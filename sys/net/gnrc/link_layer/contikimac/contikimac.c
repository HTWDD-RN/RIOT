#include "periph/rtt.h"
#include "net/gnrc/contikimac/contikimac.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#ifndef LOG_LEVEL
/**
 * @brief Default log level define
 */
#define LOG_LEVEL LOG_INFO //LOG_WARNING
#endif

#include "log.h"

/**
 * @brief  CONTIKIMAC thread's PID
 */
kernel_pid_t contikimac_pid;

bool packet_received;
bool sending_broadcast;
xtimer_t xtimer;

static void contikimac_set_netdev_state(gnrc_netdev_t *gnrc_netdev, netopt_state_t devstate);
static bool contikimac_perform_cca_phase(netdev_t *dev);
static void _pass_on_packet(gnrc_pktsnip_t *pkt);
static void rtt_cb(void *arg);
static void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev);

void contikimac_set_netdev_state(gnrc_netdev_t *gnrc_netdev, netopt_state_t devstate)
{
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_STATE, &devstate, sizeof(devstate));
}

/**
 * @brief			Performs a cca phase.
 *
 * @param[in] dev	Pointer to radio driver structure.
 * 
 * With at86rf2xx a call to dev->driver->get for NETOPT_IS_CHANNEL_CLR takes 914 us. The actual cca measurement duration is 128 us at 2.4 GHz.
 *
 * @return          True, if a packet was deteckted.
 */
bool contikimac_perform_cca_phase(netdev_t *dev)
{
	netopt_enable_t channel_free;
	xtimer_ticks32_t last_wakeup;

	for (int ccas = 0; ccas < GNRC_CONTIKIMAC_CCA_COUNT_MAX; ccas++) {
		if (ccas > 0) {
			last_wakeup = xtimer_now();
			xtimer_periodic_wakeup(&last_wakeup, GNRC_CONTIKIMAC_CCA_SLEEP_TIME_US);
		}

		/* Uncomment the two LED0-1 lines below, if you want to measure the time a cca call takes on pin PA19 (LED0 pin, board samr21-xpro).
		 * Remember to comment out all other LED0-* lines. Note that LED0 is low active, that means, switching the LED off let become pin PA19 high. */
		//LED0_OFF; // LED0-1
		dev->driver->get(dev, NETOPT_IS_CHANNEL_CLR, &channel_free, sizeof(channel_free));
		//LED0_ON; // LED0-1

		if (!channel_free) return true;
	}
	
	return false;
}

/**
 * @brief   RTT call back function.
 *
 */
static void rtt_cb(void *arg)
{
    msg_t msg;

    msg.content.value = ((uint32_t) arg) & 0xffff;
    msg.type = GNRC_CONTIKIMAC_EVENT_RTT_TYPE;
    msg_send(&msg, contikimac_pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev)
{
    uint32_t alarm;

    switch (event & 0xffff) {
    	/* A new CCA phase starts. */
    	case GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE:
    		gnrc_netdev->contikimac.last_wakeup = rtt_get_counter();
    		
    		if (gnrc_netdev->contikimac.state != GNRC_CONTIKIMAC_TRANSMITTING && // If we are transmitting, postpone cca phase. 
    				contikimac_perform_cca_phase(gnrc_netdev->dev)) { // A packet was detected.
    			LED0_TOGGLE; // LED0-0: Uncomment this line, if you want to toggle LED0 every time a packet was detected. Remember to comment out all other LED0-* lines.
    			contikimac_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
    			alarm = rtt_get_counter() + GNRC_CONTIKIMAC_LISTEN_TIME_AFTER_PACKET_DETECTED;
    			rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_CONTIKIMAC_EVENT_RTT_PACKET_DETECTED);
    		} else { // No packet was detected.
    			alarm = rtt_get_counter() + GNRC_CONTIKIMAC_CYCLE_TIME;
    			rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE);
    		}
    		
    		break;
    	/* After wait time after a packet was detected. */
    	case GNRC_CONTIKIMAC_EVENT_RTT_PACKET_DETECTED:
    		// Schedule next CCA phase and let the radio sleep again.
    		alarm = gnrc_netdev->contikimac.last_wakeup + GNRC_CONTIKIMAC_CYCLE_TIME;
    		rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE);
    		contikimac_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
    		break;
    		
        default:
            break;
    }
}

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) dev->context;

    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\n", event);
        switch(event) {
            case NETDEV_EVENT_RX_COMPLETE:
                {
                    gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);

                    if (pkt) {
                    	DEBUG("[CONTIKIMAC] Received packet of length %u.\n", gnrc_pkt_len(pkt));
                        _pass_on_packet(pkt);
                    }

                    break;
                }
            case NETDEV_EVENT_TX_COMPLETE:
            	DEBUG("[CONTIKIMAC] A packet was successfully transmitted.\n");
            	packet_received = true;
#ifdef MODULE_NETSTATS_L2
				dev->stats.tx_success++;
#endif
            	break;
#ifdef MODULE_NETSTATS_L2
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
#endif
            default:
                DEBUG("gnrc_netdev: warning: unhandled event %u.\n", event);
        }
    }
}

static void _pass_on_packet(gnrc_pktsnip_t *pkt)
{
    /* throw away packet if no one is interested */
    if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
        DEBUG("gnrc_netdev: unable to forward packet of type %i\n", pkt->type);
        gnrc_pktbuf_release(pkt);
        return;
    }
}

/**
 * @brief   Startup code and event loop of the ContikiMAC layer
 *
 * @param[in] args          expects a pointer to the underlying netdev device
 *
 * @return                  never returns
 */
static void *_contikimac_thread(void *args)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *)args;
    netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[GNRC_CONTIKIMAC_IPC_MSG_QUEUE_SIZE];
	
    LOG_INFO("[CONTIKIMAC] Starting CONTIKIMAC\n");

    /* RTT is used for scheduling wakeup */
    rtt_init();

    /* Store pid globally, so that IRQ can use it to send msg */
    contikimac_pid = thread_getpid();

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, GNRC_CONTIKIMAC_IPC_MSG_QUEUE_SIZE);

    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void *) gnrc_netdev;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);

    /* Enable RX- and TX-started interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    dev->driver->set(dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    uint16_t src_len = 8;
    dev->driver->set(dev, NETOPT_SRC_LEN, &src_len, sizeof(src_len));

    /* Get own address from netdev */
    gnrc_netdev->l2_addr_len = dev->driver->get(dev, NETOPT_ADDRESS_LONG, &gnrc_netdev->l2_addr, IEEE802154_LONG_ADDRESS_LEN);

    assert(gnrc_netdev->l2_addr_len > 0);

    /* Initialize broadcast sequence number. This at least differs from board
     * to board */
    //gnrc_netdev->tx.bcast_seqnr = gnrc_netdev->l2_addr[0];

    /* Put radio to sleep and schedule first CCA phase using RTT. */
    contikimac_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
	gnrc_netdev->contikimac.last_wakeup = rtt_get_counter();
	rtt_handler(GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE, gnrc_netdev);
	
    /* start the event loop */
    while (1) {
        msg_receive(&msg);

        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
        	/* RTT raised an interrupt */
			case GNRC_CONTIKIMAC_EVENT_RTT_TYPE:
				rtt_handler(msg.content.value, gnrc_netdev);
				break;
            case NETDEV_MSG_TYPE_EVENT:
                DEBUG("[CONTIKIMAC] GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("[CONTIKIMAC] GNRC_NETAPI_MSG_TYPE_SND received\n");
                gnrc_pktsnip_t *pkt = msg.content.ptr;
                gnrc_netdev->send(gnrc_netdev, pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("[CONTIKIMAC] GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("[CONTIKIMAC] response of netdev->set: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("[CONTIKIMAC] GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("[CONTIKIMAC] response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            default:
                DEBUG("[CONTIKIMAC] Unknown command %" PRIu16 "\n", msg.type);
                break;
        }
    }
    
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_contikimac_init(char *stack, int stacksize, char priority, const char *name, gnrc_netdev_t *dev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (dev == NULL || dev->dev == NULL) {
        LOG_ERROR("ERROR: [CONTIKIMAC] No netdev supplied or driver not set\n");
        return -ENODEV;
    }

    /* create new LWMAC thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST, _contikimac_thread, (void *)dev, name);
    if (res <= 0) {
        LOG_ERROR("ERROR: [CONTIKIMAC] Couldn't create thread\n");
        return -EINVAL;
    }
    
    return res;
}
