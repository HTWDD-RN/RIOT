#include "periph/rtt.h"
#include "net/gnrc/contikimac/contikimac.h"

#define ENABLE_DEBUG    (0)
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
xtimer_t timer1;

static bool contikimac_send(gnrc_netdev_t *netdev, gnrc_pktsnip_t *pkt);
static void contikimac_set_netdev_state(gnrc_netdev_t *gnrc_netdev, netopt_state_t devstate);
static bool contikimac_perform_cca_phase(netdev_t *dev);
static void _pass_on_packet(gnrc_pktsnip_t *pkt);
static void contikimac_set_state(gnrc_netdev_t *gnrc_netdev, gnrc_contikimac_state_t newstate);
static void rtt_cb(void *arg);
static void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev);

/**
 * @brief				ContikiMAC's send function.
 *
 * @param[in] netdev	Pointer to the underlying netdev device.
 *
 * @return          	True, if the packet was successful transmitted.
 */
bool contikimac_send(gnrc_netdev_t *netdev, gnrc_pktsnip_t *pkt)
{
	xtimer_ticks32_t transmission_start, now;

	gnrc_contikimac_state_t old_state = netdev->contikimac.state;
	netdev->contikimac.state = GNRC_CONTIKIMAC_TRANSMITTING;
	
	transmission_start = xtimer_now();
	
	/* Disable things that make packet sending slow. */
	netopt_enable_t netop = NETOPT_DISABLE;
	netdev->dev->driver->set(netdev->dev, NETOPT_AUTOACK, &netop, sizeof(netop));
	netdev->dev->driver->set(netdev->dev, NETOPT_CSMA, &netop, sizeof(netop));
	netdev->dev->driver->set(netdev->dev, NETOPT_TX_START_IRQ, &netop, sizeof(netop));
	
	packet_received = false;
	DEBUG("[CONTIKIMAC] Sending packet!\n");
	
	do {
		/* Don't let the packet be released yet, we want to send it again */
		gnrc_pktbuf_hold(pkt, 1);
		
		//LED0_ON;
		netdev->send(netdev, pkt);
		//LED0_OFF;
		
		//now = xtimer_now();
		LED0_ON;
		//xtimer_periodic_wakeup(&now, GNRC_CONTIKIMAC_INTER_PACKET_INTERVAL_US);
		xtimer_usleep(GNRC_CONTIKIMAC_INTER_PACKET_INTERVAL_US - 60);
		LED0_OFF;
		
		now = xtimer_now();
		//DEBUG("[CONTIKIMAC] Packet sent! %i\n", status);
		
	} while(!packet_received && ((now.ticks32 - transmission_start.ticks32) < GNRC_CONTIKIMAC_STROBE_TIME));
	
	gnrc_pktbuf_release(pkt);
	netdev->contikimac.state = old_state;
	
	DEBUG("[CONTIKIMAC] Sending packet done!\n");
	
	return true;
}

/* Not so well working version of ContikiMACs sending procedure. This procedure schedules itselfe repeatedly
 * using xtimer and IPC until all packets were send. */
//
// Alle 25666 us, erster Aufruf dauert 10544 us, alle weiteren ~6838 us. 25656 us zwischen den Aufrufen.
//bool contikimac_send(gnrc_netdev_t *netdev, gnrc_pktsnip_t *pkt)
//{
//	LED0_ON;
//	static msg_t msg;
//	//static xtimer_ticks32_t now1;
//	
//	/* We are not already transmitting, so it must be the first strobe packet. */
//	if (netdev->contikimac.state != GNRC_CONTIKIMAC_TRANSMITTING) {
//		netdev->contikimac.state = GNRC_CONTIKIMAC_TRANSMITTING;
//		
//		/* check if the packet is for broadcast */
//		if (gnrc_netif_hdr_get_flag(pkt) & (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
//			DEBUG("[CONTIKIMAC] We've got a broadcast packet.\n");
//			sending_broadcast = true;
//		} else {
//			DEBUG("[CONTIKIMAC] We've got a unicast packet.\n");
//			sending_broadcast = false;
//		}
//		
//		/* Disable things that make packet sending slow. */
//		netopt_enable_t netop = NETOPT_DISABLE;
//		netdev->dev->driver->set(netdev->dev, NETOPT_AUTOACK, &netop, sizeof(netop));
//		netdev->dev->driver->set(netdev->dev, NETOPT_CSMA, &netop, sizeof(netop));
//		netdev->dev->driver->set(netdev->dev, NETOPT_TX_START_IRQ, &netop, sizeof(netop));
//		
//		netdev->dev->driver->set(netdev->dev, NETOPT_TX_END_IRQ, &netop, sizeof(netop));
//		
//		netdev->contikimac.strobe_start = xtimer_now();
//		packet_received = false;
//	} /*else { // DEBUG!
//		xtimer_ticks32_t now2 = xtimer_now();
//		DEBUG("[CONTIKIMAC] contikimac_send() %lu us between calls.\n", now2.ticks32 - now1.ticks32);
//	}*/
//	
//	gnrc_pktbuf_hold(pkt, 1);
//	
//	//xtimer_ticks32_t now = xtimer_now();
//	netdev->send(netdev, pkt); // 109 Bytes => 6803 us
//	//xtimer_ticks32_t now2 = xtimer_now();
//	//DEBUG("[CONTIKIMAC] Sending packet done! %lu\n", now2.ticks32 - now.ticks32); // -> 6803 us
//	
//	xtimer_ticks32_t now = xtimer_now();
//	
//	/* No ACK was received so far und strobe is not over now. Schedule the next packet. */
//	if ((!packet_received || sending_broadcast) && ((now.ticks32 - netdev->contikimac.strobe_start.ticks32) < GNRC_CONTIKIMAC_STROBE_TIME)) {
//		msg.type = GNRC_CONTIKIMAC_EVENT_SND_TYPE;
//		msg.content.ptr = pkt;
//		//msg_send(&msg, contikimac_pid);
//		//msg_send_to_self(&msg);
//		
//		xtimer_set_msg(&timer1, GNRC_CONTIKIMAC_INTER_PACKET_INTERVAL_US, &msg, contikimac_pid);
//	} else {
//		gnrc_pktbuf_release(pkt);
//		netdev->contikimac.state = 0; // TODO: Set the right state.
//	}
//	
//	//now1 = xtimer_now(); // DEBUG!
//	LED0_OFF;
//	return true;
//}

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

		dev->driver->get(dev, NETOPT_IS_CHANNEL_CLR, &channel_free, sizeof(channel_free));

		if (!channel_free) return true;
	}
	
	return false;
}

void contikimac_set_state(gnrc_netdev_t *gnrc_netdev, gnrc_contikimac_state_t newstate)
{
//    gnrc_contikimac_state_t oldstate = gnrc_netdev->lwmac.state;
//
//    if (newstate == oldstate) {
//        return;
//    }
//
//    if (newstate >= GNRC_LWMAC_STATE_COUNT) {
//        LOG_ERROR("ERROR: [LWMAC] Trying to set invalid state %u\n", newstate);
//        return;
//    }
//
//    /* Already change state, but might be reverted to oldstate when needed */
//    gnrc_netdev->lwmac.state = newstate;
//
//    /* Actions when leaving old state */
//    switch (oldstate) {
//        case GNRC_LWMAC_RECEIVING:
//        case GNRC_LWMAC_TRANSMITTING: {
//            /* Enable duty cycling again */
//            rtt_handler(GNRC_LWMAC_EVENT_RTT_RESUME, gnrc_netdev);
//#if (GNRC_LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
//            /* Output duty-cycle ratio */
//            uint64_t duty;
//            duty = (uint64_t) rtt_get_counter();
//            duty = ((uint64_t) gnrc_netdev->lwmac.awake_duration_sum_ticks) * 100 /
//                   (duty - (uint64_t)gnrc_netdev->lwmac.system_start_time_ticks);
//            printf("[LWMAC]: achieved duty-cycle: %lu %% \n", (uint32_t)duty);
//#endif
//            break;
//        }
//        case GNRC_LWMAC_SLEEPING: {
//            gnrc_lwmac_clear_timeout(gnrc_netdev, GNRC_LWMAC_TIMEOUT_WAKEUP_PERIOD);
//            break;
//        }
//        default:
//            break;
//    }
//
    /* Actions when entering new state */
    switch (newstate) {
//        /*********************** Operation states *********************************/
//        case GNRC_LWMAC_LISTENING: {
//            _gnrc_lwmac_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
//            break;
//        }
//        case GNRC_LWMAC_SLEEPING: {
//            /* Put transceiver to sleep */
//            _gnrc_lwmac_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
//            /* We may have come here through RTT handler, so timeout may still be active */
//            gnrc_lwmac_clear_timeout(gnrc_netdev, GNRC_LWMAC_TIMEOUT_WAKEUP_PERIOD);
//
//            if (gnrc_netdev_lwmac_get_phase_backoff(gnrc_netdev)) {
//                gnrc_netdev_lwmac_set_phase_backoff(gnrc_netdev, false);
//                uint32_t alarm;
//
//                rtt_clear_alarm();
//                alarm = random_uint32_range(RTT_US_TO_TICKS((3 * GNRC_LWMAC_WAKEUP_DURATION_US / 2)),
//                                            RTT_US_TO_TICKS(GNRC_LWMAC_WAKEUP_INTERVAL_US -
//                                                            (3 * GNRC_LWMAC_WAKEUP_DURATION_US / 2)));
//                LOG_WARNING("WARNING: [LWMAC] phase backoffed: %lu us\n", RTT_TICKS_TO_US(alarm));
//                gnrc_netdev->lwmac.last_wakeup = gnrc_netdev->lwmac.last_wakeup + alarm;
//                alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
//                                            RTT_US_TO_TICKS(GNRC_LWMAC_WAKEUP_INTERVAL_US));
//                rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_LWMAC_EVENT_RTT_WAKEUP_PENDING);
//            }
//
//            /* Return immediately, so no rescheduling */
//            return;
//        }
//        /* Trying to send data */
//        case GNRC_LWMAC_TRANSMITTING: {
//            rtt_handler(GNRC_LWMAC_EVENT_RTT_PAUSE, gnrc_netdev);    /**< No duty cycling while RXing */
//            _gnrc_lwmac_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);  /**< Power up netdev */
//            break;
//        }
//        /* Receiving incoming data */
//        case GNRC_LWMAC_RECEIVING: {
//            rtt_handler(GNRC_LWMAC_EVENT_RTT_PAUSE, gnrc_netdev);    /**< No duty cycling while TXing */
//            _gnrc_lwmac_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);  /**< Power up netdev */
//            break;
//        }
//        case GNRC_LWMAC_STOPPED: {
//            _gnrc_lwmac_set_netdev_state(gnrc_netdev, NETOPT_STATE_OFF);
//            break;
//        }
        /*********************** Control states ***********************************/
#if GNRC_CONTIKIMAC_RDC_ON
        case GNRC_CONTIKIMAC_START:
        		contikimac_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
        		gnrc_netdev->contikimac.last_wakeup = rtt_get_counter();
              rtt_handler(GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE, gnrc_netdev);
//            lwmac_set_state(gnrc_netdev, GNRC_LWMAC_LISTENING);
            break;
#endif /* GNRC_CONTIKIMAC_RDC_ON */
//        case GNRC_LWMAC_STOP: {
//            rtt_handler(GNRC_LWMAC_EVENT_RTT_STOP, gnrc_netdev);
//            lwmac_set_state(gnrc_netdev, GNRC_LWMAC_STOPPED);
//            break;
//        }
//        case GNRC_LWMAC_RESET: {
//            LOG_WARNING("WARNING: [LWMAC] Reset not yet implemented\n");
//            lwmac_set_state(gnrc_netdev, GNRC_LWMAC_STOP);
//            lwmac_set_state(gnrc_netdev, GNRC_LWMAC_START);
//            break;
//        }
        /**************************************************************************/
        default: {
            LOG_DEBUG("[CONTIKIMAC] No actions for entering state %u\n", newstate);
            return;
        }
    }
//
//    lwmac_schedule_update(gnrc_netdev);
}

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

//void xtimer_handler(gnrc_contikimac_xtimer_t *type, gnrc_netdev_t *gnrc_netdev)
//{
//	switch (type) {
//		case GNRC_CONTIKIMAC_XTIMER_SEND_STROBE:
//			break;
//	}
//	
//	return;
//}

void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev)
{
    uint32_t alarm;

    switch (event & 0xffff) {
    	/* A new CCA phase starts. */
    	case GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE:
    		//LED0_ON;
    		gnrc_netdev->contikimac.last_wakeup = rtt_get_counter(); //rtt_get_alarm();
    		
    		if (gnrc_netdev->contikimac.state != GNRC_CONTIKIMAC_TRANSMITTING && // If we are transmitting, postpone cca phase. 
    				contikimac_perform_cca_phase(gnrc_netdev->dev)) { // A packet was detected.
    			//LED0_TOGGLE;
    			contikimac_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
    			alarm = rtt_get_counter() + GNRC_CONTIKIMAC_LISTEN_TIME_AFTER_PACKET_DETECTED;
    			rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_CONTIKIMAC_EVENT_RTT_PACKET_DETECTED);
    		} else { // No packet was detected.
    			alarm = rtt_get_counter() + GNRC_CONTIKIMAC_CYCLE_TIME;
    			rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE);
    		}
    		
    		//LED0_OFF;
    		break;
    	/* After wait time after a packet was detected. */
    	case GNRC_CONTIKIMAC_EVENT_RTT_PACKET_DETECTED:
    		alarm = gnrc_netdev->contikimac.last_wakeup + GNRC_CONTIKIMAC_CYCLE_TIME;
    		rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE);
    		contikimac_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
    		break;
//        case GNRC_LWMAC_EVENT_RTT_WAKEUP_PENDING: {
//            /* A new cycle starts, set sleep timing and initialize related MAC-info flags. */
//            gnrc_netdev->lwmac.last_wakeup = rtt_get_alarm();
//            alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
//                                        RTT_US_TO_TICKS(GNRC_LWMAC_WAKEUP_DURATION_US));
//            rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_LWMAC_EVENT_RTT_SLEEP_PENDING);
//            gnrc_netdev_lwmac_set_quit_tx(gnrc_netdev, false);
//            gnrc_netdev_lwmac_set_quit_rx(gnrc_netdev, false);
//            gnrc_netdev_lwmac_set_phase_backoff(gnrc_netdev, false);
//            gnrc_netdev->rx.rx_bad_exten_count = 0;
//            lwmac_set_state(gnrc_netdev, GNRC_LWMAC_LISTENING);
//            break;
//        }
//        case GNRC_LWMAC_EVENT_RTT_SLEEP_PENDING: {
//            /* Set next wake-up timing. */
//            alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
//                                        RTT_US_TO_TICKS(GNRC_LWMAC_WAKEUP_INTERVAL_US));
//            rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_LWMAC_EVENT_RTT_WAKEUP_PENDING);
//            lwmac_set_state(gnrc_netdev, GNRC_LWMAC_SLEEPING);
//            break;
//        }
//        /* Set initial wake-up alarm that starts the cycle */
//        case GNRC_LWMAC_EVENT_RTT_START: {
//            LOG_DEBUG("[LWMAC] RTT: Initialize duty cycling\n");
//            alarm = rtt_get_counter() + RTT_US_TO_TICKS(GNRC_LWMAC_WAKEUP_DURATION_US);
//            rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_LWMAC_EVENT_RTT_SLEEP_PENDING);
//            gnrc_netdev_lwmac_set_dutycycle_active(gnrc_netdev, true);
//            break;
//        }
//        case GNRC_LWMAC_EVENT_RTT_STOP:
//        case GNRC_LWMAC_EVENT_RTT_PAUSE: {
//            rtt_clear_alarm();
//            LOG_DEBUG("[LWMAC] RTT: Stop duty cycling, now in state %u\n",
//                           gnrc_netdev->lwmac.state);
//            gnrc_netdev_lwmac_set_dutycycle_active(gnrc_netdev, false);
//            break;
//        }
//        case GNRC_LWMAC_EVENT_RTT_RESUME: {
//            LOG_DEBUG("[LWMAC] RTT: Resume duty cycling\n");
//            rtt_clear_alarm();
//            alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
//                                        RTT_US_TO_TICKS(GNRC_LWMAC_WAKEUP_INTERVAL_US));
//            rtt_set_alarm(alarm, rtt_cb, (void *) GNRC_LWMAC_EVENT_RTT_WAKEUP_PENDING);
//            gnrc_netdev_lwmac_set_dutycycle_active(gnrc_netdev, true);
//            break;
//        }
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
                    	//dev->contikimac.state = GNRC_CONTIKIMAC_RECEIVING;
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

//    assert(gnrc_netdev->l2_addr_len > 0);

    /* Initialize broadcast sequence number. This at least differs from board
     * to board */
//    gnrc_netdev->tx.bcast_seqnr = gnrc_netdev->l2_addr[0];

    /* Reset all timeouts just to be sure */
//    gnrc_lwmac_reset_timeouts(gnrc_netdev);

    /* Start duty cycling */
    contikimac_set_state(gnrc_netdev, GNRC_CONTIKIMAC_START);
	
    /* start the event loop */
    while (1) {
        msg_receive(&msg);

        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
        /* RTT raised an interrupt */
			case GNRC_CONTIKIMAC_EVENT_RTT_TYPE:
				if (1 /*gnrc_netdev_lwmac_get_dutycycle_active(gnrc_netdev)*/) {
					rtt_handler(msg.content.value, gnrc_netdev);
//					lwmac_schedule_update(gnrc_netdev);
				}
				else {
					LOG_DEBUG("[CONTIKIMAC] Ignoring late RTT event while dutycycling is off\n");
				}
				break;
			/* An ContikiMAC timeout occured */
			case GNRC_CONTIKIMAC_EVENT_SND_TYPE: {
				DEBUG("[CONTIKIMAC] GNRC_CONTIKIMAC_EVENT_SND_TYPE received\n");
				
				gnrc_pktsnip_t *pkt = msg.content.ptr;
				contikimac_send(gnrc_netdev, pkt);
				
				break;
			}
            case NETDEV_MSG_TYPE_EVENT:
                DEBUG("[CONTIKIMAC] GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("[CONTIKIMAC] GNRC_NETAPI_MSG_TYPE_SND received\n");
                
                if (gnrc_netdev->contikimac.state == GNRC_CONTIKIMAC_TRANSMITTING) break; // If we are still transmitting, just quit.
                		
                gnrc_pktsnip_t *pkt = msg.content.ptr;
                contikimac_send(gnrc_netdev, pkt);
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
