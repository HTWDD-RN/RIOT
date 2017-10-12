#ifndef NET_GNRC_CONTIKIMAC_CONTIKIMAC_H
#define NET_GNRC_CONTIKIMAC_CONTIKIMAC_H

#include "kernel_types.h"
#include "net/gnrc/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ContikiMAC's default channel check rate in Hz.
 *
 * ContikiMAC's default channel check rate is 8 Hz, hence the cca phase performs every 125 ms.
 *
 */
#ifndef GNRC_CONTIKIMAC_CHANNEL_CHECK_RATE
#define GNRC_CONTIKIMAC_CHANNEL_CHECK_RATE	(8U)
#endif

/**
 * @brief ContikiMAC's cycle time.
 *
 * The cycle time is the time between two successive cca phases.
 *
 */
#define GNRC_CONTIKIMAC_CYCLE_TIME RTT_FREQUENCY / GNRC_CONTIKIMAC_CHANNEL_CHECK_RATE

/**
 * @brief Turn on (1) or off (0) ContikiMAC's radio duty cycling.
 *
 * Mains powered devices, e.g. border routers, don't need to do radio duty cycling. They perform better without.
 *
 */
#ifndef GNRC_CONTIKIMAC_RDC_ON
#define GNRC_CONTIKIMAC_RDC_ON 1
#endif

/**
 * @brief Number of CCAs ContikiMAC performs each cca phase.
 *
 * Each cycle ContikiMAC performs a cca phase consists of GNRC_CONTIKIMAC_CCA_COUNT_MAX
 * (usually two) subsequent CCAs.
 *
 */
#ifndef GNRC_CONTIKIMAC_CCA_COUNT_MAX
#define GNRC_CONTIKIMAC_CCA_COUNT_MAX (2U)
#endif

/**
 * @brief The sleep time between each successive CCA in a cca phase. 
 *
 * The sleep time have to be set in microseconds. The default value is 500 us.
 *
 */
#ifndef GNRC_CONTIKIMAC_CCA_SLEEP_TIME_US
#define GNRC_CONTIKIMAC_CCA_SLEEP_TIME_US (500U)
#endif

/**
 * @brief Time we stat awake after packet detection.
 *
 * Time we stat awake after packet detection to receive a full data packet. Default is 12.5 ms.
 *
 */
#ifndef GNRC_CONTIKIMAC_LISTEN_TIME_AFTER_PACKET_DETECTED
#define GNRC_CONTIKIMAC_LISTEN_TIME_AFTER_PACKET_DETECTED RTT_FREQUENCY / 80
#endif

/**
 * @brief Maximum duration a packet will repeatedly send (maximum strobe time).
 *
 */
#ifndef GNRC_CONTIKIMAC_STROBE_TIME
#define GNRC_CONTIKIMAC_STROBE_TIME XTIMER_HZ / GNRC_CONTIKIMAC_CHANNEL_CHECK_RATE
#endif

/**
 * @brief Time between two successive packet transmissions.
 * 
 * The time between two successive packet transmissions, e.g. during sending a strobe. Default is 0.4 ms.
 *
 */
#ifndef GNRC_CONTIKIMAC_INTER_PACKET_INTERVAL_US
#define GNRC_CONTIKIMAC_INTER_PACKET_INTERVAL_US (400U)
#endif

/**
 * @brief Default message queue size to use for the LWMAC thread.
 *
 * The value of this macro should be enough for supporting the manipulation of
 * LWMAC.
 *
 */
#ifndef GNRC_CONTIKIMAC_IPC_MSG_QUEUE_SIZE
#define GNRC_CONTIKIMAC_IPC_MSG_QUEUE_SIZE        (8U)
#endif

kernel_pid_t gnrc_contikimac_init(char *stack, int stacksize, char priority, const char *name, gnrc_netdev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_CONTIKIMAC_CONTIKIMAC_H */