#ifndef NET_GNRC_CONTIKIMAC_TYPES_H
#define NET_GNRC_CONTIKIMAC_TYPES_H

//#include "msg.h"
//#include "xtimer.h"
//#include "net/gnrc/contikimac/hdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   CONTIKIMAC RTT event type.
 */
#define GNRC_CONTIKIMAC_EVENT_RTT_TYPE            (0x4300)

/**
 * @brief   CONTIKIMAC RTT perform CCA phase event type.
 */
#define GNRC_CONTIKIMAC_EVENT_RTT_PERFORM_CCAPHASE  (0x4301)

/**
 * @brief   CONTIKIMAC RTT packet was detected event type.
 */
#define GNRC_CONTIKIMAC_EVENT_RTT_PACKET_DETECTED  (0x4302)

/**
 * @brief   Internal states of CONTIKIMAC
 */
typedef enum {
    GNRC_CONTIKIMAC_UNDEF = -1,     /**< Undefined state of CONTIKIMAC */
    GNRC_CONTIKIMAC_STOPPED,        /**< CONTIKIMAC's main state machine has been stopped */
    GNRC_CONTIKIMAC_START,          /**< Start CONTIKIMAC's main state machine */
    GNRC_CONTIKIMAC_STOP,           /**< Stop CONTIKIMAC's main state machine */
    GNRC_CONTIKIMAC_RESET,          /**< Reset CONTIKIMAC's main state machine */
    GNRC_CONTIKIMAC_LISTENING,      /**< Listen the channel for receiving packets */
    GNRC_CONTIKIMAC_RECEIVING,      /**< RX is handled in own state machine */
    GNRC_CONTIKIMAC_TRANSMITTING,   /**< TX is handled in own state machine */
    GNRC_CONTIKIMAC_SLEEPING,       /**< Turn off radio to conserve power */
    GNRC_CONTIKIMAC_STATE_COUNT     /**< Count of CONTIKIMAC's states */
} gnrc_contikimac_state_t;

/**
 * @brief   CONTIKIMAC specific structure for storing internal states.
 */
typedef struct contikimac {
    gnrc_contikimac_state_t state;                                /**< Internal state of MAC layer */
    xtimer_ticks32_t strobe_start;							/** Time when the current strobe was started */
    uint32_t last_wakeup;                                    /**< Used to calculate wakeup times */
    uint8_t contikimac_info;                                      /**< CONTIKIMAC's internal informations (flags) */
} gnrc_contikimac_t;

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_CONTIKIMAC_TYPES_H */
