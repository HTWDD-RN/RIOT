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
 * @brief   CONTIKIMAC send strobe packet.
 */
#define GNRC_CONTIKIMAC_EVENT_SND_TYPE  (0x4400)

///**
// * @brief   CONTIKIMAC RTT start event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_RTT_START           (0x4301)
//
///**
// * @brief   CONTIKIMAC RTT stop event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_RTT_STOP            (0x4302)
//
///**
// * @brief   CONTIKIMAC RTT pause event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_RTT_PAUSE           (0x4303)
//
///**
// * @brief   CONTIKIMAC RTT resume event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_RTT_RESUME          (0x4304)
//
///**
// * @brief   CONTIKIMAC RTT wakeup pending event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_RTT_WAKEUP_PENDING  (0x4305)
//
///**
// * @brief   CONTIKIMAC RTT sleep pending event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_RTT_SLEEP_PENDING   (0x4306)
//
///**
// * @brief   CONTIKIMAC timeout event type.
// */
//#define GNRC_CONTIKIMAC_EVENT_TIMEOUT_TYPE        (0x4400)
//
///**
// * @brief   CONTIKIMAC duty-cycle active flag.
// *
// * Keep track of duty cycling to avoid late RTT events after stopping.
// */
//#define GNRC_CONTIKIMAC_DUTYCYCLE_ACTIVE          (0x01)
//
///**
// * @brief   CONTIKIMAC needs reschedule flag.
// *
// * Used internally for rescheduling state machine update, e.g. after state
// * transition caused in update.
// */
//#define GNRC_CONTIKIMAC_NEEDS_RESCHEDULE          (0x02)
//
///**
// * @brief   CONTIKIMAC check radio's on/off state flag.
// */
//#define GNRC_CONTIKIMAC_RADIO_IS_ON               (0x04)
//
///**
// * @brief   Enable/disable duty-cycle record and print out.
// *          Set "1" to enable, set "0" to disable.
// */
//#ifndef GNRC_CONTIKIMAC_ENABLE_DUTYCYLE_RECORD
//#define GNRC_CONTIKIMAC_ENABLE_DUTYCYLE_RECORD    (0U)
//#endif
//
///**
// * @brief The default largest number of parallel timeouts in CONTIKIMAC
// */
//#ifndef GNRC_CONTIKIMAC_TIMEOUT_COUNT
//#define GNRC_CONTIKIMAC_TIMEOUT_COUNT             (3U)
//#endif

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

///**
// * @brief   TX states of CONTIKIMAC
// */
//typedef enum {
//    GNRC_CONTIKIMAC_TX_STATE_STOPPED,           /**< Tx schedule stopped, stop sending packet */
//    GNRC_CONTIKIMAC_TX_STATE_INIT,              /**< Initiate transmission */
//    GNRC_CONTIKIMAC_TX_STATE_SEND_BROADCAST,    /**< directly goes to SUCCESSFUL or FAILED when finished */
//    GNRC_CONTIKIMAC_TX_STATE_SEND_WR,           /**< Send a wakeup request */
//    GNRC_CONTIKIMAC_TX_STATE_WAIT_WR_SENT,      /**< Wait until WR sent to set timeout */
//    GNRC_CONTIKIMAC_TX_STATE_WAIT_FOR_WA,       /**< Wait for dest node's wakeup ackknowledge */
//    GNRC_CONTIKIMAC_TX_STATE_SEND_DATA,         /**< Send the actual payload data */
//    GNRC_CONTIKIMAC_TX_STATE_WAIT_FEEDBACK,     /**< Wait if packet was ACKed */
//    GNRC_CONTIKIMAC_TX_STATE_SUCCESSFUL,        /**< Transmission has finished successfully */
//    GNRC_CONTIKIMAC_TX_STATE_FAILED             /**< Payload data couldn't be delivered to dest */
//} gnrc_contikimac_tx_state_t;
//
///**
// * @brief   Static initializer for gnrc_contikimac_tx_state_t.
// */
//#define GNRC_CONTIKIMAC_TX_STATE_INITIAL GNRC_CONTIKIMAC_TX_STATE_STOPPED
//
///**
// * @brief   RX states of CONTIKIMAC
// */
//typedef enum {
//    GNRC_CONTIKIMAC_RX_STATE_STOPPED,       /**< Rx schedule stopped */
//    GNRC_CONTIKIMAC_RX_STATE_INIT,          /**< Initiate reception */
//    GNRC_CONTIKIMAC_RX_STATE_WAIT_FOR_WR,   /**< Wait for a wakeup request */
//    GNRC_CONTIKIMAC_RX_STATE_SEND_WA,       /**< Send wakeup ackknowledge to requesting node */
//    GNRC_CONTIKIMAC_RX_STATE_WAIT_WA_SENT,  /**< Wait until WA sent to set timeout */
//    GNRC_CONTIKIMAC_RX_STATE_WAIT_FOR_DATA, /**< Wait for actual payload data */
//    GNRC_CONTIKIMAC_RX_STATE_SUCCESSFUL,    /**< Recption has finished successfully */
//    GNRC_CONTIKIMAC_RX_STATE_FAILED         /**< Reception over, but nothing received */
//} gnrc_contikimac_rx_state_t;
//
///**
// * @brief   Static initializer for gnrc_contikimac_rx_state_t.
// */
//#define GNRC_CONTIKIMAC_RX_STATE_INITIAL GNRC_CONTIKIMAC_RX_STATE_STOPPED
//
///**
// * @brief   CONTIKIMAC uninitialized phase value
// */
//#define GNRC_CONTIKIMAC_PHASE_UNINITIALIZED   (0)
//
///**
// * @brief   CONTIKIMAC max phase value
// */
//#define GNRC_CONTIKIMAC_PHASE_MAX             (-1)

/**
 * @brief   CONTIKIMAC xtimer types
 */
typedef enum {
    GNRC_CONTIKIMAC_XTIMER_SEND_STROBE,              /**< Send packet that belongs to a strobe */
//    GNRC_CONTIKIMAC_TIMEOUT_WR,                    /**< WR timeout, waiting WA */
//    GNRC_CONTIKIMAC_TIMEOUT_NO_RESPONSE,           /**< Maximum WR duration timeout awaiting WA */
//    GNRC_CONTIKIMAC_TIMEOUT_DATA,                  /**< Timeout awaiting data packet from receiver */
//    GNRC_CONTIKIMAC_TIMEOUT_WAIT_DEST_WAKEUP,      /**< Timeout for waiting receiver's wake-up phase */
//    GNRC_CONTIKIMAC_TIMEOUT_WAKEUP_PERIOD,         /**< Wake up period timeout for going to sleep */
//    GNRC_CONTIKIMAC_TIMEOUT_NEXT_BROADCAST,        /**< Timeout for waiting to send the next broadcast packet */
//    GNRC_CONTIKIMAC_TIMEOUT_BROADCAST_END,         /**< Timeout awaiting the end of the whole broadcast period */
} gnrc_contikimac_xtimer_type_t;

///**
// * @brief   CONTIKIMAC timeout structure
// */
//typedef struct {
//    xtimer_t timer;                 /**< xtimer entity */
//    msg_t msg;                      /**< msg entity */
//    bool expired;                   /**< If type != DISABLED, this indicates if timeout has expired */
//    gnrc_contikimac_timeout_type_t type; /**< timeout type */
//} gnrc_contikimac_timeout_t;

/**
 * @brief   CONTIKIMAC specific structure for storing internal states.
 */
typedef struct contikimac {
    gnrc_contikimac_state_t state;                                /**< Internal state of MAC layer */
    xtimer_ticks32_t strobe_start;							/** Time when the current strobe was started */
    uint32_t last_wakeup;                                    /**< Used to calculate wakeup times */
    uint8_t contikimac_info;                                      /**< CONTIKIMAC's internal informations (flags) */
//    gnrc_contikimac_timeout_t timeouts[GNRC_CONTIKIMAC_TIMEOUT_COUNT]; /**< Store timeouts used for protocol */

//#if (GNRC_CONTIKIMAC_ENABLE_DUTYCYLE_RECORD == 1)
//    /* Parameters for recording duty-cycle */
//    uint32_t last_radio_on_time_ticks;                       /**< The last time in ticks when radio is on */
//    uint32_t radio_off_time_ticks;                           /**< The time in ticks when radio is off */
//    uint32_t system_start_time_ticks;                        /**< The time in ticks when chip is started */
//    uint32_t awake_duration_sum_ticks;                       /**< The sum of time in ticks when radio is on */
//    uint32_t pkt_start_sending_time_ticks;                   /**< The time in ticks when the packet is started
//                                                                  to be sent */
//#endif
} gnrc_contikimac_t;

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_CONTIKIMAC_TYPES_H */
