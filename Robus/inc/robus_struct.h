/******************************************************************************
 * @file robus_struct
 * @brief definition protocole robus structure
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _ROBUS_STRUCT_H_
#define _ROBUS_STRUCT_H_

#include "stdint.h"
#include "config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/******************************************************************************
 * @struct memory_stats_t
 * @brief store informations about RAM occupation
 ******************************************************************************/
typedef struct __attribute__((__packed__))
{
    uint8_t msg_stack_ratio;
    uint8_t luos_stack_ratio;
    uint8_t msg_drop_number;
} memory_stats_t;

/*
 * This structure is used to get the message addressing mode list.
 */
typedef enum
{
    ID,        /*!< Unique or virtual ID, used to send something to only one container. */
    IDACK,     /*!< Unique or virtual ID with reception Acknoledgment (ACK). */
    TYPE,      /*!< Type mode, used to send something to all container of the same type. */
    BROADCAST, /*!< Broadcast mode, used to send something to everybody. */
    MULTICAST, /*!< Multicast mode, used to send something to multiple containers. */
    NODEID,    /*!< Node mode, used to send something to all containers of a node. */
    NODEIDACK  /*!< Node mode with reception Acknoledgment (ACK). */
} target_mode_t;

/* This structure is used specify data and destination of datas.
 * please refer to the documentation
 */
typedef struct __attribute__((__packed__))
{
    union
    {
        struct __attribute__((__packed__))
        {
            uint16_t protocol : 4;    /*!< Protocol version. */
            uint16_t target : 12;     /*!< Target address, it can be (ID, Multicast/Broadcast, Type). */
            uint16_t target_mode : 4; /*!< Select targeting mode (ID, ID+ACK, Multicast/Broadcast, Type). */
            uint16_t source : 12;     /*!< Source address, it can be (ID, Multicast/Broadcast, Type). */
            uint8_t cmd;              /*!< msg definition. */
            uint16_t size;            /*!< Size of the data field. */
        };
        uint8_t unmap[7]; /*!< Unmaped form. */
    };
} header_t;

/* This structure is used to receive or send messages between containers in slave
 * and master mode.
 * please refer to the documentation
 */
typedef struct __attribute__((__packed__))
{
    union
    {
        struct __attribute__((__packed__))
        {
            header_t header;                       /*!< Header filed. */
            uint8_t data[MAX_DATA_MSG_SIZE]; /*!< Data with size known. */
        };
        uint8_t stream[sizeof(header_t) + MAX_DATA_MSG_SIZE]; /*!< unmaped option. */
    };
} msg_t;

/* This structure is used to manage virtual containers
 * please refer to the documentation
 */
typedef struct __attribute__((__packed__))
{

    // Container infomations
    uint16_t id;   /*!< Container ID. */
    uint16_t type; /*!< Container type. */

    // Variables
    uint16_t max_multicast_target;                         /*!< Position pointer of the last multicast target. */
    uint16_t multicast_target_bank[MAX_MULTICAST_ADDRESS]; /*!< multicast target bank. */
    uint16_t dead_container_spotted;                       /*!< The ID of a container that don't reply to a lot of ACK msg */
} ll_container_t;

/******************************************************************************
 * @struct error_return_t
 * @brief Return function error global convention
 ******************************************************************************/
typedef enum
{
    SUCESS,     /*!< function work properly. */
    FAIL = 0xFF /*!< function fail. */
} error_return_t;

/******************************************************************************
 * @struct node_t
 * @brief node informations structure
 ******************************************************************************/
typedef struct __attribute__((__packed__))
{
    union
    {
        struct __attribute__((__packed__))
        {
            struct __attribute__((__packed__))
            {
                uint16_t node_id : 12;  /*!< Node id */
                uint16_t certified : 4; /*!< True if the node have a certificate */
            };
            uint16_t port_table[NBR_PORT]; /*!< Phisical port connections */
        };
        uint8_t unmap[NBR_PORT + 2]; /*!< Uncmaped form. */
    };
} node_t;

typedef enum
{
    // protocol level command
    WRITE_NODE_ID,   /*!< Get and save a new given node ID. */
    RESET_DETECTION, /*!< Reset detection*/
    SET_BAUDRATE,    /*!< Set Robus baudrate*/
    ROBUS_PROTOCOL_NB,
} robus_cmd_t;

typedef void (*RX_CB)(ll_container_t *ll_container, msg_t *msg);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/

#endif /* _ROBUS_STRUCT_H_ */
