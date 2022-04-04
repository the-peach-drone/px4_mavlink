#pragma once
// MESSAGE VEHICLE_STATUS PACKING

#define MAVLINK_MSG_ID_VEHICLE_STATUS 180


typedef struct __mavlink_vehicle_status_t {
 uint32_t condition; /*<  */
} mavlink_vehicle_status_t;

#define MAVLINK_MSG_ID_VEHICLE_STATUS_LEN 4
#define MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN 4
#define MAVLINK_MSG_ID_180_LEN 4
#define MAVLINK_MSG_ID_180_MIN_LEN 4

#define MAVLINK_MSG_ID_VEHICLE_STATUS_CRC 197
#define MAVLINK_MSG_ID_180_CRC 197



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VEHICLE_STATUS { \
    180, \
    "VEHICLE_STATUS", \
    1, \
    {  { "condition", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_vehicle_status_t, condition) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VEHICLE_STATUS { \
    "VEHICLE_STATUS", \
    1, \
    {  { "condition", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_vehicle_status_t, condition) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param condition  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, condition);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#else
    mavlink_vehicle_status_t packet;
    packet.condition = condition;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
}

/**
 * @brief Pack a vehicle_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param condition  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, condition);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#else
    mavlink_vehicle_status_t packet;
    packet.condition = condition;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
}

/**
 * @brief Encode a vehicle_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vehicle_status_t* vehicle_status)
{
    return mavlink_msg_vehicle_status_pack(system_id, component_id, msg, vehicle_status->condition);
}

/**
 * @brief Encode a vehicle_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vehicle_status_t* vehicle_status)
{
    return mavlink_msg_vehicle_status_pack_chan(system_id, component_id, chan, msg, vehicle_status->condition);
}

/**
 * @brief Send a vehicle_status message
 * @param chan MAVLink channel to send the message
 *
 * @param condition  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vehicle_status_send(mavlink_channel_t chan, uint32_t condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, condition);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, buf, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#else
    mavlink_vehicle_status_t packet;
    packet.condition = condition;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#endif
}

/**
 * @brief Send a vehicle_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vehicle_status_send_struct(mavlink_channel_t chan, const mavlink_vehicle_status_t* vehicle_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vehicle_status_send(chan, vehicle_status->condition);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, (const char *)vehicle_status, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_VEHICLE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vehicle_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, condition);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, buf, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#else
    mavlink_vehicle_status_t *packet = (mavlink_vehicle_status_t *)msgbuf;
    packet->condition = condition;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, (const char *)packet, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_STATUS UNPACKING


/**
 * @brief Get field condition from vehicle_status message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_vehicle_status_get_condition(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a vehicle_status message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_vehicle_status_decode(const mavlink_message_t* msg, mavlink_vehicle_status_t* vehicle_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vehicle_status->condition = mavlink_msg_vehicle_status_get_condition(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VEHICLE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_VEHICLE_STATUS_LEN;
        memset(vehicle_status, 0, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
    memcpy(vehicle_status, _MAV_PAYLOAD(msg), len);
#endif
}
