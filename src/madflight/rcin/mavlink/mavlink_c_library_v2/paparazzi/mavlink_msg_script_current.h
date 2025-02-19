#pragma once
// MESSAGE SCRIPT_CURRENT PACKING

#define MAVLINK_MSG_ID_SCRIPT_CURRENT 184


typedef struct __mavlink_script_current_t {
 uint16_t seq; /*<  Active Sequence*/
} mavlink_script_current_t;

#define MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN 2
#define MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN 2
#define MAVLINK_MSG_ID_184_LEN 2
#define MAVLINK_MSG_ID_184_MIN_LEN 2

#define MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC 40
#define MAVLINK_MSG_ID_184_CRC 40



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SCRIPT_CURRENT { \
    184, \
    "SCRIPT_CURRENT", \
    1, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_script_current_t, seq) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SCRIPT_CURRENT { \
    "SCRIPT_CURRENT", \
    1, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_script_current_t, seq) }, \
         } \
}
#endif

/**
 * @brief Pack a script_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Active Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_script_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#else
    mavlink_script_current_t packet;
    packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCRIPT_CURRENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
}

/**
 * @brief Pack a script_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Active Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_script_current_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#else
    mavlink_script_current_t packet;
    packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCRIPT_CURRENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#endif
}

/**
 * @brief Pack a script_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  Active Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_script_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#else
    mavlink_script_current_t packet;
    packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCRIPT_CURRENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
}

/**
 * @brief Encode a script_current struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param script_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_script_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_script_current_t* script_current)
{
    return mavlink_msg_script_current_pack(system_id, component_id, msg, script_current->seq);
}

/**
 * @brief Encode a script_current struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param script_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_script_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_script_current_t* script_current)
{
    return mavlink_msg_script_current_pack_chan(system_id, component_id, chan, msg, script_current->seq);
}

/**
 * @brief Encode a script_current struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param script_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_script_current_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_script_current_t* script_current)
{
    return mavlink_msg_script_current_pack_status(system_id, component_id, _status, msg,  script_current->seq);
}

/**
 * @brief Send a script_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  Active Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_script_current_send(mavlink_channel_t chan, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_CURRENT, buf, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
#else
    mavlink_script_current_t packet;
    packet.seq = seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_CURRENT, (const char *)&packet, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
#endif
}

/**
 * @brief Send a script_current message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_script_current_send_struct(mavlink_channel_t chan, const mavlink_script_current_t* script_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_script_current_send(chan, script_current->seq);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_CURRENT, (const char *)script_current, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_script_current_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_CURRENT, buf, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
#else
    mavlink_script_current_t *packet = (mavlink_script_current_t *)msgbuf;
    packet->seq = seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_CURRENT, (const char *)packet, MAVLINK_MSG_ID_SCRIPT_CURRENT_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN, MAVLINK_MSG_ID_SCRIPT_CURRENT_CRC);
#endif
}
#endif

#endif

// MESSAGE SCRIPT_CURRENT UNPACKING


/**
 * @brief Get field seq from script_current message
 *
 * @return  Active Sequence
 */
static inline uint16_t mavlink_msg_script_current_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a script_current message into a struct
 *
 * @param msg The message to decode
 * @param script_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_script_current_decode(const mavlink_message_t* msg, mavlink_script_current_t* script_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    script_current->seq = mavlink_msg_script_current_get_seq(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN? msg->len : MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN;
        memset(script_current, 0, MAVLINK_MSG_ID_SCRIPT_CURRENT_LEN);
    memcpy(script_current, _MAV_PAYLOAD(msg), len);
#endif
}
