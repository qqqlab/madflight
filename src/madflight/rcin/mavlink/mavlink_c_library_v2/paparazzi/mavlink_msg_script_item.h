#pragma once
// MESSAGE SCRIPT_ITEM PACKING

#define MAVLINK_MSG_ID_SCRIPT_ITEM 180


typedef struct __mavlink_script_item_t {
 uint16_t seq; /*<  Sequence*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char name[50]; /*<  The name of the mission script, NULL terminated.*/
} mavlink_script_item_t;

#define MAVLINK_MSG_ID_SCRIPT_ITEM_LEN 54
#define MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN 54
#define MAVLINK_MSG_ID_180_LEN 54
#define MAVLINK_MSG_ID_180_MIN_LEN 54

#define MAVLINK_MSG_ID_SCRIPT_ITEM_CRC 231
#define MAVLINK_MSG_ID_180_CRC 231

#define MAVLINK_MSG_SCRIPT_ITEM_FIELD_NAME_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SCRIPT_ITEM { \
    180, \
    "SCRIPT_ITEM", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_script_item_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_script_item_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_script_item_t, seq) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 50, 4, offsetof(mavlink_script_item_t, name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SCRIPT_ITEM { \
    "SCRIPT_ITEM", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_script_item_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_script_item_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_script_item_t, seq) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 50, 4, offsetof(mavlink_script_item_t, name) }, \
         } \
}
#endif

/**
 * @brief Pack a script_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param seq  Sequence
 * @param name  The name of the mission script, NULL terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_script_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t seq, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_ITEM_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_char_array(buf, 4, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#else
    mavlink_script_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.name, name, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCRIPT_ITEM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
}

/**
 * @brief Pack a script_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param seq  Sequence
 * @param name  The name of the mission script, NULL terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_script_item_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t seq, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_ITEM_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_char_array(buf, 4, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#else
    mavlink_script_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.name, name, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCRIPT_ITEM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#endif
}

/**
 * @brief Pack a script_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param seq  Sequence
 * @param name  The name of the mission script, NULL terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_script_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t seq,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_ITEM_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_char_array(buf, 4, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#else
    mavlink_script_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.name, name, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCRIPT_ITEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
}

/**
 * @brief Encode a script_item struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param script_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_script_item_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_script_item_t* script_item)
{
    return mavlink_msg_script_item_pack(system_id, component_id, msg, script_item->target_system, script_item->target_component, script_item->seq, script_item->name);
}

/**
 * @brief Encode a script_item struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param script_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_script_item_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_script_item_t* script_item)
{
    return mavlink_msg_script_item_pack_chan(system_id, component_id, chan, msg, script_item->target_system, script_item->target_component, script_item->seq, script_item->name);
}

/**
 * @brief Encode a script_item struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param script_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_script_item_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_script_item_t* script_item)
{
    return mavlink_msg_script_item_pack_status(system_id, component_id, _status, msg,  script_item->target_system, script_item->target_component, script_item->seq, script_item->name);
}

/**
 * @brief Send a script_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param seq  Sequence
 * @param name  The name of the mission script, NULL terminated.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_script_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCRIPT_ITEM_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_char_array(buf, 4, name, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_ITEM, buf, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
#else
    mavlink_script_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.name, name, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_ITEM, (const char *)&packet, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
#endif
}

/**
 * @brief Send a script_item message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_script_item_send_struct(mavlink_channel_t chan, const mavlink_script_item_t* script_item)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_script_item_send(chan, script_item->target_system, script_item->target_component, script_item->seq, script_item->name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_ITEM, (const char *)script_item, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SCRIPT_ITEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_script_item_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t seq, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_char_array(buf, 4, name, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_ITEM, buf, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
#else
    mavlink_script_item_t *packet = (mavlink_script_item_t *)msgbuf;
    packet->seq = seq;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->name, name, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCRIPT_ITEM, (const char *)packet, MAVLINK_MSG_ID_SCRIPT_ITEM_MIN_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN, MAVLINK_MSG_ID_SCRIPT_ITEM_CRC);
#endif
}
#endif

#endif

// MESSAGE SCRIPT_ITEM UNPACKING


/**
 * @brief Get field target_system from script_item message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_script_item_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from script_item message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_script_item_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field seq from script_item message
 *
 * @return  Sequence
 */
static inline uint16_t mavlink_msg_script_item_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field name from script_item message
 *
 * @return  The name of the mission script, NULL terminated.
 */
static inline uint16_t mavlink_msg_script_item_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 50,  4);
}

/**
 * @brief Decode a script_item message into a struct
 *
 * @param msg The message to decode
 * @param script_item C-struct to decode the message contents into
 */
static inline void mavlink_msg_script_item_decode(const mavlink_message_t* msg, mavlink_script_item_t* script_item)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    script_item->seq = mavlink_msg_script_item_get_seq(msg);
    script_item->target_system = mavlink_msg_script_item_get_target_system(msg);
    script_item->target_component = mavlink_msg_script_item_get_target_component(msg);
    mavlink_msg_script_item_get_name(msg, script_item->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SCRIPT_ITEM_LEN? msg->len : MAVLINK_MSG_ID_SCRIPT_ITEM_LEN;
        memset(script_item, 0, MAVLINK_MSG_ID_SCRIPT_ITEM_LEN);
    memcpy(script_item, _MAV_PAYLOAD(msg), len);
#endif
}
