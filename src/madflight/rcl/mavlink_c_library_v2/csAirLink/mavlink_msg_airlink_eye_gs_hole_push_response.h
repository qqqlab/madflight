#pragma once
// MESSAGE AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE PACKING

#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE 52003


typedef struct __mavlink_airlink_eye_gs_hole_push_response_t {
 uint32_t ip_port; /*<  port*/
 uint8_t resp_type; /*<  Hole push response type*/
 uint8_t ip_version; /*<  ip version*/
 uint8_t ip_address_4[4]; /*<  ip 4 address*/
 uint8_t ip_address_6[16]; /*<  ip 6 address*/
} mavlink_airlink_eye_gs_hole_push_response_t;

#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN 26
#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN 26
#define MAVLINK_MSG_ID_52003_LEN 26
#define MAVLINK_MSG_ID_52003_MIN_LEN 26

#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC 166
#define MAVLINK_MSG_ID_52003_CRC 166

#define MAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_4_LEN 4
#define MAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_6_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE { \
    52003, \
    "AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE", \
    5, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, resp_type) }, \
         { "ip_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_version) }, \
         { "ip_address_4", NULL, MAVLINK_TYPE_UINT8_T, 4, 6, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_address_4) }, \
         { "ip_address_6", NULL, MAVLINK_TYPE_UINT8_T, 16, 10, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_address_6) }, \
         { "ip_port", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_port) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE { \
    "AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE", \
    5, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, resp_type) }, \
         { "ip_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_version) }, \
         { "ip_address_4", NULL, MAVLINK_TYPE_UINT8_T, 4, 6, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_address_4) }, \
         { "ip_address_6", NULL, MAVLINK_TYPE_UINT8_T, 16, 10, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_address_6) }, \
         { "ip_port", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_airlink_eye_gs_hole_push_response_t, ip_port) }, \
         } \
}
#endif

/**
 * @brief Pack a airlink_eye_gs_hole_push_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Hole push response type
 * @param ip_version  ip version
 * @param ip_address_4  ip 4 address
 * @param ip_address_6  ip 6 address
 * @param ip_port  port
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp_type, uint8_t ip_version, const uint8_t *ip_address_4, const uint8_t *ip_address_6, uint32_t ip_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN];
    _mav_put_uint32_t(buf, 0, ip_port);
    _mav_put_uint8_t(buf, 4, resp_type);
    _mav_put_uint8_t(buf, 5, ip_version);
    _mav_put_uint8_t_array(buf, 6, ip_address_4, 4);
    _mav_put_uint8_t_array(buf, 10, ip_address_6, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#else
    mavlink_airlink_eye_gs_hole_push_response_t packet;
    packet.ip_port = ip_port;
    packet.resp_type = resp_type;
    packet.ip_version = ip_version;
    mav_array_memcpy(packet.ip_address_4, ip_address_4, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.ip_address_6, ip_address_6, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
}

/**
 * @brief Pack a airlink_eye_gs_hole_push_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Hole push response type
 * @param ip_version  ip version
 * @param ip_address_4  ip 4 address
 * @param ip_address_6  ip 6 address
 * @param ip_port  port
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t resp_type, uint8_t ip_version, const uint8_t *ip_address_4, const uint8_t *ip_address_6, uint32_t ip_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN];
    _mav_put_uint32_t(buf, 0, ip_port);
    _mav_put_uint8_t(buf, 4, resp_type);
    _mav_put_uint8_t(buf, 5, ip_version);
    _mav_put_uint8_t_array(buf, 6, ip_address_4, 4);
    _mav_put_uint8_t_array(buf, 10, ip_address_6, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#else
    mavlink_airlink_eye_gs_hole_push_response_t packet;
    packet.ip_port = ip_port;
    packet.resp_type = resp_type;
    packet.ip_version = ip_version;
    mav_array_memcpy(packet.ip_address_4, ip_address_4, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.ip_address_6, ip_address_6, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#endif
}

/**
 * @brief Pack a airlink_eye_gs_hole_push_response message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp_type  Hole push response type
 * @param ip_version  ip version
 * @param ip_address_4  ip 4 address
 * @param ip_address_6  ip 6 address
 * @param ip_port  port
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp_type,uint8_t ip_version,const uint8_t *ip_address_4,const uint8_t *ip_address_6,uint32_t ip_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN];
    _mav_put_uint32_t(buf, 0, ip_port);
    _mav_put_uint8_t(buf, 4, resp_type);
    _mav_put_uint8_t(buf, 5, ip_version);
    _mav_put_uint8_t_array(buf, 6, ip_address_4, 4);
    _mav_put_uint8_t_array(buf, 10, ip_address_6, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#else
    mavlink_airlink_eye_gs_hole_push_response_t packet;
    packet.ip_port = ip_port;
    packet.resp_type = resp_type;
    packet.ip_version = ip_version;
    mav_array_memcpy(packet.ip_address_4, ip_address_4, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.ip_address_6, ip_address_6, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
}

/**
 * @brief Encode a airlink_eye_gs_hole_push_response struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_gs_hole_push_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airlink_eye_gs_hole_push_response_t* airlink_eye_gs_hole_push_response)
{
    return mavlink_msg_airlink_eye_gs_hole_push_response_pack(system_id, component_id, msg, airlink_eye_gs_hole_push_response->resp_type, airlink_eye_gs_hole_push_response->ip_version, airlink_eye_gs_hole_push_response->ip_address_4, airlink_eye_gs_hole_push_response->ip_address_6, airlink_eye_gs_hole_push_response->ip_port);
}

/**
 * @brief Encode a airlink_eye_gs_hole_push_response struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_gs_hole_push_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airlink_eye_gs_hole_push_response_t* airlink_eye_gs_hole_push_response)
{
    return mavlink_msg_airlink_eye_gs_hole_push_response_pack_chan(system_id, component_id, chan, msg, airlink_eye_gs_hole_push_response->resp_type, airlink_eye_gs_hole_push_response->ip_version, airlink_eye_gs_hole_push_response->ip_address_4, airlink_eye_gs_hole_push_response->ip_address_6, airlink_eye_gs_hole_push_response->ip_port);
}

/**
 * @brief Encode a airlink_eye_gs_hole_push_response struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_gs_hole_push_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_airlink_eye_gs_hole_push_response_t* airlink_eye_gs_hole_push_response)
{
    return mavlink_msg_airlink_eye_gs_hole_push_response_pack_status(system_id, component_id, _status, msg,  airlink_eye_gs_hole_push_response->resp_type, airlink_eye_gs_hole_push_response->ip_version, airlink_eye_gs_hole_push_response->ip_address_4, airlink_eye_gs_hole_push_response->ip_address_6, airlink_eye_gs_hole_push_response->ip_port);
}

/**
 * @brief Send a airlink_eye_gs_hole_push_response message
 * @param chan MAVLink channel to send the message
 *
 * @param resp_type  Hole push response type
 * @param ip_version  ip version
 * @param ip_address_4  ip 4 address
 * @param ip_address_6  ip 6 address
 * @param ip_port  port
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airlink_eye_gs_hole_push_response_send(mavlink_channel_t chan, uint8_t resp_type, uint8_t ip_version, const uint8_t *ip_address_4, const uint8_t *ip_address_6, uint32_t ip_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN];
    _mav_put_uint32_t(buf, 0, ip_port);
    _mav_put_uint8_t(buf, 4, resp_type);
    _mav_put_uint8_t(buf, 5, ip_version);
    _mav_put_uint8_t_array(buf, 6, ip_address_4, 4);
    _mav_put_uint8_t_array(buf, 10, ip_address_6, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE, buf, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
#else
    mavlink_airlink_eye_gs_hole_push_response_t packet;
    packet.ip_port = ip_port;
    packet.resp_type = resp_type;
    packet.ip_version = ip_version;
    mav_array_memcpy(packet.ip_address_4, ip_address_4, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.ip_address_6, ip_address_6, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
#endif
}

/**
 * @brief Send a airlink_eye_gs_hole_push_response message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airlink_eye_gs_hole_push_response_send_struct(mavlink_channel_t chan, const mavlink_airlink_eye_gs_hole_push_response_t* airlink_eye_gs_hole_push_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airlink_eye_gs_hole_push_response_send(chan, airlink_eye_gs_hole_push_response->resp_type, airlink_eye_gs_hole_push_response->ip_version, airlink_eye_gs_hole_push_response->ip_address_4, airlink_eye_gs_hole_push_response->ip_address_6, airlink_eye_gs_hole_push_response->ip_port);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE, (const char *)airlink_eye_gs_hole_push_response, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airlink_eye_gs_hole_push_response_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp_type, uint8_t ip_version, const uint8_t *ip_address_4, const uint8_t *ip_address_6, uint32_t ip_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, ip_port);
    _mav_put_uint8_t(buf, 4, resp_type);
    _mav_put_uint8_t(buf, 5, ip_version);
    _mav_put_uint8_t_array(buf, 6, ip_address_4, 4);
    _mav_put_uint8_t_array(buf, 10, ip_address_6, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE, buf, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
#else
    mavlink_airlink_eye_gs_hole_push_response_t *packet = (mavlink_airlink_eye_gs_hole_push_response_t *)msgbuf;
    packet->ip_port = ip_port;
    packet->resp_type = resp_type;
    packet->ip_version = ip_version;
    mav_array_memcpy(packet->ip_address_4, ip_address_4, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->ip_address_6, ip_address_6, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE UNPACKING


/**
 * @brief Get field resp_type from airlink_eye_gs_hole_push_response message
 *
 * @return  Hole push response type
 */
static inline uint8_t mavlink_msg_airlink_eye_gs_hole_push_response_get_resp_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ip_version from airlink_eye_gs_hole_push_response message
 *
 * @return  ip version
 */
static inline uint8_t mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field ip_address_4 from airlink_eye_gs_hole_push_response message
 *
 * @return  ip 4 address
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_address_4(const mavlink_message_t* msg, uint8_t *ip_address_4)
{
    return _MAV_RETURN_uint8_t_array(msg, ip_address_4, 4,  6);
}

/**
 * @brief Get field ip_address_6 from airlink_eye_gs_hole_push_response message
 *
 * @return  ip 6 address
 */
static inline uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_address_6(const mavlink_message_t* msg, uint8_t *ip_address_6)
{
    return _MAV_RETURN_uint8_t_array(msg, ip_address_6, 16,  10);
}

/**
 * @brief Get field ip_port from airlink_eye_gs_hole_push_response message
 *
 * @return  port
 */
static inline uint32_t mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a airlink_eye_gs_hole_push_response message into a struct
 *
 * @param msg The message to decode
 * @param airlink_eye_gs_hole_push_response C-struct to decode the message contents into
 */
static inline void mavlink_msg_airlink_eye_gs_hole_push_response_decode(const mavlink_message_t* msg, mavlink_airlink_eye_gs_hole_push_response_t* airlink_eye_gs_hole_push_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airlink_eye_gs_hole_push_response->ip_port = mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_port(msg);
    airlink_eye_gs_hole_push_response->resp_type = mavlink_msg_airlink_eye_gs_hole_push_response_get_resp_type(msg);
    airlink_eye_gs_hole_push_response->ip_version = mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_version(msg);
    mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_address_4(msg, airlink_eye_gs_hole_push_response->ip_address_4);
    mavlink_msg_airlink_eye_gs_hole_push_response_get_ip_address_6(msg, airlink_eye_gs_hole_push_response->ip_address_6);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN? msg->len : MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN;
        memset(airlink_eye_gs_hole_push_response, 0, MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN);
    memcpy(airlink_eye_gs_hole_push_response, _MAV_PAYLOAD(msg), len);
#endif
}
