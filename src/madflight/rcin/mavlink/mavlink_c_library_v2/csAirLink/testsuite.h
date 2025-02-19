/** @file
 *    @brief MAVLink comm protocol testsuite generated from csAirLink.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef CSAIRLINK_TESTSUITE_H
#define CSAIRLINK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_csAirLink(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_csAirLink(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_airlink_auth(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRLINK_AUTH >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airlink_auth_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW","YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTU"
    };
    mavlink_airlink_auth_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.login, packet_in.login, sizeof(char)*50);
        mav_array_memcpy(packet1.password, packet_in.password, sizeof(char)*50);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airlink_auth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_pack(system_id, component_id, &msg , packet1.login , packet1.password );
    mavlink_msg_airlink_auth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.login , packet1.password );
    mavlink_msg_airlink_auth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airlink_auth_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_send(MAVLINK_COMM_1 , packet1.login , packet1.password );
    mavlink_msg_airlink_auth_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRLINK_AUTH") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRLINK_AUTH) != NULL);
#endif
}

static void mavlink_test_airlink_auth_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airlink_auth_response_t packet_in = {
        5
    };
    mavlink_airlink_auth_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp_type = packet_in.resp_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airlink_auth_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_response_pack(system_id, component_id, &msg , packet1.resp_type );
    mavlink_msg_airlink_auth_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp_type );
    mavlink_msg_airlink_auth_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airlink_auth_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_auth_response_send(MAVLINK_COMM_1 , packet1.resp_type );
    mavlink_msg_airlink_auth_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRLINK_AUTH_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE) != NULL);
#endif
}

static void mavlink_test_airlink_eye_gs_hole_push_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airlink_eye_gs_hole_push_request_t packet_in = {
        5
    };
    mavlink_airlink_eye_gs_hole_push_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp_type = packet_in.resp_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airlink_eye_gs_hole_push_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_request_pack(system_id, component_id, &msg , packet1.resp_type );
    mavlink_msg_airlink_eye_gs_hole_push_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp_type );
    mavlink_msg_airlink_eye_gs_hole_push_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airlink_eye_gs_hole_push_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_request_send(MAVLINK_COMM_1 , packet1.resp_type );
    mavlink_msg_airlink_eye_gs_hole_push_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRLINK_EYE_GS_HOLE_PUSH_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST) != NULL);
#endif
}

static void mavlink_test_airlink_eye_gs_hole_push_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airlink_eye_gs_hole_push_response_t packet_in = {
        963497464,17,84,{ 151, 152, 153, 154 },{ 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178 }
    };
    mavlink_airlink_eye_gs_hole_push_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ip_port = packet_in.ip_port;
        packet1.resp_type = packet_in.resp_type;
        packet1.ip_version = packet_in.ip_version;
        
        mav_array_memcpy(packet1.ip_address_4, packet_in.ip_address_4, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.ip_address_6, packet_in.ip_address_6, sizeof(uint8_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airlink_eye_gs_hole_push_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_response_pack(system_id, component_id, &msg , packet1.resp_type , packet1.ip_version , packet1.ip_address_4 , packet1.ip_address_6 , packet1.ip_port );
    mavlink_msg_airlink_eye_gs_hole_push_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp_type , packet1.ip_version , packet1.ip_address_4 , packet1.ip_address_6 , packet1.ip_port );
    mavlink_msg_airlink_eye_gs_hole_push_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airlink_eye_gs_hole_push_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_gs_hole_push_response_send(MAVLINK_COMM_1 , packet1.resp_type , packet1.ip_version , packet1.ip_address_4 , packet1.ip_address_6 , packet1.ip_port );
    mavlink_msg_airlink_eye_gs_hole_push_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE) != NULL);
#endif
}

static void mavlink_test_airlink_eye_hp(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRLINK_EYE_HP >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airlink_eye_hp_t packet_in = {
        5
    };
    mavlink_airlink_eye_hp_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp_type = packet_in.resp_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_hp_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airlink_eye_hp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_hp_pack(system_id, component_id, &msg , packet1.resp_type );
    mavlink_msg_airlink_eye_hp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_hp_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp_type );
    mavlink_msg_airlink_eye_hp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airlink_eye_hp_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_hp_send(MAVLINK_COMM_1 , packet1.resp_type );
    mavlink_msg_airlink_eye_hp_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRLINK_EYE_HP") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRLINK_EYE_HP) != NULL);
#endif
}

static void mavlink_test_airlink_eye_turn_init(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airlink_eye_turn_init_t packet_in = {
        5
    };
    mavlink_airlink_eye_turn_init_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp_type = packet_in.resp_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_turn_init_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airlink_eye_turn_init_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_turn_init_pack(system_id, component_id, &msg , packet1.resp_type );
    mavlink_msg_airlink_eye_turn_init_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_turn_init_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp_type );
    mavlink_msg_airlink_eye_turn_init_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airlink_eye_turn_init_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airlink_eye_turn_init_send(MAVLINK_COMM_1 , packet1.resp_type );
    mavlink_msg_airlink_eye_turn_init_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRLINK_EYE_TURN_INIT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT) != NULL);
#endif
}

static void mavlink_test_csAirLink(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_airlink_auth(system_id, component_id, last_msg);
    mavlink_test_airlink_auth_response(system_id, component_id, last_msg);
    mavlink_test_airlink_eye_gs_hole_push_request(system_id, component_id, last_msg);
    mavlink_test_airlink_eye_gs_hole_push_response(system_id, component_id, last_msg);
    mavlink_test_airlink_eye_hp(system_id, component_id, last_msg);
    mavlink_test_airlink_eye_turn_init(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CSAIRLINK_TESTSUITE_H
