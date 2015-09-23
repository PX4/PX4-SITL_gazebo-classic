// MESSAGE VTOL_STATE PACKING

#define MAVLINK_MSG_ID_VTOL_STATE 245

typedef struct __mavlink_vtol_state_t
{
 uint8_t state; /*< The VTOL state the MAV is in*/
} mavlink_vtol_state_t;

#define MAVLINK_MSG_ID_VTOL_STATE_LEN 1
#define MAVLINK_MSG_ID_245_LEN 1

#define MAVLINK_MSG_ID_VTOL_STATE_CRC 83
#define MAVLINK_MSG_ID_245_CRC 83



#define MAVLINK_MESSAGE_INFO_VTOL_STATE { \
	"VTOL_STATE", \
	1, \
	{  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_vtol_state_t, state) }, \
         } \
}


/**
 * @brief Pack a vtol_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state The VTOL state the MAV is in
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vtol_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VTOL_STATE_LEN];
	_mav_put_uint8_t(buf, 0, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#else
	mavlink_vtol_state_t packet;
	packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VTOL_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VTOL_STATE_LEN, MAVLINK_MSG_ID_VTOL_STATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
}

/**
 * @brief Pack a vtol_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state The VTOL state the MAV is in
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vtol_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VTOL_STATE_LEN];
	_mav_put_uint8_t(buf, 0, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#else
	mavlink_vtol_state_t packet;
	packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VTOL_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VTOL_STATE_LEN, MAVLINK_MSG_ID_VTOL_STATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
}

/**
 * @brief Encode a vtol_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vtol_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vtol_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vtol_state_t* vtol_state)
{
	return mavlink_msg_vtol_state_pack(system_id, component_id, msg, vtol_state->state);
}

/**
 * @brief Encode a vtol_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vtol_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vtol_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vtol_state_t* vtol_state)
{
	return mavlink_msg_vtol_state_pack_chan(system_id, component_id, chan, msg, vtol_state->state);
}

/**
 * @brief Send a vtol_state message
 * @param chan MAVLink channel to send the message
 *
 * @param state The VTOL state the MAV is in
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vtol_state_send(mavlink_channel_t chan, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VTOL_STATE_LEN];
	_mav_put_uint8_t(buf, 0, state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, buf, MAVLINK_MSG_ID_VTOL_STATE_LEN, MAVLINK_MSG_ID_VTOL_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, buf, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
#else
	mavlink_vtol_state_t packet;
	packet.state = state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, (const char *)&packet, MAVLINK_MSG_ID_VTOL_STATE_LEN, MAVLINK_MSG_ID_VTOL_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, (const char *)&packet, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_VTOL_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vtol_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, buf, MAVLINK_MSG_ID_VTOL_STATE_LEN, MAVLINK_MSG_ID_VTOL_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, buf, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
#else
	mavlink_vtol_state_t *packet = (mavlink_vtol_state_t *)msgbuf;
	packet->state = state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, (const char *)packet, MAVLINK_MSG_ID_VTOL_STATE_LEN, MAVLINK_MSG_ID_VTOL_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VTOL_STATE, (const char *)packet, MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE VTOL_STATE UNPACKING


/**
 * @brief Get field state from vtol_state message
 *
 * @return The VTOL state the MAV is in
 */
static inline uint8_t mavlink_msg_vtol_state_get_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a vtol_state message into a struct
 *
 * @param msg The message to decode
 * @param vtol_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_vtol_state_decode(const mavlink_message_t* msg, mavlink_vtol_state_t* vtol_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	vtol_state->state = mavlink_msg_vtol_state_get_state(msg);
#else
	memcpy(vtol_state, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_VTOL_STATE_LEN);
#endif
}
