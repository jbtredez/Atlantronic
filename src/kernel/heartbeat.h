#ifndef HEARTBEAT_H
#define HEARTBEAT_H

//! @file heartbeat.h
//! @brief Gestion heartbeat
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

void heartbeat_update();

enum
{
	HEARTBEAT_UPDATE,
	HEARTBEAT_DISABLE,
};

//------------------ interface usb -------------------
struct heartbeat_cmd_arg
{
	int type;
} __attribute__((packed));

#ifdef __cplusplus
}
#endif

#endif
