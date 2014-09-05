#ifndef HEARTBEAT_H
#define HEARTBEAT_H

//! @file heartbeat.h
//! @brief Gestion heartbeat
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_HEARTBEAT
#define WEAK_HEARTBEAT __attribute__((weak, alias("nop_function") ))
#endif

void heartbeat_update() WEAK_HEARTBEAT;

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
