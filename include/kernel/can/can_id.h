#ifndef CAN_ID_H
#define CAN_ID_H

//! @file can_id.h
//! @brief Can - message id
//! @author Atlantronic

// uint8_t us_id uint16_t val
#define CAN_US                    0x10

// uint8_t us_activate_mask
#define CAN_US_ACTIVATE           0x11
#define CAN_KINEMATICS_1          0x12
#define CAN_KINEMATICS_2          0x13
#define CAN_KINEMATICS_3          0x14

// data hokuyo (4 uint16_t par message)
#define CAN_HOKUYO_DATA_RESET     0x13
#define CAN_HOKUYO_DATA           0x14

#endif

