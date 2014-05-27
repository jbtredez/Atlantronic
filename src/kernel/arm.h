#ifndef ARM_H
#define ARM_H

//! @file arm.h
//! @brief Gestion du bras
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/matrix_homogeneous.h"
#include "kernel/asm/asm_base_func.h"

#ifndef WEAK_ARM
#define WEAK_ARM __attribute__((weak, alias("nop_function") ))
#endif

#define ARM_SLIDER_POSITION_Y                        75.0f
#define ARM_SLIDER_POSITION_Z                       241.0f
#define ARM_SLIDER_L1                               100.0f
#define ARM_SLIDER_L2                               100.0f
#define ARM_SLIDER_L3                                27.0f
#define ARM_SHOULDER_POSITION_X                      70.0f
#define ARM_DIST_SHOULDER_TO_SHOULDER_ELBOW          87.5f
#define ARM_DIST_SHOULDER_ELBOW_TO_WRIST_ELBOW       67.5f
#define ARM_DIST_WRIST_ELBOW_TO_WRIST                67.5f
#define ARM_DIST_WRIST_TO_SUCKER                     33.7f

enum arm_cmd_type
{
	ARM_CMD_DISABLE = 0,
	ARM_CMD_HOMING,
	ARM_CMD_TAKE_FRONT_TRIANGLE,
	ARM_CMD_PROTECT_TORCH,
	ARM_CMD_SWALOW_TORCH,
	ARM_CMD_TAKE_RIGHT_FINGER,
	ARM_CMD_RELEASE_RIGHT_FINGER,
	ARM_CMD_TAKE_LEFT_FINGER,
	ARM_CMD_RELEASE_LEFT_FINGER,
};

enum
{
	ARM_AXIS_SLIDER = 0,
	ARM_AXIS_SHOULDER,
	ARM_AXIS_WRIST,
	ARM_AXIS_SHOULDER_ELBOW,
	ARM_AXIS_WRIST_ELBOW,
	ARM_AXIS_MAX,
};

enum
{
	ARM_STATE_DISABLED,
	ARM_STATE_HOMING_0,
	ARM_STATE_HOMING_1,
	ARM_STATE_HOMING_2,
	ARM_STATE_TAKE_FRONT_TRIANGLE_0,
	ARM_STATE_TAKE_FRONT_TRIANGLE_1,
	ARM_STATE_TAKE_FRONT_TRIANGLE_2,
	ARM_STATE_PROTECT_TORCH,
	ARM_STATE_SWALOW_TORCH,
	ARM_STATE_TAKE_RIGHT_FINGER,
	ARM_STATE_RELEASE_RIGHT_FINGER,
	ARM_STATE_TAKE_LEFT_FINGER,
	ARM_STATE_RELEASE_LEFT_FINGER,
	ARM_STATE_MAX,
};

struct arm_cmd
{
	uint32_t cmdType;
	float val[5];
} __attribute__((packed));


void arm_get_matrix(MatrixHomogeneous* mat) WEAK_ARM;

#endif
