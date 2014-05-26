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

#define ARM_SLIDER_POSITION_Y                        78.0f
#define ARM_SLIDER_POSITION_Z                       225.6f
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
	ARM_CMD_DISABLE,
	ARM_CMD_HOMING,
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
	ARM_STATE_MAX,
};

struct arm_cmd
{
	uint32_t cmdType;
	float val[5];
} __attribute__((packed));

//!< met le bras à la position souhaitée
//void arm_goto(struct arm_cmd cmd);

// TODO refiler MatriceHomogene (a compiler dans glplot)
void arm_get_matrix(float* mat) WEAK_ARM;

#endif
