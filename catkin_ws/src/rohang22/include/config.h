#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "global_def.h"


// == 기본 설정 ==
#define FRAME_MODE LLH
#define ROS_FREQUENCY_HZ 10.0
#define FILE_PATH "/home/mecha/wpt_time.csv"

/* 
 * KONKUK_BIG_QUAD, KONKUK_SMALL_QUAD, HANAM, YANGPYEONG, GONGSA, TOECHON, OTHER
 */
#define MISSION_NUMBER HANAM
// #define QUAD_MODE

// == 비행 특성 ==
#define FW_HORIZONTAL_ERROR 2.0
#define FW_VERTICAL_ERROR 2.0
#define MC_HORIZONTAL_ERROR 1.5
#define MC_VERTICAL_ERROR 1.0
#define HEADING_ERROR (6*(M_PI/180))

#define BACK_TRANSITION_DIST 70

// 아래 특성은 이름 의미와는 달리 임의의 상수로, 실험적으로 찾아진 것
#define FW_SPEED 15
#define MC_SPEED 3
#define MIN_ROTATION_RADIUS 26
#define CLIME_RATE 6
// radius = ((FW_SPEED^2)/(tan(max_roll)*9.81*VEHICLE_MASS))

// == 유도 관련 설정 ==
#define KP 1.3
// #define KP_MC 0.5

#endif
