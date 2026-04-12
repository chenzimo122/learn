/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ????
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ??
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//??????????
#define SHOOT_RC_MODE_CHANNEL       1

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//????????? ??
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//????? ??????,????,?????
#define SHOOT_DONE_KEY_OFF_TIME     15
//??????
#define PRESS_LONG_TIME             3
//??????????????? ?????? ????
#define RC_S_LONG_TIME              2000
//????? ?? ??
#define UP_ADD_TIME                 300
//?????????
#define HALF_ECD_RANGE              4936
#define ECD_RANGE                   9871
//??rmp ??? ???????
#define MOTOR_RPM_TO_SPEED      (0.00290888f * 0.83)
#define MOTOR_ECD_TO_ANGLE      (0.0000403679f * 0.83)
#define FULL_COUNT                  18
//????
#define TRIGGER_SPEED               (-350.0f*0.83)
//??????
#define MAX_ALLOWED_SPEED           100.0f
//??????
#define CONTINUE_TRIGGER_SPEED      (3.5f*0.83)
#define READY_TRIGGER_SPEED         -6.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//???? ??????
#define BLOCK_TRIGGER_SPEED         1.0f    // ??????????
#define BLOCK_TIME                  150     // ?????????
#define REVERSE_TIME                80      // ???????(????)
#define REVERSE_SPEED_LIMIT         -30.0f   // ????,????????
#define REVERSE_ANGLE_LIMIT         0.05f   // ??????(??),?3?

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.316f

//?????PID
#define FRIC12_KP                   8.0f
#define FRIC12_KI                   0.01f
#define FRIC12_KD                   0.0f

#define FRIC12_PID_MAX_OUT          20000.0f
#define FRIC12_PID_MAX_IOUT         3000.0f

//???????PID
#define TRIGGER_ANGLE_PID_KP        300.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        80.0f   // ??50,???????????

//???????PID
#define TRIGGER_SPEED_PID_KP        6000.0f
#define TRIGGER_SPEED_PID_KI        800.0f
#define TRIGGER_SPEED_PID_KD        200.0f

#define TRIGGER_BULLET_PID_MAX_OUT  20000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   20000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE     20

//???????
#define FRIC12_FRIC_UP          7800    //????30m/s
#define FRIC12_FRIC_MID         4880    //????18m/s
#define FRIC12_FRIC_DOWN        4500    //????15m/s
#define FRIC12_FRIC_DOWN_DOWN   4000    //????11.8m/s

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
    pid_type_def trigger_motor_pid2;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    fp32 ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;

    // ?????
    const motor_measure_t *fric1_measure;
    const motor_measure_t *fric2_measure;
    pid_type_def fric1_pid;
    pid_type_def fric2_pid;
    int16_t fric1_given_current;
    int16_t fric2_given_current;
    fp32 fric1_speed;
    fp32 fric2_speed;
    fp32 fric12_speed_set;

    uint16_t cooling_rate;  // ?????
    uint16_t speed_limit;   // ??????

    // ????:??????????,??????????
    fp32 reverse_start_angle;

} shoot_control_t;

extern void shoot_init(void);
extern void shoot_control_loop(void);
extern shoot_control_t shoot_control;

#endif