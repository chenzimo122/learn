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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#define shoot_laser_on()    laser_on()
#define shoot_laser_off()   laser_off()
//???IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

//发射机构PID全局声明
fp32 speed_set = 1;//速度设定值
fp32 speed;//速度实际值
//声明速度环PID控制转速
fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_ANGLE_PID_KD};
//声明位置环PID控制角度
fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};

//一系列函数的声明
/**
  * @brief          ???????
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);

/**
  * @brief          ??????
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ????????
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ??????,?????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);


shoot_control_t shoot_control;  //????


/**
  * @brief          ?????,???PID,?????,????
  * @param[in]      void
  * @retval         ???
  */
  //发射系统初始化函数
void shoot_init(void)
{
	//存储摩擦轮速度PID参数
    static const fp32 Fric12_speed_pid[3] = {FRIC12_KP, FRIC12_KI, FRIC12_KD};
    shoot_control.shoot_mode = SHOOT_STOP;//射击初始状态为停止
    //?????
    shoot_control.shoot_rc = get_remote_control_point();//获取遥控器数据的指针
    //????
	//获取电机反馈数据指针，拨盘电机和两个摩擦轮电机的CAN反馈数据指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric1_measure = get_gimbal_fric1_measure_point();
    shoot_control.fric2_measure = get_gimbal_fric2_measure_point();
    //???PID
	//PID控制器初始化
    PID_init(&shoot_control.trigger_motor_pid,  PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.trigger_motor_pid2, PID_POSITION, Trigger_angle_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric1_pid, PID_POSITION, Fric12_speed_pid, FRIC12_PID_MAX_OUT, FRIC12_PID_MAX_IOUT);
    PID_init(&shoot_control.fric2_pid, PID_POSITION, Fric12_speed_pid, FRIC12_PID_MAX_OUT, FRIC12_PID_MAX_IOUT);
    //????
	//射击反馈更新
    shoot_feedback_update();
    //??????
	//摩擦轮相关变量清零
    shoot_control.fric1_given_current = 0;
    shoot_control.fric2_given_current = 0;
    shoot_control.fric1_speed = 0;
    shoot_control.fric2_speed = 0;
    shoot_control.fric12_speed_set = 0;

    //拨盘电机相关变量初始化
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;//目标角度为当前位置
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;//判断拨盘是否在运动中
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;

    //???????
	//卡弹/反转
    shoot_control.block_time = 0;//拨盘堵转时记录时长，超时后执行反转退弹
    shoot_control.reverse_time = 0;
    shoot_control.reverse_start_angle = 0.0f;//记录反转开始时的角度，判断反转是否完成
}

/**
  * @brief          ????
  * @param[in]      void
  * @retval         ??can????
  */
//射击控制循环
void shoot_control_loop(void)
{
	//设置当前射击模式
    shoot_set_mode();        //?????
	//更新传感器反馈
    shoot_feedback_update(); //????

    if (shoot_control.shoot_mode == SHOOT_STOP)//停止模式
    {
        shoot_control.speed_set = 0.0f;//摩擦轮速度
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)//预摩擦模式
    {
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)//待击发模式
    {
        shoot_control.speed_set = 0.0f;
		//初始化拨弹电机的PID参数
        // ??PID??,?????????????????????
        shoot_control.trigger_motor_pid.Iout  = 0.0f;//Iout是PID积分的输出值，清零是为了消除积分累计，避免影响控制精度
        shoot_control.trigger_motor_pid2.Iout = 0.0f;
        shoot_control.trigger_motor_pid.max_out  = TRIGGER_READY_PID_MAX_OUT;//总输出上限，防止零件损坏
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;//积分项输出上限，防止积分过度累积出现超调或震荡
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)//就绪模式
    {
        shoot_control.speed_set = 0.0f;
        // ??PID??,????????????
        shoot_control.trigger_motor_pid.Iout  = 0.0f;
        shoot_control.trigger_motor_pid2.Iout = 0.0f;
    }
    /***************************************************************************************/
    else if (shoot_control.shoot_mode == SHOOT_BULLET)//单发击发模式
    {
        shoot_control.trigger_motor_pid.max_out  = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();//调用此函数执行单发击发动作
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)//连续击发模式
    {
        shoot_control.speed_set = 3.5f * CONTINUE_TRIGGER_SPEED;//让摩擦轮达到连续射击所需转速
        trigger_motor_turn_back();//控制拨弹机复位或循环转动
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)//射击完成模式
    {
        shoot_control.speed_set = 0.0f;//摩擦轮停止转动
        shoot_control.trigger_motor_pid.Iout  = 0.0f;//清零积分项
        shoot_control.trigger_motor_pid2.Iout = 0.0f;
    }

    //????
    if (shoot_control.shoot_mode == SHOOT_STOP)//停止模式处理
    {
        shoot_laser_off();//关闭激光发射模块
        shoot_control.given_current = 0;//电流为0，停止电机
		//计算摩1/2的PID输出，PID参数结构体，实际转速反馈
        shoot_control.fric1_given_current = PID_calc(&shoot_control.fric1_pid, shoot_control.fric1_measure->speed_rpm, 0);
        shoot_control.fric2_given_current = PID_calc(&shoot_control.fric2_pid, shoot_control.fric2_measure->speed_rpm, 0);
    }
    else
    {
        shoot_laser_on();//开启激光发射模块
        //???????PID
		//触发电机的PID计算
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        //???PID
		//摩擦轮1/2PID计算
        PID_calc(&shoot_control.fric1_pid, shoot_control.fric1_measure->speed_rpm,  shoot_control.fric12_speed_set);
        PID_calc(&shoot_control.fric2_pid, shoot_control.fric2_measure->speed_rpm, -shoot_control.fric12_speed_set);
        //??????
		//将触发电机的PID强制转换为int16类型赋值给给定电流,适配电调的输入格式
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        //?????????
        if (shoot_control.shoot_mode < SHOOT_READY_BULLET)//判断是不是待弹状态，不是，则电机电流清零
        {
            shoot_control.given_current = 0;
        }
        //???????
		//将摩擦轮1/2的PID强制转换为int16类型赋值给给定电流
        shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_pid.out);
        shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_pid.out);
    }
}

/**
  * @brief          ?????
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)//处理射击模式的切换逻辑
{
    static int8_t last_s = RC_SW_UP;//初始为开关向上（记录上一次开关状态）

    //????,?????
	//判断开关是否处于向上位置
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;//摩擦轮就绪模式
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;//停止模式
    }

    //???????
	//判断开关是否处于中间位置
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //???????
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //?????????????READY
	//摩擦轮就绪模式转射击就绪模式
	//摩擦轮12方向相反，一个在目标值50%以上，一个在-50%以下
    if (shoot_control.shoot_mode == SHOOT_READY_FRIC &&
        -shoot_control.fric12_speed_set / 2 >= shoot_control.fric2_measure->speed_rpm &&
         shoot_control.fric12_speed_set / 2 <= shoot_control.fric1_measure->speed_rpm)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
	//射击就绪模式转发射模式
    else if (shoot_control.shoot_mode == SHOOT_READY || shoot_control.shoot_mode == SHOOT_READY_BULLET)
    {
        //?????????????
		//遥控器模式开关从非向下切换到向下或左键按下或右键按下
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) ||
            (shoot_control.press_l && shoot_control.last_press_l == 0) ||
            (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }

	//当前模式的优先等级高于摩擦轮就绪模式
    if (shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        //??????
		//长按左右键或遥控器开关
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) ||
            (shoot_control.press_r_time == PRESS_LONG_TIME) ||
            (shoot_control.rc_s_time    == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//切换为连续发射模式
        }
        else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;//切换为待弹模式
        }
    }

    //????????
	//判断云台是否发送停止射击指令
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;//切换为停止模式
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];//更新上一次遥控器开关状态
}

/**
  * @brief          ??????
  * @param[in]      void
  * @retval         void
  */
//射击反馈更新函数
static void shoot_feedback_update(void)
{
	//三阶滤波器的状态寄存器
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //????????
	//滤波器系数，传递函数分母或分子系数
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //????
	//转速滤波更新，滤除电机转速的高频噪声、抖动，输出平滑的转速，避免PID震荡，电机失控
    speed_fliter_1 = speed_fliter_2;//上一时刻的值前移，可以腾出位置存储新数据
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] +
                     (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;//y(n) = a1*y(n-1) + a2*y(n-2) + b0*x(n)，新滤波值计算，x（n）先读取拨弹电机的原始转速rpm，用单位转换系数把rpm转换成实际物理速度m/s

    //????,???????
	//编码器多圈计数，解决编码器单圈超量程问题，给位置PID提供精准的真实角度反馈
	//ecd当前编码器的原始值，last_ecd上一个周期编码器原始值，求差值，看是否大于半量程，若大于则表示反转一圈，计数器减一，累计反转次数
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
	//若小于则表示正转一圈，计数器加一，累计正转次数
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    //??????
	//实际物理角度计算，圈数*单圈量程+当前编码器值，得到总编码器值，再把这个转换成实际物理角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //?????
	//发射状态按键读取，判断发射按键是否按下
    shoot_control.key = BUTTEN_TRIG_PIN;
    //??????
	//把当前变量存到上一次变量里面，再读取新的当前状态，通过对比两次的数值组合判断按键的状态
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;//把遥控器或鼠标传过来的按键状态存储到自己状态变量里
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;

    //??????
	//左键长按计时逻辑，让系统识别短按，长按不同操作的意图
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
            shoot_control.press_l_time++;//累计按键按下的时长，达到阈值后就不变
    }
    else
    {
        shoot_control.press_l_time = 0;//松开后计时就清零
    }

	//右键长按计时逻辑
    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
            shoot_control.press_r_time++;
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //?????????
	//遥控器开关长按计时，处于工作状态，且遥控器开关被拨下，用于区分短拨和长按
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
            shoot_control.rc_s_time++;//没达到阈值计时自增
    }
    else
    {
        shoot_control.rc_s_time = 0;//保证下次从零开始计时
    }

    //???????
    static int up_time = 0;//摩擦轮降速剩余持续时间
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;//降速持续时长，按下就给时长
    }
    else
    {
        up_time = 0;//右键松开，先执行降速，后不降
    }

    if (up_time > 0)
    {
        shoot_control.fric12_speed_set = FRIC12_FRIC_DOWN;//把摩擦轮的目标速度设置为降速值
        up_time--;//倒计时自减
    }
    else
    {
        shoot_control.fric12_speed_set = FRIC12_FRIC_DOWN;//等降速结束后一直保持这个速度
    }
}

/**
  * @brief          ??????????(SHOOT_CONTINUE_BULLET??)
  * @param[in]      void
  * @retval         void
  */
//触发电机反转函数
static void trigger_motor_turn_back(void)
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        //???,????????
        shoot_control.speed_set = shoot_control.speed_set;
    }
    else
    {
        //??,??
        shoot_control.speed_set = REVERSE_SPEED_LIMIT;//堵转时长超过阈值则反转，触发退弹
    }

	//电机速度过低，小于堵转判定速度，且堵转时长还未到
    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;//堵转时长自增
        shoot_control.reverse_time = 0;//反转时长清零，因为还没达到反转条件
    }
	//当达到堵转时长阈值且反转时长还没到上限时
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;//反转计时自增
    }
    else
    {
        shoot_control.block_time = 0;//反转时长达到上限，或电机不在堵转就清零堵转计时
    }
}

/**
  * @brief          ??????,???????????????
  *                 ?????????????:
  *                 - ????? REVERSE_ANGLE_LIMIT ????,?????
  *                 - ?? READY ??? PID ??,???????????
  * @param[in]      void
  * @retval         void
  */
//发射子弹控制函数
static void shoot_bullet_control(void)
{
    //??????(?????? PI/3,?60?,??????)
	//电机运动标志位，0表示静止待命，1表示正在执行运动
    if (shoot_control.move_flag == 0)
    {
		//给电机设置前进60°的目标位置，对应拨弹轮转动一格，完成一发子弹的补给
        shoot_control.set_angle   = shoot_control.angle + PI / 3;
        shoot_control.move_flag   = 1;//标记电机正在执行补给动作，避免重复操作
        //??????????,??????
        shoot_control.block_time   = 0;//清除历史数据避免误触发反转，导致逻辑混乱
        shoot_control.reverse_time = 0;
    }

    //??????????(???0.05????????)
	//判断电机当前角度与目标阈值的误差，若误差超过0.05f则表示电机还在补给过程中，继续控制，若没超过，则表示结束，可停止控制
    if (shoot_control.angle - shoot_control.set_angle >  0.05f ||
        shoot_control.angle - shoot_control.set_angle < -0.05f)
    {
        // ===== ???? =====
		//实时检测电机是否卡弹
        if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED)
        {
            shoot_control.block_time++;
        }
        else
        {
            shoot_control.block_time = 0;
        }

        // ===== ???? =====
		//堵转超时判断
        if (shoot_control.block_time > BLOCK_TIME)
        {
            //???????????????
            if (shoot_control.reverse_time == 0)
            {
                shoot_control.reverse_start_angle = shoot_control.angle;//还没开始反转时记录电机当前角度，作为位置基准
            }

            shoot_control.reverse_time++;

            //???????? ? ?????,??????READY
			//角度加时间控制，满足任意一个则停止反转
            if (fabs(shoot_control.angle - shoot_control.reverse_start_angle) > REVERSE_ANGLE_LIMIT ||
                shoot_control.reverse_time > REVERSE_TIME)
            {
				//计时/标志位清零
                shoot_control.block_time   = 0;
                shoot_control.reverse_time = 0;
                shoot_control.move_flag    = 0;
                //??PID??,????READY???????
				//PID积分项清零，防止积分饱和，导致误动
                shoot_control.trigger_motor_pid.Iout  = 0.0f;
                shoot_control.trigger_motor_pid2.Iout = 0.0f;
                shoot_control.speed_set = 0.0f;//将电机目标速度清零，强制停止转动
                //????READY,??????,?????
				//将发射模式调到发射就绪状态，等待射击指令
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
            else
            {
                //????,?????TRIGGER_SPEED(??)??
                //????????,? REVERSE_SPEED_LIMIT ????
                shoot_control.speed_set = REVERSE_SPEED_LIMIT;//没达到退弹到位条件，就一直保持反转速度，直到满足退出条件
            }
        }
        else
        {
            //????:???PID??????
			//正常没卡弹时，调用位置环PID计算函数，输入当前角度和目标角度，输出控制量
            PID_calc(&shoot_control.trigger_motor_pid2, shoot_control.angle, shoot_control.set_angle);
            shoot_control.speed_set = shoot_control.trigger_motor_pid2.out;//将PID计算的控制量作为电机目标速度
        }
    }
    else
    {
        //??????,??????
        shoot_control.move_flag    = 0;//表示已完成此次动作，允许触发下一次
        shoot_control.block_time   = 0;//清除历史数据
        shoot_control.reverse_time = 0;
        //??PID??,??????????????
		//拨弹电机的两个PID控制器，积分项输出清零，让下次启动从零开始计算
        shoot_control.trigger_motor_pid.Iout  = 0.0f;
        shoot_control.trigger_motor_pid2.Iout = 0.0f;
        shoot_control.speed_set = 0.0f;//电机目标速度清零
        shoot_control.shoot_mode = SHOOT_READY_BULLET;//等待下一次发射指令，整个动作闭环
    }
}