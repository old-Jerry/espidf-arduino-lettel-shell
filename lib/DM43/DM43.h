/*
* @file DM43.h
* @brief DM43电机控制类
* @details DM43电机控制类，用于控制DM43电机的运动
* @version 1.0
* @date 2021-7-15
 * 妙达使能帧：ff ff ff ff ff ff ff fd（canid=1）
 * 妙达失能帧：ff ff ff ff ff ff ff fc（canid=1）
 * 
 * 妙达旋转电机（位置模式）（canid=1+100=101）
 * 收回can帧：00 00 00 00 00 00 40 40  （位置0，速度3）
 * 发送can帧：00 00 40 C0 00 00 40 40  （位置-3，速度3）
 * 
 * 达妙伸缩电机（MIT模式-没拆下校准，无法使用位置模式）（canid=1+0=1）
 * 归零CAn帧：70 A3 7F F0 51 99 98 71     （位置-1.5： 速度：0 KP：10 KD：3 转矩：1）
 * 90°can帧：7F FF 7F F0 51 99 98 71    （位置：0 速度：0 KP：10 KD：3 转矩：1）
 * 180°can帧：8F 5B 7F F0 51 99 98 71   （位置：1.5 速度：0 KP：10 KD：3 转矩：1）
*/
#pragma once

// #include "User.h"
#include <stdio.h>

// DM43的CAN命令定义
#define DM43_SET_CAN_ID 0x8B
#define DM43_POSITION_MODE 0xFD
#define DM43_VELOCITY_MODE 0xF6
#define DM43_DRIVER_CONTROL 0xF3
#define DM43_ABS_POSITION_MODE 0xF5
#define DM43_GO_HOME 0x91
#define DM43_READ_MULTI_POSITION 0x31

// DM43的事件定义
#define DM43_EVENT_ALL 0xFF
#define DM43_EVENT_BEGIN 0b00000001
#define DM43_EVENT_DONE 0b00000010
#define DM43_EVENT_ERROR 0b00000100
#define DM43_EVENT_LIMIT 0b00001000

#ifdef __cplusplus
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f


typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;


typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

typedef struct
{
	int8_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;      // 0: MIT模式   1: 位置速度模式   2: 速度模式
	motor_ctrl_t cmd;
}motor_t;

// 定义发送CAN帧的函数指针类型
typedef void (*sendFrameFunc)(uint32_t id, uint8_t* data, uint8_t dataLength);

// DM43类定义
class DM43 {
   public:
    // 构造函数，初始化CAN ID
    DM43(uint8_t canId);

    // 初始化DM43，发送使能信号
    void init(uint8_t motor_id);
    // 设置发送CAN帧的函数
    void setSendFrame(sendFrameFunc sendFrame);
    // 设置位置
    void mit_ctrl(uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);//MIT模式控制函数
    void pos_speed_ctrl(uint16_t motor_id, float pos, float vel);//位置速度控制函数
    void speed_ctrl(uint16_t motor_id, float _vel);//速度控制函数
    void save_pos_zero(uint16_t motor_id, uint16_t mode_id);//保存零点函数

    // 参数计算
    int decode(uint8_t* data, uint8_t len);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);//无符号整数转换为浮点数函数
    int float_to_uint(float x_float, float x_min, float x_max, int bits);//浮点数转换为无符号整数函数

    void enable(motor_t *motor);//使能电机函数
    void disable(motor_t *motor);//失能电机函数
    void ctrl_send_mode(motor_t *motor);//切换控制模式函数
    void set_para(motor_t *motor);//设置电机参数函数//初始化需要加上
    
    void set_clear_err(motor_t *motor);//清除电机错误函数
    void fbdata(motor_t *motor, uint8_t *rx_data);//反馈数据处理函数//还没写入回调函数解析


    uint8_t status; // 状态
    uint8_t runningMode; // 运行模式

   private:
    uint8_t _canId; // CAN ID
    int64_t _position, _lastSendPosition; // 位置和上次发送的位置
    uint16_t _velocity, _lastSendVelocity; // 速度和上次发送的速度
    uint8_t _acceleration; // 加速度

    sendFrameFunc _sendFrame; // 发送CAN帧的函数指针
    // 发送CAN帧
    void sendFrame(uint32_t id, uint8_t* data, uint8_t dataLength);

    void disable_motor_mode(uint16_t motor_id, uint16_t mode_id);//失能电机模式函数
    void enable_motor_mode(uint16_t motor_id, uint16_t mode_id);//使能电机模式函数
    void clear_err(uint16_t motor_id, uint16_t mode_id);//清除错误函数
    void clear_para(motor_t *motor);//清除电机参数函数

};

#endif
