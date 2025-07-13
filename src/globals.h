// globals.h，承担main.cpp中全局变量的声明和定义
// 以及其他文件中全局变量的引用，避免重复定义和包含问题。
// 在其他文件中使用main中的函数时，只需先在此文件注册函数头再包含此头文件即可。
#pragma once
// #include "SMS_STS.h"  // 包含舵机控制库
// #include "Servo16.h"    // 包含Servo库
// #include "LKMOTOR.h"  // 包含领控电机库
// #include "SimpleSerialShell.h"  // 包含shell库
#include "DM43.h"
// 在项目头文件中
#define CAN_TX_GPIO ((gpio_num_t)CAN_TX_PIN)
#define CAN_RX_GPIO ((gpio_num_t)CAN_RX_PIN)

// 电机ID定义
#define MOTOR_ID 1  // 基础ID
#define MOTOR_ID_POS (MOTOR_ID + 100)  // 位置模式ID

// 声明全局对象
// extern SMS_STS sms_sts;
// extern Servo gdw;
// extern LKMOTOR lkmotor;
extern DM43 dm43;

// void DM_pen(int pen); // 声明DM电机换笔函数