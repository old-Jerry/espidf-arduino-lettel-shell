#ifndef INC_ESP32CAN_H
#define INC_ESP32CAN_H

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/twai.h" // TWAI 是 ESP32 的 CAN 控制器驱动模块

/* 宏定义 ------------------------------------------------------------------- */
// 开启该宏可以启用串口调试输出
// #define ESP32CAN_DEBUG

/* 调试输出宏定义 */
#ifdef ESP32CAN_DEBUG
#define debugPrint(x) Serial.print(x)
#define debugPrintln(x) Serial.println(x)
#else
#define debugPrint(x)
#define debugPrintln(x)
#endif

/* 类型定义 ----------------------------------------------------------------- */

/**
 * @brief ESP32CAN 返回状态定义
 */
typedef enum
{
    ESP32CAN_NOK = 0, // 操作失败
    ESP32CAN_OK = 1   // 操作成功
} ESP32CAN_status_t;

/**
 * @brief CAN 波特率设置枚举（单位：Kbps）
 */
typedef enum
{
    ESP32CAN_SPEED_100KBPS = 100, // 100 kbps
    ESP32CAN_SPEED_125KBPS = 125, // 125 kbps
    ESP32CAN_SPEED_250KBPS = 250, // 250 kbps
    ESP32CAN_SPEED_500KBPS = 500, // 500 kbps
    ESP32CAN_SPEED_800KBPS = 800, // 800 kbps
    ESP32CAN_SPEED_1MBPS = 1000,  // 1 Mbps
} ESP32CAN_timing_t;

/* Globals ------------------------------------------------------------------- */

/* Function Prototypes ------------------------------------------------------- */

/* Class --------------------------------------------------------------------- */
class ESP32CAN
{
public:
    ESP32CAN_status_t CANInit(gpio_num_t tx_pin, gpio_num_t rx_pin, ESP32CAN_timing_t baud);
    ESP32CAN_status_t CANStop(); // 停止并卸载 CAN 接口
    ESP32CAN_status_t CANWriteFrame(const twai_message_t *p_frame);
    ESP32CAN_status_t CANReadFrame(twai_message_t *p_frame);
    ESP32CAN_status_t CANReceiveFlush(); // 清除接收缓冲区
    // 设置过滤器，仅接受指定 11 位 ID 的帧
    int CANConfigFilter(uint16_t id);
    // 设置过滤器，自定义验收码和掩码（支持 11 或 29 位）
    int CANConfigFilter(uint32_t acceptance_code, uint32_t acceptance_mask, bool single_filter);

private:
    uint16_t _id;                                                     // 当前设置的 CAN ID
    twai_filter_config_t _f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // 默认接收所有 ID
};

/* External Globals ---------------------------------------------------------- */
extern ESP32CAN ESP32Can;

#endif
