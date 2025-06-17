/**
 * @file shell_port.c
 * @author Letter (NevermindZZT@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-02-22
 *
 * @copyright (c) 2019 Letter
 *在这里加shell的功能
 */
#include <stdio.h>
#include "shell_port.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <driver/gpio.h>
#include "driver/uart.h"

#include "log.h"
#include "shell.h"
#include "shell_cpp.h"
#include "sdkconfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include "KTech.h"
#include "DM43.h"
#include "main.h"
// 使用usbcdc需要修改文件为.cpp
#define SHELL_UART HWCDCSerial // UART0-默认的调试串口-UART_NUM_0
#define SHELL_BUFFER_SIZE 512
#define SHELL_TASK_STACK_SIZE 2048
#define SHELL_TASK_PRIORITY 10
/*
log对象
*/
char shellBuffer[SHELL_BUFFER_SIZE];
Shell shell;


void uartLogWrite(char *buffer, short len)
{
    // if (uartLog.shell)
    // {
    shellWriteEndLine(&shell, buffer, len);
    // }
}
Log uartLog = {.write = uartLogWrite, .active = 1, .level = LOG_DEBUG};

/**
 * @brief 用户shell写
 * @param data 数据
 * @param len 数据长度
 * @return short int 写入实际长度
 */
short int userShellWrite(char *data, short unsigned int len)
{
    //  return uart_write_bytes(SHELL_UART, (const char *)data, len);
    return Serial.write((uint8_t *)data, len);
}

/**
 * @brief 用户shell读
 * @param data 数据
 * @param len 数据长度
 * @return short int 读取实际长度
 */
short int userShellRead(char *data, short unsigned int len)
{
    //  return uart_read_bytes(SHELL_UART, (uint8_t *)data, len, portMAX_DELAY);
    int i = 0;
    while (i < len)
    {
        if (Serial.available()){data[i++] = Serial.read();}
        else{vTaskDelay(1);}
    }
    return i;
}
// short int userShellRead(char *data, short unsigned int len)
// {
//     int totalRead = 0;
//     unsigned long start = millis();

//     while ((millis() - start) < 20 && totalRead < len)
//     {
//         if (Serial.available())
//         {
//             int byteRead = Serial.read();
//             if (byteRead >= 0) {
//                 data[totalRead++] = (char)byteRead;
//             }
//         }
//         else
//         {
//             vTaskDelay(pdMS_TO_TICKS(1));
//         }
//     }

//     return totalRead;
// }
/**
 * @brief 用户shell初始化
 *
 */
void userShellInit(void)
{
    //  硬件UART需要下面的初始化
    // uart_config_t uartConfig = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    //  uart_param_config(SHELL_UART, &uartConfig);
    //  uart_driver_install(SHELL_UART, 256 * 2, 0, 0, NULL, 0);
    shell.write = userShellWrite;
    shell.read = userShellRead;
    shellInit(&shell, shellBuffer, 512);
    logRegister(&uartLog, &shell);
    xTaskCreate(shellTask, "shell", 4096, &shell, 10, NULL);
}

void shellReboot() {
    esp_restart();
    shellPrint(&shell, "Rebooting...\r\n");
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 reboot,
                 shellReboot,
                 Reboot the system);

void shellShowIP() {
    if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        shellPrint(&shell, "IP Address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
    } else {
        shellPrint(&shell, "WiFi not connected.\r\n");
    }
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 ip,
                 shellShowIP,
                 Show current IP address);
/**
 * @brief 设置 KTech[0] 的转速
 * 
 * 从命令行参数中解析速度值，并调用 ktech[0].runVelocity()，
 * 使用格式: KT_speed <value>
 */
// 定义 KTech 数组（声明和定义只出现一次）
// extern KTech ktech[6];
// int shellSetVelocity(int argc, char* argv[])
// {
//     if (argc != 2) {
//         shellPrint(&shell, "Usage: KT_speed <value>\r\n");
//         return 0;
//     }
//     int value = strtol(argv[1], NULL, 10);
//     ktech[0].runVelocity(value);
//     ktech[3].runVelocity(value);
//     shellPrint(&shell, "KTech[0] runVelocity set to %d\r\n", value);
//     return 0;
// }

// SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
//                  KT_speed,
//                  shellSetVelocity,
//                  Set KTech[0] runVelocity using input value);

/**
 * @brief 设置 DM43(id1) 的转速
 * 
 * 从命令行参数中解析速度值，并调用 dm43.speed_ctrl(id,speed);
 * 使用格式: DM_speed <value>
 */
// 定义 DM 数组（声明和定义只出现一次）
// extern DM43 dm43;
int shellSetDMVelocity(int argc, char* argv[])
{
    if (argc != 2) {
        shellPrint(&shell, "Usage: DM_speed <value>\r\n");
        return 0;
    }
    int value = strtol(argv[1], NULL, 10);//支持小数
    float speed = value / 10.0; // 减小值，10对应1//适配vofa
    DM43 dm43(1);  // CAN ID 为 1
    dm43.setSendFrame(send2CAN);
    dm43.init(1);
    dm43.speed_ctrl(1,speed);
    shellPrint(&shell, "DM_speed runVelocity set to %d\r\n", value);
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 DM_speed,
                 shellSetDMVelocity,
                 Set DM_speed runVelocity using input value);