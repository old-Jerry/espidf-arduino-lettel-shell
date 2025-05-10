/**
 * @file shell_port.c
 * @author Letter (NevermindZZT@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-22
 * 
 * @copyright (c) 2019 Letter
 * 
 */
 #include "shell_port.h"
 #include "freertos/FreeRTOS.h"
 #include "driver/uart.h"
 #include "log.h"
 #include "shell.h"
 
 #define     SHELL_UART      UART_NUM_0
 /*
 log对象//实现log写buffer函数
 */
 void uartLogWrite(char* buffer, short len) {
    // if (uartLog.shell)
    // {
    shellWriteEndLine(&shell, buffer, len);
    // }
}
Log uartLog = {.write = uartLogWrite, .active = 1, .level = LOG_DEBUG};//定义log对象


char shellBuffer[512];
Shell shell;
 /**
  * @brief 用户shell写
  * 
  * @param data 数据
  * @param len 数据长度
  * 
  * @return short int 写入实际长度
  */
 short int userShellWrite(char *data, short unsigned int len)
 {
     return uart_write_bytes(SHELL_UART, (const char *)data, len);
 }
 
 
 /**
  * @brief 用户shell读
  * 
  * @param data 数据
  * @param len 数据长度
  * 
  * @return short int 读取实际长度
  */
 short int userShellRead(char *data, short unsigned int len)
 {
     return uart_read_bytes(SHELL_UART, (uint8_t *)data, len, portMAX_DELAY);
 }
 
 
 /**
  * @brief 用户shell初始化
  * 
  */
 void userShellInit(void)
 {
     uart_config_t uartConfig = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity    = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
     };
     uart_param_config(SHELL_UART, &uartConfig);
     uart_driver_install(SHELL_UART, 256 * 2, 0, 0, NULL, 0);
     shell.write = userShellWrite;
     shell.read = userShellRead;
     shellInit(&shell, shellBuffer, 512);
     logRegister(&uartLog, &shell);//注册log对象
 
     xTaskCreate(shellTask, "shell", 2048, &shell, 10, NULL);
 }
 