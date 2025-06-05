/**
 * @file shell_port.h
 * @author Letter (NevermindZZT@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-22
 * 
 * @copyright (c) 2019 Letter
 * 
 */

#ifndef __SHELL_PORT_H__
#define __SHELL_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize shell interface
 */
void userShellInit(void);

/**
 * @brief Shell write function
 * 
 * @param data data to write
 * @param len data length
 * @return short actually written length
 */
short userShellWrite(char *data, unsigned short len);

/**
 * @brief Shell read function
 * 
 * @param data buffer to store read data
 * @param len expected read length
 * @return short actually read length
 */
short userShellRead(char *data, unsigned short len);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_PORT_H */
