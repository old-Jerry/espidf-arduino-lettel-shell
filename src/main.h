#ifndef MAN_H
#define MAN_H

#include <driver/twai.h>
#include "shell.h"        // 包含 shellPrint 的声明
#include "ESP32CAN.h"     // 包含 ESP32Can 对象的声明

void send2CAN(uint32_t id, uint8_t *data, uint8_t dataLength);


#endif // MAN_H