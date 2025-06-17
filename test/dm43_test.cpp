/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// 它根据 CONFIG_AUTOSTART_ARDUINO 这个宏来选择使用 FreeRTOS 任务调度还是直接使用 Arduino 的 setup() 和 loop() 函数。

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "ESPTelnet.h"
#include <WebServer.h>  // 新增
WebServer server(80);   // 新增：HTTP 服务运行在 80 端口

#include "log.h"
#include "shell_cpp.h"
#include "shell_port.h"
#include <driver/twai.h>
#include "ESP32CAN.h"
#include "DM43.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO

#ifndef PIN_RGB_LED
#define PIN_RGB_LED 4
#endif

/******************************************************************************************************************************************************************/

ESPTelnet telnet;
IPAddress ip;
uint16_t port = 23;
extern Shell shell;


#define SERIALDEBUG telnet

const char *name = "oldjerry_esp32";
// const char* ssid = "Work";
// const char* password = "xhaxx518";
const char *ssid = "quest2";
const char *password = "meiyijia";
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/

void wifiScan();
void send2CAN(uint32_t id, uint8_t *data, uint8_t dataLength);
void testDM43_id_scan();
/******************************************************************************************************************************************************************/

void setup()
{
    // 初始化CAN线
    ESP32Can.CANInit(GPIO_NUM_40, GPIO_NUM_41, ESP32CAN_SPEED_1MBPS);

    // DM43 dm43(1); // 假设CAN ID为1
    // dm43.setSendFrame(send2CAN);
    // dm43.init(1);
    

    Serial.begin(115200);
    // Serial.setRxTimeout(0);
    userShellInit();
    delay(10);
    
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid, password);
    // // 等待 WiFi 连接
    // while (WiFi.status() != WL_CONNECTED) {
    //     delay(1000);
    //     logInfo("Connecting to WiFi...");
    // }

    // // 打印 WiFi 连接信息
    // ip = WiFi.localIP();
    // logInfo("WiFi connected");
    // logInfo("IP Address     : %s", ip.toString().c_str());
    // logInfo("MAC Address    : %s", WiFi.macAddress().c_str());
    // logInfo("SSID           : %s", WiFi.SSID().c_str());
    // logInfo("RSSI           : %d dBm", WiFi.RSSI());
    // logInfo("Hostname       : %s", WiFi.getHostname());
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
//     ArduinoOTA.setHostname(name);
//     ArduinoOTA
//         .onStart([]()
//                  {
//             String type;
//             if (ArduinoOTA.getCommand() == U_FLASH)
//                 type = "sketch";
//             else  // U_SPIFFS
//                 type = "filesystem";

//             // NOTE: if updating SPIFFS this would be the place to unmount
//             // SPIFFS using SPIFFS.end()
//             // logPrintln("Start updating " + type);
//             logPrintln("Start updating %s", type.c_str()); })
//         .onEnd([]()
//                { logPrintln("\nEnd"); })
//         .onProgress([](unsigned int progress, unsigned int total)
//                     { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
//         .onError([](ota_error_t error)
//                  {
//             Serial.printf("Error[%u]: ", error);
//             if (error == OTA_AUTH_ERROR) {
//                 logError("Auth Failed");
//             } else if (error == OTA_BEGIN_ERROR) {
//                 logError("Begin Failed");
//             } else if (error == OTA_CONNECT_ERROR) {
//                 logError("Connect Failed");
//             } else if (error == OTA_RECEIVE_ERROR) {
//                 logError("Receive Failed");
//             } else if (error == OTA_END_ERROR) {
//                 logError("End Failed");
//             } });
//     ArduinoOTA.begin();
//     telnet.begin(port);
// /*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
//     // 启动 Web 服务
//     server.on("/", []() {
//         server.send(200, "text/plain", "ESP32 OTA Ready!");
//     });
//     server.begin();
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/

    pinMode(PIN_RGB_LED, OUTPUT);
}
void loop()
{
    // telnet.loop();
    // server.handleClient();     // 浏览器访问处理
    // ArduinoOTA.handle();
    digitalWrite(PIN_RGB_LED, !digitalRead(PIN_RGB_LED));
    logInfo("controlTask-loop-ktech——test");
    delay(1000);
    testDM43_id_scan();
}
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
void send2CAN(uint32_t id, uint8_t *data, uint8_t dataLength)
{
    twai_message_t tx_frame;
    tx_frame.extd = 0; /* CAN ID is standard 11bit, for 29bit set to 1*/
    tx_frame.rtr = 0;
    tx_frame.data_length_code = dataLength; /* send 8 bytes of data */
    tx_frame.identifier = id;               /* CAN id is 0x123 */
    for (int i = 0; i < dataLength; i++)
    {
        tx_frame.data[i] = data[i];
    }
    ESP32Can.CANWriteFrame(&tx_frame); /* send the CAN message */
    
    // 使用 shellPrint 打印 CAN ID 和 data 信息
    shellPrint(&shell, "CAN Message sent: ID=0x%X, Data=", id);
    for (int i = 0; i < dataLength; i++) {
        shellPrint(&shell, "%02X ", data[i]);
    }
    shellPrint(&shell, "\r\n");
}

void testDM43_id_scan() {
    for (int id = 1; id <= 24; id++) {
        // 创建 DM43 对象并初始化（假设构造函数接收的是 CAN ID）
        DM43 dm43(id);
        dm43.setSendFrame(send2CAN);
        dm43.init(id);

        // 设置MIT模式下的位置
        motor_t motor;
        motor.id = id;
        motor.ctrl.mode = 0; // MIT模式
        motor.ctrl.pos_set = 0.0f; // 设定位置
        motor.ctrl.vel_set = 0.0f; // 设定速度
        motor.ctrl.kp_set = 8.0f; // 设定KP
        motor.ctrl.kd_set = 2.5f; // 设定KD
        motor.ctrl.tor_set = 2.0f; // 设定扭矩
        
        float shengsuo_vel = 15.0f;//旋转电机速度

        // 在 shell 中打印当前测试的 ID
        shellPrint(&shell, "Testing DM43 with ID: %d\r\n", id);
        
        // 发送运行指令（例如 pos_speed_ctrl(channel, value, duration)）
        // 这里设定一个示例值 0.8f，持续 1 秒
        // dm43.pos_speed_ctrl(id, 0.8f, 1.0f);
        dm43.mit_ctrl(motor.id,-1.5f, motor.ctrl.vel_set, motor.ctrl.kp_set, motor.ctrl.kd_set, motor.ctrl.tor_set);
        delay(1000);
        
        // 执行紧急停止（假设 DM43 类提供 stop() 方法）
        // dm43.pos_speed_ctrl(id, -3.3f, 5.0f);//位置速度控制函数
        dm43.mit_ctrl(motor.id,0.0f, motor.ctrl.vel_set, motor.ctrl.kp_set, motor.ctrl.kd_set, motor.ctrl.tor_set);
        shellPrint(&shell, "Emergency stop issued for ID: %d\r\n", id);
        
        // 每个测试之间添加短暂延时
        delay(500);
    }
}

