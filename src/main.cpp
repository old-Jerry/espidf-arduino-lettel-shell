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

// extern "C" {               // ← 只用 ASCII 直引号
//     #include "shell_port.h"    // 确保能被找到
// }
#include "log.h"
#include "shell_cpp.h"
#include "shell_port.h"


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

#define SERIALDEBUG telnet
#define     SHELL_UART      HWCDCSerial

const char *name = "oldjerry_esp32";
// const char* ssid = "Work";
// const char* password = "xhaxx518";
const char *ssid = "quest2";
const char *password = "meiyijia";
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/

void wifiScan();
/******************************************************************************************************************************************************************/

void setup()
{
    Serial.begin(115200);
    // Serial.setRxTimeout(0);
    userShellInit();
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    // 等待 WiFi 连接
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        logInfo("Connecting to WiFi...");
    }

    // 打印 WiFi 连接信息
    ip = WiFi.localIP();
    logInfo("WiFi connected");
    logInfo("IP Address     : %s", ip.toString().c_str());
    logInfo("MAC Address    : %s", WiFi.macAddress().c_str());
    logInfo("SSID           : %s", WiFi.SSID().c_str());
    logInfo("RSSI           : %d dBm", WiFi.RSSI());
    logInfo("Hostname       : %s", WiFi.getHostname());
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
    ArduinoOTA.setHostname(name);
    ArduinoOTA
        .onStart([]()
                 {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else  // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            // logPrintln("Start updating " + type);
            logPrintln("Start updating %s", type.c_str()); })
        .onEnd([]()
               { logPrintln("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                logError("Auth Failed");
            } else if (error == OTA_BEGIN_ERROR) {
                logError("Begin Failed");
            } else if (error == OTA_CONNECT_ERROR) {
                logError("Connect Failed");
            } else if (error == OTA_RECEIVE_ERROR) {
                logError("Receive Failed");
            } else if (error == OTA_END_ERROR) {
                logError("End Failed");
            } });
    ArduinoOTA.begin();
    telnet.begin(port);
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/
    // 启动 Web 服务
    server.on("/", []() {
        server.send(200, "text/plain", "ESP32 OTA Ready!");
    });
    server.begin();

    pinMode(PIN_RGB_LED, OUTPUT);
}
void loop()
{
    telnet.loop();
    server.handleClient();     // 浏览器访问处理
    ArduinoOTA.handle();
    digitalWrite(PIN_RGB_LED, !digitalRead(PIN_RGB_LED));
    logInfo("controlTask-loop-hello-OTA2");
    delay(1000);
}
/*——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————*/



