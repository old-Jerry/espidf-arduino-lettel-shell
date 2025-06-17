#include "ESP32CAN.h"
// 创建 ESP32Can 全局实例，用户可直接调用 ESP32Can.CANInit(...) 等
ESP32CAN ESP32Can;
/**
 * @brief 初始化 CAN（TWAI）驱动
 * 
 * @param tx_pin CAN TX 引脚号（GPIO）
 * @param rx_pin CAN RX 引脚号（GPIO）
 * @param baud   波特率枚举
 * @return ESP32CAN_status_t 成功或失败状态码
 */
ESP32CAN_status_t ESP32CAN::CANInit(gpio_num_t tx_pin, gpio_num_t rx_pin, ESP32CAN_timing_t baud) {
  // 初始化 TWAI 总线的通用配置，使用标准工作模式
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);

  // 接收过滤器配置（使用之前设置的过滤器 _f_config）
  twai_filter_config_t f_config = _f_config;

  // 定义波特率配置结构体
  twai_timing_config_t t_config;

  // 根据传入的波特率枚举选择对应的 TWAI 波特率配置
  switch (baud) {
    case ESP32CAN_SPEED_100KBPS:
      t_config = TWAI_TIMING_CONFIG_100KBITS();
      break;
    case ESP32CAN_SPEED_125KBPS:
      t_config = TWAI_TIMING_CONFIG_125KBITS();
      break;
    case ESP32CAN_SPEED_250KBPS:
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case ESP32CAN_SPEED_500KBPS:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case ESP32CAN_SPEED_800KBPS:
      t_config = TWAI_TIMING_CONFIG_800KBITS();
      break;
    case ESP32CAN_SPEED_1MBPS:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:
      debugPrintln("TWAI: undefined buad rate");
      return ESP32CAN_NOK;
  }

  // 安装 TWAI 驱动
  switch (twai_driver_install(&g_config, &t_config, &f_config)) {
    case ESP_OK:
      debugPrintln("TWAI INSTALL: ok");
      break;
    case ESP_ERR_INVALID_ARG:
      debugPrintln("TWAI INSTALL: ESP_ERR_INVALID_ARG");
      return ESP32CAN_NOK;
    case ESP_ERR_NO_MEM:
      debugPrintln("TWAI INSTALL: ESP_ERR_NO_MEM");
      return ESP32CAN_NOK;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI INSTALL: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    default:
      debugPrintln("TWAI INSTALL: unknown error");
      return ESP32CAN_NOK;
  }

  // 启动 TWAI 总线
  switch (twai_start()) {
    case ESP_OK:
      debugPrintln("TWAI START: ok");
      break;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI START: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    default:
      debugPrintln("TWAI START: unknown error");
      return ESP32CAN_NOK;
  }

  return ESP32CAN_OK;
}

/**
 * @brief 停止并卸载 CAN（TWAI）驱动
 */
ESP32CAN_status_t ESP32CAN::CANStop() {
  // 停止总线
  switch (twai_stop()) {
    case ESP_OK:
      debugPrintln("TWAI STOP: ok");
      break;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI STOP: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    default:
      debugPrintln("TWAI STOP: unknown error");
      return ESP32CAN_NOK;
  }

  // 卸载驱动
  switch (twai_driver_uninstall()) {
    case ESP_OK:
      debugPrintln("TWAI UNINSTALL: ok");
      break;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI UNINSTALL: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    default:
      break;
  }

  return ESP32CAN_OK;
}

/**
 * @brief 发送 CAN 帧
 * 
 * @param p_frame 指向要发送的帧结构体 twai_message_t
 * @return ESP32CAN_status_t 成功或失败
 */
ESP32CAN_status_t ESP32CAN::CANWriteFrame(const twai_message_t* p_frame) {
  // 尝试发送帧，超时 10 毫秒
  switch (twai_transmit(p_frame, pdMS_TO_TICKS(10))) {
    case ESP_OK:
      break;
    case ESP_ERR_INVALID_ARG:
      debugPrintln("TWAI TX: ESP_ERR_INVALID_ARG");
      return ESP32CAN_NOK;
    case ESP_ERR_TIMEOUT:
      debugPrintln("TWAI TX: ESP_ERR_TIMEOUT");
      return ESP32CAN_NOK;
    case ESP_FAIL:
      debugPrintln("TWAI TX: ESP_FAIL");
      return ESP32CAN_NOK;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI TX: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    case ESP_ERR_NOT_SUPPORTED:
      debugPrintln("TWAI TX: ESP_ERR_NOT_SUPPORTED");
      return ESP32CAN_NOK;
    default:
      debugPrintln("TWAI TX: unknown error");
      return ESP32CAN_NOK;
  }

  return ESP32CAN_OK;
}

/**
 * @brief 接收 CAN 帧
 * 
 * @param p_frame 指向 twai_message_t，用于存储接收到的帧数据
 */
ESP32CAN_status_t ESP32CAN::CANReadFrame(twai_message_t* p_frame) {
  // 从接收队列中读取帧，超时 10 毫秒
  switch (twai_receive(p_frame, pdMS_TO_TICKS(10))) {
    case ESP_OK:
      break;
    case ESP_ERR_TIMEOUT:
      debugPrintln("TWAI RX: ESP_ERR_TIMEOUT");
      return ESP32CAN_NOK;
    case ESP_ERR_INVALID_ARG:
      debugPrintln("TWAI RX: ESP_ERR_INVALID_ARG");
      return ESP32CAN_NOK;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI RX: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    default:
      debugPrintln("TWAI RX: unknown error");
      return ESP32CAN_NOK;
  }

  return ESP32CAN_OK;
}

/**
 * @brief 清空 CAN 接收队列
 */
ESP32CAN_status_t ESP32CAN::CANReceiveFlush() {
  switch (twai_clear_receive_queue()) {
    case ESP_OK:
      break;
    case ESP_ERR_INVALID_STATE:
      debugPrintln("TWAI RX FLUSH: ESP_ERR_INVALID_STATE");
      return ESP32CAN_NOK;
    default:
      debugPrintln("TWAI RX FLUSH: unknown error");
      return ESP32CAN_NOK;
  }
  return ESP32CAN_OK;
}

/**
 * @brief 配置过滤器，仅接受指定 ID（自动左移 21 位）
 * 
 * @param id 需要接受的标准帧 ID（11 位）
 */
int ESP32CAN::CANConfigFilter(uint16_t id) {
  _id = id;
  uint32_t acc_code, acc_mask;

  acc_code = id << 21;              // ID 左移 21 位填充标准帧位置
  acc_code = acc_code & 0xFFE00000; // 保留高位，屏蔽低位
  acc_mask = 0x001FFFFF;            // 标准掩码，保留低 21 位

  // 设置过滤器配置结构体
  _f_config = {.acceptance_code = acc_code, .acceptance_mask = acc_mask, .single_filter = true};
  return 0;
}

/**
 * @brief 设置过滤器，自定义验收码和掩码（支持 11 或 29 位）
 * 
 * @param acceptance_code 接受码
 * @param acceptance_mask 接受掩码
 * @param single_filter 是否使用单一过滤器
 * @return int 操作结果（0 表示成功）
 */
int ESP32CAN::CANConfigFilter(uint32_t acceptance_code, uint32_t acceptance_mask, bool single_filter) {
  _f_config = {.acceptance_code = acceptance_code, .acceptance_mask = acceptance_mask, .single_filter = single_filter};
  return 0;
}


