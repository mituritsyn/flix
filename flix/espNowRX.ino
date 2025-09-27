#include <Arduino.h>  // Добавляем базовый заголовок
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>  // Добавляем WiFi заголовок
#include <esp_system.h> // Добавляем системный заголовок

// Добавляем определения из ESP-IDF
#include <esp_wifi_types.h>

extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlArmed, controlMode;
extern uint16_t channels[16];
// Структура для получаемых данных (должна совпадать с отправителем)
typedef struct {
  int8_t sticks[4];
  bool button;
} message_struct;

message_struct rxPacket;
bool newMessageReceived = false; // Флаг для обработки в основном цикле

// Функция обратного вызова при получении сообщения (новая версия ESP32 Core)
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Минимальная обработка в прерывании
  memcpy(&rxPacket, incomingData, sizeof(rxPacket));
  newMessageReceived = true;
}

void setupEspNowRX() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
}

void readEspNowRX() {
  if (newMessageReceived) {
    newMessageReceived = false;

    // Обработка полученных данных
    controlRoll = rxPacket.sticks[0] / 100.0f;
    controlPitch = rxPacket.sticks[1] / 100.0f;
    controlThrottle = rxPacket.sticks[2] / 100.0f;
    controlYaw = rxPacket.sticks[3] / 100.0f;
    controlArmed = rxPacket.button;
    // controlMode = rxPacket.mode;

    // Обновление каналов
    // for (int i = 0; i < 16; i++) {
    //   channels[i] = rxPacket.channels[i];
    // }
  }
}