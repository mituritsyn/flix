#include <esp_now.h>
#include <WiFi.h>

extern float controlRoll, controlPitch, controlThrottle, controlYaw;
extern uint16_t channels[16];
// Структура для получаемых данных (должна совпадать с отправителем)
typedef struct {
  int8_t sticks[4];
  bool button;
} message_struct;

message_struct rxPacket;
bool newMessageReceived = false; // Флаг для обработки в основном цикле



void setupEspNowRX() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
}
// Функция обратного вызова при получении сообщения (новая версия ESP32 Core)
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Минимальная обработка в прерывании
  memcpy(&rxPacket, incomingData, sizeof(rxPacket));
  newMessageReceived = true;
}
void readEspNowRX() {
  if (newMessageReceived) {
    newMessageReceived = false;

    // Обработка полученных данных
    controlRoll = rxPacket.sticks[1] / 127.0f;
    controlPitch = rxPacket.sticks[0] / 127.0f;
    controlThrottle = rxPacket.sticks[3] / 127.0f;
    controlYaw = rxPacket.sticks[2] / 127.0f;
    // Serial.printf("controlRoll: %f, controlPitch: %f, controlThrottle: %f, controlYaw: %f\n", controlRoll, controlPitch, controlThrottle, controlYaw);
    // controlArmed = rxPacket.button;
    // controlMode = rxPacket.mode;

    // Обновление каналов
    // for (int i = 0; i < 16; i++) {
    //   channels[i] = rxPacket.channels[i];
    // }
  }
}