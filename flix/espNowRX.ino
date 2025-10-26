#include <esp_now.h>
#include <WiFi.h>

extern float controlRoll, controlPitch, controlThrottle, controlYaw;
extern uint16_t channels[16];
extern float controlTime;
// Структура для получаемых данных (должна совпадать с отправителем)
typedef struct {
  int8_t sticks[4];
  bool button;
} message_struct;

volatile message_struct rxPacket;
bool newMessageReceived = false; // Флаг для обработки в основном цикле
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void setupEspNowRX() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
}
// Функция обратного вызова при получении сообщения (новая версия ESP32 Core)
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    portENTER_CRITICAL(&mux);
    memcpy((void*)&rxPacket, incomingData, sizeof(rxPacket));
    newMessageReceived = true;
    portEXIT_CRITICAL(&mux);
}

void readEspNowRX() {
    if (newMessageReceived) {
        message_struct packet;
        portENTER_CRITICAL(&mux);
        memcpy(&packet, (const void*)&rxPacket, sizeof(packet));
        newMessageReceived = false;
        portEXIT_CRITICAL(&mux);
        
        // Process local copy of data
        controlRoll = packet.sticks[1] / 127.0f;
        controlPitch = packet.sticks[0] / 127.0f;
        // Convert [-127;127] to [0;1]
        controlThrottle = 0.7 * (packet.sticks[3] + 127) / 254.0f;
        controlYaw = packet.sticks[2] / 127.0f;
        controlTime = t;
    }
}
