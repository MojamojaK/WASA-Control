#define SENSORY_SERIAL Serial3
#define SENSORY_BAUDRATE 9600

// 計測基板送信用パケット最大バイト数
#define SENSORY_COMM_MAX_BYTES 64
// 計測値送信用パケット
uint8_t sensory_tx_packet[SENSORY_COMM_MAX_BYTES] = {0};

uint16_t cooldown = 200;
uint32_t last_time = 0;

void sensory_setup() {
  SENSORY_SERIAL.begin(SENSORY_BAUDRATE);
}

uint8_t *sensory_packet(uint8_t id) {
  return sensory_tx_packet + 3 + (id - 1) * 15;
}

void sensory_transmit() {
  if ((uint32_t)(millis() - last_time) > cooldown) {
    sensory_tx_packet[0] = 0x7C;
    sensory_tx_packet[1] = 0xC7;
    sensory_tx_packet[2] = 30;
    sensory_tx_packet[33] = checksum(sensory_tx_packet, 33);
    SENSORY_SERIAL.write(sensory_tx_packet, 34);
    SENSORY_SERIAL.flush();
  }
}
