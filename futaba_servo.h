#include <EEPROM.h>

#define SERVO_SERIAL Serial2
#define SERVO_BAUDRATE 9600

#define SERVO_COUNT_MAX 2

#define TAIL_COMM_ENABLE_PIN 8

#define REQUEST_COOLDOWN 200UL
#define DEBUG_COOLDOWN 200UL

#define MIN 0
#define NEU 1
#define MAX 2

#define SERVO_COMM_MAX_BYTES 32

// 尾翼サーボ用送受信パケット
uint8_t servo_tx_packet[SERVO_COMM_MAX_BYTES] = {0}; //送信用パケット
uint8_t servo_rx_packet[SERVO_COMM_MAX_BYTES] = {0}; //受信用パケット

#define SWEEP_STEPS 8

int16_t sweep_durations[2][SWEEP_STEPS] = {
  {0, 100, 250, 100, 500, 100, 250, 100}, // 低速試験動作　時間(10ms) 合計14秒以上
  {0, 10, 50, 0, 100, 0, 50, 0}   // 高速試験動作　時間(10ms) 合計2.1秒以上
};

uint8_t sweep_angles[SWEEP_STEPS] = {
  NEU, NEU, MIN, MIN, MAX, MAX, NEU, NEU
};

typedef struct ServoInfo {
  uint8_t id;
  String alias;

  int16_t controller_pin;

  int16_t l_min = 0;
  int16_t c_min = 500;
  int16_t c_max = 524;
  int16_t h_max = 1023;

  int16_t control_value;
  int16_t val = 0;
  int16_t val_threshold[3] = { -50, 0, 50};

  boolean test_mode = false;
  boolean sweep_mode = false;

  uint8_t sweep_speed = 0;
  uint8_t sweep_next_step = 0;
  uint32_t sweep_begin_time = 0;
  uint32_t sweep_last_step_time = 0;

  uint32_t last_request_time;

  uint8_t torque_mode = 1;
  uint8_t actual_torque_mode = 0;
  uint8_t torque_percentage = 100;
  int16_t actual_position = 0;
  uint8_t temp_limit = 0;
  uint8_t temp_limit_alarm = 0;
  uint8_t rom_write_error = 0;
  uint8_t packet_error = 0;
  int16_t load = 0;
  int16_t temperature = 0;
  int16_t voltage = 0;
} ServoInfo;

ServoInfo servo_info[SERVO_COUNT_MAX];

int16_t servo_count = 0;

uint32_t last_debug_time = 0;

// ID から DATA の末尾までの各バイトのXOR
byte checksum(uint8_t * data, uint8_t len) {
  byte sum = 0;
  for (uint8_t i = 2; i < len; i++) sum ^= data[i];
  return sum;
}

void transmit_packet(uint8_t len) {
  digitalWrite(TAIL_COMM_ENABLE_PIN, HIGH);       //送信許可
  SERVO_SERIAL.write(servo_tx_packet, len);            //サーボに送信
  SERVO_SERIAL.flush();                            //リードバッファを初期化(送信データがすべて送信されるまで待つ)
  digitalWrite(TAIL_COMM_ENABLE_PIN, LOW);        //送信禁止
}

void servo_reboot(uint8_t id) {
  servo_tx_packet[0] = 0xFA;                                     //Header
  servo_tx_packet[1] = 0xAF;                                     //Header
  servo_tx_packet[2] = id;                                       //ID
  servo_tx_packet[3] = 0x20;                                     //Flags
  servo_tx_packet[4] = 0xFF;                                     //Address
  servo_tx_packet[5] = 0x00;                                     //Length
  servo_tx_packet[6] = 0x00;                                     //Count
  servo_tx_packet[7] = checksum(servo_tx_packet, 7);               //sum

  transmit_packet(8);
  delay(30);
}

// サーボのトルク％を設定(基本使わない)
void servo_torque_value(uint8_t id, uint8_t value) {
  servo_tx_packet[0] = 0xFA;                                     //Header
  servo_tx_packet[1] = 0xAF;                                     //Header
  servo_tx_packet[2] = id;                                       //ID
  servo_tx_packet[3] = 0x00;                                     //Flags
  servo_tx_packet[4] = 0x23;                                     //Address
  servo_tx_packet[5] = 0x01;                                     //Length
  servo_tx_packet[6] = 0x01;                                     //Count
  servo_tx_packet[7] = (uint8_t)value;                           //ON/OFF
  servo_tx_packet[8] = checksum(servo_tx_packet, 8);               //sum

  if (!(0 <= value && value <= 100)) return;
  transmit_packet(9);
}

// トルクの種類を設定
// Address 0x24 (RAM範囲)
// 1 byte (0 or 1 or 2)
// 0: OFF
// 1: ON
// 2: BREAK MODE (手で動かせるぐらいの弱いトルクにする)
void servo_set_torque_mode(uint8_t id, uint8_t state) {
  servo_tx_packet[0] = 0xFA;                                     //Header
  servo_tx_packet[1] = 0xAF;                                     //Header
  servo_tx_packet[2] = id;                                       //ID
  servo_tx_packet[3] = 0x00;                                     //Flags
  servo_tx_packet[4] = 0x24;                                     //Address
  servo_tx_packet[5] = 0x01;                                     //Length
  servo_tx_packet[6] = 0x01;                                     //Count
  servo_tx_packet[7] = (uint8_t) state & 0x00FF;                    //ON/OFF
  servo_tx_packet[8] = checksum(servo_tx_packet, 8);               //sum

  if (!(state == 0 || state == 1 || state == 2)) return;
  transmit_packet(9);
}


void servo_move(uint8_t id, uint16_t o_angle, uint16_t o_time) {
  servo_tx_packet[0] = 0xFA;                            //Header
  servo_tx_packet[1] = 0xAF;                            //Header
  servo_tx_packet[2] = id;                              //ID
  servo_tx_packet[3] = 0x00;                            //Flags
  servo_tx_packet[4] = 0x1E;                            //Address
  servo_tx_packet[5] = 0x04;                            //Length
  servo_tx_packet[6] = 0x01;                            //Count
  servo_tx_packet[7] = lowByte(o_angle);                //目標位置データ(下位バイト)
  servo_tx_packet[8] = highByte(o_angle);               //目標位置データ(上位バイト)
  servo_tx_packet[9] = lowByte(o_time);                 //目標時間データ(下位バイト)
  servo_tx_packet[10] = highByte(o_time);               //目標時間データ(上位バイト)
  servo_tx_packet[11] = checksum(servo_tx_packet, 11);    //Checksum

  transmit_packet(12);
}

void servo_angle_eeprom_set(uint8_t id, uint8_t type, int16_t val) {
  EEPROM.put(id * 6 + type * 2, val);
}

uint8_t get_index(uint8_t id) {
  for (uint8_t i = 0; i < servo_count; i++) {
    if (servo_info[i].id == id) return i;
  }
  return 0xFF;
}

bool servo_add(uint8_t id, String alias, int16_t controller_pin, int16_t l_min, int16_t c_min, int16_t c_max, int16_t h_max, int16_t val_min, int16_t val_neu, int16_t val_max) {
  if (servo_count >= SERVO_COUNT_MAX) return false;
  if (val_min == val_neu) {
    val_min--;
    servo_angle_eeprom_set(id, MIN, val_min);
  }
  if (val_max == val_neu) {
    val_max++;
    servo_angle_eeprom_set(id, MAX, val_max);
  }
  if (val_min < -1500 || val_min > 1500) return false;
  if (val_neu < -1500 || val_neu > 1500) return false;
  if (val_max < -1500 || val_max > 1500) return false;
  servo_info[servo_count].id = id;
  servo_info[servo_count].alias = alias;
  servo_info[servo_count].controller_pin = controller_pin;
  servo_info[servo_count].l_min = l_min;
  servo_info[servo_count].c_min = c_min;
  servo_info[servo_count].c_max = c_max;
  servo_info[servo_count].h_max = h_max;
  servo_info[servo_count].val_threshold[MIN] = val_min;
  servo_info[servo_count].val_threshold[NEU] = val_neu;
  servo_info[servo_count].val_threshold[MAX] = val_max;
  servo_count++;
  pinMode(controller_pin, INPUT);
  return true;
};

void command_send_all();

void servo_maintain() {
  for (uint8_t i = 0; i < servo_count; i++) {
    if (servo_info[i].actual_torque_mode != servo_info[i].torque_mode) {
      servo_reboot(servo_info[i].id);
      servo_set_torque_mode(servo_info[i].id, servo_info[i].torque_mode);
    }
    if (servo_info[i].sweep_mode) {
      if (millis() - servo_info[i].sweep_begin_time > 12500) {
        servo_info[i].sweep_mode = false;
        servo_info[i].sweep_next_step = 0;
        command_send_all();
        continue;
      }
      if (millis() - servo_info[i].sweep_last_step_time < sweep_durations[servo_info[i].sweep_speed][servo_info[i].sweep_next_step] * 10) continue;
      servo_info[i].sweep_next_step++;
      if (servo_info[i].sweep_next_step >= SWEEP_STEPS) {
        servo_info[i].sweep_mode = false;
        servo_info[i].sweep_next_step = 0;
        command_send_all();
        continue;
      }
      servo_info[i].val = servo_info[i].val_threshold[sweep_angles[servo_info[i].sweep_next_step]];
      servo_move(servo_info[i].id, servo_info[i].val_threshold[sweep_angles[servo_info[i].sweep_next_step]], sweep_durations[servo_info[i].sweep_speed][servo_info[i].sweep_next_step]);
      servo_info[i].sweep_last_step_time = millis();
      DEBUG_SERIAL.print(F("Moved sweep ID: "));
      DEBUG_SERIAL.print(servo_info[i].id);
      DEBUG_SERIAL.print(F(" step: "));
      DEBUG_SERIAL.print(servo_info[i].sweep_next_step);
      DEBUG_SERIAL.print(F(" angle: "));
      DEBUG_SERIAL.print(servo_info[i].val_threshold[sweep_angles[servo_info[i].sweep_next_step]]);
      DEBUG_SERIAL.print(F(" duration: "));
      DEBUG_SERIAL.println(sweep_durations[servo_info[i].sweep_speed][servo_info[i].sweep_next_step] * 10);
    }
  }
}

void servo_setup() {
  SERVO_SERIAL.begin(SERVO_BAUDRATE);
  pinMode(TAIL_COMM_ENABLE_PIN, OUTPUT);
  servo_maintain();
}

void servo_control_all() {
  for (uint8_t i = 0; i < servo_count; i++) {
    if (servo_info[i].test_mode || servo_info[i].sweep_mode) continue;
    servo_info[i].control_value = analogRead(servo_info[i].controller_pin);
    if (servo_info[i].control_value < servo_info[i].c_min)      servo_info[i].val = map(servo_info[i].control_value, servo_info[i].l_min, servo_info[i].c_min, servo_info[i].val_threshold[MIN], servo_info[i].val_threshold[NEU]);
    else if (servo_info[i].control_value > servo_info[i].c_max) servo_info[i].val = map(servo_info[i].control_value, servo_info[i].c_max, servo_info[i].h_max, servo_info[i].val_threshold[NEU], servo_info[i].val_threshold[MAX]);
    else                                                        servo_info[i].val = servo_info[i].val_threshold[NEU];
    servo_move(servo_info[i].id, servo_info[i].val, 20);
  }
}

void servo_request_data(uint8_t id, uint8_t address, uint8_t len) {
  servo_tx_packet[0] = 0xFA; //Header
  servo_tx_packet[1] = 0xAF; //Header
  servo_tx_packet[2] = id;   //ID
  servo_tx_packet[3] = 0x0F; //Flags
  servo_tx_packet[4] = address; //Address
  servo_tx_packet[5] = len;
  servo_tx_packet[6] = 0x00; //Count
  servo_tx_packet[7] = checksum(servo_tx_packet, 7); //sum

  /*DEBUG_SERIAL.print("SENT: ");
    for (uint8_t i = 0; i < 8; i++){
    DEBUG_SERIAL.print(servo_tx_packet[i], HEX);
    DEBUG_SERIAL.print(' ');
    }
    DEBUG_SERIAL.println();*/
  for (uint8_t i = 0; SERVO_SERIAL.available() && i < 255; i++) SERVO_SERIAL.read();
  transmit_packet(8);
}

bool servo_receive_data(uint8_t len) {
  delayMicroseconds(150);
  uint32_t wait_time;   // リターンパケット待ち用引数
  // リターンパケットの最初のバイトが来るまで待つ(最大10ms)
  // そもそも返信が来ているか（接続されているか）をチェック
  wait_time = millis();
  while (!SERVO_SERIAL.available()) if (millis() - wait_time > 10UL) return false;
  // リターンパケットの最初のバイトが0xFDになるまで待つ(最大25ms) // パケットズレ防止
  // リターンパケットが来ても、以前受信に失敗した分かも知れないのでヘッダーが来るまで流す
  wait_time = millis();
  while ((servo_rx_packet[0] = SERVO_SERIAL.read()) != 0xFD) if (millis() - wait_time > 25UL) return false;
  delayMicroseconds(1050);
  // 残りのバイトを読み取る
  wait_time = millis();
  for (uint8_t i = 1; i < 8 + len; i++) {
    while (!SERVO_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 3UL) return false;
    servo_rx_packet[i] = SERVO_SERIAL.read();
    wait_time = millis();
    delayMicroseconds(1010);
  }
  if ((servo_rx_packet[0] == 0xFD) && (servo_rx_packet[1] == 0xDF)) {
    if (servo_rx_packet[7 + len] == checksum(servo_rx_packet, 7 + len)) return true;
  }
  return false;
}

void servo_pack_info(uint8_t id, uint8_t* packet) {
  uint8_t index;
  if ((index = get_index(id)) == 0xFF) return;
  if ((uint32_t)(millis() - servo_info[index].last_request_time) < REQUEST_COOLDOWN) return;
  servo_request_data(servo_info[index].id, 30, 24);
  if (servo_receive_data(24)) {
    packet[0 ] = servo_tx_packet[3];    // flag
    packet[1 ] = servo_rx_packet[7];    // goal position L
    packet[2 ] = servo_rx_packet[8];    // goal position H
    packet[3 ] = servo_rx_packet[9];    // goal time L
    packet[4 ] = servo_rx_packet[10];   // goal time H
    packet[5 ] = servo_rx_packet[12];   // max torque
    packet[6 ] = servo_rx_packet[13];   // torque mode
    packet[7 ] = servo_rx_packet[19];   // present position L
    packet[8 ] = servo_rx_packet[20];   // present position H
    packet[9 ] = servo_rx_packet[21];   // present time L
    packet[10] = servo_rx_packet[22];   // present time H
    packet[11] = servo_rx_packet[23];   // present speed L
    packet[12] = servo_rx_packet[24];   // present speed H
    packet[13] = servo_rx_packet[25];   // present load L
    packet[14] = servo_rx_packet[26];   // present load H
    packet[15] = servo_rx_packet[27];   // present temperature L
    packet[16] = servo_rx_packet[28];   // present temperature H
    packet[17] = servo_rx_packet[29];   // present voltage L
    packet[18] = servo_rx_packet[30];   // present voltage H
    servo_info[index].temp_limit          = (servo_rx_packet[3] & B10000000) >> 7;
    servo_info[index].temp_limit_alarm    = (servo_rx_packet[3] & B00100000) >> 5;
    servo_info[index].rom_write_error     = (servo_rx_packet[3] & B00001000) >> 3;
    servo_info[index].packet_error        = (servo_rx_packet[3] & B00000010) >> 1;
    servo_info[index].actual_torque_mode  = servo_rx_packet[13];
    servo_info[index].torque_percentage   = servo_rx_packet[12];
    servo_info[index].actual_position     = ((uint16_t)servo_rx_packet[20] << 8) | (uint16_t)servo_rx_packet[19];
    servo_info[index].load                = ((uint16_t)servo_rx_packet[26] << 8) | (uint16_t)servo_rx_packet[25];
    servo_info[index].temperature         = ((uint16_t)servo_rx_packet[28] << 8) | (uint16_t)servo_rx_packet[27];
    servo_info[index].voltage             = ((uint16_t)servo_rx_packet[30] << 8) | (uint16_t)servo_rx_packet[29];
  }
  else {
    servo_info[index].actual_torque_mode = 0;
  }
}

void print_debug_info() {
  if (millis() - last_debug_time < DEBUG_COOLDOWN) return;
  for (uint8_t i = 0; i < servo_count; i++) {
    // if (i == 0)
    DEBUG_SERIAL.println(servo_info[i].alias + F("\t") + String(servo_info[i].control_value)
                         + F("\t") + String(servo_info[i].val)
                         + F("\t") + String(servo_info[i].actual_position) + "\t" + String(servo_info[i].load)
                         + F("\t") + String(servo_info[i].temperature) + "\t" + String(servo_info[i].voltage)
                         + F("\t") + String(servo_info[i].actual_torque_mode));
  }
  last_debug_time = millis();
}
