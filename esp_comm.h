#include <EEPROM.h>
#include "futaba_servo.h"

#define CMD_LOG 0x00
#define CMD_SET 0x01
#define CMD_REQ 0x02
#define CMD_RBT 0x03
#define CMD_TQS 0x04
#define CMD_TMS 0x05
#define CMD_TMD 0x06
#define CMD_TMV 0x07
#define CMD_SWP 0x08
#define CMD_PRP 0xF0

#define SET_RUD_MIN B00000001
#define SET_RUD_NEU B00000010
#define SET_RUD_MAX B00000011
#define SET_ELE_MIN B00000101
#define SET_ELE_NEU B00000110
#define SET_ELE_MAX B00000111

#define REQ_INI 0x01

#define DCM_DSP 0x00
#define DCM_PRP 0x01

#define ESP_SERIAL Serial1
#define ESP_BAUDRATE 9600

#define ESP_PACKET_SIZE 128
uint8_t esp_rx_packet[ESP_PACKET_SIZE] = {0};
uint8_t esp_tx_packet[ESP_PACKET_SIZE] = {0};

#define ESP_CONFIRM_COOLDOWN 5000UL

uint8_t command_len = 0;
uint8_t command_data_len = 0;

uint8_t command_device = 0;
uint8_t command_id = 0;

bool confirm_wait = false;
uint32_t confirm_wait_time = 0;
uint8_t confirm_device = 0;
uint8_t confirm_command_id = 0xFF;
uint16_t confirm_param[3] = {0};

void command_setup() {
  ESP_SERIAL.begin(ESP_BAUDRATE);
}

bool command_receive() {
  if (ESP_SERIAL.available()) {
    uint32_t wait_time;   // リターンパケット待ち用引数
    // リターンパケットの最初のバイトが0x8Fになるまで待つ(最大10ms) // パケットズレ防止
    // リターンパケットが来ても、以前受信に失敗した分かも知れないのでヘッダーが来るまで流す
    wait_time = millis();
    while ((esp_rx_packet[0] = ESP_SERIAL.read()) != 0x8F) if ((uint32_t)(millis() - wait_time) > 10UL) return false;
    // 残りのバイトを読み取る
    wait_time = millis();
    for (uint8_t j = 1; j < 5; j++) {
      while (!ESP_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 3UL) return false;
      esp_rx_packet[j] = ESP_SERIAL.read();
      wait_time = millis();
    }
    command_id = esp_rx_packet[3];
    command_device = esp_rx_packet[2];
    command_data_len = esp_rx_packet[4];
    command_len = command_data_len + 6;
    if (command_len > ESP_PACKET_SIZE) return false;
    wait_time = millis();
    for (uint8_t j = 5; j < command_len; j++) {
      while (!ESP_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 4UL) return false;
      esp_rx_packet[j] = ESP_SERIAL.read();
      wait_time = millis();
    }
    /*DEBUG_SERIAL.println("ESP RECEIVED:");
      for (uint8_t i = 0; i < command_len; i++) {
      DEBUG_SERIAL.print(esp_rx_packet[i], HEX);
      DEBUG_SERIAL.print(' ');
      }
      DEBUG_SERIAL.print(" actual checksum is " + String(checksum(esp_rx_packet, command_len - 1), HEX));
      DEBUG_SERIAL.println();*/
    if (esp_rx_packet[0] == 0x8F && esp_rx_packet[1] == 0xF8 && esp_rx_packet[command_len - 1] == checksum(esp_rx_packet, command_len - 1)) return true;
  }
  return false;
}

void command_log();
void command_confirm_set();
void command_respond();
void command_confirm_reboot();
void command_confirm_torque_percentage_set();
void command_confirm_torque_mode_set();
void command_confirm_test_mode_set();
void command_test_move();
void command_confirm_sweep();

void command_confirm();
void command_send_all();

void command_interpret() {
  // DEBUG_SERIAL.println("Command ID is " + String(command_id));
  switch (command_id) {
    case CMD_LOG:
      command_log();
      break;
    case CMD_SET:
      command_confirm_set();
      break;
    case CMD_REQ:
      command_respond();
      break;
    case CMD_RBT:
      command_confirm_reboot();
      break;
    case CMD_TQS:
      command_confirm_torque_percentage_set();
      break;
    case CMD_TMS:
      command_confirm_torque_mode_set();
      break;
    case CMD_TMD:
      command_confirm_test_mode_set();
      break;
    case CMD_TMV:
      command_test_move();
      break;
    case CMD_SWP:
      command_confirm_sweep();
      break;
    case CMD_PRP:
      command_confirm();
      command_send_all();
      break;
  }
}

void command_handle() {
  if (confirm_wait && millis() - confirm_wait_time > ESP_CONFIRM_COOLDOWN) confirm_wait = false;
  if (command_receive()) {
    command_interpret();
  }
}

void command_log() {
  DEBUG_SERIAL.print(F("[DEVICE_ID: "));
  DEBUG_SERIAL.print(esp_rx_packet[2]);
  DEBUG_SERIAL.print(F(", LOG: "));
  for (uint8_t i = 5; i < esp_rx_packet[4] + 5; i++) {
    if (esp_rx_packet[i] != '\n') DEBUG_SERIAL.write(esp_rx_packet[i]);
  }
  DEBUG_SERIAL.print(F(", COMMAND: "));
  for (uint8_t i = 0; i < command_len; i++) {
    DEBUG_SERIAL.print(esp_rx_packet[i], HEX);
    DEBUG_SERIAL.print(F(" "));
  }
  DEBUG_SERIAL.println(F("]"));
}

void command_confirm_set() {
  if (confirm_wait) return;
  if (command_data_len != 3) return;
  int16_t new_value = ((((uint16_t)esp_rx_packet[7] << 8) & 0xFF00) | (esp_rx_packet[6] & 0x00FF));
  if (new_value > 1500 || new_value < -1500) return;
  uint8_t servo_id = ((esp_rx_packet[5] >> 2) & 0x01) + 1;
  uint8_t value_type = (esp_rx_packet[5] & 0x03) - 1;
  uint8_t index = get_index(servo_id);
  if (index == 0xFF) return;
  if (servo_info[index].val_threshold[value_type] == new_value) return;
  int16_t tmp_threshold[3] = {0};
  for (uint8_t i = 0; i < 3; i++) tmp_threshold[i] = servo_info[index].val_threshold[i];
  tmp_threshold[value_type] = new_value;
  if (tmp_threshold[MIN] >= tmp_threshold[NEU] || tmp_threshold[MIN] >= tmp_threshold[MAX] || tmp_threshold[NEU] >= tmp_threshold[MAX]) return;
  if (servo_info[index].sweep_mode || servo_info[index].test_mode) return;

  esp_tx_packet[0]  = 0x8D;                                                   // ヘッダー
  esp_tx_packet[1]  = 0xD8;                                                   // ヘッダー
  esp_tx_packet[2]  = command_device;                                         // 送信先デバイスID
  esp_tx_packet[3]  = (uint8_t)DCM_PRP;                                       // デバイス用コマンド
  esp_tx_packet[4]  = (uint8_t)6;                                             // データ長
  esp_tx_packet[5]  = command_id;                                             // データ：操舵基板コマンド確認用
  esp_tx_packet[6]  = esp_rx_packet[5];                                       // データ：対象のサーボ角値指定
  esp_tx_packet[7]  = lowByte (servo_info[index].val_threshold[value_type]);  // データ：変更前の値
  esp_tx_packet[8]  = highByte(servo_info[index].val_threshold[value_type]);  // データ：変更前の値
  esp_tx_packet[9]  = lowByte (new_value);                                    // データ：変更後の値
  esp_tx_packet[10]  = highByte(new_value);                                   // データ：変更後の値
  esp_tx_packet[11] = checksum(esp_tx_packet, 11);                            // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 12);                                        // 送信
  ESP_SERIAL.flush();                                                         // 送信完了待ち

  confirm_wait = true;                        // 確認待ちを有効にする
  confirm_wait_time = millis();               // 確認待ちの開始時間
  confirm_command_id = command_id;            // 確認待ちのコマンド
  confirm_device = command_device;
  confirm_param[0] = (uint16_t)index;         // 確認待ちのコマンド処理用付加データ(サーボINDEX)
  confirm_param[1] = (uint16_t)value_type;    // 確認待ちのコマンド処理用付加データ(MIN|NEU|MAXの区別)
  confirm_param[2] = (uint16_t)new_value;     // 確認待ちのコマンド処理用付加データ(角度)
}

void command_respond() {
  if (command_data_len != 1) return;
  switch (esp_rx_packet[5]) {
    case REQ_INI:
      DEBUG_SERIAL.println("Requested initial data");
      command_send_all();
      break;
  }
}

void command_confirm_reboot() {
  if (confirm_wait) return;
  if (command_data_len != 1) return;
  uint8_t servo_id = esp_rx_packet[5];
  uint8_t index = get_index(servo_id);
  if (index == 0xFF) return;
  if (servo_info[index].sweep_mode) return;

  esp_tx_packet[0] = 0x8D;                                      // ヘッダー
  esp_tx_packet[1] = 0xD8;                                      // ヘッダー
  esp_tx_packet[2] = command_device;                            // 送信先デバイスID
  esp_tx_packet[3] = (uint8_t)DCM_PRP;                          // デバイス用コマンド
  esp_tx_packet[4] = (uint8_t)2;                                // データ長
  esp_tx_packet[5] = command_id;                                // データ：操舵基板用コマンドの種類
  esp_tx_packet[6] = servo_id;                                  // データ：対象のサーボID
  esp_tx_packet[7] = checksum(esp_tx_packet, 7);                // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 8);                           // 送信
  ESP_SERIAL.flush();                                           // 送信完了待ち

  confirm_wait = true;                        // 確認待ちを有効にする
  confirm_wait_time = millis();               // 確認待ちの開始時間
  confirm_command_id = command_id;            // 確認待ちのコマンド
  confirm_device = command_device;
  confirm_param[0] = (uint16_t)index;         // 確認待ちのコマンド処理用付加データ(サーボINDEX)
}

void command_confirm_torque_percentage_set() {
  if (confirm_wait) return;
  if (command_data_len != 2) return;
  uint8_t servo_id = esp_rx_packet[5];
  uint8_t index = get_index(servo_id);
  if (index == 0xFF) return;
  uint8_t new_value = esp_rx_packet[6];
  if (servo_info[index].torque_percentage == new_value) return;
  if (new_value < 0 || new_value > 100) return;

  esp_tx_packet[0] = 0x8D;                                        // ヘッダー
  esp_tx_packet[1] = 0xD8;                                        // ヘッダー
  esp_tx_packet[2] = command_device;                              // 送信先デバイスID
  esp_tx_packet[3] = (uint8_t)DCM_PRP;                            // デバイス用コマンド
  esp_tx_packet[4] = (uint8_t)4;                                  // データ長
  esp_tx_packet[5] = command_id;                                  // データ：操舵基板用コマンドの種類
  esp_tx_packet[6] = servo_id;                                    // データ：対象のサーボID
  esp_tx_packet[7] = servo_info[index].torque_percentage;         // データ：変更前トルク%
  esp_tx_packet[8] = new_value;                                   // データ：変更後トルク%
  esp_tx_packet[9] = checksum(esp_tx_packet, 9);                  // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 10);                            // 送信
  ESP_SERIAL.flush();                                             // 送信完了待ち

  confirm_wait = true;                        // 確認待ちを有効にする
  confirm_wait_time = millis();               // 確認待ちの開始時間
  confirm_command_id = command_id;            // 確認待ちのコマンド
  confirm_device = command_device;
  confirm_param[0] = (uint16_t)index;         // 確認待ちのコマンド処理用付加データ(サーボINDEX)
  confirm_param[1] = (uint16_t)new_value;     // 確認待ちのコマンド処理用付加データ(トルク%)
}

void command_confirm_torque_mode_set() {
  if (confirm_wait) return;
  if (command_data_len != 2) return;
  uint8_t servo_id = esp_rx_packet[5];
  uint8_t index = get_index(servo_id);
  if (index == 0xFF) return;
  uint8_t new_value = esp_rx_packet[6];
  if (servo_info[index].torque_mode == new_value) return;
  if (new_value < 0 || new_value > 2) return;
  if (servo_info[index].sweep_mode) return;
  if (new_value != 0x01 && servo_info[index].test_mode) return;

  esp_tx_packet[0] = 0x8D;                                        // ヘッダー
  esp_tx_packet[1] = 0xD8;                                        // ヘッダー
  esp_tx_packet[2] = command_device;                              // 送信先デバイスID
  esp_tx_packet[3] = (uint8_t)DCM_PRP;                            // デバイス用コマンド
  esp_tx_packet[4] = (uint8_t)4;                                  // データ長
  esp_tx_packet[5] = command_id;                                  // データ：操舵基板用コマンドの種類
  esp_tx_packet[6] = servo_id;                                    // データ：対象のサーボID
  esp_tx_packet[7] = servo_info[index].torque_mode;               // データ：変更前トルクモード
  esp_tx_packet[8] = new_value;                                   // データ：変更後トルクモード
  esp_tx_packet[9] = checksum(esp_tx_packet, 9);                  // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 10);                            // 送信
  ESP_SERIAL.flush();                                             // 送信完了待ち

  confirm_wait = true;                        // 確認待ちを有効にする
  confirm_wait_time = millis();               // 確認待ちの開始時間
  confirm_command_id = command_id;            // 確認待ちのコマンド
  confirm_device = command_device;
  confirm_param[0] = (uint16_t)index;         // 確認待ちのコマンド処理用付加データ(サーボINDEX)
  confirm_param[1] = (uint16_t)new_value;     // 確認待ちのコマンド処理用付加データ(トルクモード)
}

void command_confirm_test_mode_set() {
  if (confirm_wait) return;
  if (command_data_len != 2) return;
  uint8_t servo_id = esp_rx_packet[5];
  uint8_t index = get_index(servo_id);
  if (index == 0xFF) return;
  uint8_t new_value = esp_rx_packet[6];
  if (servo_info[index].test_mode == new_value) return;
  if (new_value < 0 || new_value > 1) return;
  if (servo_info[index].sweep_mode) return;
  if (servo_info[index].torque_mode != 0x01) return;

  esp_tx_packet[0] = 0x8D;                                        // ヘッダー
  esp_tx_packet[1] = 0xD8;                                        // ヘッダー
  esp_tx_packet[2] = esp_rx_packet[2];                            // 送信先デバイスID
  esp_tx_packet[3] = (uint8_t)DCM_PRP;                            // デバイス用コマンド
  esp_tx_packet[4] = (uint8_t)4;                                  // データ長
  esp_tx_packet[5] = command_id;                                  // データ：操舵基板用コマンドの種類
  esp_tx_packet[6] = servo_id;                                    // データ：対象のサーボID
  esp_tx_packet[7] = servo_info[index].test_mode;                 // データ：変更前テストモード
  esp_tx_packet[8] = new_value;                                   // データ：変更後テストモード
  esp_tx_packet[9] = checksum(esp_tx_packet, 9);                  // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 10);                            // 送信
  ESP_SERIAL.flush();                                             // 送信完了待ち

  confirm_wait = true;                        // 確認待ちを有効にする
  confirm_wait_time = millis();               // 確認待ちの開始時間
  confirm_command_id = command_id;            // 確認待ちのコマンド
  confirm_device = command_device;
  confirm_param[0] = (uint16_t)index;         // 確認待ちのコマンド処理用付加データ(サーボINDEX)
  confirm_param[1] = (uint16_t)new_value;     // 確認待ちのコマンド処理用付加データ(テストモード)
}

void command_test_move() {
  int16_t tmp_value = ((((uint16_t)esp_rx_packet[7] << 8) & 0xFF00) | (esp_rx_packet[6] & 0x00FF));
  if (tmp_value > 1500 || tmp_value < -1500) return;
  uint8_t servo_id = esp_rx_packet[5];
  uint8_t index = get_index(servo_id);
  if (!servo_info[index].test_mode) return;
  servo_info[index].val = tmp_value;
  servo_move(servo_info[index].id, servo_info[index].val, 100);
}

void command_confirm_sweep() {
  if (confirm_wait) return;
  if (command_data_len != 2) return;
  uint8_t servo_id = esp_rx_packet[5];
  uint8_t index = get_index(servo_id);
  if (index == 0xFF) return;
  uint8_t new_value = 0x01;
  uint8_t sweep_speed = esp_rx_packet[6];
  if (servo_info[index].sweep_mode == new_value) return;
  if (sweep_speed < 1 || sweep_speed > 2) return;
  if (servo_info[index].torque_mode != 0x01) return;

  esp_tx_packet[0] = 0x8D;                                        // ヘッダー
  esp_tx_packet[1] = 0xD8;                                        // ヘッダー
  esp_tx_packet[2] = esp_rx_packet[2];                            // 送信先デバイスID
  esp_tx_packet[3] = (uint8_t)DCM_PRP;                            // デバイス用コマンド
  esp_tx_packet[4] = (uint8_t)3;                                  // データ長
  esp_tx_packet[5] = command_id;                                  // データ：操舵基板用コマンドの種類
  esp_tx_packet[6] = servo_id;                                    // データ：対象のサーボID
  esp_tx_packet[7] = sweep_speed;                                 // データ：試験動作の速さ
  esp_tx_packet[8] = checksum(esp_tx_packet, 8);                  // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 9);                             // 送信
  ESP_SERIAL.flush();                                             // 送信完了待ち

  confirm_wait = true;                        // 確認待ちを有効にする
  confirm_wait_time = millis();               // 確認待ちの開始時間
  confirm_command_id = command_id;            // 確認待ちのコマンド
  confirm_device = command_device;
  confirm_param[0] = (uint16_t)index;         // 確認待ちのコマンド処理用付加データ(サーボINDEX)
  confirm_param[1] = (uint16_t)new_value;     // 確認待ちのコマンド処理用付加データ(試験動作モード)
  confirm_param[2] = (uint16_t)sweep_speed;   // 確認待ちのコマンド処理用付加データ(試験動作の速さ)
}

void command_confirm() {
  if (!confirm_wait) return;
  if (confirm_device != command_device) return;
  if (confirm_command_id != esp_rx_packet[5]) return;
  confirm_wait = false;
  if (esp_rx_packet[6] != 0x01) return;
  switch (confirm_command_id) {
    case CMD_SET:
      DEBUG_SERIAL.print(F("Executed Value Set ["));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].id);
      DEBUG_SERIAL.print(F("] "));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].val_threshold[(uint8_t)confirm_param[1]]);
      DEBUG_SERIAL.print(F(" ==> "));
      DEBUG_SERIAL.println((int16_t)confirm_param[2]);
      servo_info[(uint8_t)confirm_param[0]].val_threshold[(uint8_t)confirm_param[1]] = (int16_t)confirm_param[2];
      EEPROM.put((uint8_t)confirm_param[0] * 6 + (uint8_t)confirm_param[1] * 2, (int16_t)confirm_param[2]);
      break;
    case CMD_RBT:
      DEBUG_SERIAL.print(F("Executed Reboot Servo ID: "));
      DEBUG_SERIAL.println(servo_info[(uint8_t)confirm_param[0]].id);
      servo_reboot(servo_info[(uint8_t)confirm_param[0]].id);
      break;
    case CMD_TQS:
      DEBUG_SERIAL.print(F("Executed Torque Percentage Set ["));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].id);
      DEBUG_SERIAL.print(F("] "));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].torque_percentage);
      DEBUG_SERIAL.print(F(" ==> "));
      DEBUG_SERIAL.println((uint8_t)confirm_param[1]);
      servo_torque_value(servo_info[(uint8_t)confirm_param[0]].id, (uint8_t)confirm_param[1]);
      break;
    case CMD_TMS:
      DEBUG_SERIAL.print(F("Executed Torque Mode Set ["));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].id);
      DEBUG_SERIAL.print(F("] "));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].torque_mode);
      DEBUG_SERIAL.print(F(" ==> "));
      DEBUG_SERIAL.println((uint8_t)confirm_param[1]);
      servo_info[(uint8_t)confirm_param[0]].torque_mode = (uint8_t)confirm_param[1];
      servo_set_torque_mode(servo_info[(uint8_t)confirm_param[0]].id, (uint8_t)confirm_param[1]);
      break;
    case CMD_TMD:
      DEBUG_SERIAL.print(F("Executed Test Mode Set ["));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].id);
      DEBUG_SERIAL.print(F("] "));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].test_mode);
      DEBUG_SERIAL.print(F(" ==> "));
      DEBUG_SERIAL.println((uint8_t)confirm_param[1]);
      servo_info[(uint8_t)confirm_param[0]].test_mode = (uint8_t)confirm_param[1];
      break;
    case CMD_SWP:
      DEBUG_SERIAL.print(F("Executed Sweep Mode Toggle ["));
      DEBUG_SERIAL.print(servo_info[(uint8_t)confirm_param[0]].id);
      DEBUG_SERIAL.print(F("] Speed : "));
      DEBUG_SERIAL.println((uint8_t)confirm_param[2]);
      servo_info[(uint8_t)confirm_param[0]].sweep_mode = (bool)confirm_param[1];
      servo_info[(uint8_t)confirm_param[0]].sweep_speed = (uint8_t)confirm_param[2] - 1;
      servo_info[(uint8_t)confirm_param[0]].sweep_last_step_time = millis();
      servo_info[(uint8_t)confirm_param[0]].sweep_begin_time = millis();
  }
}

void command_send_all() {
  // ----------初期値の送信--------------------------------------------------------------------- //
  esp_tx_packet[0]  = 0x8D;                                       // ヘッダー
  esp_tx_packet[1]  = 0xD8;                                       // ヘッダー
  esp_tx_packet[2]  = 0xFA;                                       // 送信先デバイスID
  esp_tx_packet[3]  = (uint8_t)DCM_DSP;                           // デバイス用コマンド
  esp_tx_packet[4]  = (uint8_t)32;                                // データ長
  esp_tx_packet[5]  = SET_RUD_MIN;                                // データ：ラダー最小
  esp_tx_packet[6]  = lowByte (servo_info[0].val_threshold[MIN]); // データ：ラダー最小
  esp_tx_packet[7]  = highByte(servo_info[0].val_threshold[MIN]); // データ：ラダー最小
  esp_tx_packet[8]  = SET_RUD_NEU;                                // データ：ラダーニュ
  esp_tx_packet[9]  = lowByte (servo_info[0].val_threshold[NEU]); // データ：ラダーニュ
  esp_tx_packet[10] = highByte(servo_info[0].val_threshold[NEU]); // データ：ラダーニュ
  esp_tx_packet[11] = SET_RUD_MAX;                                // データ：ラダー最大
  esp_tx_packet[12] = lowByte (servo_info[0].val_threshold[MAX]); // データ：ラダー最大
  esp_tx_packet[13] = highByte(servo_info[0].val_threshold[MAX]); // データ：ラダー最大
  esp_tx_packet[14] = SET_ELE_MIN;                                // データ：エレベータ最小
  esp_tx_packet[15] = lowByte (servo_info[1].val_threshold[MIN]); // データ：エレベータ最小
  esp_tx_packet[16] = highByte(servo_info[1].val_threshold[MIN]); // データ：エレベータ最小
  esp_tx_packet[17] = SET_ELE_NEU;                                // データ：エレベータニュ
  esp_tx_packet[18] = lowByte (servo_info[1].val_threshold[NEU]); // データ：エレベータニュ
  esp_tx_packet[19] = highByte(servo_info[1].val_threshold[NEU]); // データ：エレベータニュ
  esp_tx_packet[20] = SET_ELE_MAX;                                // データ：エレベータ最大
  esp_tx_packet[21] = lowByte (servo_info[1].val_threshold[MAX]); // データ：エレベータ最大
  esp_tx_packet[22] = highByte(servo_info[1].val_threshold[MAX]); // データ：エレベータ最大
  esp_tx_packet[23] = 0x08;                                       // データ：ラダーテストモード
  esp_tx_packet[24] = (uint8_t)servo_info[0].test_mode;           // データ：ラダーテストモード
  esp_tx_packet[25] = 0x09;                                       // データ：エレベータテストモード
  esp_tx_packet[26] = (uint8_t)servo_info[1].test_mode;           // データ：エレベータテストモード
  esp_tx_packet[27] = 0x0A;                                       // データ：ラダートルクモード
  esp_tx_packet[28] = (uint8_t)servo_info[0].torque_mode;         // データ：ラダートルクモード
  esp_tx_packet[29] = 0x0B;                                       // データ：エレベータトルクモード
  esp_tx_packet[30] = (uint8_t)servo_info[1].torque_mode;         // データ：エレベータトルクモード
  esp_tx_packet[31] = 0x0C;                                       // データ：ラダースイープモード
  esp_tx_packet[32] = (uint8_t)servo_info[0].sweep_mode;          // データ：ラダースイープモード
  esp_tx_packet[33] = (uint8_t)servo_info[0].sweep_speed;         // データ：ラダースイープモード
  esp_tx_packet[34] = 0x0D;                                       // データ：エレベータスイープモード
  esp_tx_packet[35] = (uint8_t)servo_info[1].sweep_mode;          // データ：エレベータスイープモード
  esp_tx_packet[36] = (uint8_t)servo_info[1].sweep_speed;         // データ：エレベータスイープモード
  esp_tx_packet[37] = checksum(esp_tx_packet, 37);                // チェックサム
  ESP_SERIAL.write(esp_tx_packet, 38);                            // 送信
  ESP_SERIAL.flush();                                             // 送信完了待ち
  // ------------------------------------------------------------------------------------------ //
}
