/*
 * マルチSerialを利用しているため
 * ARDUINO MEGA のみで動作可能
 */

#define DEBUG_SERIAL Serial

// tony test

// 各サーボのID
#define RUD_ID 1
#define ELE_ID 2

#include "esp_comm.h" // EEPROM.h と futaba_servo.h がインクルードされる
#include "sensory.h"

// 操縦桿アナログ信号ピン
#define RUD_CONT_PIN A0
#define ELE_CONT_PIN A1

// 操縦桿電圧値範囲
// RUD_CONT_L_MIN ~ RUD_CONT_C_MIN: ラダー右
// RUD_CONT_C_MIN ~ RUD_CONT_C_MAX: ラダーニュートラル
// RUD_CONT_C_MAX ~ RUD_CONT_H_MAX: ラダー左
// ELE_CONT_L_MIN ~ ELE_CONT_C_MIN: エレベータ下
// ELE_CONT_C_MIN ~ ELE_CONT_C_MAX: エレベータニュートラル
// ELE_CONT_C_MAX ~ ELE_CONT_H_MAX: エレベータ上
#define RUD_CONT_L_MIN 0
#define RUD_CONT_C_MIN 440
#define RUD_CONT_C_MAX 451
#define RUD_CONT_H_MAX 1023

#define ELE_CONT_L_MIN 0
#define ELE_CONT_C_MIN 475
#define ELE_CONT_C_MAX 485
#define ELE_CONT_H_MAX 1023

// 各サーボ角の最小値、最大値、ニュートラル   単位は0.1°
int16_t rud_min = -500;     //ラダー最小角          B001
int16_t rud_neu = 0;        //ラダーニュートラル角   B010
int16_t rud_max = 500;      //ラダー最大角          B011
int16_t ele_min = -500;     //エレベータ最大角       B101
int16_t ele_neu = 0;        //エレベータ最大角       B110
int16_t ele_max = 500;      //エレベータ最大角       B111

void setup(){
  DEBUG_SERIAL.begin(9600);
  EEPROM.get(0x00, rud_min);
  EEPROM.get(0x02, rud_neu);
  EEPROM.get(0x04, rud_max);
  EEPROM.get(0x06, ele_min);
  EEPROM.get(0x08, ele_neu);
  EEPROM.get(0x0A, ele_max);
  servo_add(RUD_ID, "RUDDER  ", RUD_CONT_PIN, RUD_CONT_L_MIN, RUD_CONT_C_MIN, RUD_CONT_C_MAX, RUD_CONT_H_MAX, rud_min, rud_neu, rud_max);
  servo_add(ELE_ID, "ELEVATOR", ELE_CONT_PIN, ELE_CONT_L_MIN, ELE_CONT_C_MIN, ELE_CONT_C_MAX, ELE_CONT_H_MAX, ele_min, ele_neu, ele_max);
  servo_setup();
  sensory_setup();
  command_setup();
  command_send_all();
}

void loop(){
  servo_control_all();
  servo_pack_info(RUD_ID, sensory_packet(RUD_ID));
  servo_control_all();
  servo_pack_info(ELE_ID, sensory_packet(ELE_ID));
  servo_control_all();
  servo_maintain();
  servo_control_all();
  sensory_transmit();
  servo_control_all();
  print_debug_info();
  servo_control_all();
  command_handle();
}
