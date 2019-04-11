#include <Tone.h>

#define BUZZER1 11
#define BUZZER2 12

Tone tone1;
Tone tone2;

boolean playing;

#define TONES 4
uint8_t tone_state = 0;
uint32_t tone_time = 0;
uint16_t tone_freq[TONES] = {1047, 0, 1047, 0};
uint16_t tone_dura[TONES + 1] = {0, 250, 250, 250, 250};

void initTone() {
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);
  tone1.begin(BUZZER1);
  tone2.begin(BUZZER2);
  playing = false;
}

void playAlert() {
  playing = true;
}

void handleTone () {
  if (playing) {
    if (millis() - tone_time > tone_dura[tone_state]) {
      if (tone_state == TONES) {
        tone_state = 0;
        playing = false;
      } else {
        if (tone_freq[tone_state] != 0) {
          tone1.play(tone_freq[tone_state]);
        } else {
          tone1.stop();
        }
        tone_state++;
        tone_time = millis();
      }
    }
  }
}

void tonePlayLevelUp() {
  tone1.play(1397);
  delay(10);
  tone2.play(1047);
  delay(40);
  tone1.stop();
  delay(10);
  tone2.stop();
  delay(40);
  tone1.play(1397);
  delay(10);
  tone2.play(988);
  delay(40);
  tone1.stop();
  delay(10);
  tone2.stop();
  delay(40);
  tone1.play(1397);
  delay(10);
  tone2.play(932);
  delay(40);
  tone1.stop();
  delay(10);
  tone2.stop();
  delay(40);
  tone1.play(1397);
  delay(10);
  tone2.play(880);
  delay(40);
  tone1.stop();
  delay(10);
  tone2.stop();
  delay(140);
  tone1.play(1245);
  delay(10);
  tone2.play(784);
  delay(40);
  tone1.stop();
  delay(10);
  tone2.stop();
  delay(140);
  tone1.play(1568);
  delay(10);
  tone2.play( 988);
  delay(40);
  tone1.stop();
  delay(10);
  tone2.stop();
  delay(190);
  tone1.play(1397);
  delay(10);
  tone2.play( 880);
  delay(490);
  tone1.stop();
  delay(10);
  tone2.stop();
}
