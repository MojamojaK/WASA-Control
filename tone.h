#include <Tone.h>

#define BUZZER1 11
#define BUZZER2 12

Tone tone1;
Tone tone2;

void initTone() {
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);
  tone1.begin(BUZZER1);
  tone2.begin(BUZZER2);
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
