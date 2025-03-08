#include <TimerFreeTone.h>
#include <IRremote.hpp>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "packet.hpp"

#define BUZZER_PIN 4
#define GUNSHOT_HIT_BUZZER_FREQ 2000
#define GUNSHOT_HIT_BUZZER_DURATION 200
#define INVALID_HP 255
#define PLAYER_FULL_HP 100
#define RESPAWN_BUZZER_FREQ 1200
#define RESPAWN_BUZZER_DURATION 300
#define IR_COMMAND 0x34
#define IR_RECV_PIN 5
#define LED_PIN LED_BUILTIN
#define LED_STRIP_PIN 3
#define NUM_HP_LED 10
// Player-specific macros
#define PLAYER_ID 1
#define IR_ADDRESS_PLAYER_1 0x1234
#define IR_ADDRESS_PLAYER_2 0xABCD
#define GET_OUR_IR_ADDRESS() ((PLAYER_ID == 1) ? IR_ADDRESS_PLAYER_1 : IR_ADDRESS_PLAYER_2)

void irReceiverSetup();
void checkHealth();
void doDamage();
void doGunshotHit();
void doRespawn();
bool getIsShotFromIr();
void giveLife();
void updateHpLed(uint8_t givenPlayerHp);

void createVestPacketData(bool mIsHit, byte packetData[PACKET_DATA_SIZE]) {
  packetData[0] = mIsHit ? 1 : 0;
}
