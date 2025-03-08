#include "gun.hpp"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* Internal comms */
void processAllIncomingPackets();
void processIncomingPacket();
void retransmitLastPacket();
void updateDisplay();

HandshakeStatus handshakeStatus = STAT_NONE;
// Zero-initialise lastSentPacket
BlePacket lastSentPacket = {};
unsigned long lastSentPacketTime = 0;
uint16_t receiverSeqNum = INITIAL_SEQ_NUM;
uint16_t senderSeqNum = INITIAL_SEQ_NUM;
bool isWaitingForAck = false;
uint8_t numRetries = 0;
uint8_t numInvalidPacketsReceived = 0;
// Used to maintain (RETRANSMIT_DELAY) ms period of retransmissions
unsigned long lastRetransmitTime = 0;
unsigned long lastReadPacketTime = 0;
// Used to maintain keep alive interval and transmit keep alive packets periodically
unsigned long lastKeepAliveTime = 0;
// Used to maintain retransmit interval and retransmit upon timeout
unsigned long lastRawDataSentTime = 0;

/* IR Transmitter */
/* Gun state */
uint8_t bulletCount = GUN_MAGAZINE_SIZE;
bool isReloading = false;
bool isFiring = false;
bool isFired = false;
unsigned long lastReloadTime = 0;
bool checkSerialReload = false;

void setup() {
  Serial.begin(BAUDRATE);

  // Initialize the OLED display
  /*if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // If display initialization fails, flash the LED
    pinMode(LED_BUILTIN, OUTPUT);
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }
  
  // Show initial display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Gun with BLE");
  display.println("Initializing...");
  display.display();
  delay(1000);

  // Setup gun-specific logic
  gunSetup(); */

  // Set up internal comms 
  setupBle();
  
  // Show ready display
  // updateDisplay();
}

void loop() {
  if (!hasHandshake()) {
    handshakeStatus = doHandshake();
  }
  unsigned long currentTime = millis();
  
  // Check for serial reload command
  /*if(Serial.available() > 0) {
    char command = Serial.read();
    if(command == 'R' || command == 'r') {
      if(bulletCount == 0) {
        isReloading = true;
        lastReloadTime = millis();
        Serial.println("Reloading...");
        updateDisplay();
      } else {
        Serial.println("You still have ammo. Reload only when empty.");
      }
    }
  }
  
  // Handle reloading
  if(isReloading) {
    if(millis() - lastReloadTime >= RELOAD_TIME) {
      bulletCount = GUN_MAGAZINE_SIZE;
      isReloading = false;
      Serial.println("Reload complete. Ready to fire!");
      reload(); // For sound effect
      updateDisplay();
    }
  }*/
  
  // Retransmit last sent packet on timeout
  if (isWaitingForAck && (currentTime - lastRetransmitTime) >= RETRANSMIT_DELAY
      && (currentTime - lastRawDataSentTime) >= BLE_TIMEOUT) { 
    // Maintain at least RETRANSMIT_DELAY millisecs in between consecutive retransmits
    if (numRetries < MAX_RETRANSMITS) {
      // Retransmit only if numRetries less than max limit
      retransmitLastPacket();
      numRetries += 1;
    } else {
      // Clear serial input/output buffers to restart transmission from clean state
      clearSerialInputBuffer();
      Serial.flush();
      // Laptop might have disconnected, re-enter handshake
      handshakeStatus = STAT_NONE;
      numRetries = 0;
    }
  } else if (!isWaitingForAck && (currentTime - lastSentPacketTime) >= TRANSMIT_DELAY
      && hasRawData()) { // Send raw data packets(if any)
    // Only send new packet if previous packet has been ACK-ed and there's new sensor data to send
    //   but maintain TRANSMIT_DELAY millisecs in between sensor data packet transmissions
    // Read sensor data and generate a BlePacket encapsulating that data
    BlePacket mRawDataPacket = createRawDataPacket();
    // Send updated sensor data to laptop
    sendPacket(mRawDataPacket);
    // Update last sent packet to latest sensor data packet
    lastSentPacket = mRawDataPacket;
    // Update last raw data packet sent time to maintain transmit delay
    lastSentPacketTime = millis();
    // Update last raw data packet sent time to track timeout
    lastRawDataSentTime = lastSentPacketTime;
    isWaitingForAck = true;
  } else if (!isWaitingForAck && 
      (currentTime - lastRawDataSentTime) >= KEEP_ALIVE_INTERVAL &&
      (currentTime - lastKeepAliveTime) >= KEEP_ALIVE_INTERVAL &&
      (currentTime - lastSentPacketTime) >= TRANSMIT_DELAY) {
    // Keep alive interval has passed since the last sensor/keep alive packet transmission but no sensor data is available to transmit
    // -> Send keep alive packet periodically when no sensor packet is transmitted so laptop knows Beetle is responding
    BlePacket keepAlivePacket = createKeepAlivePacket(senderSeqNum);
    sendPacket(keepAlivePacket);
    // Update last raw data packet sent time to maintain transmit delay
    lastSentPacketTime = millis();
    // Update lastSentPacketTime to support periodic keep alive packet transmission
    lastKeepAliveTime = lastSentPacketTime;
    // Don't require ACK for keep alive packets
  }
  // Always process incoming packets regardless of what sender logic does
  if ((millis() - lastReadPacketTime) >= READ_PACKET_DELAY) { // Handle incoming packets
    // Received some bytes from laptop, process them wwhile maintaining at least READ_PACKET_DELAY
    //   in between reading of 2 consecutive packets 
    processAllIncomingPackets();
  }
}

/*void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Player ID
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Player: ");
  display.println(PLAYER_ID);
  
  // Ammo count
  display.setTextSize(2);
  display.setCursor(0,16);
  display.print("Ammo:");
  display.println(bulletCount);
  
  // BLE Status
  display.setTextSize(1);
  display.setCursor(0,40);
  display.print("BLE: ");
  display.println(hasHandshake() ? "Connected" : "Disconnected");
  
  // Status
  display.setCursor(0,48);
  if(isReloading) {
    display.println("RELOADING...");
    display.println("Please wait");
  } else if(bulletCount == 0) {
    display.println("OUT OF AMMO!");
    display.println("Press 'R' to reload");
  } else {
    display.println("Ready to fire!");
  }
  
  display.display();
} */

HandshakeStatus doHandshake() {
  unsigned long mLastPacketSentTime = millis();
  BlePacket mLastSentPacket;
  byte mSeqNum = INITIAL_SEQ_NUM;
  bool mIsWaitingForAck = false;
  uint8_t mNumInvalidPacketsReceived = 0;
  
  // Update display to show handshake in progress
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("BLE Handshake");
  display.println("in progress...");
  display.display();
  
  while (handshakeStatus != STAT_SYN) {
    // No packet sent yet or not yet timed out or last packet sent is invalid
    switch (handshakeStatus) {
      case HandshakeStatus::STAT_NONE:
        {
          // Whether at least 1 HELLO packet has been received
          bool mHasHello = false;
          int mNumBytesAvailable = Serial.available();
          // Process all HELLO packets received so far at once
          // -> This should minimise cycling between NONE and HELLO handshake states
          //      when laptop transmits multiple HELLO packets
          while (mNumBytesAvailable >= PACKET_SIZE) {
            // At least 1 complete packet in serial input buffer, read it
            BlePacket mReceivedPacket = readPacket();
            // Decrease the number of bytes left to read from serial input
            mNumBytesAvailable -= PACKET_SIZE;
            if (!isPacketValid(mReceivedPacket)) {
              mNumInvalidPacketsReceived += 1;
              if (mNumInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
                // Clear serial input buffer as an attempt to recover from unusual amount of packet corruption
                clearSerialInputBuffer();
                mNumInvalidPacketsReceived = 0;
                mSeqNum = INITIAL_SEQ_NUM;
              }
              BlePacket mNackPacket;
              // Create NACK packet indicating that packet received is invalid or has wrong seq num
              createNackPacket(mNackPacket, mSeqNum, "Invalid/seqNum");
              // Notify the laptop by transmitting the NACK packet
              sendPacket(mNackPacket);
            } else if (getPacketTypeOf(mReceivedPacket) == PacketType::HELLO) {
              // Reset invalid packet count when a valid packet is received
              mNumInvalidPacketsReceived = 0;
              if (mReceivedPacket.seqNum != mSeqNum) {
                // Drop packet if seq num does not match
                continue;
              }
              // Indicate that at least 1 valid HELLO packet has been received
              mHasHello = true;
            }
            // Drop all non-HELLO packets received
            // Continue processing the other remaining HELLO packets in serial input
          }
          if (mHasHello) {
            // Progress to the next handshake stage as long as at least 1 HELLO packet is received
            handshakeStatus = STAT_HELLO;
          }
          // Continue waiting for HELLO packet from laptop in the next iteration of while() if none is received so far
          break;
        }
      case HandshakeStatus::STAT_HELLO:
        {
          unsigned long mTransmitPeriod = millis() - mLastPacketSentTime;
          if (mTransmitPeriod < TRANSMIT_DELAY) {
            // Maintain at least (TRANSMIT_DELAY) ms delay between transmissions to avoid overwhelming the Beetle
            delay(TRANSMIT_DELAY - mTransmitPeriod);
          }
          BlePacket ackPacket;
          createHandshakeAckPacket(ackPacket, mSeqNum);  
          sendPacket(ackPacket);
          mLastSentPacket = ackPacket;
          mLastPacketSentTime = millis();
          mSeqNum += 1;
          handshakeStatus = HandshakeStatus::STAT_ACK;
          mIsWaitingForAck = true;
          break;
        }
      case HandshakeStatus::STAT_ACK:
        {
          unsigned long mCurrentTime = millis();
          if (mIsWaitingForAck && (mCurrentTime - mLastPacketSentTime) >= BLE_TIMEOUT) {
            handshakeStatus = STAT_HELLO;
            mSeqNum = INITIAL_SEQ_NUM;
            // Invalidate mLastSentPacket to prevent retransmission in any edge case
            mLastSentPacket.metadata = PLACEHOLDER_METADATA;
            // Reset mIsWaitingForAck as well to fully reset transmission state
            mIsWaitingForAck = false;
            continue;
          }
          if (Serial.available() < PACKET_SIZE) {
            // Skip this iteration since we haven't received a full 20-byte packet
            continue;
          }
          BlePacket receivedPacket = readPacket();
          if (!isPacketValid(receivedPacket)) {
            mNumInvalidPacketsReceived += 1;
            if (mNumInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
              // Clear serial input buffer as an attempt to recover from unusual amount of packet corruption
              clearSerialInputBuffer();
              mNumInvalidPacketsReceived = 0;
              mSeqNum = INITIAL_SEQ_NUM;
            }
            BlePacket nackPacket;
            createNackPacket(nackPacket, mSeqNum, "Corrupted");
            sendPacket(nackPacket);
          } else { // Packet received is valid
            // Reset invalid packet count when a valid packet is received
            mNumInvalidPacketsReceived = 0;
            if (getPacketTypeOf(receivedPacket) == PacketType::ACK) {
              if (receivedPacket.seqNum > mSeqNum) {
                BlePacket nackPacket;
                // Use existing seqNum for NACK packet to indicate current packet is not received
                createNackPacket(nackPacket, mSeqNum, "Over seqNum");
                sendPacket(nackPacket);
                continue;
              }
              if (receivedPacket.seqNum < mSeqNum) {
                // Likely a delayed ACK packet, drop it
                continue;
              }
              handshakeStatus = HandshakeStatus::STAT_SYN;
              mSeqNum += 1;
              mIsWaitingForAck = false;
              // Drop duplicate SYN+ACK packets received from laptop so transmission logic 
              //   in loop() doesn't process leftover SYN+ACK packets from handshake
              clearSerialInputBuffer();
              // Break switch block since handshake process is complete
              break;
            } else if (getPacketTypeOf(receivedPacket) == PacketType::HELLO &&
                (mCurrentTime - mLastPacketSentTime) >= BLE_TIMEOUT) {
              // Return to HELLO state only if we sent ACK a sufficiently long time ago(handshake has restarted or timeout occurred)
              handshakeStatus = STAT_HELLO;
              mSeqNum = INITIAL_SEQ_NUM;
              // Drop the HELLO packet if we just sent an ACK to avoid cycling between HELLO and ACK states
            } else if (getPacketTypeOf(receivedPacket) == PacketType::NACK &&
                receivedPacket.seqNum == (mSeqNum - 1) && isPacketValid(mLastSentPacket)) {
              sendPacket(mLastSentPacket);
              mIsWaitingForAck = true;
            }
          }
        }
    }
  }
  
  // Update display to show handshake completed
  // updateDisplay();
  
  return handshakeStatus;
}

/**
 * Setup for the BLE internal communications-related logic and variables
 */
void setupBle() {
  // Clear the serial output buffer
  Serial.flush();

  // Clear the serial input buffer
  clearSerialInputBuffer();

  /* Initialise lastSentPacket with invalid metadata
    to ensure it's detected as corrupted if ever
    sent without assigning actual (valid) packet */
  lastSentPacket.metadata = PLACEHOLDER_METADATA;
}

BlePacket createGunPacket(bool mIsFired) {
  BlePacket gunPacket = {};
  byte packetData[PACKET_DATA_SIZE] = {};
  getPacketDataFor(mIsFired, packetData);
  createPacket(gunPacket, PacketType::IR_TRANS, senderSeqNum, packetData);
  return gunPacket;
}

void createHandshakeAckPacket(BlePacket &ackPacket, uint16_t givenSeqNum) {
  byte packetData[PACKET_DATA_SIZE] = {};
  uint16_t seqNumToSyn = senderSeqNum;
  if (isWaitingForAck && isPacketValid(lastSentPacket)) {
    seqNumToSyn = lastSentPacket.seqNum;
  }
  packetData[0] = (byte) seqNumToSyn;
  packetData[1] = (byte) (seqNumToSyn >> BITS_PER_BYTE);
  createPacket(ackPacket, PacketType::ACK, givenSeqNum, packetData);
}

BlePacket createRawDataPacket() {
  return createGunPacket(isFired);
}

uint8_t getBulletCountFrom(const BlePacket &gamePacket) {
  return gamePacket.data[0];
}

/*
 * Update internal variables based on the new game state received
 */
void handleGamePacket(const BlePacket &gamePacket) {
  uint8_t newBulletCount = getBulletCountFrom(gamePacket);
  if (bulletCount == 0 && newBulletCount > bulletCount) {
    bulletCount = newBulletCount;
   //  reload();
  } else if (newBulletCount >= 0 && newBulletCount <= GUN_MAGAZINE_SIZE) {
    bulletCount = newBulletCount;
  }
 //  updateDisplay();
}

bool hasHandshake() {
  return handshakeStatus == HandshakeStatus::STAT_SYN;
}

/**
 * Checks whether the connected sensors of this Beetle has raw data to send to laptop.
 */
bool hasRawData() {
  // isFired must be false unless fireGun() sets it to true(when gun has ammo and trigger is pressed)
  isFired = false;
  // Check whether gun trigger is pressed
  /* if (getIsFired()) {
    // Trigger gunfire-related game logic
    fireGun();
    // Indicate that there's sensor data to send to laptop
    return true;
  } */
  // Gun trigger isn't pressed, no sensor data to send to laptop
  return false;
}

void processGivenPacket(const BlePacket &packet) {
  char givenPacketType = getPacketTypeOf(packet);
  switch (givenPacketType) {
    case PacketType::HELLO:
      handshakeStatus = STAT_HELLO;
      break;
    case PacketType::ACK:
      if (!isWaitingForAck) {
        // Not expecting an ACK, so this ACK is likely delayed and we drop it
        return;
      }
      // Have been waiting for an ACK and we received it
      if (packet.seqNum > senderSeqNum) {
        BlePacket nackPacket;
        createNackPacket(nackPacket, senderSeqNum, "seqNum too high");
        // Inform laptop about seq num mismatch by sending a NACK with our current seq num
        sendPacket(nackPacket);
        return;
      } else if (packet.seqNum < senderSeqNum) {
        // If packet.seqNum < senderSeqNum, it's (likely) a delayed ACK packet and we ignore it
        return;
      }
      // Valid ACK received, so stop waiting for incoming ACK
      isWaitingForAck = false;
      // Increment senderSeqNum upon every ACK
      senderSeqNum += 1;
      isFired = false;
      numRetries = 0;
      break;
    case PacketType::NACK:
      if (!isWaitingForAck) {
        // Didn't send a packet, there's nothing to NACK
        // Likely a delayed packet so we just drop it
        return;
      }
      // Sent a packet but received a NACK, attempt to retransmit
      if (packet.seqNum == senderSeqNum) {
        if (isPacketValid(lastSentPacket) && getPacketTypeOf(lastSentPacket) != PacketType::NACK) {
          // Only retransmit if packet is valid
          sendPacket(lastSentPacket);
        }
        // No else{}: Don't retransmit a corrupted packet or another NACK packet
      }
      // If packet.seqNum < senderSeqNum, NACK packet is likely delayed and we drop it
      break;
    case GAME_STAT:
      {
        uint16_t seqNumToAck = receiverSeqNum;
        bool shouldHandlePacket = false;
        if (receiverSeqNum == packet.seqNum) {
          shouldHandlePacket = true;
          receiverSeqNum += 1;
        } else if (receiverSeqNum > packet.seqNum) {
          /* If receiverSeqNum > packet.seqNum, I incremented receiverSeqNum after sending ACK 
              but sender did not receive ACK and thus retransmitted packet
            */
          // ACK the packet but don't decrement my sequence number
          seqNumToAck = packet.seqNum;
          // Don't process the same packet again
        }
        BlePacket ackPacket;
        createAckPacket(ackPacket, seqNumToAck);
        sendPacket(ackPacket);
        if (numInvalidPacketsReceived > 0) {
          numInvalidPacketsReceived = 0;
        }
        if (shouldHandlePacket) {
          // Process the packet to handle specific game logic(e.g. updating Beetle's internal game state)
          handleGamePacket(packet);
        }
        break;
      }
    case INVALID_PACKET_ID:
    default:
      // All other packet types are unsupported, inform sender that packet is rejected
      BlePacket nackPacket;
      createNackPacket(nackPacket, receiverSeqNum, "Invalid type");
      sendPacket(nackPacket);
  } // switch (receivedPacketType)
}

void processAllIncomingPackets() {
  int numBytesAvailable = Serial.available();
  // Read as many packets as are available in the serial input buffer at the moment processAllIncomingPackets() is called
  while (numBytesAvailable >= PACKET_SIZE) {
    // Complete packet received, read packet bytes from receive buffer as BlePacket
    BlePacket receivedPacket = readPacket();
    // Update lastReadPackeTime to maintain read packet delay in loop()
    lastReadPacketTime = millis();
    // Read PACKET_SIZE number of bytes, decrease number of bytes available accordingly
    numBytesAvailable -= PACKET_SIZE;
    if (!isPacketValid(receivedPacket)) {
      numInvalidPacketsReceived += 1;
      if (numInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
        // Clear serial input buffer as an attempt to recover from unusual amount of packet corruption
        clearSerialInputBuffer();
        numInvalidPacketsReceived = 0;
        // Cleared all received packets, nothing left to parse
        numBytesAvailable = 0;
      }
      BlePacket nackPacket;
      createNackPacket(nackPacket, receiverSeqNum, "Corrupted");
      // Received invalid packet, request retransmit with NACK
      sendPacket(nackPacket);
      continue;
    } else {
      // Valid packet received, reset invalid packet count
      numInvalidPacketsReceived = 0;
      // Process valid packet
      processGivenPacket(receivedPacket);
    }
  }
}

void processIncomingPacket() {
  if (Serial.available() < PACKET_DATA_SIZE) {
    // Don't read from serial input buffer unless 1 complete packet is received
    return;
  }
  // Complete 20-byte packet received, read 20 bytes from receive buffer as packet
  BlePacket receivedPacket = readPacket();
  // Update lastReadPackeTime to maintain read packet delay in loop()
  lastReadPacketTime = millis();
  if (!isPacketValid(receivedPacket)) {
    numInvalidPacketsReceived += 1;
    if (numInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
      clearSerialInputBuffer();
      delay(BLE_TIMEOUT);
      numInvalidPacketsReceived = 0;
      return;
    }
    BlePacket nackPacket;
    createNackPacket(nackPacket, receiverSeqNum, "Corrupted");
    // Received invalid packet, request retransmit with NACK
    sendPacket(nackPacket);
  } else {
    if (numInvalidPacketsReceived > 0) {
      numInvalidPacketsReceived = 0;
    }
    processGivenPacket(receivedPacket);
  }
}

int readIntoRecvBuffer(MyQueue<byte> &mRecvBuffer) {
  int numOfBytesRead = 0;
  while (Serial.available() > 0) {
    byte nextByte = (byte) Serial.read();
    if (isHeadByte(nextByte) || !mRecvBuffer.isEmpty()) {
      mRecvBuffer.push_back(nextByte);
      numOfBytesRead += 1;
    }
  }
  return numOfBytesRead;
}

void retransmitLastPacket() {
  if (isPacketValid(lastSentPacket)) {
    sendPacket(lastSentPacket);
    // Update last retransmit time to maintain retransmit delay
    lastRetransmitTime = millis();
    // Update last sent packet time to maintain transmit delay independently from retransmit delay
    lastSentPacketTime = lastRetransmitTime;
    // Update sent time and wait for ACK again
    lastRawDataSentTime = lastRetransmitTime;
  } else {
    isWaitingForAck = false;
  }
}

BlePacket sendGunPacket(bool mIsFired) {
  BlePacket gunPacket = {};
  byte packetData[PACKET_DATA_SIZE] = {};
  getPacketDataFor(mIsFired, packetData);
  createPacket(gunPacket, PacketType::IR_TRANS, senderSeqNum, packetData);
  sendPacket(gunPacket);
  return gunPacket;
}

/* IR Transmitter with OLED Display */
/*void gunSetup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(IR_TRN_PIN, OUTPUT);
  IrSender.begin(IR_TRN_PIN);
  
  // Print startup message on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Gun initialized");
  display.println("Player ID: " + String(PLAYER_ID));
  display.println("Bullets: " + String(bulletCount));
  display.display();
  delay(1000);
}

void send38kHzBurst(int durationMs) {
  unsigned long endTime = millis() + durationMs;
    
  while(millis() < endTime) {
    digitalWrite(IR_TRN_PIN, HIGH);
    delayMicroseconds(13);
    digitalWrite(IR_TRN_PIN, LOW);
    delayMicroseconds(13);
  }
}

void sendShot() {
  // Header burst (50ms)
  send38kHzBurst(50);
  delay(25);
  
  // Player ID burst (10ms for Player 1)
  send38kHzBurst(10);
  delay(25);
  
  // End marker (20ms)
  send38kHzBurst(20);
}

byte getButtonState() {
  byte newButtonState = (byte) digitalRead(BUTTON_PIN);
  return newButtonState;
}

bool getIsFired() {
  byte befButtonState = getButtonState();
  delay(BUTTON_DEBOUNCE_DELAY);
  byte aftButtonState = getButtonState();
  return befButtonState == LOW && aftButtonState == HIGH && !isReloading;
}
 */
/*
 * Attempts to trigger a gun fire.
 */

 /*
void fireGun() {
  if (bulletCount > 0 && !isReloading) {
    // Send IR signal
    sendShot();
    bulletCount--;
    // Gun trigger is pressed and gun has ammo, set isFired to true
    isFired = true;
    
    // Show firing animation on OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("FIRING!");
    display.display();
    
    TimerFreeTone(BUZZER_PIN, GUNFIRE_BUZZER_FREQ, GUNFIRE_BUZZER_DURATION);
    delay(100);
    updateDisplay();
  } else if (bulletCount == 0) {
    TimerFreeTone(BUZZER_PIN, GUN_MAGAZINE_EMPTY_BUZZER_FREQ, GUN_MAGAZINE_EMPTY_BUZZER_DURATION);
    updateDisplay();
  }
}

void reload() {
  TimerFreeTone(BUZZER_PIN, RELOAD_BUZZER_FREQ, RELOAD_BUZZER_DURATION);
  
  // Show reload animation on OLED
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,16);
  display.println("RELOADED!");
  display.display();
  delay(500);
  
  updateDisplay();
} */
