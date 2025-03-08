


#include "vest.hpp"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/* Internal comms */
void handleGamePacket(const BlePacket &gamePacket);
void processAllIncomingPackets();
void processIncomingPacket();
void retransmitLastPacket();

// Define display
Adafruit_SH1106G display(128, 64, &Wire, 4);

// Define pins for new hardware
#define IR_PIN 5
#define BUZZ_PIN 2
#define DAMAGE_PER_HIT 5
#define HIT_COOLDOWN 800

HandshakeStatus handshakeStatus = STAT_NONE;
BlePacket lastSentPacket = {};
unsigned long lastSentPacketTime = 0;
uint16_t receiverSeqNum = INITIAL_SEQ_NUM;
uint16_t senderSeqNum = INITIAL_SEQ_NUM;
bool isWaitingForAck = false;
uint8_t numRetries = 0;
uint8_t numInvalidPacketsReceived = 0;
unsigned long lastRetransmitTime = 0;
unsigned long lastReadPacketTime = 0;
unsigned long lastKeepAliveTime = 0;
unsigned long lastRawDataSentTime = 0;

// Vest game state
bool isHit = false; // NEW
uint8_t playerHp = 100; // NEW
byte hitCount = 0;
byte lastPlayerID = 0;
byte rejected = 0;
unsigned long lastHitTime = 0;

// Signal processing variables from second code
unsigned long signalStart = 0;
bool signalActive = false;
byte burstCount = 0;
unsigned long burstDuration = 0;

// Strict Player ID detection thresholds
#define P1_MIN_DURATION 7   // Player 1: 7-14ms
#define P1_MAX_DURATION 14
#define P2_MIN_DURATION 16  // Player 2: 16-25ms
#define P2_MAX_DURATION 25

void setup() {
  Serial.begin(BAUDRATE);
  
  // Setup IR receiver-specific logic with new hardware
  irReceiverSetup();

  // Set up internal comms 
  setupBle();
}

void loop() {
    if (!hasHandshake()) {
    handshakeStatus = doHandshake();
  }
  unsigned long currentTime = millis();
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
  
  // Process IR signals for hit detection
  processIrSignals(); 
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(1);
  
  // Health
  
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("HP:");
  display.println(playerHp);
  
  
  // Health bar
  display.drawRect(0, 20, 100, 10, 1);
  display.fillRect(0, 20, playerHp, 10, 1);
  
  // Status info
  /*
  display.setTextSize(1);
  display.setCursor(0,35);
  display.print("Hits: ");
  display.println(hitCount);
  
  if (lastPlayerID > 0) {
    display.setCursor(0,45);
    display.print("Last: P");
    display.println(lastPlayerID);
    display.print("Rej: ");
    display.println(rejected);
  } 
  
  // Player identifier and connection status
  display.setCursor(0,55);
  display.print("PLAYER 1 VEST"); */
  if (hasHandshake()) {
    display.print(" CONN");
  }
  
  display.display();
}

void playHitSound() {
  if (playerHp > 0) {
    tone(BUZZ_PIN, 2000, 100);
  } else {
    // Death sound
    tone(BUZZ_PIN, 500, 200);
    delay(250);
    tone(BUZZ_PIN, 400, 200);
    delay(250);
    tone(BUZZ_PIN, 300, 200);
  }
}

byte identifyPlayer(unsigned long duration) {
  // Strict player identification with explicit thresholds
  if (duration >= P1_MIN_DURATION && duration <= P1_MAX_DURATION) {
    return 1;
  } else if (duration >= P2_MIN_DURATION && duration <= P2_MAX_DURATION) {
    return 2;
  } else {
    return 0; // Invalid ID
  }
}

void processHit(byte playerID) {
  // Only accept hits from Player 1
  if (playerID != 1) {
    rejected++;
    updateDisplay(); // Update display to show rejected count
    return;
  }
  
  // Check cooldown
  if (millis() - lastHitTime < HIT_COOLDOWN) {
    return;
  }
  
  hitCount++;
  lastHitTime = millis();
  lastPlayerID = playerID;
  
  if (playerHp > 0) {
    playerHp -= DAMAGE_PER_HIT;
    if (playerHp < DAMAGE_PER_HIT) playerHp = 0;
    
    // Set isHit to true so that the BLE logic can send it to the laptop
    isHit = true;
    
    playHitSound();
    updateDisplay();
  }
}

void processIrSignals() {
  bool currentState = digitalRead(IR_PIN);
  unsigned long currentTime = millis();
  static bool lastState = HIGH;
  
  // IR state changed
  if (currentState != lastState) {
    lastState = currentState;
    
    // Start of signal burst (IR LOW)
    if (currentState == LOW && !signalActive) {
      signalStart = currentTime;
      signalActive = true;
    }
    // End of signal burst (IR HIGH)
    else if (currentState == HIGH && signalActive) {
      unsigned long duration = currentTime - signalStart;
      signalActive = false;
      
      // Process based on burst sequence with strict validation
      if (burstCount == 0) {
        // First burst is header - confirm it's approximately 50ms
        if (duration >= 40 && duration <= 60) {
          burstCount = 1;
        } else {
          // Invalid header, reset
          burstCount = 0;
        }
      }
      else if (burstCount == 1) {
        // Second burst is player ID - store duration for analysis
        burstDuration = duration;
        burstCount = 2;
      }
      else if (burstCount == 2) {
        // Third burst is end marker - process complete signal
        // Confirm it's approximately 20ms
        if (duration >= 15 && duration <= 25) {
          // Identify player using strict thresholds
          byte playerID = identifyPlayer(burstDuration);
          
          if (playerID > 0) {
            processHit(playerID);
          } else {
            // Invalid player ID
            rejected++;
            updateDisplay();
          }
        } else {
          // Invalid end marker
          rejected++;
          updateDisplay();
        }
        
        // Reset for next signal
        burstCount = 0;
      }
    }
  }
  
  // Reset if signal stays active too long
  if (signalActive && (currentTime - signalStart > 100)) {
    signalActive = false;
    burstCount = 0;
  }
} 

HandshakeStatus doHandshake() {
  unsigned long mLastPacketSentTime = millis();
  BlePacket mLastSentPacket;
  byte mSeqNum = INITIAL_SEQ_NUM;
  bool mIsWaitingForAck = false;
  uint8_t mNumInvalidPacketsReceived = 0;
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
  return handshakeStatus;
}

/**
 * Setup for the BLE internal communications-related logic and variables
 */
void setupBle() {
  // Clear the serial output buffer
  //   WARNING: This sends out all existing data in the output buffer over BLE though
  Serial.flush();

  // Clear the serial input buffer
  clearSerialInputBuffer();

  /* Initialise lastSentPacket with invalid metadata
    to ensure it's detected as corrupted if ever
    sent without assigning actual (valid) packet */
  lastSentPacket.metadata = PLACEHOLDER_METADATA;
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

/**
 * Reads raw data from connected sensors and returns a BlePacket encapsulating the data
 */
BlePacket createRawDataPacket() {
  BlePacket vestPacket = {};
  byte packetData[PACKET_DATA_SIZE] = {};
  createVestPacketData(isHit, packetData);
  createPacket(vestPacket, PacketType::IR_RECV, senderSeqNum, packetData);
  return vestPacket;
}

bool getIsHitFrom(const BlePacket &gamePacket) {
  return gamePacket.data[0] == 1;
}

uint8_t getPlayerHpFrom(const BlePacket &gamePacket) {
  return gamePacket.data[1];
}

/* 
 * Update internal variables based on the new game state received
 */
void handleGamePacket(const BlePacket &gamePacket) {
  bool newIsHit = getIsHitFrom(gamePacket);
  uint8_t newPlayerHp = getPlayerHpFrom(gamePacket);
  if (newIsHit) {
    playHitSound();
  }
  if (newPlayerHp != INVALID_HP && newPlayerHp != playerHp) {
    // Update the HP on display
    playerHp = newPlayerHp;
    updateDisplay();
    
    // Handle respawn or damage sounds if needed
    if (newPlayerHp > playerHp) {
      // Player respawned
      tone(BUZZ_PIN, 1000, 500); // Respawn sound
    }
  }
  isHit = newIsHit;
}

bool hasHandshake() {
  return handshakeStatus == HandshakeStatus::STAT_SYN;
}

/**
 * Checks whether the connected sensors of this Beetle has raw data to send to laptop.
 */
bool hasRawData() {
  // We set isHit to true in processHit when a hit is detected
  return isHit;
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
      // Reset isHit state so next gunshot can be registered
      isHit = false;
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
      }
      break;
    case GAME_STAT:
      {
        uint16_t seqNumToAck = receiverSeqNum;
        if (receiverSeqNum == packet.seqNum) {
          // Process the packet to handle specific game logic(e.g. updating Beetle's internal game state)
          handleGamePacket(packet);
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

BlePacket sendVestPacket() {
  BlePacket vestPacket = {};
  byte packetData[PACKET_DATA_SIZE] = {};
  createVestPacketData(isHit, packetData);
  createPacket(vestPacket, PacketType::IR_RECV, senderSeqNum, packetData);
  sendPacket(vestPacket);
  return vestPacket;
}

/* New IR Receiver Setup with Adafruit Display */
void irReceiverSetup() {
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(BUZZ_PIN, OUTPUT);
  
  // Initialize the Adafruit display
  display.begin(0x3C, true);
  //display.clearDisplay();
  
  // Test buzzer
  tone(BUZZ_PIN, 1000, 100);
  delay(200);
  tone(BUZZ_PIN, 1500, 100);
  
  // Show initial state on display
  updateDisplay();
} 

