// Player 2 Vest Code with Shield Feature
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Use exactly the same initialization as the original working code
Adafruit_SH1106G display(128, 64, &Wire, 4);

#define IR_PIN 5
#define BUZZ_PIN 2

// Game constants
#define INITIAL_HEALTH 100
#define DAMAGE_PER_HIT 5
#define HIT_COOLDOWN 800
#define INITIAL_SHIELDS 3
#define SHIELD_COOLDOWN 3000

// Game state
byte health = INITIAL_HEALTH;
byte hitCount = 0;
byte lastPlayerID = 0;
byte rejected = 0;
byte shieldsRemaining = INITIAL_SHIELDS;
bool shieldActive = false;
unsigned long lastShieldTime = 0;

// Signal processing
unsigned long signalStart = 0;
bool signalActive = false;
unsigned long lastHitTime = 0;
byte burstCount = 0;
unsigned long burstDuration = 0;

// Shield animation
byte shieldAnimFrame = 0;
unsigned long shieldAnimTime = 0;
#define SHIELD_ANIM_SPEED 150

// Strict Player ID detection thresholds
#define P1_MIN_DURATION 7   // Player 1: 7-14ms
#define P1_MAX_DURATION 14
#define P2_MIN_DURATION 16  // Player 2: 16-25ms
#define P2_MAX_DURATION 25

void playHitSound() {
  if (health > 0) {
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

void playShieldSound() {
  // Shield activation sound
  tone(BUZZ_PIN, 1500, 50);
  delay(50);
  tone(BUZZ_PIN, 2000, 50);
}

void playShieldBlockSound() {
  // Sound when shield blocks damage
  tone(BUZZ_PIN, 2500, 50);
  delay(50);
  tone(BUZZ_PIN, 1500, 50);
}

// Modified updateDisplay function with enhanced aesthetics
void updateDisplay() {
  display.clearDisplay();
  
  // Health - with custom color (we can only use white on OLED, so use different patterns)
  display.setTextSize(2);
  display.setCursor(0,0);
  display.setTextColor(1);  // Normal white text for HP
  display.print("HP:");
  display.println(health);
  
  // Health bar with color indicators based on health percentage
  display.drawRect(0, 20, 100, 10, 1);  // Outline
  
  // Health bar fill with different patterns based on health level
  byte fillStyle = 0;
  if (health > 75) {
    // Green (76-100%): Solid fill
    fillStyle = 1;
  } else if (health > 50) {
    // Orange (51-75%): Every other pixel
    fillStyle = 2;
  } else if (health > 25) {
    // Yellow (26-50%): Sparse pattern
    fillStyle = 3;
  } else {
    // Red (0-25%): Very sparse pattern
    fillStyle = 4;
  }
  
  // Fill health bar based on style
  switch (fillStyle) {
    case 1:  // Solid (Green)
      display.fillRect(0, 20, health, 10, 1);
      break;
    case 2:  // Medium (Orange) - checkerboard pattern
      for (int x = 0; x < health; x += 2) {
        for (int y = 20; y < 30; y += 2) {
          display.drawPixel(x, y, 1);
        }
        for (int y = 21; y < 30; y += 2) {
          display.drawPixel(x+1, y, 1);
        }
      }
      break;
    case 3:  // Low (Yellow) - dots pattern
      for (int x = 0; x < health; x += 3) {
        for (int y = 20; y < 30; y += 3) {
          display.drawPixel(x, y, 1);
        }
      }
      break;
    case 4:  // Critical (Red) - sparse dots
      for (int x = 0; x < health; x += 4) {
        for (int y = 20; y < 30; y += 4) {
          display.drawPixel(x, y, 1);
        }
      }
      break;
  }
  
  // Shield count - inverted color for contrast
  display.fillRect(0, 35, 128, 18, 1);  // Background fill
  display.setTextColor(0);  // Black text on white background
  display.setTextSize(2);
  display.setCursor(0, 37);
  display.print("Shields:");
  display.println(shieldsRemaining);
  
  // Shield active indicator
  if (shieldActive) {
    display.setTextColor(1);  // Back to white text
    display.setTextSize(1);
    display.setCursor(70, 55);
    display.print("ACTIVE");
  }
  
  display.display();
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
  // Only accept hits from Player 1 (changed from Player 2 in the original)
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
  
  // Check if shield is active
  if (shieldActive) {
    shieldActive = false;  // Consume the shield
    playShieldBlockSound();
    updateDisplay();
    return;  // No damage taken when shield blocks
  }
  
  // Normal damage processing (no shield active)
  if (health > 0) {
    health -= DAMAGE_PER_HIT;
    if (health < DAMAGE_PER_HIT) health = 0;
    
    playHitSound();
    updateDisplay();
  }
}

// Simplified serial shield check
void checkSerialShield() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == 'S' || c == 's') {
      // Only activate if we have shields and no active shield
      if (shieldsRemaining > 0 && !shieldActive && 
         (millis() - lastShieldTime > SHIELD_COOLDOWN)) {
        
        shieldsRemaining--;
        shieldActive = true;
        lastShieldTime = millis();
        
        // Confirm activation
        Serial.println("Shield activated");
        
        playShieldSound();
        updateDisplay();
      }
    }
  }
}

void setup() {
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(BUZZ_PIN, OUTPUT);
  
  // Serial at 115200 baud
  Serial.begin(115200);
  
  // Display initialization - use original code
  display.begin(0x3C, true);
  
  // Test buzzer
  tone(BUZZ_PIN, 1000, 100);
  delay(200);
  tone(BUZZ_PIN, 1500, 100);
  
  // Print setup message only once
  Serial.println("Player 2 Vest ready. Send 'S' to activate shield.");
  
  updateDisplay();
}

void loop() {
  bool currentState = digitalRead(IR_PIN);
  unsigned long currentTime = millis();
  static bool lastState = HIGH;
  
  // Check for shield activation via serial
  checkSerialShield();
  
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