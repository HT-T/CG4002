// Enhanced Player 1 Gun with Serial Reload
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Simple display initialization
Adafruit_SSD1306 display(128, 64, &Wire, 4);

#define TRIGGER_PIN 2
#define IR_LED_PIN 3

// Game constants
#define PLAYER_ID 1          // Player 1's ID
#define INITIAL_BULLETS 6
#define RELOAD_TIME 500      // Short reload time since we're using serial input

// Game state
byte bulletCount = INITIAL_BULLETS;
unsigned long lastReloadTime = 0;
bool isReloading = false;

void send38kHzBurst(int durationMs) {
    unsigned long endTime = millis() + durationMs;
    
    while(millis() < endTime) {
        digitalWrite(IR_LED_PIN, HIGH);
        delayMicroseconds(13);
        digitalWrite(IR_LED_PIN, LOW);
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

void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(1);
    
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
    
    // Status
    display.setTextSize(1);
    display.setCursor(0,48);
    if(isReloading) {
        display.println("RELOADING...");
        display.println("Press 'R' to reload");
    } else if(bulletCount == 0) {
        display.println("OUT OF AMMO!");
        display.println("Press 'R' to reload");
    } else {
        display.println("Ready to fire!");
    }
    
    display.display();
}

void setup() {
    Serial.begin(9600);  // Start serial for reload command
    
    pinMode(TRIGGER_PIN, INPUT);
    pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, LOW);
    
    // Simplified display initialization
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    
    // Test display
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0,0);
    display.println("Player 1 Gun");
    display.println("Initializing...");
    display.display();
    delay(1000);
    
    updateDisplay();
    
    Serial.println("Player 1 Gun Ready");
    Serial.println("Press 'R' to reload when out of ammo");
}

void loop() {
    static bool lastTrigger = false;
    bool currentTrigger = digitalRead(TRIGGER_PIN);
    
    // Check for serial reload command
    if(Serial.available() > 0) {
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
            bulletCount = INITIAL_BULLETS;
            isReloading = false;
            Serial.println("Reload complete. Ready to fire!");
            updateDisplay();
        }
    }
    
    // Handle trigger press
    if(currentTrigger && !lastTrigger) {
        delay(10); // Debounce
        if(digitalRead(TRIGGER_PIN)) {
            if(bulletCount > 0 && !isReloading) {
                // Fire!
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(1);
                display.setCursor(0,0);
                display.println("FIRING!");
                display.display();
                
                sendShot();
                bulletCount--;
                Serial.print("Shot fired! Ammo remaining: ");
                Serial.println(bulletCount);
                
                delay(100);
                updateDisplay();
            }
            else if(bulletCount == 0 && !isReloading) {
                // Out of ammo
                Serial.println("Out of ammo! Press 'R' to reload.");
                updateDisplay();
            }
        }
    }
    
    lastTrigger = currentTrigger;
}