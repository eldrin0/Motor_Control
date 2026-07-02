#include <Arduino.h>
#include <ezButton.h>

class Motor {
  public:
    int pwmPin, rotPin, fbPin;
    volatile int pulseCount = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    Motor(int setupPwmPin, int setupRotPin, int setupFbPin) {
      pwmPin = setupPwmPin;
      rotPin = setupRotPin;
      fbPin = setupFbPin;
    }

    void begin() {
      pinMode(rotPin, OUTPUT);
      digitalWrite(rotPin, LOW); 
      pinMode(fbPin, INPUT_PULLUP);
      
      // Setup LEDC (PWM)
      ledcAttach(pwmPin, 10000, 8); // 10kHz, 8-bit resolution (0-255)
      ledcWrite(pwmPin, 0);         // Start stopped
    }

    void IRAM_ATTR onPulse() {

      pulseCount++;
    }

    // Safely reads the counts and resets them to 0 for the next cycle
    int getAndResetPulses() {
      int pulses;
      portENTER_CRITICAL(&mux);
      pulses = pulseCount;
      pulseCount = 0;
      portEXIT_CRITICAL(&mux);
      return pulses;
    }

    void command(int pwm, int dir) {
      // Limit PWM to safe 8-bit bounds
      if(pwm > 255) pwm = 255;
      if(pwm < 0) pwm = 0;

      // Handle Direction pin
      if (dir == 1) {
        digitalWrite(rotPin, LOW);
      } else {
        digitalWrite(rotPin, HIGH);
      }

      ledcWrite(pwmPin, pwm);
    }
};

// Instantiate with your exact pinouts
Motor motor1(27, 23, 18);
Motor motor2(26, 33, 5);
Motor motor3(25, 32, 19);
Motor motor4(14, 4, 34);

ezButton limitSwitch1(2);
ezButton limitSwitch2(15);

// Interrupt Service Routines
void IRAM_ATTR motor1_ISR() { motor1.onPulse(); }
void IRAM_ATTR motor2_ISR() { motor2.onPulse(); }
void IRAM_ATTR motor3_ISR() { motor3.onPulse(); }
void IRAM_ATTR motor4_ISR() { motor4.onPulse(); }

unsigned long lastTelemetryTime = 0;
const int telemetryInterval = 20; // Send data every 20ms (50Hz)

unsigned long lastCommandTime = 0;
const int watchdogTimeout = 500; // Stop motors if no PC command for 500ms

// --- Global Direction State ---
bool is_reversed = false; 

void setup() {
  Serial.begin(115200);

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  
  // NOTE: You updated these to CHANGE to increase resolution
  attachInterrupt(motor1.fbPin, motor1_ISR, CHANGE);
  attachInterrupt(motor2.fbPin, motor2_ISR, CHANGE);
  attachInterrupt(motor3.fbPin, motor3_ISR, CHANGE);
  attachInterrupt(motor4.fbPin, motor4_ISR, CHANGE);

  // ezButton handles the bouncing automatically
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
}

void loop() {
  // ezButton loop MUST run continuously to read pins
  limitSwitch1.loop();
  limitSwitch2.loop();
  
  // --- ABSOLUTE STATE LOGIC ---
  if (limitSwitch1.isPressed()) {
    is_reversed = false; // Force direction A
  }
  else if (limitSwitch2.isPressed()) {
    is_reversed = true;  // Force direction B
  }

  unsigned long now = millis();

  // 1. TELEMETRY: Send data to PC at 50Hz
  if (now - lastTelemetryTime >= telemetryInterval) {
    lastTelemetryTime = now;
    
    int m1_pulses = motor1.getAndResetPulses();
    int m2_pulses = motor2.getAndResetPulses();
    int m3_pulses = motor3.getAndResetPulses();
    int m4_pulses = motor4.getAndResetPulses();
    
    // Convert boolean state to 1 or 0 for Python
    int rev_state = is_reversed ? 1 : 0; 

    // Format: "counts1,counts2,counts3,counts4,rev_state"
    char outBuffer[64];
    snprintf(outBuffer, sizeof(outBuffer), "%d,%d,%d,%d,%d\n", 
             m1_pulses, m2_pulses, m3_pulses, m4_pulses, rev_state);
    Serial.print(outBuffer);
  }

  // 2. COMMAND LISTENER: Read instructions from PC
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    lastCommandTime = now; // Feed the watchdog

    // Expected format from PC: "pwm1,dir1,pwm2,dir2,pwm3,dir3,pwm4,dir4"
    int p1, d1, p2, d2, p3, d3, p4, d4;
    
    // Parse the comma-separated string
    if (sscanf(incoming.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &p1, &d1, &p2, &d2, &p3, &d3, &p4, &d4) == 8) {
      motor1.command(p1, d1);
      motor2.command(p2, d2);
      motor3.command(p3, d3);
      motor4.command(p4, d4);
    }
  }

  // 3. WATCHDOG TIMER: Safety shutoff
  if (now - lastCommandTime > watchdogTimeout) {
    motor1.command(0, 1);
    motor2.command(0, 1);
    motor3.command(0, 1);
    motor4.command(0, 1);
  }
}