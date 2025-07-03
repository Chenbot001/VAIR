// --- Pin Definitions for Your Specific STM32 Wiring ---
#define SLEEP1  7 // Motor 1 Enable (EN1)
#define DIR1    8  // Motor 1 Direction
#define STEP1   9  // Motor 1 Step

#define SLEEP2  5  // Motor 2 Enable (EN2)
#define DIR2    4  // Motor 2 Direction
#define STEP2   3  // Motor 2 Step

#define MICROSTEP_FACTOR 16
#define TARGET_FREQUENCY_HZ 5000 // Target speed in microsteps/second
#define PAUSE_MS            500   // Pause for 0.5 second between major steps.

// --- Global Position Tracking Variables ---
long motor1_position = 0;
long motor2_position = 0;

void setup() {
  Serial.begin(115200);
  delay(100); 
  
  // Configure all motor pins as outputs
  pinMode(SLEEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);

  Serial.println("Power OK. Enabling motors...");
  digitalWrite(SLEEP1, HIGH);
  digitalWrite(SLEEP2, HIGH);

  Serial.println("--- Starting Custom 6-Step Sequence (1/4 Microstepping) ---");
  delay(1000);

  // MODIFIED: All target positions are multiplied by the MICROSTEP_FACTOR
  
  // 1) move both motors 500 full steps (2000 microsteps)
  moveToTarget(500 * MICROSTEP_FACTOR, 500 * MICROSTEP_FACTOR, TARGET_FREQUENCY_HZ);
  delay(PAUSE_MS);

  // 2) move motor 1 to 1000 and motor 2 to 0
  moveToTarget(1000 * MICROSTEP_FACTOR, 0 * MICROSTEP_FACTOR, TARGET_FREQUENCY_HZ);
  delay(PAUSE_MS);

  // 3) move motor 1 to 0 and motor 2 to 1000
  moveToTarget(0 * MICROSTEP_FACTOR, 1000 * MICROSTEP_FACTOR, TARGET_FREQUENCY_HZ);
  delay(PAUSE_MS);

  // 4) move motor 2 back to 0
  moveToTarget(0 * MICROSTEP_FACTOR, 0 * MICROSTEP_FACTOR, TARGET_FREQUENCY_HZ);
  delay(PAUSE_MS);

  Serial.println("--- Sequence Complete. Putting motors to sleep. ---");
  digitalWrite(SLEEP1, LOW); 
  digitalWrite(SLEEP2, LOW); 
}

void loop() {
  // Nothing here. The entire sequence runs once in setup().
}

/**
 * @brief Moves both motors simultaneously to their target positions.
 * This function will not return until both motors have reached their targets.
 * @param m1_target_pos The absolute target position for Motor 1.
 * @param m2_target_pos The absolute target position for Motor 2.
 * @param frequency_hz  // MODIFIED: The desired speed in steps per second (Hz).
 */
void moveToTarget(long m1_target_pos, long m2_target_pos, float frequency_hz) {
  // NEW: Calculate the required step delay in microseconds from the frequency.
  // 1,000,000 microseconds in a second.
  // Note: Handles edge case where frequency is zero or negative to prevent division by zero.
  if (frequency_hz <= 0) {
    return; 
  }
  unsigned long step_delay_us = 1000000 / frequency_hz;

  // Set the directions for this movement
  digitalWrite(DIR1, (m1_target_pos > motor1_position) ? HIGH : LOW);
  digitalWrite(DIR2, (m2_target_pos > motor2_position) ? HIGH : LOW);
  
  unsigned long m1_last_step = 0;
  unsigned long m2_last_step = 0;

  // Loop as long as at least one motor is not at its target
  while (motor1_position != m1_target_pos || motor2_position != m2_target_pos) {

    // --- Motor 1 Stepping Logic ---
    if (motor1_position != m1_target_pos) {
      // MODIFIED: Use the calculated delay instead of the old #define
      if (micros() - m1_last_step >= step_delay_us) {
        m1_last_step = micros();
        digitalWrite(STEP1, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP1, LOW);
        motor1_position += (m1_target_pos > motor1_position) ? 1 : -1;
      }
    }

    // --- Motor 2 Stepping Logic ---
    if (motor2_position != m2_target_pos) {
      // MODIFIED: Use the calculated delay instead of the old #define
      if (micros() - m2_last_step >= step_delay_us) {
        m2_last_step = micros();
        digitalWrite(STEP2, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP2, LOW);
        motor2_position += (m2_target_pos > motor2_position) ? 1 : -1;
      }
    }
  }
}