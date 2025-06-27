// --- Pin Definitions for Arduino UNO (from your diagram) ---
#define EN1     3  // Enable Motor 1
#define DIR1    4  // Direction Motor 1
#define ST1     5  // Step Motor 1

#define EN2     8  // Enable Motor 2
#define DIR2    7  // Direction Motor 2
#define ST2     6  // Step Motor 2

// --- Motor & Homing Configuration Constants ---
#define MICROSTEP_FACTOR       16
#define FULL_TRAVEL_STEPS      1000
#define FULL_TRAVEL_MICROSTEPS (FULL_TRAVEL_STEPS * MICROSTEP_FACTOR) // 16000
#define HOMING_FREQUENCY_HZ    8000 // Speed in microsteps/sec for homing

// --- Global Position Tracking ---
long motor1_position = 0;
long motor2_position = 0;

void setup() {
  Serial.begin(115200);

  // NOTE: The while(!Serial) handshake is not needed for the Arduino UNO
  // and has been removed.

  // Configure pins using the new Arduino UNO pin numbers
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(ST1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ST2, OUTPUT);

  // Send the ready message for the Python script
  Serial.println("--- Arduino UNO Ready. Send <HOME> to begin homing sequence. ---");

  String command = "";
  while (command != "<HOME>") {
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      command.trim();
    }
  }

  // --- Homing Sequence ---
  Serial.println("--- Homing command received. Starting sequence... ---");

  // Enable motors for homing
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(10);

  // 1. Fully extend both motors
  Serial.println("Extending motors to full travel...");
  moveToTarget(FULL_TRAVEL_MICROSTEPS, FULL_TRAVEL_MICROSTEPS, HOMING_FREQUENCY_HZ);
  delay(500);

  // 2. Retract both motors to the zero position
  Serial.println("Retracting motors to zero position...");
  moveToTarget(0, 0, HOMING_FREQUENCY_HZ);
  delay(100);

  // Homing complete, disable motors and wait for motion commands
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);

  Serial.println("--- Homing Complete. Ready for motion commands. ---");
  Serial.println("Send commands in format: <M1_pos,M2_pos,freq_fullstep,microsteps>");
}

void loop() {
  // THE FIX: Set a very short timeout for reading serial commands.
  // This prevents the readStringUntil() function from blocking for a long time.
  Serial.setTimeout(10); // Set timeout to 10 milliseconds

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("<") && command.endsWith(">")) {
      String content = command.substring(1, command.length() - 1);

      long target_m1, target_m2;
      float freq;
      int microsteps;
      long temp_freq;
      
      int parsed_items = sscanf(content.c_str(), "%ld,%ld,%ld,%d", &target_m1, &target_m2, &temp_freq, &microsteps);

      if (parsed_items == 4) {
        freq = temp_freq;
        
        digitalWrite(EN1, HIGH);
        digitalWrite(EN2, HIGH);
        delay(10);

        float frequency_in_microsteps = freq * microsteps;
        long m1_target_microsteps = target_m1 * microsteps;
        long m2_target_microsteps = target_m2 * microsteps;

        moveToTarget(m1_target_microsteps, m2_target_microsteps, frequency_in_microsteps);

        digitalWrite(EN1, LOW);
        digitalWrite(EN2, LOW);

        Serial.println("Move complete. Motors disabled.");

      } else {
        Serial.println("Error: Invalid command format.");
      }
    }
  }
}

/**
 * @brief Moves both motors simultaneously to their target positions in microsteps.
 */
void moveToTarget(long m1_target_pos, long m2_target_pos, float frequency_hz) {
  if (frequency_hz <= 0) {
    return;
  }
  unsigned long step_delay_us = 1000000 / frequency_hz;

  digitalWrite(DIR1, (m1_target_pos > motor1_position) ? HIGH : LOW);
  digitalWrite(DIR2, (m2_target_pos > motor2_position) ? HIGH : LOW);

  unsigned long m1_last_step = 0;
  unsigned long m2_last_step = 0;

  while (motor1_position != m1_target_pos || motor2_position != m2_target_pos) {
    unsigned long now = micros();
    if (motor1_position != m1_target_pos) {
      if (now - m1_last_step >= step_delay_us) {
        m1_last_step = now;
        digitalWrite(ST1, HIGH);
        delayMicroseconds(5);
        digitalWrite(ST1, LOW);
        motor1_position += (m1_target_pos > motor1_position) ? 1 : -1;
      }
    }
    if (motor2_position != m2_target_pos) {
      if (now - m2_last_step >= step_delay_us) {
        m2_last_step = now;
        digitalWrite(ST2, HIGH);
        delayMicroseconds(5);
        digitalWrite(ST2, LOW);
        motor2_position += (m2_target_pos > motor2_position) ? 1 : -1;
      }
    }
  }
}