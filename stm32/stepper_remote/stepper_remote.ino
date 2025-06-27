// --- Pin Definitions ---
#define SLEEP1  PC12
#define DIR1    PB8
#define STEP1   PC9

#define SLEEP2  PD2
#define DIR2    PB9
#define STEP2   PC8

// --- Motor & Homing Configuration Constants ---
#define MICROSTEP_FACTOR      16
#define FULL_TRAVEL_STEPS     1000
#define FULL_TRAVEL_MICROSTEPS (FULL_TRAVEL_STEPS * MICROSTEP_FACTOR) // 16000
#define HOMING_FREQUENCY_HZ   8000 // Speed in microsteps/sec for homing

// --- Global Position Tracking ---
long motor1_position = 0;
long motor2_position = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(SLEEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);

  // Now, wait for the user to initiate homing
  Serial.println("--- STM32 Ready. Send <HOME> to begin homing sequence. ---");
  
  String command = "";
  while(command != "<HOME>") {
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      command.trim();
    }
  }

  // --- Homing Sequence at Startup ---
  Serial.println("--- Starting Homing Sequence ---");-
  
  // Enable motors for homing
  digitalWrite(SLEEP1, HIGH);
  digitalWrite(SLEEP2, HIGH);
  delay(10);

  // 1. Fully extend both motors
  Serial.println("Extending motors to full travel...");
  moveToTarget(FULL_TRAVEL_MICROSTEPS, FULL_TRAVEL_MICROSTEPS, HOMING_FREQUENCY_HZ);
  delay(500);

  // 2. Retract both motors to the zero position
  Serial.println("Retracting motors to zero position...");
  moveToTarget(0, 0, HOMING_FREQUENCY_HZ);
  delay(100);

  // Homing complete, disable motors and wait for commands
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);
  
  Serial.println("--- Homing Complete. Ready for Commands ---");
  Serial.println("Send commands in format: <M1_pos,M2_pos,freq_fullstep,microsteps>");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("<") && command.endsWith(">")) {
      String content = command.substring(1, command.length() - 1);
      
      long target_m1, target_m2;
      float freq;
      int microsteps;

      int parsed_items = sscanf(content.c_str(), "%ld,%ld,%f,%d", &target_m1, &target_m2, &freq, &microsteps);

      if (parsed_items == 4) {
        digitalWrite(SLEEP1, HIGH);
        digitalWrite(SLEEP2, HIGH);
        delay(10);

        // --- REVERTED: STM32 now handles microstep scaling ---
        // The 'freq' variable is from Python (full steps/sec). We scale it here.
        float frequency_in_microsteps = freq * microsteps;
        
        long m1_target_microsteps = target_m1 * microsteps;
        long m2_target_microsteps = target_m2 * microsteps;

        moveToTarget(m1_target_microsteps, m2_target_microsteps, frequency_in_microsteps);

        digitalWrite(SLEEP1, LOW);
        digitalWrite(SLEEP2, LOW);
        
        Serial.println("Move complete. Motors disabled.");
        
      } else {
        Serial.println("Error: Invalid command format.");
      }
    }
  }
}

/**
 * @brief Moves both motors simultaneously to their target positions in microsteps.
 * (This function is unchanged)
 */
void moveToTarget(long m1_target_pos, long m2_target_pos, float frequency_hz) {
  // ... (Function content is the same as previous versions)
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
        digitalWrite(STEP1, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP1, LOW);
        motor1_position += (m1_target_pos > motor1_position) ? 1 : -1;
      }
    }
    if (motor2_position != m2_target_pos) {
      if (now - m2_last_step >= step_delay_us) {
        m2_last_step = now;
        digitalWrite(STEP2, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP2, LOW);
        motor2_position += (m2_target_pos > motor2_position) ? 1 : -1;
      }
    }
  }
}