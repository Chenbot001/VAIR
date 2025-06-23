// --- Pin Definitions for Your Specific STM32 Wiring ---
#define SLEEP1  PC12 // Motor 1 Enable (EN1)
#define DIR1    PB8  // Motor 1 Direction
#define STEP1   PC9  // Motor 1 Step

#define SLEEP2  PD2  // Motor 2 Enable (EN2)
#define DIR2    PB9  // Motor 2 Direction
#define STEP2   PC8  // Motor 2 Step

// --- Global Position Tracking Variables ---
// These are important to maintain the state of the motors between commands.
long motor1_position = 0;
long motor2_position = 0;

void setup() {
  Serial.begin(115200);
  
  // Configure all motor pins as outputs
  pinMode(SLEEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);

  // --- Start with motors explicitly disabled for safety ---
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);

  Serial.println("--- Gripper Control Ready ---");
  Serial.println("Send commands in format: <M1_pos,M2_pos,freq,microsteps>");
  Serial.println("Example: <500,500,1320,16>");
}

void loop() {
  // Check if there is a complete command available from the serial port
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // --- Command Parsing Logic ---
    if (command.startsWith("<") && command.endsWith(">")) {
      // Remove the start and end markers
      command = command.substring(1, command.length() - 1);
      
      // Variables to hold parsed values
      long target_m1, target_m2;
      float freq;
      int microsteps;

      // Find the comma delimiters
      int comma1 = command.indexOf(',');
      int comma2 = command.indexOf(',', comma1 + 1);
      int comma3 = command.indexOf(',', comma2 + 1);

      // Ensure all four parts are present
      if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
        String m1_str = command.substring(0, comma1);
        String m2_str = command.substring(comma1 + 1, comma2);
        String freq_str = command.substring(comma2 + 1, comma3);
        String micro_str = command.substring(comma3 + 1);

        // Convert string parts to numbers
        target_m1 = m1_str.toInt();
        target_m2 = m2_str.toInt();
        freq = freq_str.toFloat();
        microsteps = micro_str.toInt();

        // --- Execute the Command ---
        Serial.print("Received Command: Move M1 to ");
        Serial.print(target_m1);
        Serial.print(", M2 to ");
        Serial.print(target_m2);
        Serial.print(" at ");
        Serial.print(freq);
        Serial.print(" Hz with 1/");
        Serial.print(microsteps);
        Serial.println(" microstepping.");

        // Enable motors only for the duration of the move
        digitalWrite(SLEEP1, HIGH);
        digitalWrite(SLEEP2, HIGH);
        delay(10); // Small delay for drivers to power up

        // Call the move function with the calculated target positions
        moveToTarget(target_m1 * microsteps, target_m2 * microsteps, freq);

        // Disable motors immediately after the move for safety
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
 * @brief Moves both motors simultaneously to their target positions.
 * This function is UNCHANGED from your previous version.
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
    if (motor1_position != m1_target_pos) {
      if (micros() - m1_last_step >= step_delay_us) {
        m1_last_step = micros();
        digitalWrite(STEP1, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP1, LOW);
        motor1_position += (m1_target_pos > motor1_position) ? 1 : -1;
      }
    }
    if (motor2_position != m2_target_pos) {
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