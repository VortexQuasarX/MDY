int relay1Pin = 6; // Light bulb relay
int relay2Pin = 5; // Vibration emitter relay
int relay3Pin = 4; // Electric mat relay
int relay4Pin = 3; // Humidifier relay
int buzzerPin = 8; // Pin connected to the buzzer

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);
  pinMode(buzzerPin, OUTPUT); // Set the buzzer pin as an output
  
  // Ensure all relays and buzzer are off at startup
  digitalWrite(relay1Pin, HIGH); // Light bulb off
  digitalWrite(relay2Pin, HIGH); // Vibration emitter off
  digitalWrite(relay3Pin, HIGH); // Electric mat off
  digitalWrite(relay4Pin, HIGH); // Humidifier off
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read incoming serial data
    
    // Control light bulb
    if (command == "LIGHT_ON") {
      digitalWrite(relay1Pin, LOW);
      delay(20);
      digitalWrite(relay1Pin, HIGH);
    } else if (command == "LIGHT_OFF") {
      digitalWrite(relay1Pin, HIGH);
    }

    // Control humidifier
    if (command == "HUMIDIFIER_ON") {
      digitalWrite(relay4Pin, LOW);
    } else if (command == "HUMIDIFIER_OFF") {
      digitalWrite(relay4Pin, HIGH);
    }

    // Control vibration emitter
    if (command == "VIBRATION_EMITTER_ON") {
      digitalWrite(relay2Pin, LOW);
    } else if (command == "VIBRATION_EMITTER_OFF") {
      digitalWrite(relay2Pin, HIGH);
    }

    // Control electric mat
    if (command == "ELECTRIC_MAT_ON") {
      digitalWrite(relay3Pin, LOW);
    } else if (command == "ELECTRIC_MAT_OFF") {
      digitalWrite(relay3Pin, HIGH);
    }

    // Check for the "“BUZZER_ON” command from the laptop
    if (command == “BUZZER_ON”) {
      // Create a shorter pulsing effect with a reduced duration
      for (int i = 0; i < 50; i++) { // Reduce the number of pulses
        tone(buzzerPin, 2000);  // Emit tone at 2000 Hz
        delay(5);               // Shorter on duration
        tone(buzzerPin, 3000);  // Change tone to 3000 Hz
        delay(5);               // Shorter on duration
        noTone(buzzerPin);      // Turn off buzzer briefly
        delay(2);               // Short off duration to create pulsing effect
      }
    }
  }
}