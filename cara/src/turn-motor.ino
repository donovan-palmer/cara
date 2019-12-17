#include <LiquidCrystal_I2C.h>

#define ENCODEROUTPUT 230.436 // motor encoder output pulse per rotation
#define BUTTON 7
#define HALLSEN_A 3 // Hall sensor A connected to pin 3 (external interrupt)
#define HALLSEN_B 6 // Hall sensor A connected to pin 6 (external interrupt)
#define PWM 5 // Speed
#define DIR 4 // Direction

volatile long encoderValue = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
const char* cycle = "";
int rpm = 0;
boolean measureRpm = false;
int motorPwm = 0;


void setup()
{

  Serial.begin(9600); // Initialize serial with 9600 baudrate
  
  pinMode(BUTTON, INPUT_PULLUP); // Set button as input pullup
  pinMode(HALLSEN_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_B, INPUT_PULLUP); // Set hall sensor B as input pullup
  pinMode(PWM, OUTPUT); // Set PWM pin as output
  pinMode(DIR, OUTPUT); // Set DIR pin as output

  // Attach interrupt at hall sensor A & B on each rising signal
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_B), updateEncoder, RISING);
//  Serial.print("\n\n");
//  Serial.println("Measuring DC Motor's RPM");
//  Serial.println("Press button to begin.");
//  Serial.println("Press again to stop.");

}

void loop()
{
//  lcd.setCursor(0, 0);
//  lcd.blink();
  
//    Serial.println(digitalRead(3)); //print digital hall sensor A
//    Serial.println(analogRead(7)); //print analog value of button pushing
//    digitalWrite(DIR,HIGH); // spin motor CCW looking down at shaft
//    digitalWrite(DIR,LOW); // spin motor CW looking down at shaft
  if (analogRead(BUTTON) < 200) { // If button is pressed...
    
    measureRpm = !measureRpm;
    if (measureRpm == true) { // 1st pressed, start moving the motor
      //      Serial.println("\nStart...\n");
      //      Serial.println("PWM\tRPM");
      //      playMelody();
    }
    else if (measureRpm == false) { // 2nd pressed, stop the motor
      //      Serial.println("\nStop...");
      //      tone(BUZZER, NOTE_C5, 100);
    }

    delay(500);
    encoderValue = 0;
    previousMillis = millis();
  }

  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    if (digitalRead(3) > digitalRead(6)) {
      cycle = "CCW";
    }
    else {
      cycle = "CW";
    }

    if (measureRpm == true &&
        motorPwm < 255) {
      motorPwm += 50;
      analogWrite(PWM, motorPwm);
    }
   else if (measureRpm == false &&
            motorPwm > 0) {
     motorPwm -= 25;
     analogWrite(PWM, motorPwm);
   }
    // Revolutions per minute (RPM) =
    // (total encoder pulse in 1s / motor encoder output) x 60s
    rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);
    
    if (motorPwm > 0 || rpm > 0) {
     Serial.print(motorPwm);
     Serial.print(encoderValue);
     Serial.print('\t');
     Serial.print(cycle);
     Serial.print('\t');
     Serial.print(encoderValue);
     Serial.print(" pulse / ");
     Serial.print(ENCODEROUTPUT);
     Serial.print(" pulse per rotation x 60 seconds = ");
      Serial.println(rpm);
      //
    }

    encoderValue = 0;
  }
}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
}
