
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//initialize the liquid crystal library
//the first parameter is the I2C address
// 16 cols, 2 rows -- I2C address located using I2C_scanner sketch
LiquidCrystal_I2C lcd(0x20, 16, 2);
#define DIR_L 12
#define PWM_L 11

#define DIR_R 10
#define PWM_R 9

#define ENC_A_L 19            //A channel for encoder of left motor
#define ENC_B_L 18               //B channel for encoder of left motor

#define ENC_A_R 2             //A channel for encoder of right motor
#define ENC_B_R 3              //B channel for encoder of right motor


int cpr = 230.436;
long volatile countsL = 0;
long volatile countsR = 0;


void setup() {
  Serial.begin(9600);

  // set motor dir & pwm pins to output
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
 
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);

 // set motor speeds to 0
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW); 

  
  // Define the quad encoder for left motor
  pinMode(ENC_A_L, INPUT_PULLUP); 
  pinMode(ENC_B_L, INPUT_PULLUP);
//  digitalWrite(ENC_B_L, LOW);
  

  // Define the quad encoder for right motor
  pinMode(ENC_A_R, INPUT_PULLUP); 
  pinMode(ENC_B_R, INPUT_PULLUP);
//  digitalWrite(ENC_B_R, LOW);

  // Initialize hardware interrupts
//  attachInterrupt(digitalPinToInterrupt(ENC_B_R), readEncR, RISING); //INT4 --> Pin 2
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), readEncR, RISING); //INT5 --> Pin 3
//  attachInterrupt(digitalPinToInterrupt(ENC_B_L), readEncL, RISING);//INT2 --> Pin 19
  attachInterrupt(digitalPinToInterrupt(ENC_A_L), readEncL, RISING); //INT3 --> Pin 18
  
  
}

void loop() 
{

  // set motor direction and PWM
  digitalWrite(DIR_R, HIGH);
  analogWrite(PWM_R, 0);
  digitalWrite(DIR_L, HIGH);
  analogWrite(PWM_L, 0);

  //initialize lcd screen
  lcd.init();
  // turn on the backlight
  lcd.backlight();
  
  // set cursor on row 0, col 0 // then on row 0, col 12
  lcd.setCursor(0,0);
  lcd.print("Rev_R: ");
  lcd.setCursor(12,0);
  lcd.print(countsR/cpr);

  // set cursor on row 1, col 0 // then on row 1, col 12
  lcd.setCursor(0,1);
  lcd.print("Rev_L: ");
  lcd.setCursor(12,1);
  lcd.print(countsL/cpr);

  
}

void readEncL() // occurs when A goes HIGH (RISING
{ 
  if(digitalRead(ENC_A_L)==digitalRead(ENC_B_L)) {
    countsL++;
  }
  else {
    countsL--;
  }
}

void readEncR() // occurs when A goes HIGH (RISING
{ 
  if(digitalRead(ENC_A_R)==digitalRead(ENC_B_R)) {
    countsR++;
  }
  else {
    countsR--;
  }
}
