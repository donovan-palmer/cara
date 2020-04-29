#include <Wire.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//initialize the liquid crystal library
//the first parameter is the I2C address
// 16 cols, 2 rows -- I2C address located using I2C_scanner sketch
LiquidCrystal_I2C lcd(0x20, 16, 2);

//initializing all the variables
#define LOOPTIME 100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;

#define DIR_L 11
#define PWM_L 10

#define DIR_R 9
#define PWM_R 8

#define ENC_A_L 19            //A channel for encoder of left motor
#define ENC_B_L 18               //B channel for encoder of left motor

#define ENC_A_R 2             //A channel for encoder of right motor
#define ENC_B_R 3              //B channel for encoder of right motor



long volatile countsL = 0;
long volatile countsR = 0;

unsigned long lastMilli = 0;



//--- Robot-specific constants ---
const double radius = 0.06858;                 //Wheel radius, in m
const double wheelbase = 0.4572;              //Wheelbase, in m
const double cpr = 230.436;                   //Encoder ticks or counts per rotation of output shaft
const double speed_to_pwm_ratio = 0.0035;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.3;             //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s

const double max_speed = 2.0;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor

// PID Parameters
const double PID_left_param[] = { 0 , 0, 0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0, 0, 0 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type


//__________________________________________________________________________

void setup() {

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(115200);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  //initialize lcd screen

  lcd.init();
  // turn on the backlight
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hi, I am Cara");
 
  
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

  // Define the quad encoder for right motor
  pinMode(ENC_A_R, INPUT_PULLUP);
  pinMode(ENC_B_R, INPUT_PULLUP);


  // Initialize hardware interrupts
//  attachInterrupt(digitalPinToInterrupt(ENC_B_R), readEncR, RISING); //INT4 --> Pin 2
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), readEncR, RISING); //INT5 --> Pin 3
//  attachInterrupt(digitalPinToInterrupt(ENC_B_L), readEncL, RISING);//INT2 --> Pin 19
  attachInterrupt(digitalPinToInterrupt(ENC_A_L), readEncL, RISING); //INT3 --> Pin 18


  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();

  
  if((millis()-lastMilli) >= LOOPTIME)
  {                                                                           // enter timed loop
    lastMilli = millis();


    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }

    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }

    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
   
    PID_leftMotor.Compute();
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      analogWrite(PWM_L, 0);
      digitalWrite(DIR_L, LOW);
    }
    else if (speed_req_left == 0){                        //Stopping
      analogWrite(PWM_L, 0);
      digitalWrite(DIR_L, LOW);
    }
    else if (PWM_leftMotor > 0){                          //Going forward
      analogWrite(PWM_L, abs(PWM_leftMotor));
      digitalWrite(DIR_L, HIGH);
    }
    else {                                               //Going backward
      analogWrite(PWM_L, abs(PWM_leftMotor));
      digitalWrite(DIR_L, LOW);
    }

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    PID_rightMotor.Compute();
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); //

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      analogWrite(PWM_R, 0);
      digitalWrite(DIR_R, LOW);
    }
    else if (speed_req_right == 0){                       //Stopping
      analogWrite(PWM_R, 0);
      digitalWrite(DIR_R, LOW);
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      analogWrite(PWM_R, abs(PWM_rightMotor));
      digitalWrite(DIR_R, HIGH);
    }
    else {                                                //Going backward
      analogWrite(PWM_R, abs(PWM_rightMotor));
      digitalWrite(DIR_R, LOW);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
 }

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void readEncL() {
  if (digitalRead(ENC_A_L) == digitalRead(ENC_B_L)) {
    pos_left++;
  }
  else {
    pos_left--;
  }
}

//Right motor encoder counter
void readEncR() {
  if (digitalRead(ENC_A_R) == digitalRead(ENC_B_R)) {
    pos_right++;
  }
  else {
    pos_right--;
  }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
