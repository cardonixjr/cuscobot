#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#define CMD (byte)0x00            // MD49 command address of 0                                 
#define GET_SPEED1 0x21
#define GET_ENC1 0x23
#define GET_ENC2 0X24
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32
#define ENC_RESET 0x35
#define DISABLE_TIMEOUT  0X38

float WHEEL_RADIUS = 0.06;
float BASE_LENGTH = 0.37;
int MAX_PWM = 40;

uint32_t encoder = 0;
byte enc1a, enc1b, enc1c, enc1d = 0;

int left_pwm;
int right_pwm;

float linear;
float angular;
float left_speed;
float right_speed;
float left_norm;
float right_norm;

ros::NodeHandle  nh;

std_msgs::Int32 leftEncoder;
ros::Publisher leftEncoderPublisher("left_encoder_pulses", &leftEncoder);

// right enocder publisher
std_msgs::Int32 rightEncoder;
ros::Publisher rightEncoderPublisher("right_encoder_pulses", &rightEncoder);

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  /* MD49 specifications
    RPM: from -116.5 to 116.6
    PWM: from 0 to 255  (the zero speed is the PWM 128)
    m/s: from -0.7326 to 0.7326 
  */

  linear = cmd_vel.linear.x;
  angular = cmd_vel.angular.z;

  float right_linear = ((linear/2) + (angular/BASE_LENGTH))/2;
  float left_linear =  (linear/2) - right_linear;
  
  left_pwm = map(left_linear, -0.7326,0,0.7326,254);
  right_pwm = map(right_linear, -0.7326, 0, 0.7326, 254);

  // Calculates speed
  /*
  left_speed = (2 * linear + angular * BASE_LENGTH)/(2*WHEEL_RADIUS);
  right_speed = (2 * linear - angular * BASE_LENGTH)/(2*WHEEL_RADIUS);

  int ls = left_speed/abs(left_speed);
  int rs = right_speed/abs(right_speed);

  if(abs(left_speed) > abs(right_speed)){
    left_norm = abs(left_speed)/abs(left_speed);
    right_norm = right_speed/left_speed;
  } else {
    right_norm = abs(right_speed)/abs(right_speed);
    left_norm = abs(left_speed)/abs(right_speed);
  }

  // implement speed limit by distance
  left_pwm = (int) left_norm*MAX_PWM * ls + 128;
  right_pwm = (int) right_norm*MAX_PWM * rs + 128;
  */

  Serial1.write(CMD);
  Serial1.write(SET_SPEED1);
  Serial1.write((int) left_pwm);

  Serial1.write(CMD);
  Serial1.write(SET_SPEED2);
  Serial1.write((int) right_pwm);

}

void resetEncoderCB(const std_msgs::Empty &command){
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSubscriber("cmd_vel", cmdVelCallback );
ros::Subscriber<std_msgs::Empty> encoderResetSubscriber("reset_encoder", resetEncoderCB);

void setup()
{

  Serial.begin(57600);
  // SERIAL
  Serial1.begin(9600);
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);

  // ROS
  nh.initNode();
  nh.advertise(leftEncoderPublisher);
  nh.advertise(rightEncoderPublisher);
  nh.subscribe(cmdVelSubscriber);
  nh.subscribe(encoderResetSubscriber);
}

void loop()
{
  nh.spinOnce();

  // Read Left encoder
  Serial1.write(CMD);
  Serial1.write(GET_ENC1); // Recieve encoder 1 value
  // delay(50);
  while(Serial1.available()<=3);
  if (Serial1.available())
  {
    enc1a = Serial1.read();
    enc1b = Serial1.read();
    enc1c = Serial1.read();
    enc1d = Serial1.read();
  }
  encoder = (((uint32_t)enc1a << 24) +
  ((uint32_t)enc1b << 16) +
  ((uint32_t)enc1c << 8) +
  ((uint32_t)enc1d << 0));
  leftEncoder.data = (uint32_t) encoder;
  
  // Read Right Encoder 
  Serial1.write(CMD);
  Serial1.write(GET_ENC2); // Recieve encoder right value
  // delay(50);
  while(Serial1.available()<=3);
  if (Serial1.available() > 3)
  {
    enc1a = Serial1.read();
    enc1b = Serial1.read();
    enc1c = Serial1.read();
    enc1d = Serial1.read();
  }
  encoder = (((uint32_t)enc1a << 24) +
  ((uint32_t)enc1b << 16) +
  ((uint32_t)enc1c << 8) +
  ((uint32_t)enc1d << 0));
  rightEncoder.data = (uint32_t) encoder;
  nh.spinOnce();

  leftEncoderPublisher.publish(&leftEncoder);
  rightEncoderPublisher.publish(&rightEncoder);
  nh.spinOnce();

  //CALCULO DO PID DOS PWM




  delay(500);
}
