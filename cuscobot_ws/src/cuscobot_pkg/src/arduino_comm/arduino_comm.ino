
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#define CMD        (byte)0x00            // MD49 command address of 0                                 
#define GET_SPEED1 0x21
#define GET_ENC1 0x23
#define GET_ENC2 0X24
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32
#define ENC_RESET 0x35
#define DISABLE_TIMEOUT  0X38

uint32_t encoder = 0;
byte enc1a, enc1b, enc1c, enc1d = 0;

int vel_e = 128;
int vel_d = 128;

ros::NodeHandle  nh;

std_msgs::Int32 leftEncoder;
ros::Publisher leftEncoderPublisher("left_encoder_pulses", &leftEncoder);

// right enocder publisher
std_msgs::Int32 rightEncoder;
ros::Publisher rightEncoderPublisher("right_encoder_pulses", &rightEncoder);


void leftWheelCB(const std_msgs::Int32 &leftPWM){
  // leftEncoder.data = 2*leftPWM.data;
  // leftEncoderPublisher.publish(&leftEncoder);

  Serial1.write(CMD);
  Serial1.write(SET_SPEED1);
  Serial1.write(leftPWM.data);
}

void rightWheelCB(const std_msgs::Int32 &rightPWM){
  // rightEncoder.data = 2*rightPWM.data;
  // rightEncoderPublisher.publish(&rightEncoder);

  Serial1.write(CMD);
  Serial1.write(SET_SPEED2);
  Serial1.write(rightPWM.data);
}

void resetEncoderCB(const std_msgs::Empty &command){
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);
}

ros::Subscriber<std_msgs::Int32> leftWheelSubscriber("left_wheel_pwm", leftWheelCB );
ros::Subscriber<std_msgs::Int32> rightWheelSubscriber("right_wheel_pwm", rightWheelCB );
ros::Subscriber<std_msgs::Empty> encoderResetSubscriber("reset_encoder", resetEncoderCB);


void setup()
{
  // SERIAL
  // Serial.begin(57600);
  Serial1.begin(9600);
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);

  // ROS
  nh.initNode();
  nh.advertise(leftEncoderPublisher);
  nh.advertise(rightEncoderPublisher);
  nh.subscribe(leftWheelSubscriber);
  nh.subscribe(rightWheelSubscriber);
  nh.subscribe(encoderResetSubscriber);
}

void loop()
{
  nh.spinOnce();

  // leftEncoder.data = 123;
  // leftEncoderPublisher.publish(&leftEncoder);

  // rightEncoder.data = 321;
  // rightEncoderPublisher.publish(&rightEncoder);

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
  leftEncoder.data = (int) encoder;
  
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
  rightEncoder.data = (int) encoder;
  nh.spinOnce();

  leftEncoderPublisher.publish(&leftEncoder);
  rightEncoderPublisher.publish(&rightEncoder);

  nh.spinOnce();
  delay(500);
}
