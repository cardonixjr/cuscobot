# include <ros.h>
# include <std_msgs/Int32.h>
# include <std_msgs/String.h>

#define CMD (byte)0x00 // MD49 command address of 0
#define GET_SPEED1 0x21
#define GET_ENC1 0x23
#define GET_ENC2 0X24
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32
#define ENC_RESET 0x35

uint32_t encoder = 0;
byte enc1a, enc1b, enc1c, enc1d = 0;
byte speed1a = 0;
byte speed1b = 0;

// reads the value of encoder 1 into an unsigned 32 bit int
int a, b, c, d;
unsigned int integerValue = 0; // Max value is 65535
char incomingByte;
int sensors[8] = {0};
static char outstr[15];
char inChar;
float encoder1 = 0;
float encoder2 = 0;
int vel_e = 128;
int vel_d = 128;


void callBackCommandHandle(const std_msgs::String &arduinoCommand){
  String command = arduinoCommand.data;

  char cmd = command.substring(1,2).c_str();
  String data = command.substring(2);

  // Brake command
  if (cmd == (char) 102)
  {
    Serial1.write(CMD);
    Serial1.write(SET_SPEED1);
    Serial1.write(128);

    Serial1.write(CMD);
    Serial1.write(SET_SPEED2);
    Serial1.write(128);
  }
  
  // Write Left Wheel Speed
  if(cmd == 101)
  {
    int vel_e = data.toInt();
    Serial1.write(CMD);
    Serial1.write(SET_SPEED1);
    Serial1.write(vel_e);
  }

  if(cmd == (char) 100) // roda direita
  {
    int vel_d = data.toInt();
    Serial1.write(CMD);
    Serial1.write(SET_SPEED2);
    Serial1.write(vel_d);
  }

  if(cmd == (char)114)
  {
      Serial1.write(CMD);
      Serial1.write(ENC_RESET);
  }
}

ros::NodeHandle nh;

// Publishers for the encoder pulses
// left encoder publisher
std_msgs::Int32 leftEncoderROS;
ros::Publisher leftEncoderROSPublisher("left_encoder_pulses", &leftEncoderROS);

// right enocder publisher
std_msgs::Int32 rightEncoderROS;
ros::Publisher rightEncoderROSPublisher("right_encoder_pulses", &rightEncoderROS);

// Subscribers for arduino commands
ros::Subscriber<std_msgs::String> arduinoROSSubscriber("arduino_command", callBackCommandHandle);

void setup() {
  // Setup Serial communication with Driver
  Serial1.begin(9600);
  delay(10);
  // Reset the encoders
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);
  delay(10);

  // Setup ROS
  nh.getHardware()->setBaud(9600);

  nh.initNode();

  //Publishers
  nh.advertise(leftEncoderROSPublisher);
  nh.advertise(rightEncoderROSPublisher);

  //Subscriber
  nh.subscribe(arduinoROSSubscriber);
}

void loop() {
  nh.spinOnce();
  
  // Read Left encoder
  Serial1.write(CMD);
  Serial1.write(GET_ENC1); // Recieve left encoder value
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
  leftEncoderROS.data = (int) encoder;
  
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
  rightEncoderROS.data = (int) encoder;
  nh.spinOnce();

  leftEncoderROSPublisher.publish(&leftEncoderROS);
  rightEncoderROSPublisher.publish(&rightEncoderROS);
  nh.spinOnce();
  delay(20);
}

