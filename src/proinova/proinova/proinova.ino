#include <math.h>

// ROS libs
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// Capacitive libs
#include <ADCTouch.h>

// Strain-Gauge libs
#include <HX711_ADC.h>

// MD49 commands
#define CMD (byte)0x00                             
#define GET_SPEED1 0x21
#define GET_ENC1 0x23
#define GET_ENC2 0X24
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32
#define ENC_RESET 0x35
#define DISABLE_TIMEOUT  0X38

// Capacitive definitions
#define TOUCHPIN A0
#define RESOLUTION 100
#define SMOOTH 100

// Robot dimensions
float WHEEL_RADIUS = 0.06;
float BASE_LENGTH = 0.37;
int MAX_PWM = 127;

// Strain-Gauge variables
const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 9; //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;
float sg_reading = 0;
bool storingSG = true;
float sg_avg = 0.0;
float sg_sum = 0.0;
float speedModifier = 1.0;
float sg_offset = 1000.0;
float lastSGReading[100] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0};


// Encoder
uint32_t encoder = 0;
byte enc1a, enc1b, enc1c, enc1d = 0;
int left_pwm = 128;
int right_pwm = 128;
float linear = 0;
float angular = 0;

// Capacitive variables
float multiplier = 1.02;
int previousReadings[SMOOTH];
int currentIndex = 0;
int reading;
bool isCapacitivePressed = false;
int falseCapacitiveCounter = 0;

// Vib motors variables
int motorA_PWM = 10; //Controle de velocidade do motor A (Esquerdo)
int motorB_PWM = 11; //Controle de velocidade do motor B (Direito)
int motorA_EN = 12; //Controle de direção do motor A (Esquerdo))
int motorB_EN = 13; //Controle de direção do motor B (Direito)
int velocidade = 127; //variável para controle de velocidade de rotação dos motores,sendo 0 o valor de velocidade mínimo e 255 o valor de velocidade máxima. 


// Button
const int buttonPin = 40;  // the number of the pushbutton pin
//const int ledPin = 13;    // the number of the LED pin
int buttonState = 0;
int pastButtonState = 0;
bool isActivated = false;
unsigned long pressedTime;

int clicks = 0;
unsigned long timePress = 0;
unsigned long timePressLimit = 0;

///********** DEFINIÇÃO DE TOPICOS ROS **********/
ros::NodeHandle  nh;

// Publishers
// left encoder publisher
std_msgs::Int32 leftEncoder;
ros::Publisher leftEncoderPublisher("left_encoder_pulses", &leftEncoder);

// right enocder publisher
std_msgs::Int32 rightEncoder;
ros::Publisher rightEncoderPublisher("right_encoder_pulses", &rightEncoder);

// strain gauge publisher
std_msgs::Float32 strainGaugeReading;
ros::Publisher strainGaugePublisher("strain_gauge", &strainGaugeReading);

// capacitive publisher
std_msgs::Bool isTouching;
ros::Publisher capacitivePublisher("capacitive", &isTouching);

// button publisher
std_msgs::Bool isActive;
ros::Publisher buttonPublisher("button", &isActive);

void leftVibCallback(const std_msgs::Int32 &left_vib_msg){
  int leftVibPWM = left_vib_msg.data;
  analogWrite(motorA_PWM, leftVibPWM);
}

void rightVibCallback(const std_msgs::Int32 &right_vib_msg){
  int rightVibPWM = right_vib_msg.data;
  analogWrite(motorB_PWM, rightVibPWM);
}
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  /* MD49 specifications
    RPM: from -116.5 to 116.6
    PWM: from 0 to 255  (the zero speed is the PWM 128)
    m/s: from -0.7326 to 0.7326 
  */
  // PWM calc variables

  linear = cmd_vel.linear.x;
  angular = cmd_vel.angular.z;
}

void resetEncoderCB(const std_msgs::Empty &command){
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);
}
ros::Subscriber<geometry_msgs::Twist> cmdVelSubscriber("cmd_vel", cmdVelCallback );
ros::Subscriber<std_msgs::Empty> encoderResetSubscriber("reset_encoder", resetEncoderCB);
ros::Subscriber<std_msgs::Int32> leftVibSubscriber("left_vib_msg", leftVibCallback);
ros::Subscriber<std_msgs::Int32> rightVibSubscriber("right_vib_msg", rightVibCallback);


void setup() {
// Serial
  Serial1.begin(9600);
  delay(50);
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);

  // ROS
  nh.initNode();
  nh.advertise(buttonPublisher);
  nh.advertise(leftEncoderPublisher);
  nh.advertise(rightEncoderPublisher);
  nh.advertise(strainGaugePublisher);
  nh.advertise(capacitivePublisher);
  nh.subscribe(cmdVelSubscriber);
  nh.subscribe(encoderResetSubscriber);
  nh.subscribe(leftVibSubscriber);
  nh.subscribe(rightVibSubscriber);

  // Capacitive
  for(int i = 0; i < SMOOTH; i++){
    previousReadings[i] = ADCTouch.read(TOUCHPIN, RESOLUTION);
  }

  // Strain-Gauge
  float calibrationValue; // calibration value
  calibrationValue = 696.0; // uncomment this if you want to set this value in the sketch
  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);

  if (LoadCell.getTareTimeoutFlag()) {
    // Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    // Serial.println("Startup is complete");
  }

  while (!LoadCell.update());

  // Vib motors
  pinMode(motorA_PWM, OUTPUT);
  pinMode(motorA_EN, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  // digitalWrite(motorA_PWM, 0);
  // digitalWrite(motorB_PWM, 0);
  delay(500);

// Button
//pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  nh.spinOnce();
  // read the state of the pushbutton value:
  buttonState = !digitalRead(buttonPin);
  
  isActive.data = (bool) buttonState;


  if(buttonState == 1 && pastButtonState ==0){
    pressedTime = millis();
  }

  if(buttonState == 0 && pastButtonState == 1){
    if(millis() - pressedTime > 1000){
      speedModifier += 0.25;
      if(speedModifier > 1.0){speedModifier = 1.0;}
    } else {
      isActivated = !isActivated;
    }
  };
  pastButtonState = buttonState;
  


  nh.spinOnce();
  
  /********** Encoder Reading ***********/
  // Read Left Encoder
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

  // Strain-Gauge
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      sg_reading = -LoadCell.getData();
      newDataReady = 0;
      t = millis();
    }
  }

  sg_avg = sg_sum/100;
  if(storingSG && sg_reading > sg_avg + sg_offset){
    storingSG = false;
    speedModifier -= 0.25;
    if(speedModifier < 0.25){speedModifier=0.25;}
  }

  if(!storingSG && sg_reading < sg_avg + sg_offset){
    storingSG = true;
  }

  if(storingSG){
    sg_sum = 0;
    for (byte i = 0; i < 100; i = i + 1) {
      if (i < 99){lastSGReading[i] = lastSGReading[i+1];
      } else {
        lastSGReading[i] = sg_reading;
      }
      sg_sum += lastSGReading[i];
    }
  }

  strainGaugeReading.data = sg_reading;
  nh.spinOnce();


  // Capacitive reading
  reading = ADCTouch.read(TOUCHPIN, RESOLUTION);
  if(reading > average() * multiplier){
    isCapacitivePressed = true;

  }else{
    isCapacitivePressed = false;
    previousReadings[currentIndex] = reading;
    currentIndex++;
    if(currentIndex >= SMOOTH){
      currentIndex = 0;
    }
  }
  isTouching.data = (bool) isCapacitivePressed;
  nh.spinOnce();

  if(!isCapacitivePressed){
    falseCapacitiveCounter += 1;
  } else {
    falseCapacitiveCounter = 0;
  }


  // Calculate target linear velocity for both wheels
  float right_linear = ((linear*2) + (angular*BASE_LENGTH))/2;
  float left_linear =  (linear*2) - right_linear;


  /// COLOCAR MODIFICADOR DE VELOCIDADE NA ANGULAR E LINEAR



  // Remap motor's speed into a PWM
  left_pwm = (int) round(((left_linear*speedModifier + 0.7326) * (255 - 0) / (0.7326 + 0.7326)));
  right_pwm = (int) round(((right_linear*speedModifier + 0.7326) * (255 - 0)/ (0.7326 + 0.7326)));

  if(left_pwm > 128+MAX_PWM){left_pwm = 128+MAX_PWM;}
  if(left_pwm < 128-MAX_PWM){left_pwm = 128-MAX_PWM;}
  if(right_pwm > 128+MAX_PWM){right_pwm = 128+MAX_PWM;}
  if(right_pwm < 128-MAX_PWM){right_pwm = 128-MAX_PWM;}

  

  // Execute only if capacitive sensor is being hold and the atcivation button has been pressed
  if(isCapacitivePressed && isActivated){
    Serial1.write(CMD);
    Serial1.write(SET_SPEED1);
    Serial1.write(left_pwm);
    
    Serial1.write(CMD);
    Serial1.write(SET_SPEED2);
    Serial1.write(right_pwm);
  } else if(falseCapacitiveCounter > 20 || !isActivated){
    Serial1.write(CMD);
    Serial1.write(SET_SPEED1);
    Serial1.write(128);
    
    Serial1.write(CMD);
    Serial1.write(SET_SPEED2);
    Serial1.write(128);

    isActivated = false;
  }

//  if(falseCapacitiveCounter > 20 || !isActivated){
//    Serial1.write(CMD);
//    Serial1.write(SET_SPEED1);
//    Serial1.write(128);
//    
//    Serial1.write(CMD);
//    Serial1.write(SET_SPEED2);
//    Serial1.write(128);
//
//    isActivated = false;
//  }

    /********** Publish data ***********/
  leftEncoderPublisher.publish(&leftEncoder);
  rightEncoderPublisher.publish(&rightEncoder);
  strainGaugePublisher.publish(&strainGaugeReading);
  capacitivePublisher.publish(&isTouching);
  buttonPublisher.publish(&isActive);

}

int average(){
  // calculate the sum of all previous readings
  unsigned long sum = 0;
  for(int i = 0; i < SMOOTH; i++){
    sum += previousReadings[i];
  }

  // return the sum divided by the number of elements
  // or, in other words, the average of all previous readings
  //Serial.println(sum/SMOOTH);
  return sum / SMOOTH;
}