#include <math.h>

// ROS libs
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// Capacitive
#include <ADCTouch.h>

//strain gauge:
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
int MAX_PWM = 40;

// Vibrational motors
int motorA_PWM = 10; //Controle de velocidade do motor A (Esquerdo)
int motorB_PWM = 11; //Controle de velocidade do motor B (Direito)
int motorA_EN = 12; //Controle de direção do motor A (Esquerdo))
int motorB_EN = 13; //Controle de direção do motor B (Direito)

// Strain Gauge
const int HX711_dout = 9; //mcu > HX711 dout pin
const int HX711_sck = 8; //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;
float i = 0.0;

// Capacitive variables
float multiplier = 1.1;
int previousReadings[SMOOTH];
int currentIndex = 0;
int reading;
bool isCapacitivePressed = false;

// Encoder
uint32_t encoder = 0;
byte enc1a, enc1b, enc1c, enc1d = 0;

/********** DEFINIÇÃO DE TOPICOS ROS **********/
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
  int left_pwm;
  int right_pwm;
  float linear;
  float angular;

  linear = cmd_vel.linear.x;
  angular = cmd_vel.angular.z;

  float right_linear = ((linear*2) + (angular*BASE_LENGTH))/2;
  float left_linear =  (linear*2) - right_linear;

  left_pwm = (int) round((left_linear + 0.7326) * (255 - 0) / (0.7326 + 0.7326));
  right_pwm = (int) round((right_linear + 0.7326) * (255 - 0) / (0.7326 + 0.7326));

  // Execute only if capacitive sensor is being hold
  if(isCapacitivePressed){
    Serial1.write(CMD);
    Serial1.write(SET_SPEED1);
    Serial1.write(left_pwm);
    
    Serial1.write(CMD);
    Serial1.write(SET_SPEED2);
    Serial1.write(right_pwm);
  } else {
    Serial1.write(CMD);
    Serial1.write(SET_SPEED1);
    Serial1.write(128);
    
    Serial1.write(CMD);
    Serial1.write(SET_SPEED2);
    Serial1.write(128);
  }
}

void resetEncoderCB(const std_msgs::Empty &command){
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSubscriber("cmd_vel", cmdVelCallback );
ros::Subscriber<std_msgs::Empty> encoderResetSubscriber("reset_encoder", resetEncoderCB);
ros::Subscriber<std_msgs::Int32> leftVibSubscriber("left_vib_msg", leftVibCallback);
ros::Subscriber<std_msgs::Int32> rightVibSubscriber("right_vib_msg", rightVibCallback);

void setup()
{
  // Serial.begin(9600);
  // SERIAL
  Serial1.begin(9600);
  Serial1.write(CMD);
  Serial1.write(ENC_RESET);

  // ROS
  nh.initNode();
  nh.advertise(leftEncoderPublisher);
  nh.advertise(rightEncoderPublisher);
  nh.advertise(strainGaugePublisher);
  nh.advertise(capacitivePublisher);
  nh.subscribe(cmdVelSubscriber);
  nh.subscribe(encoderResetSubscriber);
  nh.subscribe(leftVibSubscriber);
  nh.subscribe(rightVibSubscriber);

  // Vib motors
  pinMode (motorA_PWM, OUTPUT);
  pinMode (motorA_EN, OUTPUT);
  pinMode (motorB_PWM, OUTPUT);
  pinMode (motorB_EN, OUTPUT);

  // Capacitive
  // Fill the [previousReaings] array with readings
  for(int i = 0; i < SMOOTH; i++){previousReadings[i] = ADCTouch.read(TOUCHPIN, RESOLUTION);}

  // Strain-Gauge
  float calibrationValue; // calibration value
  calibrationValue = 696.0; // uncomment this if you want to set this value in the sketch
  LoadCell.begin();
  
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  /*if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }*/
  while (!LoadCell.update());
  /*Serial.print("Calibration value: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }*/
}

void loop()
{
  nh.spinOnce();

  /********** Capacitive Reading ***********/
  // read the sensor
  reading = ADCTouch.read(TOUCHPIN, RESOLUTION);

  // check if triggered
  if(reading > average() * multiplier){
    // executes if the sensor is triggered
    isCapacitivePressed = true;
  }else{
    // executes if the sensor is not triggered
    isCapacitivePressed = false; // Lock robot
    // use this reading to compensate for environmental changes (i.e. smoothing)
    previousReadings[currentIndex] = reading;
    currentIndex++;
    if(currentIndex >= SMOOTH){ currentIndex = 0; }
  }
  isTouching.data = (bool) isCapacitivePressed;
  nh.spinOnce();



  /********** Encoder Reading ***********/
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




  /********** Strain Gauge Reading ***********/
  static boolean newDataReady = 0;
  const int serialPrintInterval = 500; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      i = LoadCell.getData();
      newDataReady = 0;
      t = millis();
    }
  }
  strainGaugeReading.data = i;
  nh.spinOnce();


  /********** Publish data ***********/
  leftEncoderPublisher.publish(&leftEncoder);
  rightEncoderPublisher.publish(&rightEncoder);
  strainGaugePublisher.publish(&strainGaugeReading);
  capacitivePublisher.publish(&isTouching);


  nh.spinOnce();
  delay(500);
}


// calculate the average of the previous readings
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
