/*   Hoverboard_Serial_Test
 *   Controls the speed, brake, and direction of a single hoverboard motor
 *   via commands sent through the serial port.
 *   Measures the speed of a hoverboard motor asynchronously
 *   using a custom ReadSpeed function.  Uses the SC speed pulse output of the
 *   RioRand 400W 6-60V PWM DC Brushless Electric Motor Speed Controller with Hall.
 *   Outputs the speed data to the serial port.
 *     
 *   created 2021
 *   Mad-EE  (Mad Electrical Engineer)
 *   www.mad-ee.com
 *   
 *   This example code is in the public domain.
 *   
 *   Platform:  Arduino UNO
 */

// Constants
const unsigned long SPEED_TIMEOUT = 500000;
const unsigned int UPDATE_TIME = 500;             
const unsigned int BUFFER_SIZE = 16;              
const double BAUD_RATE = 9600;                  
const double WHEEL_DIAMETER_IN = 6.8;            
const double WHEEL_CIRCUMFERENCE_IN = 10.51181;     
const double WHEEL_DIAMETER_CM = 17;           
const double WHEEL_CIRCUMFERENCE_CM = 26.70;      

// pinagem driver esquerdo
const int PIN_DIRe = 2;      
const int PIN_BRAKEe = 3;    
const int PIN_PWMe = 9;     
const int PIN_SPEEDe = 8;   

// pinagem driver direito
const int PIN_DIRd = 4;      
const int PIN_BRAKEd = 6;    
const int PIN_PWMd = 10;      
const int PIN_SPEEDd = 11;   
String _command = "";       
int _data = 0;              
int pulsoe = 0;
int pulsod = 0;           
// variaveis auxiliares
bool DIRE = true;
bool DIRD = false;

// This is ran only once at startup
void setup() 
{
    // Set pin directions
    pinMode(PIN_SPEEDe, INPUT);
    pinMode(PIN_PWMe, OUTPUT);
    pinMode(PIN_BRAKEe, OUTPUT);
    pinMode(PIN_DIRe, OUTPUT);

    pinMode(PIN_SPEEDd, INPUT);
    pinMode(PIN_PWMd, OUTPUT);
    pinMode(PIN_BRAKEd, OUTPUT);
    pinMode(PIN_DIRd, OUTPUT);
    
    // Set initial pin states
    digitalWrite(PIN_BRAKEe, false);
    digitalWrite(PIN_DIRe, DIRE);
    analogWrite(PIN_PWMe, 0);

    digitalWrite(PIN_BRAKEd, false);
    digitalWrite(PIN_DIRd, DIRD);
    analogWrite(PIN_PWMd, 0);
    
    // Initialize serial port
    Serial.begin(BAUD_RATE);
    while (!Serial){;}
    // Serial.println("---- Program Started ----");
}

// This is the main program loop that runs repeatedly
void loop() 
{
    // Read serial data and set dataReceived to true if command is ready to be processed
    Serial.flush();
    bool dataReceived = ReadFromSerial();

    // Process the received command if available
    if (dataReceived == true)
        ProcessCommand(_command, _data);

    // Read the speed from input pin (sets _rpm and _mph)
    ReadSpeed();

    // Outputs the speed data to the serial port 
    WriteToSerial(); 
}

// Receives string data from the serial port
// Data should be in the format <command>,<data>
// Data should be terminated with a carriage return
// Function returns true if termination character received 
bool ReadFromSerial()
{    
    // Local variables   
    static String cmdBuffer;        // Stores the received command
    static String dataBuffer;       // Stores the received data
    static bool isCommand = true;   // Flag to store received bytes in command or data buffer
    byte recByte;                   // Byte received from the serial port
    
    // Check if any new data is available, if not exit
    if (Serial.available() == false)
      return false;
    
    // Read single byte from serial port
    recByte = Serial.read();
    // Serial.print(recByte);
    
    // Check if byte is termination character (carriage return)
    if (recByte == '\r')
    {
        // Save buffers to global variables
        cmdBuffer.toUpperCase();
        _command = cmdBuffer;
        _data = dataBuffer.toInt();
      
        // Write what was received back to the serial port
        // Serial.print("Received: "); 
        // Serial.print(_command); 
        // Serial.print(",");
        // Serial.println(_data);
      
        // Clear local variables
        cmdBuffer = "";
        dataBuffer = "";
        isCommand = true;

        return true;
    }
    
    // Check if byte is a comma which separates the command from the data
    if ((char)recByte == ',')
    {
        isCommand = false;  // Next byte will be a data byte
        return false;
    }

    // Save data to one of the receive buffers
    if (isCommand)
        cmdBuffer += (char)recByte;
    else
        dataBuffer += (char)recByte;
    
    return false;
}

// Processes the command and data, sends result to serial port
void ProcessCommand(String command, int data)
{ 
  //  Serial.println("OK");

    // Process SPEED command
    if (command == "PWM")
    {
      // Serial.print("Setting speed:  ");
      // Serial.println(data);
      analogWrite(PIN_PWMd, data);
      analogWrite(PIN_PWMe, data);
    }

    if (command == "PWME")
    {
      // Serial.print("Setting left speed:  ");
      // Serial.println(data);
      analogWrite(PIN_PWMe, data);
    }

    if (command == "PWMD")
    {
      // Serial.print("Setting right speed:  ");
      // Serial.println(data);
      analogWrite(PIN_PWMd, data);
    }
        
    // Process BRAKE command
    if (command == "BRAKE")
    {
      // Serial.print("Setting brake:  ");
      // Serial.println(data);
      digitalWrite(PIN_BRAKEe, data);
      digitalWrite(PIN_BRAKEd, data);
    }

    // Process DIR command
    if (command == "DIR")
    {
      // Serial.print("Setting direction:  ");
      // Serial.println(data);
      DIRE = !data;
      DIRD = data;
      digitalWrite(PIN_DIRd, DIRD);
      digitalWrite(PIN_DIRe, DIRE);
    }

    // Process DIR command
    if (command == "DIRE")
    {
      // Serial.print("Setting left direction:  ");
      // Serial.println(data);
      DIRE = !data;
      digitalWrite(PIN_DIRe, DIRE);
    }

    if (command == "DIRD")
    {
      // Serial.print("Setting right direction:  ");
      // Serial.println(data);
      DIRD = data;
      digitalWrite(PIN_DIRd, DIRD);
    }
}

// Reads the speed from the input pin and calculates RPM and MPH
// Monitors the state of the input pin and measures the time (µs) between pin transitions
void ReadSpeed()
{
    static bool lastStatee = false;    // Saves the last state of the speed pin
    static bool lastStated = false;
    static unsigned long last_uSe;     // The time (µs) when the speed pin changes
    static unsigned long timeout_uSe;  // Timer used to determine the wheel is not spinning
    static unsigned long last_uSd;     // The time (µs) when the speed pin changes
    static unsigned long timeout_uSd;

    // Read the current state of the input pin
    bool statee = digitalRead(PIN_SPEEDe);
    bool stated = digitalRead(PIN_SPEEDd);

    // Check if the pin has changed state
    if (statee != lastStatee)
    {
      // Calculate how long has passed since last transition
      unsigned long current_uSe = micros();
      unsigned long elapsed_uSe = current_uSe - last_uSe;
      if(DIRE)
      {
        pulsoe++;
      } else {
        pulsoe--;
      };

      // Save the last state and next timeout time
      last_uSe = current_uSe;
      timeout_uSe = last_uSe + SPEED_TIMEOUT;
      lastStatee = statee;
    }
    if (stated != lastStated)
    {
      // Calculate how long has passed since last transition
      unsigned long current_uSd = micros();
      unsigned long elapsed_uSd = current_uSd - last_uSd;
      if(!DIRD)
      {
        pulsod++;
      } else {
        pulsod--;
      };

      // Save the last state and next timeout time
      last_uSd = current_uSd;
      timeout_uSd = last_uSd + SPEED_TIMEOUT;
      lastStated = stated;
    }
    // If too long has passed then the wheel has probably stopped
    if (micros() > timeout_uSd)
    {
        last_uSd = micros();
    }

    if (micros() > timeout_uSe)
    {
        last_uSe = micros();
    }
}

// Writes the RPM and MPH to the serial port at a set interval
void WriteToSerial()
{
    // Local variables
    static unsigned long updateTime;
    
    if (millis() > updateTime)
    {
        // Write data to the serial port

        // {"pd" : 13127, "pe" : 45}
        Serial.println( "{"+String(char(34)) + "pe"+String(char(34))+":" + String(pulsoe) + "," + 
                            String(char(34)) + "pd"+String(char(34))+":" + String(pulsod) + "}");

        // Calculate next update time
        updateTime = millis() + UPDATE_TIME;
    }
}