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
String stringOne = "Hello String";
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

void setup()
{
Serial.begin(9600);
Serial1.begin(9600);
}
void loop()
{
if(Serial.available() > 0)
{
int roda = Serial.read();
if (roda == 102){
Serial1.write(CMD);
Serial1.write(SET_SPEED1);
Serial1.write(128);

Serial1.write(CMD);
Serial1.write(SET_SPEED2);
Serial1.write(128);
}
if(roda == 101) // roda esquerda
{
vel_e = Serial.parseInt();
Serial1.write(CMD);
Serial1.write(SET_SPEED1);
Serial1.write(vel_e);
}
if(roda == 100) // roda direita
{
vel_d = Serial.parseInt();
Serial1.write(CMD);
Serial1.write(SET_SPEED2);
Serial1.write(vel_d);
}
if(roda == 105)
{
Serial1.write(CMD);
Serial1.write(GET_ENC1); // Recieve encoder 1 value
delay(50);
if (Serial1.available() > 3)
{
enc1a = Serial1.read();
enc1b = Serial1.read();
enc1c = Serial1.read();
enc1d = Serial1.read();
}
int count = sizeof(enc1a);
char* chars;
encoder = (((uint32_t)enc1a << 24) +
((uint32_t)enc1b << 16) +
((uint32_t)enc1c << 8) +
((uint32_t)enc1d << 0));
Serial.println(encoder,DEC);
delay(300);
}
if(roda==106)



{
Serial1.write(CMD);
Serial1.write(GET_ENC2); // Recieve encoder 1 value
delay(50);
if (Serial1.available() > 3)
{
enc1a = Serial1.read();
enc1b = Serial1.read();
enc1c = Serial1.read();
enc1d = Serial1.read();
}
int count = sizeof(enc1a);
char* chars;
encoder = (((uint32_t)enc1a << 24) +
((uint32_t)enc1b << 16) +
((uint32_t)enc1c << 8) +
((uint32_t)enc1d << 0));
Serial.println(encoder,DEC);
delay(300);
}
}
}