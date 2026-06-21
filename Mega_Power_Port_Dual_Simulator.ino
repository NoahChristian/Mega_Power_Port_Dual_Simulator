//This is a simple power port simulator for two channels
//
//Written by Noah Christian (C) 2026
//MIT License
//This code is provided "as is", without warranty of any kind, express or implied. In no event shall the authors be liable for any claim, damages or other liability, whether in an action of contract, tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in the software.
//The purpose of this code is to simulate the output of a dual-channel power port, which is commonly used in grid-tied solar or battery inverters. The simulator generates random power values for two channels (L1 and L2) and outputs them over two separate serial ports in a specific format. The current power values are also displayed on an OLED screen for easy monitoring. This can be useful for testing and debugging communication interfaces with controllers or software that interact with such power ports.
//
//Use case: Debugging and testing with hardware-in-the-loop
//
//Uses some code from the Adafruit SSD1306 library examples for display output
//Including the little arduino splash screen
//The simulator outputs random power values for two channels (L1 and L2) at a rate of 2 Hz, with a correction factor applied to simulate real-world conditions. The output is sent over two separate serial ports (Serial1 and Serial2) in a specific format, and the current power values are displayed on an OLED screen. The code also includes version information and a simple display of the simulator's name and version on startup.
//for hardware, wire two RS-485 converters to UART 1 and 2 output
//This duplicates the message format for the Y-H (or Y&H) grid tied solar or battery inverter
//Use for debugging either controller or software interfaces
//
//Note: The random power values are generated and output on the UARTs as well as the display.
//
//Use as a hardware debugger for testing power port communication
//
//Version history:
//Version 1.0.0 - output on two ports random numbers at 2 Hz rate
//Version 1.0.1 - bug fixes from 1.0.0
//Version 1.0.2 - send both messages out of phase


//Add Display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// Pin connected to onboard LED
const int ledPin = 13;

String receivedMessage;
String sendMessage;
String receivedMessage_Hex;

#define VERSION_POWER "1.0.2"
//version 1.0.0 - output on two ports random numbers at 2 Hz rate
//version 1.0.1 - bug fixes from 1.0.0
//version 1.0.2 - send both messages out of phase

bool verbose=false;

uint8_t buffer1[8];
uint8_t highByte1,lowByte1,cksum1;
uint8_t buffsize1;
uint8_t receivedChar1;
bool messageValid1 = false;
bool bSendMessage1 = true;
uint8_t head1 = 0, tail1 = 0;

uint8_t buffer2[8];
uint8_t highByte2,lowByte2,cksum2;
uint8_t buffsize2;
uint8_t receivedChar2;
bool messageValid2 = false;
bool bSendMessage2 = true;
bool bDisplayMessage = false;
uint8_t head2 = 0, tail2 = 0;
const float L1_CorrectionFactor=0.683966;
const float L2_CorrectionFactor=0.683966;
uint16_t L1_Power = 0;
uint16_t L1_LastPower = 0;
uint16_t L2_Power = 0;
float L1_CorrectedPower = 0.0;
float L2_CorrectedPower = 0.0;

uint16_t L1_State = 0;
uint16_t L2_State = 0;

// uint16_t L1 = 0;
// uint16_t L2 = 0;

unsigned int Power1=0;
unsigned int SetpointPower1=0;
unsigned int PowerCk1=0;
unsigned int Power2=0;
unsigned int SetpointPower2=0;
unsigned int PowerCk2=0;

//Screen Settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64

//Screen other settings
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


void setup() {
  pinMode(ledPin, OUTPUT); // Set pin 13 as output

  // initialize both serial ports:
  //Serial.begin(9600); //the COM PORT on USB Monitor
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial2.begin(4800);
  
    //Display Setup
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();

  displayVersion();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.display();
  delay(2000);

  Serial.print("Power Port Simulator Version ");Serial.println(VERSION_POWER);
}

unsigned long currentTime=millis();
unsigned long previousTime=millis();

void loop() {

  currentTime=millis();
  if (currentTime > (previousTime + 500))
  {
    bSendMessage1 = true;
    bSendMessage2 = true;
    Power1 = random(900);
    Power2 = random(900);
    L1_CorrectedPower = Power1 * L1_CorrectionFactor;
    L2_CorrectedPower = Power2 * L2_CorrectionFactor;
    previousTime = millis();
  }
  if (bSendMessage1){
    //Algorithm for control test on L1
    uint16_t L1Out = (uint16_t) L1_CorrectedPower;
    highByte1 = trunc(L1Out / 256);
    lowByte1 = ( L1Out - highByte1*256 );
    cksum1 = 264 - ((highByte1 + lowByte1) & 0xFF);
    {
      //Serial.print(highByte); Serial.print(";"); Serial.print(lowByte);Serial.print("\n");
      
      buffer1[0]=0x24; buffer1[1]=0x56;
      buffer1[2]=0x0; buffer1[3]='!'; buffer1[4]=highByte1;
      buffer1[5]=lowByte1; buffer1[6]=0x80;
      buffer1[7]=cksum1;
      // for(int i=0;i++;i<8)
      {
        Serial1.write(buffer1, 8);  //write outputs bytes at low level, not null terminated
        //add SetpointPower1
        Serial.print("L1 = ");Serial.print(L1Out);
      }
      //sendMessage = (char) (buffer)
    }
    messageValid1=false; //reset status
    tail1 = 0;
    bSendMessage1=false;
    bDisplayMessage=true;
  }
  delay(250);
  if (bSendMessage2){
    //Algorithm for copy test on L2
    highByte2 = trunc(L2Out / 256);
    lowByte2 = ( L2Out - highByte2*256 );
    cksum2 = 264 - ((highByte2 + lowByte2) & 0xFF);
    {
      //Serial.print(highByte); Serial.print(";"); Serial.print(lowByte);Serial.print("\n");
      
      buffer2[0]=0x24; buffer2[1]=0x56;
      buffer2[2]=0x0; buffer2[3]='!'; buffer2[4]=highByte2;
      buffer2[5]=lowByte2; buffer2[6]=0x80;
      buffer2[7]=cksum2;
      // for(int i=0;i++;i<8)
      {
        Serial2.write(buffer2, 8);  //write outputs bytes at low level, not null terminated
        Serial.print(" L2 = ");Serial.println(L2Out);
      }
      //sendMessage = (char) (buffer)
    }
    messageValid2=false; //reset status
    tail2 = 0;
    bSendMessage2=false;
    bDisplayMessage=true;
  }
  if(bDisplayMessage) {
    displayValues();
    bDisplayMessage=false;
  }
  
}

void displayValues() {
  char buffer[50]; // Ensure the buffer is large enough
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  sprintf(buffer, "L1 out =\n %d W\nL2 out =\n %d W\n", (uint16_t) L1_CorrectedPower, (uint16_t) L2_CorrectedPower);
  display.write(buffer);
  display.display();
}

void displayVersion() {
  char buffer[50]; // Ensure the buffer is large enough
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  sprintf(buffer, "Power Port\n Simulator\n V%s",VERSION_POWER);
  display.write(buffer);
  display.display();
}
