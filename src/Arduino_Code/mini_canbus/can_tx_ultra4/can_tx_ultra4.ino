// CAN Send Example
//

#include <mcp_can.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#define NODE_ID 4
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MCP_CAN CAN0(10);     // Set CS to pin 10
SoftwareSerial mySerial(7,8); // RX, TX
unsigned char data_ultra[4]={};
int distance;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600); 
  pinMode(3, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) 
  {
    display.clearDisplay();    //clear display
    display.setTextSize(1,2);
    display.setCursor(10,0);
    display.print("CP2515 Initialized Successfully");
    Serial.println("MCP2515 Initialized Successfully!");
    
  }
  else 
  {
    display.clearDisplay();    //clear display
    display.setTextSize(1,2);
    display.setCursor(10,0);
    display.print("Error Initializing MCP2515");
    Serial.println("Error Initializing MCP2515...");
    digitalWrite(A3, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);
    digitalWrite(A3, LOW);
    delay(1000);
  }

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

//int data1 = 23;

void loop()
{
  
  // Ultra 
  do{
     for(int i=0;i<4;i++)
     {
       data_ultra[i]=mySerial.read();
     }
  }while(mySerial.read()==0xff);
  
//  mySerial.flush();

  if(data_ultra[0]==0xff)
    {
      //Serial.print("tes");
      int sum;
      sum=(data_ultra[0]+data_ultra[1]+data_ultra[2])&0x00FF;
      //Serial.print(sum);
      if(sum==data_ultra[3])
      {
        distance=((data_ultra[1]<<8)+data_ultra[2])/10;
      }
     }
     //delay(100);
 
  // Canbus
  Serial.println(distance);
  
  unsigned char ultraNodeData[1] = {0};
  ultraNodeData[0] = distance;
  CAN0.sendMsgBuf(NODE_ID, 0, 1, ultraNodeData);

  delay(100);

  //Display
  display.clearDisplay();    //clear display
  display.setTextSize(1,2);
  display.setCursor(30,0);
  display.print("Ultrasonik 4: ");
  display.setTextSize(4);
  display.setCursor(0,30);
  display.print(distance);
  display.print(" ");
  display.setTextSize(1);
  display.cp437(true);
  //display.write(167);
  display.setTextSize(2);
  display.print("CM");

  display.display(); 
  
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
