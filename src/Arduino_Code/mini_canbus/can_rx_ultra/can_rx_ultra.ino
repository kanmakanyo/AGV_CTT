// CAN BUS recive data from 8 ultrasound
// Created by : Afghan

#include <mcp_can.h>
#include <SPI.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#define BAUD_ROS_SERIAL 500000
#include <sensor_msgs/Range.h>

// ROS Configure
// sensor_msgs::Range range_msg;
geometry_msgs::PoseWithCovarianceStamped range_msg;
ros::Publisher pub("/ultrasonic", &range_msg);
ros::NodeHandle  nh;

int ros_period = 20; // Milisecond, 20 Hz
unsigned long ros_time = 0; // Milisecond
// char frameid[] = "/ultrasound";

// Canbus
long unsigned int senderID;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
float ultra1, ultra2, ultra3, ultra4, ultra5, ultra6, ultra7, ultra8; 

#define CAN_INT 21                              // Set INT to pin 2
MCP_CAN CAN(53);                               // Set CS to pin 10


void setup()
{
  // ROS
  nh.getHardware()-> setBaud(BAUD_ROS_SERIAL);
  nh.initNode();
  nh.advertise(pub);

  // range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  // range_msg.header.frame_id =  frameid;
  // range_msg.field_of_view = 0.1;  // fake
  // range_msg.min_range = 0.0;
  // range_msg.max_range = 6.47;

  // Canbus
//  Serial.begin(115200);
  pinMode(27, OUTPUT);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  //pinMode(CAN_INT, INPUT);                            // Configuring pin for /INT input
}

void loop()
{
  //Serial.print("tes");
  // Variabel untuk menampung ukuran data yg akan diterima
  unsigned char dataLength = 0; 
  
  // Variabel yg digunakan untuk menampung modify progress bar qt5 designer data (2 byte data yg akan diterima)
  unsigned char dataBuffer[1]; 

  // Check jika ada data yg masuk
  if (CAN_MSGAVAIL == CAN.checkReceive())           
  {
    digitalWrite(27, HIGH);   // turn the LED on (HIGH is the voltage level)
    // Membaca data dari CAN Bus
    // Simpan ke variabel dataBuffer
    CAN.readMsgBuf(&senderID, &dataLength, dataBuffer);    
    // Membaca ID dari pengirim data
    // senderID = CAN.getCanId();
    
    // Jika data yang diterima dari ID = 1
    if (senderID == 1) 
    {
      ultra1 = (float)dataBuffer[0];
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra1);
      // Serial.println(" cm"); 
    }

    // Jika data yang diterima dari ID = 2
    else if(senderID == 2) 
    {
      ultra2 = (float)dataBuffer[0];
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra2);
      // Serial.println(" cm"); 
    }
    else if(senderID == 3) 
    {
      ultra3 = (float)dataBuffer[0];    
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra3);
      // Serial.println(" cm"); 
    }
    else if(senderID == 4) 
    {
      ultra4 = (float)dataBuffer[0];    
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra4);
      // Serial.println(" cm"); 
    }
    else if(senderID == 5) 
    {
      ultra5 = (float)dataBuffer[0];
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra5);
      // Serial.println(" cm"); 
    }
    else if(senderID == 6) 
    {
      ultra6 = (float)dataBuffer[0];
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra6);
      // Serial.println(" cm"); 
    }
    else if(senderID == 7) 
    {
      ultra7 = (float)dataBuffer[0];
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra7);
      // Serial.println(" cm"); 
    }
    else if(senderID == 8) 
    {
      ultra8 = (float)dataBuffer[0];
      // Serial.print("Ultra ID: ");
      // Serial.print(senderID);
      // Serial.print(" Data: ");
      // Serial.print(dataBuffer[0], HEX);
      // Serial.print(" ");
      // Serial.print(" Jarak: ");
      // Serial.print(ultra8);
      // Serial.println(" cm"); 
    }
  }
  // ROS
  if ((millis() - ros_time) >= ros_period) 
  {   // Publish the pub_msg
      ros_time = millis();
      range_msg.header.stamp = nh.now();
      range_msg.pose.covariance[0]=ultra1;
      range_msg.pose.covariance[1]=ultra2;
      range_msg.pose.covariance[2]=ultra3;
      range_msg.pose.covariance[3]=ultra4;
      range_msg.pose.covariance[4]=ultra5;
      range_msg.pose.covariance[5]=ultra6;
      range_msg.pose.covariance[6]=ultra7;
      range_msg.pose.covariance[7]=ultra8;
      pub.publish( &range_msg);
  }  
  nh.spinOnce();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
