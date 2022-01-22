/* Author: Junsu Kang */

#include <DynamixelSDK.h>


#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

// SDK Protocol
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define ADDR_VEL_LIM                    44
#define ADDR_TORQUE_ENABLE              64
#define ADDR_OPERATING_MODE             11
#define ADDR_GOAL_VELOCITY              104

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define VELOCITY_MODE                   1

#define BAUDRATE_COMM  115200
#define BAUDRATE  1000000
#define DXL_ID_L    1
#define DXL_ID_R    2

#define VEL_LIM 1000 // velocity limit is 330 by default for XM430-W210. Use Dynamixel SDK to change this.

bool check_result(dynamixel::PacketHandler *packetHandler, int dxl_comm_result, uint8_t dxl_error){
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
    return false;
  }
    return true;
}

// Serial Packet Communication
#define CMD_DISABLE   0
#define CMD_ENABLE    1
#define CMD_VEL_LIM   2
#define CMD_VEL_TAR   3
#define CMD_DEBUG     100
#define PACKET_LEN    8
#define PACKET_SOP    0x55
#define PACKET_EOP    0xAA

bool debug_mode=false;

bool read_packet(int* command, int* value1, int* value2, char* buffer){
  int first_byte;
  first_byte = Serial.peek();
  if (first_byte<0){ // no serial input - no command processing
    return false;
  }
  Serial.readBytes(buffer, PACKET_LEN);
  if (first_byte!=PACKET_SOP){
    if(debug_mode){
      Serial.print("Wrong SOP: ");
      Serial.print(first_byte);
      Serial.print(" / ");
      Serial.println(PACKET_SOP); 
    }
    return false;
  }
  if(buffer[7] != PACKET_EOP){
    if(debug_mode){
      Serial.print("Wrong EOP: ");
      Serial.print(buffer[7]);
      Serial.print(" / ");
      Serial.println(PACKET_EOP);
    }
    return false;
  }
  int checksum = (buffer[1]+buffer[2]+buffer[3]+buffer[4]+buffer[5])%0x100;
  if(checksum != buffer[6]){
    if(debug_mode){
      Serial.print("Check sum fail: ");
      Serial.print(buffer[6]);
      Serial.print(" / ");
      Serial.println(checksum);
    }
    return false;
  }
  *command = buffer[1];
  *value1 = buffer[2]*0x100+buffer[3];
  *value1 = (*value1)%0x8000-((*value1)/0x8000)*0x8000;
  *value2 = buffer[4]*0x100+buffer[5];
  *value2 = (*value2)%0x8000-((*value2)/0x8000)*0x8000;
  return true;
}

#define MIN(A,B) (A > B ? B:A)
#define MAX(A,B) (A > B ? A:B)
#define CLIP_V(vel, vel_lim) (int32_t) (MAX(MIN(vel, vel_lim), -vel_lim))

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

int dxl_comm_result = COMM_TX_FAIL;             // Communication result

int vel_lim=VEL_LIM;
uint8_t dxl_error = 0;                          // Dynamixel error
int32_t dxl_present_position = 0;               // Present position

void setup() 
{
  Serial.begin(BAUDRATE_COMM);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler->openPort())
  {
    if(debug_mode){
      Serial.println("Succeeded to open the port!");
    }
  }
  else
  {
    if(debug_mode){
      Serial.println("Failed to open the port!");
    }
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    if(debug_mode){
      Serial.println("Succeeded to change the baudrate!");
    }
  }
  else
  {
    if(debug_mode){
      Serial.println("Failed to change the baudrate!");
    }
    return;
  }
}

void loop() 
{
  bool ret;
  int command, value1, value2;
  char buffer[PACKET_LEN];
  
  ret = read_packet(&command, &value1, &value2, buffer);
  if(debug_mode){
    delay(500); 
  }
  if(debug_mode){
      Serial.println("");
      Serial.println("--------------------");
      for(int i=0; i<PACKET_LEN; i++){
        Serial.print((int)buffer[i]); 
        Serial.print(" ");
      }
      Serial.println("");
      Serial.println("--------------------");
  }
  if(!ret){
    return;
  }

  switch(command){
    case CMD_DISABLE:
    
      // Disable Dynamixel Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_L, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Left Dynamixel has been successfully disabled");
        }
      }
    
      // Disable Dynamixel Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_R, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Right Dynamixel has been successfully disabled");
        }
      }
      break;
    case CMD_ENABLE:
      // Change left vel lim
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_L, ADDR_VEL_LIM, VEL_LIM, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println(
            "Left Velocity Limit Update");
        }
      }
      
      // Set velocity mode
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_L, ADDR_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Left Wheel Mode Set");
        }
      }
    
      // Enable Dynamixel Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_L, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Left Dynamixel has been successfully connected");
        }
      }
    
      // Change right vel lim
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_R, ADDR_VEL_LIM, VEL_LIM, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Right Velocity Limit Update");
        }
      }
      
      // Set velocity mode
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_R, ADDR_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Right Wheel Mode Set");
        }
      }
    
      // Enable Dynamixel Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_R, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.println("Right Dynamixel has been successfully connected");
        }
      }
      break;
    case CMD_VEL_LIM:
      vel_lim = value1;
      // Change left vel lim
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_L, ADDR_VEL_LIM, vel_lim, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.print("Left Velocity Limit Update to ");
          Serial.println(vel_lim);
        }
      }
      // Change right vel lim
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_R, ADDR_VEL_LIM, vel_lim, &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.print("Right Velocity Limit Update to ");
          Serial.println(vel_lim);
        }
      }
      break;
    case CMD_VEL_TAR:
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_L, ADDR_GOAL_VELOCITY, CLIP_V(value1, vel_lim), &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.print("Left Velocity Set to ");
          Serial.println(value1);
        }
      }
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_R, ADDR_GOAL_VELOCITY, CLIP_V(value2, vel_lim), &dxl_error);
      if (check_result(packetHandler, dxl_comm_result, dxl_error)){
        if(debug_mode){
          Serial.print("Right Velocity Set to ");
          Serial.println(value2);
        }
      }
      break;
    case CMD_DEBUG:
      debug_mode = value1;
      break;
  }
  Serial.write(buffer, PACKET_LEN);
  delay(10); 
}
