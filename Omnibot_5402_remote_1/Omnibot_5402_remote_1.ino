#include <esp_now.h>
#include <WiFi.h>
#include <math.h>


// Pin Definitions
static const int joy1XPin = 35;
static const int joy1YPin = 34;
static const int joy2XPin = 36;

// Trim adjustments for non linear crap pots. #####Note to create auto trim on startup in future
const int joy1XTrim = 90;
const int joy1YTrim = 30;
const int joy2XTrim = 20;

float x;      // Joystick 1 X axis mapped as floating point decimal -1 to 1
float y;      // Joystick 1 Y axis mapped as floating point decimal -1 to 1
double turn;  // Joystick 2 X axis mapped as floating point decimal -1 to 1

double theta; // Requested angle from 45 degrees positive x-y
double Sin;   // Offset for -45 wheels
double Cos;   // Offset for 45 wheels
double Max;   // Tracker for max angle value to prevent square joystick values
double power; // Amount of difference between polar center and joystick position - used to calculate speed in any direction

// Receiver MAC address for espNow communication
uint8_t receiverMacAddress[] = {0x94,0xB9,0x7E,0xD4,0x0D,0x58};  //94:B9:7E:D4:0D:58

// espNow packet data structure
struct PacketData
{
  double leftfront;   // Calculated direction and speed for Left Front wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  double rightfront;  // Calculated direction and speed for Right Front wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  double leftrear;    // Calculated direction and speed for Left Rear wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  double rightrear;   // Calculated direction and speed for Right Rear wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
};
PacketData data; 

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t ");
  // Serial.println(status);
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  } 
     
}
 
void loop() {

    // Get analog values and map to floats between -1 and 1
  x = float(map(analogRead(joy1XPin)+joy1XTrim, 0, 4095, -1024, 1024)) / float(1024);
  y = float(map(analogRead(joy1YPin)+joy1YTrim, 0, 4095, -1024, 1024)) / float(1024);
  turn = float(map(analogRead(joy2XPin)+joy2XTrim, 0, 4095, -1024, 1024)) / float(1024);
  Serial.println(turn);
  
  // Base calculations
  theta = atan2(y,x);
  power = sqrt(x*x + y*y);
  Sin = sin(theta-PI/4);
  Cos = cos(theta-PI/4);
  Max = max(abs(Sin),abs(Cos));

  // Calculate raw proportional wheel rates and directions
  data.leftfront = power*Cos/Max + turn;
  data.rightfront = power*Sin/Max - turn;
  data.leftrear = power*Sin/Max + turn;
  data.rightrear = power*Cos/Max - turn;

  // Make sure that none of the values combined are greater than range -1 to 1 and if so, flatten
  if((power+abs(turn))>1) {
    data.leftfront /= power + abs(turn);
    data.rightfront /= power + abs(turn);
    data.leftrear /= power + abs(turn);
    data.rightrear /= power + abs(turn);
  }  

  // char inputValuesString[100];
  // sprintf(inputValuesString, 
  //         "Left Front: %3f  Right Front: %3f     Left Rear: %3f  Right Rear: %3f",
  //          data.leftfront,
  //          data.rightfront,
  //          data.leftrear,
  //          data.rightrear);
  // Serial.println(inputValuesString); 
  
  // Send the 4 wheel metrics to the receiver
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
  if (result == ESP_OK) 
  {
    //Serial.println("Sent with success");
  }
  else 
  {
    Serial.println("Error sending the data");
  }    
  
  delay(10);
}



