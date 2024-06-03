#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Adafruit_MCP23X17.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

//L298N driver pins
#define ltf1 3
#define ltf2 2
#define ltfpwm 27
#define ltr1 1
#define ltr2 0
#define ltrpwm 14
#define rtf1 7
#define rtf2 6
#define rtfpwm 12
#define rtr1 5
#define rtr2 4
#define rtrpwm 13

//L298N EN PWM attributes
const int ltfFreq = 5000;
const int ltfChannel = 2;
const int ltfResolution = 10;
const int ltrFreq = 5000;
const int ltrChannel = 3;
const int ltrResolution = 10;
const int rtfFreq = 5000;
const int rtfChannel = 4;
const int rtfResolution = 10;
const int rtrFreq = 5000;
const int rtrChannel = 5;
const int rtrResolution = 10;

const int leftServoPin = 25;
const int rightServoPin = 26;

const int MCP17Address = 0x21;

int servodata[][4] = {  // Min, Max, Home, current position
  {30, 150, 150, 150},    // Left servo
  {30, 150, 150, 0}     // Right servo
};

// The joysticks have a center point that isn't always stable
// These two variables mark the dead band area where the incoming signals jump around when the joysticks are centered
// Use values slightly above the fluctuation minimum and maximums observed
const float centerDeadbandLow = -.15;
const float centerDeadbandHigh = .15;

unsigned long lastRecvTime = 0; //variable to track how long it has been since a packet was received from the sender

// espNow packet data structure
struct PacketData
{
  double leftfront;   // Calculated direction and speed for Left Front wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  double rightfront;  // Calculated direction and speed for Right Front wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  double leftrear;    // Calculated direction and speed for Left Rear wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  double rightrear;   // Calculated direction and speed for Right Rear wheel (Negative values reverse -1 to 0; forward values are positive 0 to 1)
  int leftarm;        // Current position of the left arm shoulder servo
  int rightarm;       // Current position of the left arm shoulder servo
};
PacketData receiverData;
Adafruit_MCP23X17 mcp;
Servo LeftArm;
Servo RightArm;

//Assign default input received values
void setInputDefaultValues()
{
  receiverData.leftfront = 0;
  receiverData.rightfront = 0;
  receiverData.leftrear = 0;
  receiverData.rightrear = 0;
  receiverData.leftarm = servodata[0][3];
  receiverData.rightarm = servodata[0][3];
}

// Update the outputs with current values.
void mapAndWriteValues(){
  int ltf = 0;
  int ltr = 0;
  int rtf = 0;
  int rtr = 0;

  // Calculate the PWM values (integers between 0-1023) for each wheel from the data provided by the remote
  ltf = abs(int(receiverData.leftfront * 1023));
  ltr = abs(int(receiverData.leftrear * 1023));
  rtf = abs(int(receiverData.rightfront * 1023));
  rtr = abs(int(receiverData.rightrear * 1023));

  // For each wheel set the direction (ie ltf1 and ltf2) to the forward or reverse direction based on if the
  // wheel data is positive or negative and also if the wheel value is outside the center deadband

  // First the left front wheel
  if ((receiverData.leftfront < centerDeadbandLow) || (receiverData.leftfront > centerDeadbandHigh)){
    if (receiverData.leftfront > 0){    // If the value is positive then set pins for forward
      mcp.digitalWrite(ltf1, HIGH);
      mcp.digitalWrite(ltf2, LOW);
      ledcWrite(ltfChannel, ltf);
    } else {                            // Otherwise set the pins for reverse
      mcp.digitalWrite(ltf1, LOW);
      mcp.digitalWrite(ltf2, HIGH);
      ledcWrite(ltfChannel, ltf);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    mcp.digitalWrite(ltf1, LOW);
    mcp.digitalWrite(ltf2, LOW);
    ledcWrite(ltfChannel, 0);
  }
  
  // Next the left rear wheel
  if ((receiverData.leftrear < centerDeadbandLow) || (receiverData.leftrear > centerDeadbandHigh)){
    if (receiverData.leftrear > 0){    // If the value is positive then set pins for forward
      mcp.digitalWrite(ltr1, HIGH);
      mcp.digitalWrite(ltr2, LOW);
      ledcWrite(ltrChannel, ltr);
    } else {                            // Otherwise set the pins for reverse
      mcp.digitalWrite(ltr1, LOW);
      mcp.digitalWrite(ltr2, HIGH);
      ledcWrite(ltrChannel, ltr);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    mcp.digitalWrite(ltr1, LOW);
    mcp.digitalWrite(ltr2, LOW);
    ledcWrite(ltrChannel, 0);
  }
     
  // Next the right front wheel
  if ((receiverData.rightfront < centerDeadbandLow) || (receiverData.rightfront > centerDeadbandHigh)){
    if (receiverData.rightfront > 0){    // If the value is positive then set pins for forward
      mcp.digitalWrite(rtf1, HIGH);
      mcp.digitalWrite(rtf2, LOW);
      ledcWrite(rtfChannel, rtf);
    } else {                            // Otherwise set the pins for reverse
      mcp.digitalWrite(rtf1, LOW);
      mcp.digitalWrite(rtf2, HIGH);
      ledcWrite(rtfChannel, rtf);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    mcp.digitalWrite(rtf1, LOW);
    mcp.digitalWrite(rtf2, LOW);
    ledcWrite(rtfChannel, 0);
  }
  
  // Next the right rear wheel
  if ((receiverData.rightrear < centerDeadbandLow) || (receiverData.rightrear > centerDeadbandHigh)){
    if (receiverData.rightrear > 0){    // If the value is positive then set pins for forward
      mcp.digitalWrite(rtr1, HIGH);
      mcp.digitalWrite(rtr2, LOW);
      ledcWrite(rtrChannel, rtr);
    } else {                            // Otherwise set the pins for reverse
      mcp.digitalWrite(rtr1, LOW);
      mcp.digitalWrite(rtr2, HIGH);
      ledcWrite(rtrChannel, rtr);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    mcp.digitalWrite(rtr1, LOW);
    mcp.digitalWrite(rtr2, LOW);
    ledcWrite(rtrChannel, 0);
  }

  // **** ARMS
  Serial.print(receiverData.leftarm); Serial.print(" "); Serial.println(receiverData.rightarm);
  // Left Arm
  servodata[0][3] = receiverData.leftarm;
  if (servodata[0][3]<servodata[0][0]){
    servodata[0][3]=servodata[0][0];
  }
  if (servodata[0][3]>servodata[0][1]){
    servodata[0][3]=servodata[0][1];
  }
  LeftArm.write(servodata[0][3]);

  // Right Arm
  servodata[1][3] = receiverData.rightarm;
  if (servodata[1][3]<servodata[1][0]){
    servodata[1][3]=servodata[1][0];
  }
  if (servodata[1][3]>servodata[1][1]){
    servodata[1][3]=servodata[1][1];
  }
  RightArm.write(servodata[1][3]);

}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;                 // There was no data to do anything with - exit
  }
  //Copy the memory contained in the incoming data block to the Struct receiverData for use througout the program
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  mapAndWriteValues();      // Now that we have the new set of data from the remote, update the outputs 
  lastRecvTime = millis();  // Update our clock variable so that we don't allow a time out 
}

void setup() 
{
  Serial.begin(115200);

  if (!mcp.begin_I2C(MCP17Address)) {
    Serial.println("Error.");
    while (1);
  }

  // Turn on wifi in Station mode
  WiFi.mode(WIFI_STA);

  // Setup the output pins 
  mcp.pinMode(ltf1, OUTPUT);
  mcp.pinMode(ltf2, OUTPUT);
  mcp.pinMode(ltr1, OUTPUT);
  mcp.pinMode(ltr2, OUTPUT);
  mcp.pinMode(rtf1, OUTPUT);
  mcp.pinMode(rtf2, OUTPUT);
  mcp.pinMode(rtr1, OUTPUT);
  mcp.pinMode(rtr2, OUTPUT);

  // Setup the PWM channels
  ledcSetup(ltfChannel, ltfFreq, ltfResolution);
  ledcSetup(ltrChannel, ltrFreq, ltrResolution);
  ledcSetup(rtfChannel, rtfFreq, rtfResolution);
  ledcSetup(rtrChannel, rtrFreq, rtrResolution);

  // Setup the PWM pins
  ledcAttachPin(ltfpwm, ltfChannel);
  ledcAttachPin(ltrpwm, ltrChannel);
  ledcAttachPin(rtfpwm, rtfChannel);
  ledcAttachPin(rtrpwm, rtrChannel);

  // Setup arm servo pins
  LeftArm.attach(leftServoPin);
  RightArm.attach(rightServoPin);

  LeftArm.write(servodata[0][2]);
  RightArm.write(servodata[1][2]);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    // If a time out has occured, set the bot outputs to default safe values
    setInputDefaultValues();        // Set the pins to their default safe state
    mapAndWriteValues();            // Update the outputs to default safe states
    Serial.println("No Signal");  
  }
}