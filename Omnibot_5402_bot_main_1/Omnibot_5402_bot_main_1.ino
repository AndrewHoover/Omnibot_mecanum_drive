#include <esp_now.h>
#include <WiFi.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

//L298N driver pins
#define ltf1 13
#define ltf2 12
#define ltfpwm 32
#define ltr1 27
#define ltr2 33
#define ltrpwm 14
#define rtf1 17
#define rtf2 16
#define rtfpwm 5
#define rtr1 19
#define rtr2 18
#define rtrpwm 21

//L298N EN PWM attributes
const int ltfFreq = 5000;
const int ltfChannel = 0;
const int ltfResolution = 10;
const int ltrFreq = 5000;
const int ltrChannel = 1;
const int ltrResolution = 10;
const int rtfFreq = 5000;
const int rtfChannel = 2;
const int rtfResolution = 10;
const int rtrFreq = 5000;
const int rtrChannel = 3;
const int rtrResolution = 10;

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
  
};
PacketData receiverData;

//Assign default input received values
void setInputDefaultValues()
{
  receiverData.leftfront = 0;
  receiverData.rightfront = 0;
  receiverData.leftrear = 0;
  receiverData.rightrear = 0;
  
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
      digitalWrite(ltf1, HIGH);
      digitalWrite(ltf2, LOW);

      ledcWrite(ltfChannel, ltf);
    } else {                            // Otherwise set the pins for reverse
      digitalWrite(ltf1, LOW);
      digitalWrite(ltf2, HIGH);
      ledcWrite(ltfChannel, ltf);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    digitalWrite(ltf1, LOW);
    digitalWrite(ltf2, LOW);
    ledcWrite(ltfChannel, 0);
  }
  
  // Next the left rear wheel
  if ((receiverData.leftrear < centerDeadbandLow) || (receiverData.leftrear > centerDeadbandHigh)){
    if (receiverData.leftrear > 0){    // If the value is positive then set pins for forward
      digitalWrite(ltr1, HIGH);
      digitalWrite(ltr2, LOW);
      ledcWrite(ltrChannel, ltr);
    } else {                            // Otherwise set the pins for reverse
      digitalWrite(ltr1, LOW);
      digitalWrite(ltr2, HIGH);
      ledcWrite(ltrChannel, ltr);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    digitalWrite(ltr1, LOW);
    digitalWrite(ltr2, LOW);
    ledcWrite(ltrChannel, 0);
  }
     
  // Next the right front wheel
  if ((receiverData.rightfront < centerDeadbandLow) || (receiverData.rightfront > centerDeadbandHigh)){
    if (receiverData.rightfront > 0){    // If the value is positive then set pins for forward
      digitalWrite(rtf1, HIGH);
      digitalWrite(rtf2, LOW);
      ledcWrite(rtfChannel, rtf);
    } else {                            // Otherwise set the pins for reverse
      digitalWrite(rtf1, LOW);
      digitalWrite(rtf2, HIGH);
      ledcWrite(rtfChannel, rtf);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    digitalWrite(rtf1, LOW);
    digitalWrite(rtf2, LOW);
    ledcWrite(rtfChannel, 0);
  }
  
  // Next the right rear wheel
  if ((receiverData.rightrear < centerDeadbandLow) || (receiverData.rightrear > centerDeadbandHigh)){
    if (receiverData.rightrear > 0){    // If the value is positive then set pins for forward
      digitalWrite(rtr1, HIGH);
      digitalWrite(rtr2, LOW);
      ledcWrite(rtrChannel, rtr);
    } else {                            // Otherwise set the pins for reverse
      digitalWrite(rtr1, LOW);
      digitalWrite(rtr2, HIGH);
      ledcWrite(rtrChannel, rtr);
    }
  } else {                              // if the wheels are inside the deadband then turn them off altogether
    digitalWrite(rtr1, LOW);
    digitalWrite(rtr2, LOW);
    ledcWrite(rtrChannel, 0);
  }

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

  // Turn on wifi in Station mode
  WiFi.mode(WIFI_STA);

  // Setup the output pins 
  pinMode(ltf1, OUTPUT);
  pinMode(ltf2, OUTPUT);
  pinMode(ltr1, OUTPUT);
  pinMode(ltr2, OUTPUT);
  pinMode(rtf1, OUTPUT);
  pinMode(rtf2, OUTPUT);
  pinMode(rtr1, OUTPUT);
  pinMode(rtr2, OUTPUT);

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