//Hack @ WPI 2022
//Project by: Mark Caleca and Issac Lau

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ESP32Servo.h> 
Servo myservo;  // create servo object to control a servo

//servo setup
// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33 
int servoPin = 18;      // GPIO pin used to connect the servo control (digital out)
// Possible ADC pins on the ESP32: 0,2,4,12-15,32-39; 34-39 are recommended for analog input
int potPin = 34;        // GPIO pin used to connect the potentiometer (analog in)
int ADC_Max = 4096;     // This is the default ADC max value on the ESP32 (12 bit ADC width);
                        // this width can be set (in low-level oode) from 9-12 bits, for a
                        // a range of max values of 512-4096
int servoReset = 15;     //reset position for servo, in degrees

//Bluetooth setup
String knownBLENames[] = {"Galaxy S9 M. C."};
int RSSI_THRESHOLD = -80; 
bool device_found;
int scanTime = 1; //In seconds
BLEScan* pBLEScan;
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      for (int i = 0; i < (sizeof(knownBLENames) / sizeof(knownBLENames[0])); i++)
      {
        if(strcmp(advertisedDevice.getName().c_str(),knownBLENames[i].c_str())==0)
                        {
          device_found = true;
                          break;
                        }
        else
          device_found = false;
      }
      //Serial.printf("Advertised Device: %s %d\n", advertisedDevice.toString().c_str(), device_found);
    }
};
void setup() {
  Serial.begin(115200); //Enable UART on ESP32

  //Bluetooth setup
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(),false); //Init Callback Function FIX
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100); // set Scan interval
  pBLEScan->setWindow(99);  // less or equal setInterval value

  //Servo setup
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(servoPin, 500, 2400);   // attaches the servo on pin 18 to the servo object
                                         // using SG90 servo min/max of 500us and 2400us
                                         // for MG995 large servo, use 1000us and 2000us,
                                         // which are the defaults, so this line could be
                                         // "myservo.attach(servoPin);"
}

bool phoneDetected(){
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  for (int i = 0; i < foundDevices.getCount(); i++)
  {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    int rssi = device.getRSSI();
    String deviceName = device.getName().c_str();
    bool found = deviceName==knownBLENames[0].c_str();
    if (rssi > RSSI_THRESHOLD && found){
      return true;}
    else if (found){
      return true;}
  }
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  return false;
}
int readServo(){
  int val = analogRead(potPin);
  val = map(val, 0, ADC_Max, 0, 180);     
  return val;
}
bool withinError(int secureAngle){return abs(readServo()- secureAngle)<5;}
void servoResetPos(){
  myservo.write(servoReset);
  delay(250);
  digitalWrite(servoPin,LOW);
}

bool confirmServoInput(int secureAngle){
  if(withinError(secureAngle)){
    delay(2000);
    if(withinError(secureAngle)){
      servoResetPos();
      return true;
    }
    else {
      return false;
    }
  }
  else if(readServo()<secureAngle){
    delay(200);
    confirmServoInput(secureAngle);
  }
  else {
    return false;
  }
}

bool servoTurnCombo(){
   return confirmServoInput(90);
}

void loop() {
  Serial.println("Please log in to the system");
  servoResetPos();
  if(phoneDetected()&&servoTurnCombo()) {
    Serial.println("Logged In");
    delay(5000);
    }
  else {
    Serial.println("Sorry, your login attempt failed, try again.");
    //Serial.printf("false :( %d \n", readServo());    
    }
}

//Special thanks to https://circuitdigest.com/microcontroller-projects/ble-based-proximity-control-using-esp32 and ESP32 servo sample code
