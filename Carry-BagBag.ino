#include <CurieBle.h>
#include "CurieImu.h"

BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService CarryBagBagService("180D"); // BLE Heart Rate Service

// BLE CarryBagBag Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic  CarryBagBagCharacteristic("2A37",                   // standard 16-bit characteristic UUID
                                                BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                                                                          // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                                                                          // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

enum
{
   enNotifyNone = 0
  ,enOmatase      // おまたせ
  ,enOdekake      // お出かけ
  ,enOwakare      // お別れ
  ,enOmukae       // おむかえ
  ,enEttaiiiiiii  // いたぃぃ
  ,enMatteeeeeee  // 待ってぇ
  ,enOhanashi     // お話
};

enum
{
   enHandleSwitchOFF = 0
  ,enHandleSwitchON
  ,enPushSwitchOFF
  ,enPushSwitchON
};

enum
{
   enCarryFall = 0
  ,enCarryStand
};

// 速度計算
float oxyz = 0;
float dxyz = 0;
bool f=1;
float velocity = 0;

// 経過時間
long previousMillis = 0;  // last time the heart rate was checked, in ms

// 通知
int previousNotify = enNotifyNone;
int currentNotify = enNotifyNone;
int flagOwakare = 0;
int flagOhanashi = 0;

// Switch
int handleSwitchPin = 7;
int pushSwitchPin = 8;

// 閾値
float ThresholdWalkSlow = 1.5f;
float ThresholdWalkMid = 1.0f;

void setup()
{
  Serial.begin(9600);
  pinMode(handleSwitchPin, INPUT);
  pinMode(pushSwitchPin, INPUT);
  
  startGyro();
  startBLE();
}

void startBLE()
{
  // set advertised local name and service UUID:
  blePeripheral.setLocalName("Carry-BagBag");
  blePeripheral.setAdvertisedServiceUuid(CarryBagBagService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(CarryBagBagService);
  blePeripheral.addAttribute(CarryBagBagCharacteristic);

  // begin advertising BLE service:
  blePeripheral.begin();

  Serial.println("Bluetooth device active, waiting for connections..."); 
}

void startGyro()
{
  CurieImu.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  if (CurieImu.testConnection()) {
    Serial.println("CurieImu connection successful");
  } else {
    Serial.println("CurieImu connection failed");
  }
  
  // The board must be resting in a horizontal position for 
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieImu.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieImu.autoCalibrateXAccelOffset(0);
  CurieImu.autoCalibrateYAccelOffset(0);
  CurieImu.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieImu.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getYAccelOffset());
  Serial.print("\t"); // -2359
  Serial.print(CurieImu.getZAccelOffset());
  Serial.print("\t"); // 1688
  Serial.print(CurieImu.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getZGyroOffset());

  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieImu.setGyroOffsetEnabled(true);
  CurieImu.setAccelOffsetEnabled(true); 
}

void loop()
{
#if 1
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();
  
  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central : ");
    // print the central's MAC address:
    Serial.println(central.address());
  
    // while the central is still connected to peripheral:
    while (central.connected())
    {
      // 1000ms phase check:
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 1000)
      {
        previousMillis = currentMillis;

        // Position
        if (getPosition() == enCarryFall)
        {
          // Switch
          if (checkHandleSwitch() == enHandleSwitchON && checkPushSwitch() == enPushSwitchON)
          {
            ;// no notify
          }
          else if (checkHandleSwitch() == enHandleSwitchON && checkPushSwitch() == enPushSwitchOFF)
          {
            currentNotify = enOwakare;// お別れ
            flagOwakare = 1;
          }
          else if (checkHandleSwitch() == enHandleSwitchOFF && checkPushSwitch() == enPushSwitchOFF)
          {
            int16_t ax, ay, az;         // raw gyro values
            float gx, gy, gz;

            // these methods (and a few others) are also available
            CurieImu.getAcceleration(&ax, &ay, &az);
            gz = convertRawAcceleration(az);
            if (gz >= 1.0)
            {
              currentNotify = enEttaiiiiiii;// いたぃぃ
            }
          }
          else if (checkHandleSwitch() == enHandleSwitchOFF && checkPushSwitch() == enPushSwitchON)
          {
            ;// no notify
          }
          // Speed
        }
        else
        {
          // Switch
          if (checkHandleSwitch() == enHandleSwitchON && checkPushSwitch() == enPushSwitchON)
          {
            ;
          }
          else if (checkHandleSwitch() == enHandleSwitchON && checkPushSwitch() == enPushSwitchOFF)
          {
            currentNotify = enOmatase;// おまたせ
          }
          else if (checkHandleSwitch() == enHandleSwitchOFF && checkPushSwitch() == enPushSwitchOFF)
          {
            if (flagOwakare != 0)
            {
              flagOwakare = 0;
              currentNotify = enOmukae;// おむかえ
              //Serial.println("おむかえ");              
            }
            else
            {
              if (currentNotify != enOhanashi && currentNotify != enMatteeeeeee)
              {
                currentNotify = enOdekake;// おでかけ
                //Serial.println("おでかけ");               
              }
              else
              {
                flagOhanashi = 1;
              }
            }
          }
          else if (checkHandleSwitch() == enHandleSwitchOFF && checkPushSwitch() == enPushSwitchON)
          {
            if (currentNotify != enOhanashi || flagOhanashi != 0)
            {
              currentNotify = enOhanashi;// おはなし
              previousNotify = enOdekake;
              flagOhanashi = 0;
            }
          }
          
          // Speed
          float speed = getSpeed();
          if(speed >= ThresholdWalkMid)
          {
            currentNotify = enMatteeeeeee;// 待ってぇ
          }
        }
        
        if (currentNotify != previousNotify)
        {

          switch(currentNotify)
          {
            case enOmatase:      // おまたせ
              Serial.println("おまたせ");
              break;
            case enOdekake:      // お出かけ
              Serial.println("おでかけ");
              break;              
            case enOwakare:      // お別れ
              Serial.println("お別れ");
              break;
            case enOmukae:       // おむかえ
              Serial.println("おむかえ");
              break;
            case enEttaiiiiiii:  // いたぃぃ
              Serial.println("いたぃぃ");
              break;
            case enMatteeeeeee:  // 待ってぇ
              Serial.println(" 待ってぇ");
              break;
            case enOhanashi:     // お話
              Serial.println("おはなし");
              break;
          };
          
          const unsigned char notifyCharArray[2] = { 0, (unsigned char)currentNotify };
          CarryBagBagCharacteristic.setValue(notifyCharArray, 2);
          previousNotify = currentNotify;
        }
      }
    }
    previousNotify = currentNotify;
  }
#else
  long currentMillis = millis();
  // if 1000ms have passed, check the heart rate measurement:
  if (currentMillis - previousMillis >= 1000)
  {
    previousMillis = currentMillis;
    #if 1 // 加速度情報
    float xyz = getXYZ();
    if (f==0)
    {
      dxyz += xyz - oxyz;
    }
    f = 0;
    velocity = abs(dxyz * 9.8f);
    oxyz = xyz;
    Serial.print("m/s:\t");
    Serial.println(velocity);
    
    if (digitalRead(handleSwitchPin))
    {
      Serial.println("switch on");
    }
    else
    {
      Serial.println("switch off");
    }
    #else
    Serial.println(currentNotify);
    const unsigned char notifyCharArray[2] = { 0, (unsigned char)currentNotify };
    CarryBagBagCharacteristic.setValue(notifyCharArray, 2);
    currentNotify = (currentNotify + 1) % 8;
    #endif
  }
#endif
}

float getSpeed()
{
  float xyz = getXYZ();
  if (f==0)
  {
    dxyz += xyz - oxyz;
  }
  f = 0;
  velocity = abs(dxyz * 9.8);// 単位(m/s)
  oxyz = xyz;

  return velocity;
}

int getPosition()
{
  int ret = 0;
  int16_t ax, ay, az;         // raw gyro values
  float gx, gy, gz;

  // these methods (and a few others) are also available
  CurieImu.getAcceleration(&ax, &ay, &az);
  gx = convertRawAcceleration(ax);
  gy = convertRawAcceleration(ay);
  gz = convertRawAcceleration(az);

  //Serial.println(gz);
  if (gz <= 0.5)
  {
    // きゃりーばぐばぐが立っている
    ret = enCarryStand;
  }
  else
  {
    // きゃりーばぐばぐが倒れている
    ret = enCarryFall;
  }

  return ret;  
}

int checkHandleSwitch()
{
  int ret = 0;
  
  if (digitalRead(handleSwitchPin))
  {
    ret = enHandleSwitchON;
//    Serial.println("HandleSwitch on");    
  }
  else
  {
    ret = enHandleSwitchOFF;
//    Serial.println("HandleSwitch off");    
  }
  
  return ret;
}

int checkPushSwitch()
{
  int ret = 0;
  
  if (digitalRead(pushSwitchPin))
  {
    ret = enPushSwitchON;
//    Serial.println("PushSwitch on");    
  }
  else
  {
    ret = enPushSwitchOFF;
//    Serial.println("PushSwitch off");
  }
  
  return ret;
}

float getXYZ()
{
  // Calculate the synthetic acceleration
  int16_t ax, ay, az;         // raw gyro values
  float gx, gy, gz;
  float gxyz;

  //CurieImu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  CurieImu.getAcceleration(&ax, &ay, &az);
  //CurieImu.getRotation(&gx, &gy, &gz);
  gx = convertRawAcceleration(ax);
  gy = convertRawAcceleration(ay);
  gz = convertRawAcceleration(az);
  // display tab-separated accel/gyro x/y/z values
/*  
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
*/
  gxyz = sqrt(pow(gx, 2.0f) + pow(gy, 2.0f) + pow(gz, 2.0f));  
/*
  Serial.print("\t");
  Serial.println(gxyz);
*/
  return gxyz;
}

float convertRawAcceleration(int aRaw)
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;

  return a;
}
