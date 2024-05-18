#include <Arduino.h>
#include "ELMduino.h"
#include "BluetoothSerial.h"
#include <HardwareSerial.h>

// BluetoothSerial SerialBT;
// #define ELM_PORT   SerialBT
HardwareSerial ELM_PORT(1);
#define DEBUG_PORT Serial
#define ESP_BLUETOOTH_NAME "ESP32"

const bool DEBUG        = false;
const int  TIMEOUT      = 2000;
const bool HALT_ON_FAIL = false;

//String MACadd = "66:1E:32:1E:F2:2B";                         //enter the ELM327 MAC address
  // 00:00:00:00:00:01
uint8_t address[6]  = {0x66, 0x1E, 0x32, 0x1E, 0xF2, 0x2B};  //enter the ELM327 MAC address after the 0x

ELM327 myELM327;

typedef enum { ENG_RPM,
               SPEED,
               BATTVOLT,
               COOLTEMP,
               ODOMETER,
               FUELLEVEL } obd_pid_states;
obd_pid_states obd_state = ODOMETER;

struct PIDinfo
{
  uint8_t pid;
  const char* command;
  const char* name;
};

unordered_set<uint8_t> PIDsSupported;
float rpm = 0;
float mph = 0;
float volts = 0;
float coolantTemp = 0;
float mileage = 0;
float fuelLevel = 0;
int i;
int j=0;
bool pidOK;
uint8_t pid;
const PIDinfo pids[] = {
  {12, "ENGINE_RPM", nullptr},
  {13, "VEHICLE_SPEED", nullptr},
  {255, "READ_VOLTAGE", "AT RV"},
  {5, "ENGINE_COOLANT_TEMP", nullptr},
  {166, "ODOMETER_MILEAGE", nullptr},
  {0x2F, "FUEL_TANK_LEVEL_INPUT", nullptr}
};

void setup()
{
  // Serial.begin(115200);
  // ELM_PORT.begin(115200);
  DEBUG_PORT.begin(115200);
  // ELM_PORT.begin(9600,SERIAL_8N1,16,17); // Set for STN1110 configuration

  Serial.println("Attempting to connect to ELM327...");

  // ELM_PORT.begin(ESP_BLUETOOTH_NAME, true);
  
  // if (!ELM_PORT.connect(address))
  // {
  //   Serial.println("Couldn't connect to OBD scanner with Address");
    // while (1);
  // }
  ELM_PORT.begin(9600,SERIAL_8N1,16,17);
  if (!myELM327.begin(ELM_PORT, DEBUG, TIMEOUT))
  {
    Serial.println("Couldn't connect to OBD scanner with myELM327");
    // while (1);
  } else {Serial.println("Connected with myELM327 object");}

  Serial.println("Connected to ELM327");
  
  //Check if all sensors are supported
  Serial.println("Checking Sensor Support...");
  
    // Serial.println("Supported PIDs:");
    // PIDsSupported = getSupportedPIDs();
    // for (it = PIDsSupported.begin(); it != SupportedPIDs.end(); ++it) {
    //     Serial.println(*it);
    // }
  // for (i = 0; i < (sizeof(pids)/ sizeof(pids[0])); i++) {
  //   pid = pids[i].pid;
  //   if (pid == 255) {
  //     DEBUG_PORT.println("skipping command");
  //     continue;
  //   }
  //   DEBUG_PORT.print("PID "); DEBUG_PORT.print(pid); DEBUG_PORT.println(" check begin.");
  //   pidOK = myELM327.isPidSupported(pid);
  //   // DEBUG_PORT.print("PID "); DEBUG_PORT.print(pid);
  //   //   if (pidOK) {
  //   //     DEBUG_PORT.println(" is supported");
  //   //   } else {DEBUG_PORT.println(" is not supported.");}
  //   Serial.print("beginning Nb_rx_state: "); Serial.println(myELM327.nb_rx_state);

  //   if (myELM327.nb_rx_state == ELM_SUCCESS)
  //   {
  //     DEBUG_PORT.print("ELM Responded to PID "); DEBUG_PORT.println(pid);
  //     if(pidOK) {
  //         Serial.print("PID "); Serial.print(pid); Serial.println(" is supported");
  //     }
  //     else 
  //     {
  //         Serial.print("PID "); Serial.print(pid); Serial.println(" is not supported");
  //     }
  //   }
  //   else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
  //   {
  //       DEBUG_PORT.println("ELM Responded with an error.");
  //       myELM327.printError();
  //   } 
  //   else { DEBUG_PORT.println("Did not get proper message from Car.");}  
  // }

  DEBUG_PORT.println("Done checking Sensors");
  delay(2000);
  Serial.println("Reporting Sensor data...");
}


void loop()
{

  switch (obd_state)
  {
    case ENG_RPM:
    {
      rpm = myELM327.rpm();
      
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        Serial.print("rpm: ");
        Serial.println(rpm);
        obd_state = SPEED;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
        myELM327.printError();
        obd_state = SPEED;
      }
      break;
    }
    
    case SPEED:
    {
      mph = myELM327.mph();
      
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        Serial.print("mph: ");
        Serial.println(mph);
        obd_state = BATTVOLT;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
        myELM327.printError();
        obd_state = BATTVOLT;
      }
      break;
    }

    case BATTVOLT:
    {
      float volts = myELM327.batteryVoltage();

      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
          Serial.print("Battery Voltage: ");
          Serial.println(volts);
          obd_state = COOLTEMP;
          // delay(10000);
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
          myELM327.printError();
          obd_state = COOLTEMP;
      }
      break;
    }

    case COOLTEMP: 
    {
      float coolantTemp = myELM327.engineCoolantTemp();

      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
          Serial.print("Engine Coolant Temp: ");
          Serial.print(coolantTemp);
          Serial.println(" F");
          obd_state = ODOMETER;
          // delay(10000);
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
          myELM327.printError();
          obd_state = ODOMETER;
      }
      break;
    }

    case ODOMETER:
    {
      mileage = myELM327.odometerReading();

      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
          Serial.print("Milage: ");
          Serial.println(mileage);
          obd_state = FUELLEVEL;
          // delay(10000);
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
          myELM327.printError();
          obd_state = FUELLEVEL;
          
      }
      break;
    }
    case FUELLEVEL:
    {
      fuelLevel = myELM327.fuelLevel();

      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
          Serial.print("Fuel Level: ");
          Serial.print(fuelLevel);
          Serial.println("%");
          obd_state = ENG_RPM;
          // obd_state = ODOMETER;
          // delay(10000);
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
          myELM327.printError();
          obd_state = ENG_RPM;
          // obd_state = ODOMETER;
      }
      break;
    }
  }
  // delay(500);
}






// HardwareSerial ELM_PORT(1);
// #define DEBUG_PORT Serial
// #define ESP_BLUETOOTH_NAME "ESP32"


// void setup()
// {
//   // Serial.begin(115200);
//   // ELM_PORT.begin(115200);
//   DEBUG_PORT.begin(115200);

//   ELM_PORT.begin(9600,SERIAL_8N1,16,17); // Set the baud rate to match your STN1110 configuration

//   // SerialBT.setPin("1234");
//   Serial.println("Attempting to connect to ELM327...");
//   // ELM_PORT.begin(ESP_BLUETOOTH_NAME, true);
//   if (ELM_PORT) {
//     DEBUG_PORT.println("ELM Connected");
//   } else {DEBUG_PORT.println("ELM couldnt connect");}
  
  