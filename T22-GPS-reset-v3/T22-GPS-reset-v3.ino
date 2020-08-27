/*************************************************************************
Original sketch by LilyGO 
https://github.com/LilyGO/TTGO-T-Beam/tree/master/GPS-T22_v1.0-20190612

Modified by ErikThevking on August 25, 2020. Updated on August 27, 2020.

Purpose:
This sketch will bring back U-blox GPS N6M & N8M NMEA 9600 baud serial
on TTGO T-beam T22- V1.0 and 1.1 and also on 0.7 T-beams (uncomment yours)

Thanks to Kizniche for his advice on
https://github.com/kizniche/ttgo-tbeam-ttn-tracker/issues/20

Based on SparkFun's Ublox Arduino Library and examples
https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
where you can download the necessary SparkFun library.

For T22_v1.0 20190612 and the T22_v1.1 20191212 boards
As the power management chipset changed, it
requires the axp20x library that can be found  // AXP192 for T-beam; AXP202 for T-watch
https://github.com/lewisxhe/AXP202X_Library
**************************************************************************/

// Select your board version
//#define T_BEAM_V07  // AKA Rev0 (first board released)
#define T_BEAM_V10  // AKA Rev1 for board versions T-beam_V1.0 and V1.1 (second board released)

#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10)
#include <Wire.h>
#include <axp20x.h>
AXP20X_Class axp;
#define I2C_SDA         21
#define I2C_SCL         22
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
int state = 0; // steps through states
HardwareSerial SerialGPS(1);


void setup()
{
  Serial.begin(115200);
  while (!Serial);  // Wait for user to open the terminal
  Serial.println("Connected to Serial");
  Wire.begin(I2C_SDA, I2C_SCL);

  #if defined(T_BEAM_V10)
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
    } else {
        Serial.println("AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // provides power to GPS backup battery
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // enables power to ESP32 on T-beam
    axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); // I foresee similar benefit for restting T-watch 
                                                 // where ESP32 is on DCDC3 but remember to change I2C pins and GPS pins!
  #endif 
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("All comms started");
  delay(100);

  do {
    if (myGPS.begin(SerialGPS)) {
      Serial.println("Connected to GPS");
      myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("GPS serial connected, output set to NMEA");
      myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
      myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("Enabled/disabled NMEA sentences");
      break;
    }
    delay(1000);
  } while(1);

}  // endofsetup

void loop()
{

  if (SerialGPS.available()) {
    Serial.write(SerialGPS.read());  // print anything comes in from the GPS
  }

  
}  // endofloop
