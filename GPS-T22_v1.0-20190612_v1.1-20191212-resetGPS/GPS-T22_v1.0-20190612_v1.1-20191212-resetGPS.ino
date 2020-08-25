/*************************************************************************
Original sketch by LilyGO
https://github.com/LilyGO/TTGO-T-Beam/tree/master/GPS-T22_v1.0-20190612

Modified by ErikThevking on August 25, 2020.

Purpose: to reset U-blox NEO GPS devices on TTGO T-beam T22- V1.0 and 1.1 
This sketch will bring back U-blox GPS N6M & N8M factory settings 
so that NMEA 9600 over the GPS serial output is enabled.

Based on SparkFun's Ublox Arduino Library and examples
https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
where you can download the necessary SparkFun library.
**************************************************************************/

/*****************************************
  ESP32 GPS VKEL 9600 Bds
This version is for T22_v1.0 20190612          // T22_v01 20190612 board
and the T22_v1.1 20191212 boards               // Vking added line
As the power management chipset changed, it
require the axp20x library that can be found
https://github.com/lewisxhe/AXP202X_Library
You must import it as gzip in sketch submenu
in Arduino IDE
This way, it is required to power up the GPS
module, before trying to read it.

Also get TinyGPS++ library from: 
https://github.com/mikalhart/TinyGPSPlus
******************************************/

//#include <TinyGPS++.h>
#include <axp20x.h>

//TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

int state = 0; // steps through auto-baud, reset, etc states


void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
//  GPS.begin(38400, SERIAL_8N1, 34, 12);
  Serial.println("All comms started");

            delay(100);

  
}  // endofsetup

void loop()
{


    Serial.print("===== STATE ");
    Serial.println(state);
    switch (state) {
    case 0: // auto-baud connection, then switch to 38400 and save config
        do {
            Serial.println("GPS: trying 38400 baud");
            GPS.begin(38400, SERIAL_8N1, 34, 12);
            if (myGPS.begin(GPS)) break;

            delay(100);
            Serial.println("GPS: trying 9600 baud");
            GPS.begin(9600, SERIAL_8N1, 34, 12);
            if (myGPS.begin(GPS)) {
                Serial.println("GPS: connected at 9600 baud, switching to 38400");
                myGPS.setSerialRate(38400);
                delay(100);
            } else {
                delay(2000); //Wait a bit before trying again to limit the Serial output flood
            }
        }  while(1);
        myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
        myGPS.saveConfiguration(); //Save the current settings to flash and BBR
        Serial.println("GPS serial connected, saved config");
        state++;
        break;
        
    case 1: // hardReset, expect to see GPS back at 38400 baud
        Serial.println("Issuing hardReset (cold start)");
        myGPS.hardReset();
        delay(3000);
        GPS.begin(38400, SERIAL_8N1, 34, 12);
        if (myGPS.begin(GPS)) {
            Serial.println("Success.");
            state++;
        } else {
            Serial.println("*** GPS did not respond at 38400 baud, starting over.");
            state = 0;
        }
        break;
        
    case 2: // factoryReset, expect to see GPS back at 9600 baud
        Serial.println("Issuing factoryReset");
        myGPS.factoryReset();
        delay(3000); // takes more than one second... a loop to resync would be best
        GPS.begin(9600, SERIAL_8N1, 34, 12);
        if (myGPS.begin(GPS)) {
            Serial.println("Success.");
            state++;
        } else {
            Serial.println("*** GPS did not come back at 9600 baud, starting over.");
            state = 0;
        }
        break;
        
    case 3: // print version info
        Serial.print("GPS protocol version: ");
        Serial.print(myGPS.getProtocolVersionHigh());
        Serial.print('.');
        Serial.println(myGPS.getProtocolVersionLow());
        Serial.println();
        state++;
 //   }
    
    case 4: // print position info
    
    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
    Serial.println();
    
 //       state++;
    delay(3000);
    }
    
    delay(3000);

  
  /*
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.print("Speed     : ");
  Serial.println(gps.speed.kmph()); 
  Serial.println("**********************");

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
  } while (millis() - start < ms);
  */

  
}  // endofloop
