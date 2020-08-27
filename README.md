# TTGO_T-beam_GPS-reset
Reset U-blox NEO GPS devices on TTGO T-beam T22- V1.0 and 1.1 to bring back NMEA output on GPS serial line.

Original sketch by LilyGO
https://github.com/LilyGO/TTGO-T-Beam/tree/master/GPS-T22_v1.0-20190612

Modified by ErikThevking on August 25, 2020. Updated on August 27, 2020.


Purpose:
The sketch T22-GPS-reset-v3 will bring back U-blox GPS N6M & N8M NMEA 9600 baud serial
on TTGO T-beam T22- V1.0 and 1.1 and also on 0.7 T-beams (uncomment yours).
In case you're having trouble because your T-beam's GPS doesn't show NMEA output anymore, you should first try this v3 sketch, which is not restoring factory settings. Normally this is sufficient to bring your NMEA output back on serial.

Thanks to Kizniche for his advice on
https://github.com/kizniche/ttgo-tbeam-ttn-tracker/issues/20

In case you still don't get your NMEA back, you might need something more drastic, like sketch T22-GPS-restoreFactory-v2. This will reset U-blox NEO GPS devices on TTGO T-beam T22- V1.0 and 1.1. It will bring back U-blox GPS N6M & N8M factory settings 
so that NMEA 9600 over the GPS serial output is enabled.

Based on SparkFun's Ublox Arduino Library and examples
https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
where you can download the necessary SparkFun library.
