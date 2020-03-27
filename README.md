# Nissan-Leaf-Inverter-Controller
OpenSource Control Unit for the Nissan Leaf Gen 1 and Gen 2 inverter.<br>
PCB files in DesignSpark 8.1 format.
<br>
<br>
Software courtesy of Perttu Ahola.<br>
http://productions.8dromeda.net/c55-leaf-inverter-protocol.html
<br>
Slightly modified to run on an Arduino Due with an EV Shield.<br>
Hardware untested as of release.<br>

16/08/19 : V1 Hardware built and tested. Software in progress. Latest version uploaded. Works with BMW E46 throttle pedal and sends CAN to E46 body to make dash and other modules work as they should. Still in early alpha. 


19/08/19 : Bare PCBs available for those wishing to build their own controller :<br>
http://www.evbmw.com/index.php/evbmw-webshop/miscellaneous-bare-pcbs/leaf-vcm-bare

24/08/19 : Testing at full torque : https://www.youtube.com/watch?v=w1eDK8kHOu0


24/11/19 : BOM Updated with corrections.

27/03/20 : Uploaded new firmware with the following changes :

-Now supports the use of ESP8266 Olimex WiFi module and web display

-Supports ISA 100A CAN based shunt for battery monitoring

-Precharge and main contactor control
