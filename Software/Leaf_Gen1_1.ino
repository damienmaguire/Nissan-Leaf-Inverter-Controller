/*
Leaf Gen1 Inverter driver. Alpha software for testing.
Runs on the Arduino Due SAM3X8E MCU.
Enter torque request on serial window.
As of now only responds to negative torque requests. e.g. -10
Positive torque requests trigger the inverter pwm but do not rotate the motor.


Copyright 2019 
Perttu Ahola (all the hard work!)
http://productions.8dromeda.net/c55-leaf-inverter-protocol.html

Damien Maguire (copy and paste).
OpenSource VCU hardware design available on Github :
https://github.com/damienmaguire/Nissan-Leaf-Inverter-Controller

2011 Nisan Leaf Gen 1 EV CAN logs on Github:
https://github.com/damienmaguire/LeafLogs


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.



*/

#include <due_can.h>  
#include <due_wire.h> 
#include <DueTimer.h>  
#include <Wire_EEPROM.h> 



#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }


CAN_FRAME outFrame;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    //structure to keep inbound inFrames


#define Throttle A0
int16_t final_torque_request = 0;
#define INVERTER_BITS_PER_VOLT 2
#define INVERTER_BITS_PER_RPM 2


struct InverterStatus {
  uint16_t voltage = 0;
  int16_t speed = 0;
  int8_t inverter_temperature = 0;
  int8_t motor_temperature = 0;
  bool error_state = false;
} inverter_status;

String readString;



void setup() 
  {
  Can0.begin(CAN_BPS_500K);   // Inverter CAN
  Can1.begin(CAN_BPS_500K);   // Vehicle CAN
  Can0.watchFor();
  Can1.watchFor();
    
    Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

    Timer3.attachInterrupt(Msgs10ms).start(10000); // mcucommand2 every 10ms

    Timer4.attachInterrupt(Msgs100ms).start(100000); // mcucommand1 every 100ms
    



  

 
  }

  
  
void loop()
{ 

delay(10);
checkCAN();

  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() >0) {
   // Serial.println(readString);  //so you can see the captured string
    final_torque_request = readString.toInt();  //convert readString into a number

  }

        readString=""; //empty for next input
}





void Msgs10ms()                       //10ms messages here
{

  static uint8_t counter_11a_d6 = 0;
  static uint8_t counter_1d4 = 0;
  static uint8_t counter_1db = 0;

  // Send VCM gear selection signal (gets rid of P3197)
  

        outFrame.id = 0x11a;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 3 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request



    
 //   CAN_FRAME inFrame;
  //  inFrame.id = 0x11a;
  //  inFrame.length = 8;

    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: 4E400055 0000017D

    // All possible gen1 values: 00 01 0D 11 1D 2D 2E 3D 3E 4D 4E
    // MSB nibble: Selected gear (gen1/LeafLogs)
    //   0: some kind of non-gear before driving
    //   1: some kind of non-gear after driving
    //   2: R
    //   3: N
    //   4: D
    // LSB nibble: ? (LeafLogs)
    //   0: sometimes at startup, not always; never when the
    //      inverted is powered on (0.06%)
    //   1: this is the usual value (55% of the time in LeafLogs)
    //   D: seems to occur for ~90ms when changing gears (0.2%)
    //   E: this also is a usual value, but never occurs with the
    //      non-gears 0 and 1 (44% of the time in LeafLogs)


    
    outFrame.data.bytes[0] = 0x4E;
    //outFrame.data.bytes[0] = 0x01;

    // 0x40 when car is ON, 0x80 when OFF, 0x50 when ECO
    outFrame.data.bytes[1] = 0x40;

    // Usually 0x00, sometimes 0x80 (LeafLogs), 0x04 seen by canmsgs
    outFrame.data.bytes[2] = 0x00;

    // Weird value at D3:4 that goes along with the counter
    // NOTE: Not actually needed, you can just send constant AA C0
    const static uint8_t weird_d34_values[4][2] = {
      {0xaa, 0xc0},
      {0x55, 0x00},
      {0x55, 0x40},
      {0xaa, 0x80},
    };
    outFrame.data.bytes[3] = weird_d34_values[counter_11a_d6][0];
    outFrame.data.bytes[4] = weird_d34_values[counter_11a_d6][1];

    // Always 0x00 (LeafLogs, canmsgs)
    outFrame.data.bytes[5] = 0x00;

    // A 2-bit counter
    outFrame.data.bytes[6] = counter_11a_d6;

    counter_11a_d6++;
    if(counter_11a_d6 >= 4)
      counter_11a_d6 = 0;

    // Extra CRC
    nissan_crc(outFrame.data.bytes, 0x85);

    /*Serial.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    Serial.println();*/

 Can1.sendFrame(outFrame);
 
    
  
  
  // Send target motor torque signal
  

        outFrame.id = 0x1d4;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 3 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request

    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: F70700E0C74430D4

    // Usually F7, but can have values between 9A...F7 (gen1)
    outFrame.data.bytes[0] = 0xF7;
    // 2016: 6E
   // outFrame.data.bytes[0] = 0x6E;

    // Usually 07, but can have values between 07...70 (gen1)
    outFrame.data.bytes[1] = 0x07;
    // 2016: 6E
    //outFrame.data.bytes[1] = 0x6E;

    // Requested torque (signed 12-bit value + always 0x0 in low nibble)
    static int16_t last_logged_final_torque_request = 0;
    if(final_torque_request != last_logged_final_torque_request){
      last_logged_final_torque_request = final_torque_request;
      //log_print_timestamp();
      Serial.print(F("Sending torque request "));
      Serial.print(final_torque_request);
      Serial.print(F(" (speed: "));
      Serial.print(inverter_status.speed / INVERTER_BITS_PER_RPM);
      Serial.print(F(" rpm)"));
      Serial.print(inverter_status.voltage / INVERTER_BITS_PER_VOLT);
      Serial.print(F(" Volts)"));
      Serial.println();
    }
    if(final_torque_request >= -2048 && final_torque_request <= 2047){
      outFrame.data.bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) |
          ((final_torque_request >> 4) & 0x7f);
      outFrame.data.bytes[3] = (final_torque_request << 4) & 0xf0;
    } else {
      outFrame.data.bytes[2] = 0x00;
      outFrame.data.bytes[3] = 0x00;
    }

    // MSB nibble: Runs through the sequence 0, 4, 8, C
    // LSB nibble: Precharge report (precedes actual precharge
    //             control)
    //   0: Discharging (5%)
    //   2: Precharge not started (1.4%)
    //   3: Precharging (0.4%)
    //   5: Starting discharge (3x10ms) (2.0%)
    //   7: Precharged (93%)
    outFrame.data.bytes[4] = 0x07 | (counter_1d4 << 6);
    //outFrame.data.bytes[4] = 0x02 | (counter_1d4 << 6);

    counter_1d4++;
    if(counter_1d4 >= 4)
      counter_1d4 = 0;

    // MSB nibble:
    //   0: 35-40ms at startup when gear is 0, then at shutdown 40ms
    //      after the car has been shut off (6% total)
    //   4: Otherwise (94%)
    // LSB nibble:
    //   0: ~100ms when changing gear, along with 11A D0 b3:0 value
    //      D (0.3%)
    //   2: Reverse gear related (13%)
    //   4: Forward gear related (21%)
    //   6: Occurs always when gear 11A D0 is 01 or 11 (66%)
    //outFrame.data.bytes[5] = 0x44;
    //outFrame.data.bytes[5] = 0x46;

    // 2016 drive cycle: 06, 46, precharge, 44, drive, 46, discharge, 06
    // 0x46 requires ~25 torque to start
    //outFrame.data.bytes[5] = 0x46;
    // 0x44 requires ~8 torque to start
    outFrame.data.bytes[5] = 0x44;

    // MSB nibble:
    //   In a drive cycle, this slowly changes between values (gen1):
    //     leaf_on_off.txt:
    //       5 7 3 2 0 1 3 7
    //     leaf_on_rev_off.txt:
    //       5 7 3 2 0 6
    //     leaf_on_Dx3.txt:
    //       5 7 3 2 0 2 3 2 0 2 3 2 0 2 3 7
    //     leaf_on_stat_DRDRDR.txt:
    //       0 1 3 7
    //     leaf_on_Driveincircle_off.txt:
    //       5 3 2 0 8 B 3 2 0 8 A B 3 2 0 8 A B A 8 0 2 3 7 
    //     leaf_on_wotind_off.txt:
    //       3 2 0 8 A B 3 7
    //     leaf_on_wotinr_off.txt:
    //       5 7 3 2 0 8 A B 3 7
    //     leaf_ac_charge.txt:
    //       4 6 E 6
    //   Possibly some kind of control flags, try to figure out
    //   using:
    //     grep 000001D4 leaf_on_wotind_off.txt | cut -d' ' -f10 | uniq | ~/projects/leaf_tools/util/hex_to_ascii_binary.py
    //   2016:
    //     Has different values!
    // LSB nibble:
    //   0: Always (gen1)
    //   1:  (2016)

    // 2016 drive cycle:
    //   E0: to 0.15s
    //   E1: 2 messages
    //   61: to 2.06s (inverter is powered up and precharge
    //                 starts and completes during this)
    //   21: to 13.9s
    //   01: to 17.9s
    //   81: to 19.5s
    //   A1: to 26.8s
    //   21: to 31.0s
    //   01: to 33.9s
    //   81: to 48.8s
    //   A1: to 53.0s
    //   21: to 55.5s
    //   61: 2 messages
    //   60: to 55.9s
    //   E0: to end of capture (discharge starts during this)

    // This value has been chosen at the end of the hardest
    // acceleration in the wide-open-throttle pull, with full-ish
    // torque still being requested, in
    //   LeafLogs/leaf_on_wotind_off.txt
    //outFrame.data.bytes[6] = 0x00;

    // This value has been chosen for being seen most of the time
    // when, and before, applying throttle in the wide-open-throttle
    // pull, in
    //   LeafLogs/leaf_on_wotind_off.txt
    //outFrame.data.bytes[6] = 0x30;

    // Value chosen from a 2016 log
    //outFrame.data.bytes[6] = 0x61;

    // Value chosen from a 2016 log
    // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
    outFrame.data.bytes[6] = 0x01;

    // Extra CRC
    nissan_crc(outFrame.data.bytes, 0x85);

    /*Serial.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    Serial.println();*/

 Can1.sendFrame(outFrame);

//We need to send 0x1db here with voltage measured by inverter

        outFrame.id = 0x1db;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 3 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=0x00;  
        outFrame.data.bytes[1]=0x00;
        outFrame.data.bytes[2]=0x00;
        outFrame.data.bytes[3]=0x00;
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0x00;
        outFrame.data.bytes[6]=counter_1db;
        outFrame.data.bytes[7]=0x00;


    counter_1db++;
    if(counter_1db >= 4)
      counter_1db = 0;

        

        Can1.sendFrame(outFrame);

 
    }
    



void Msgs100ms()                      ////100ms messages here
{

        outFrame.id = 0x50b;            // Set our transmission address ID
        outFrame.length = 7;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
    // Statistics from 2016 capture:
    //     10 00000000000000
    //     21 000002c0000000
    //    122 000000c0000000
    //    513 000006c0000000

    // Let's just send the most common one all the time
    // FIXME: This is a very sloppy implementation
  //  hex_to_data(outFrame.data.bytes, "00,00,06,c0,00,00,00");
        outFrame.data.bytes[0]=0x00;
        outFrame.data.bytes[1]=0x00;  
        outFrame.data.bytes[2]=0x06;
        outFrame.data.bytes[3]=0xc0;
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0x00;
        outFrame.data.bytes[6]=0x00;

    /*CONSOLE.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    CONSOLE.println();*/
        Can1.sendFrame(outFrame); 




}




void checkCAN()
{

  while(Can1.available())
  {
    Can1.read(inFrame);
   // Serial.println(inFrame.id, HEX);
  

  if(inFrame.id == 0x1da && inFrame.length == 8){
  //  last_received_from_inverter_timestamp = millis();

    inverter_status.voltage = ((uint16_t)inFrame.data.bytes[0] << 2) |
        (inFrame.data.bytes[1] >> 6);

    int16_t parsed_speed = ((uint16_t)inFrame.data.bytes[4] << 8) |
        (uint16_t)inFrame.data.bytes[5];
    inverter_status.speed = (parsed_speed == 0x7fff ? 0 : parsed_speed);

    inverter_status.error_state = (inFrame.data.bytes[6] & 0xb0) != 0x00;
  }

  if(inFrame.id == 0x55a && inFrame.length == 8){
   // last_received_from_inverter_timestamp = millis();

    inverter_status.inverter_temperature = fahrenheit_to_celsius(inFrame.data.bytes[2]);
    inverter_status.motor_temperature = fahrenheit_to_celsius(inFrame.data.bytes[1]);
  }

  }
  
}


static int8_t fahrenheit_to_celsius(uint16_t fahrenheit)
{
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if(result < -128)
    return -128;
  if(result > 127)
    return 127;
  return result;
}



static void nissan_crc(uint8_t *data, uint8_t polynomial)
{
  // We want to process 8 bytes with the 8th byte being zero
  data[7] = 0;
  uint8_t crc = 0;
  for(int b=0; b<8; b++)
  {
    for(int i=7; i>=0; i--)
    {
      uint8_t bit = ((data[b] &(1 << i)) > 0) ? 1 : 0;
      if(crc >= 0x80) 
        crc = (byte)(((crc << 1) + bit) ^ polynomial);
      else 
        crc = (byte)((crc << 1) + bit);
    }
  }
  data[7] = crc;
}
