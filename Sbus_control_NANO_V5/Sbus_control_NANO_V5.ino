#include <SBUS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#define SOFT_printSerial_RX 5 //  (RX 5 no use)
#define SOFT_printSerial_TX 6 //
bool RCStatus = 1;
int Revdata[18];

unsigned int sbusChlData[18];
byte sbusChlIdx = 0;
byte sbusChlBit = 0;
byte sbusPktBit = 0;
byte sbusPktData[25];
byte sbusPktPosition = 0;
byte sbusFailsafeAct = 0;
byte sbusLostFrame = 0;

int low_throw = 306;
int throw_threshold = 1050;


SoftwareSerial printSerial = SoftwareSerial(SOFT_printSerial_RX, SOFT_printSerial_TX);  // RX, TX
SBUS sbus(Serial); // RF control serial

//volatile int UWBdata[2] ;     //volatile用於ISR變數
volatile int yaw_value;
volatile int pitch_value;

void setup()
{
  init_sbus_channel();
  delay(10);
  Wire.begin(0x08);
  Wire.onReceive(wireRecvEvent);
  printSerial.begin(115200);
  Serial.begin(100000);
  sbus.begin(false);
}

void wireRecvEvent(int numBytes) {    //I2C
  int local_yaw;
  int local_pitch;
  int datals[4];
  //int yaw_ini_value = 200;
  //int pitch_ini_value = 100;

  //  printSerial.print("numBytes: "); printSerial.println(numBytes);
  if (numBytes != 3) {
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  for (int i = 0; i < 3 ; i++) {
    datals[i] = Wire.read();
  }
  //  printSerial.print("data recv:"); printSerial.print(datals[1]);printSerial.print(";"); printSerial.println(datals[2]);
  //datals[1] = yaw ,datals[2] = pitch ,  y = ax + b, a = 6, b = 400, (400~1600), a = 5, b = 500, (500~1500)

  local_yaw = datals[1] ; local_pitch = datals[2];
  
  yaw_value = 5 * local_yaw + 500;
  pitch_value = 5 * local_pitch + 500;

}


void loop()
{
  int alpha;
  sbus.process();
//  printSerial.print("row:"); printSerial.println(Revdata[1]); printSerial.print("pitch:"); printSerial.println(Revdata[2]);
//  printSerial.print("throttle:"); printSerial.println(Revdata[3]); printSerial.print("yaw:"); printSerial.println(Revdata[4]);
//  printSerial.print("mode:"); printSerial.println(Revdata[5]); 
//  printSerial.print("switch:"); printSerial.println(Revdata[9]); printSerial.println("=======================");
  Revdata[0] = sbus.getNormalizedChannel(0);
  Revdata[1] = sbus.getNormalizedChannel(1); //row
  Revdata[2] = sbus.getNormalizedChannel(2);  //pitch
  Revdata[3] = sbus.getNormalizedChannel(3);  //throttle
  Revdata[4] = sbus.getNormalizedChannel(4);  //yaw
  Revdata[5] = sbus.getNormalizedChannel(5); Revdata[6] = sbus.getNormalizedChannel(6);
  Revdata[7] = sbus.getNormalizedChannel(7); Revdata[8] = sbus.getNormalizedChannel(8);
  Revdata[9] = sbus.getNormalizedChannel(9);
  alpha = Revdata[6] / 231;

  //constrain throttle value
  if (Revdata[3] > 1350) {
    Revdata[3] = 1350;
  }

  //Build SBUS package
  sbusChlData[0] = Revdata[1];   // row
  //  sbusChlData[1] = Revdata[2];   // pitch
  sbusChlData[2] = Revdata[3];   // throttle

  //distinguish follow switch on or off
  if (Revdata[9] > 1500) {
    //    printSerial.print("yaw pitch value: "); printSerial.print(yaw_value);printSerial.print(";");
    //    printSerial.print(pitch_value);printSerial.print(";");printSerial.println(alpha);
    sbusChlData[3] = yaw_value ;   // yaw from rpi
    sbusChlData[1] = pitch_value;   // pitch from rpi
    //    printSerial.print("Revdata[4][2]: "); printSerial.print(sbusChlData[3]);printSerial.print(";");printSerial.println(sbusChlData[1]);
  }
  else {
    sbusChlData[3] = Revdata[4];
    sbusChlData[1] = Revdata[2];
  }
  sbusChlData[4] = Revdata[5];  // 手動.自穩.定高
  sbusChlData[5] = Revdata[6];   // alpha
  sbusChlData[8] = Revdata[9];  //follow switch
  sbusChlData[9] = 292;   sbusChlData[10] = 1070; sbusChlData[11] = 1070;
  sbusChlData[12] = 1028; sbusChlData[13] = 1028; sbusChlData[14] = 1028;
  sbusChlData[15] = 1028; sbusChlData[16] = 0;    sbusChlData[17] = 0;
  buildSbusPacket();
  Serial.write(sbusPktData, 25); //Send data to SBUS
  delay(7);
}

//initaial SBUS values
void init_sbus_channel(void)
{
  sbusChlData[0] = 998; // roll
  sbusChlData[1] = 1011; // pitch
  sbusChlData[2] = 306; // throttle
  sbusChlData[3] = 1000; // yaw
  sbusChlData[4] = 306; // cmd_angle 306 - 1000 - 1694
  sbusChlData[5] = 50; //
  sbusChlData[6] = 1695; sbusChlData[7] = 306; sbusChlData[8] = 1694;
  sbusChlData[9] = 1070; sbusChlData[10] = 1070; sbusChlData[11] = 1070;
  sbusChlData[12] = 1028; sbusChlData[13] = 1028; sbusChlData[14] = 1028;
  sbusChlData[15] = 1028; sbusChlData[16] = 0; sbusChlData[17] = 0;
  sbusFailsafeAct = 0;
  sbusLostFrame = 0;
  return;
}

void buildSbusPacket(void)
{
  for (sbusPktPosition = 0; sbusPktPosition < 25; sbusPktPosition++)
    sbusPktData[sbusPktPosition] = 0x00;  //Zero out packet data

  sbusPktBit = 0;
  sbusPktPosition = 0;
  sbusPktData[sbusPktPosition] = 0x0F;  //Start Byte
  sbusPktPosition++;

  for (sbusChlIdx = 0; sbusChlIdx < 16; sbusChlIdx++)
  {
    for (sbusChlBit = 0; sbusChlBit < 11; sbusChlBit++)
    {
      if (sbusPktBit > 7)
      {
        sbusPktBit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        sbusPktPosition++;       //Move to the next packet byte
      }
      sbusPktData[sbusPktPosition] |= (((sbusChlData[sbusChlIdx] >> sbusChlBit) & 0x01) << sbusPktBit); //Downshift the channel data bit, then upshift it to set the packet data byte
      sbusPktBit++;
    }
  }

  if (sbusChlData[16] > 1023)
    sbusPktData[23] |= (1 << 0); //Any number above 1023 will set the digital servo bit
  if (sbusChlData[17] > 1023)
    sbusPktData[23] |= (1 << 1);
  if (sbusLostFrame != 0)
    sbusPktData[23] |= (1 << 2);        //Any number above 0 will set the lost frame and failsafe bits
  if (sbusFailsafeAct != 0)
    sbusPktData[23] |= (1 << 3);

  sbusPktData[24] = 0x00;  //End byte
  return;
}
