#include <Arduino.h>
#include <Wire.h>

void i2c_scanner();

#define ADDR_DEVICE_ID 0x01
#define ADDR_Switches0 0x02
#define ADDR_Switches1 0x03
#define ADDR_Measure   0x04
#define ADDR_Slice     0x05
#define ADDR_Control0  0x06
#define ADDR_Control1  0x07
#define ADDR_Control2  0x08
#define ADDR_Control3  0x09
#define ADDR_Mask1     0x0A
#define ADDR_Power     0x0B
#define ADDR_Reset     0x0C
#define ADDR_OCPreg    0x0D
#define ADDR_Maska     0x0E
#define ADDR_Maskb     0x0F
#define ADDR_Control4  0x10
#define ADDR_Status0a  0x3C
#define ADDR_Interrupta  0x3E
#define ADDR_Interruptb  0x3F
#define ADDR_Status0   0x40
#define ADDR_Status1   0x41
#define ADDR_Interrupt 0x42
#define ADDR_FIFOs     0x43

struct DEVICE_ID {
  unsigned int Revision_ID : 2;
  unsigned int Product_ID : 2;
  unsigned int Version_ID : 4;
};

struct SWITCHES0 {
  unsigned int PDWN1 : 1;
  unsigned int PDWN2 : 1;
  unsigned int MEAS_CC1 : 1;
  unsigned int MEAS_CC2 : 1;
  unsigned int VCONN_CC1 : 1;
  unsigned int VCONN_CC2 : 1;
  unsigned int PU_EN1 : 1;
  unsigned int PU_EN2 : 1;
};

struct Status0a {
  unsigned int HARDRST : 1;
  unsigned int SOFTRST : 1;
  unsigned int POWER2 : 1;
  unsigned int POWER3 : 1;
  unsigned int RETRY_FAIL : 1;
  unsigned int SOFTFAIL : 1;
};

struct Status1a {
  unsigned int RXSOP : 1;
  unsigned int RXSOP_1DB : 1;
  unsigned int RXSOP_2DB : 1;
  unsigned int TOGSS1 : 1;
  unsigned int TOGSS2 : 1;
  unsigned int TOGSS3 : 1;
};

struct Status0 {
  unsigned int BC_LVL0 : 1;
  unsigned int BC_LVL1 : 1;
  unsigned int WAKE : 1;
  unsigned int ALERT : 1;
  unsigned int CRC_CHK : 1;
  unsigned int COMP : 1;
  unsigned int ACTIVITY : 1;
  unsigned int VBUSOK : 1;
};

struct Status1 {
  unsigned int OCP : 1;
  unsigned int OVRTEMP : 1;
  unsigned int TX_FULL : 1;
  unsigned int TX_EMPTY : 1;
  unsigned int RX_FULL : 1;
  unsigned int RX_EMPTY : 1;
  unsigned int RXSOP1 : 1;
  unsigned int RXSOP2 : 1;
};

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // DEVICE_ID device_id;
  Wire.beginTransmission(0x22);
  Wire.write(ADDR_DEVICE_ID);
  Wire.endTransmission();
  Wire.requestFrom(0x22, 1);
  uint8_t data__device_id = Wire.read();
  DEVICE_ID *device_id = (DEVICE_ID*)(&data__device_id);
  Serial.println(*(u_int8_t*)device_id);

  // device_id.Revision_ID = data__device_id & 0x03;
  // device_id.Product_ID = (data__device_id >> 2) & 0x03;
  // device_id.Version_ID = (data__device_id >> 4) & 0x0F;


  Wire.beginTransmission(0x22);
  Wire.write(ADDR_Switches0);
  Wire.endTransmission();
  Wire.requestFrom(0x22, 1);
  uint8_t data__switch0 = Wire.read();
  SWITCHES0 *switch0 = (SWITCHES0*)(&data__switch0);
  Serial.println(*(u_int8_t*)switch0);

  
  // switch0.PDWN1 = data__switch0 & 0x01;
  // switch0.PDWN2 = (data__switch0 >> 1) & 0x01;
  // switch0.MEAS_CC1 = (data__switch0 >> 2) & 0x01;
  // switch0.MEAS_CC2 = (data__switch0 >> 3) & 0x01;
  // switch0.VCONN_CC1 = (data__switch0 >> 4) & 0x01;
  // switch0.VCONN_CC2 = (data__switch0 >> 5) & 0x01;
  // switch0.PU_EN1 = (data__switch0 >> 6) & 0x01;
  // switch0.PU_EN2 = (data__switch0 >> 7) & 0x01;
  // Serial.println(data__switch0);



  Serial.println("===========================");
  // Serial.println(device_id.Revision_ID);
  // Serial.println(device_id.Product_ID);
  // Serial.println(device_id.Version_ID);
  
  delay(1000);
  // i2c_scanner();
}

// put function definitions here:
void i2c_scanner() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}