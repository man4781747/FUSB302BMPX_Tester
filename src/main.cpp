//https://blog.csdn.net/qq_29246181/article/details/105636135?ops_request_misc=&request_id=&biz_id=102&utm_term=FUSB302&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-9-105636135.142^v100^pc_search_result_base6&spm=1018.2226.3001.4187
#include <Arduino.h>
#include <Wire.h>
#include "FUSB302BMPX/FUSB302BMPX.h"

void i2c_scanner();


int PD_STEP = 0;
u_int8_t FIFO_buffer[40] = {0};
u_int8_t FIFO_TX_buffer[20];
int FIFO_Len;
uint8_t I2C_RequestsByteBuffer;

const u_int8_t PD_request[14] = {
  /**
   * https://www.usbzh.com/article/detail-732.html
   * SOP 指令下法: Sync−1 Sync−1 Sync−1 Sync−2
   * 
   */
  0x12, 0x12, 0x12, 0x13,
  0x86,       // 0x80 + 資料Byte數
  0b01000010, // 01(PD2.0) 0(發送訊息的是Sink耗电方) 00010(Request 请求供电)
  0b00010100, // 0(不是Extended) 001(內含N筆資料，N*4Byte) 010(Message ID) 0(PortPowerRole 發送訊息的是UFP)
  0x00, 0x00, 0x00, 0x03,
  0xff,   // JAM_CRC
  0x14,   // EOP
  0xA1    // TXON
};


String getBitString(u_int8_t val){
  String data = "";
  for (int i = 0; i < 8; i++) {
    bool b = val & 0x80;
    if (b) {
      data += "1";
    } else {
      data += "0";
    }
    val = val << 1;
  }
  return data;
}

String getBitString(u_int16_t val){
  String Low = getBitString(*(uint8_t *)(&val+1));
  String High = getBitString(*(uint8_t *)&val);
  return High+Low;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000);
  FUSB302BMPX.SetWire(&Wire);

  FUSB302BMPX.ResetI2CSetting();
  FUSB302BMPX.OpenAllPower();

  Serial.println(FUSB302BMPX.CheckCC()?"有連接":"無連接");
  FUSB302BMPX.SendHardReset();
  FUSB302BMPX.ResetI2CSetting();
  vTaskDelay(5/portTICK_PERIOD_MS);
  FUSB302BMPX.OpenAllAutoRetry();
  FUSB302BMPX.OpenUsefulInterrupt();

  //! 清空各狀態
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x06);
  Wire.write(0x00);
  Wire.endTransmission();

  //! CC1 下拉，並開啟 ADC 量測
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x02); 
  Wire.write(0b00100101);
  Wire.endTransmission();

  //! DP2.0打開，並且在CC1上啟用BMC，打開AutoCRC
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x03);  
  Wire.write(0b00100101);
  Wire.endTransmission();
  FUSB302BMPX.OpenAllPower();
  FUSB302BMPX.ClearFIFO_Rx();

  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x0F);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x0A);
  Wire.write(0xEF);
  Wire.endTransmission();
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x0C);
  Wire.write(0x02);
  Wire.endTransmission();
  pinMode(0, INPUT);
}

void loop() {
  // delay(1*1000);
  FUSB302BMPX.CheckINTERRUPT();
  DP__FIFO_Rx_Info info = FUSB302BMPX.ReadFIFO();
  DP__INTERRUPTB INTERRUPTB = FUSB302BMPX.GetInteruptB();
  if (INTERRUPTB.I_GCRCSENT & PD_STEP == 0) {
    Serial.printf("[%d]Good CRC 回應已發送\n", millis());
    PD_STEP = 1;
    Serial.printf("[%d]準備發出請求\n", millis());
    DP__HEADER_SOP RxTest;
    RxTest.parts.Extended = 0;
    RxTest.parts.MessageType = 0b00010;
    RxTest.parts.NumberOfDataObjects = 1;   //! 這邊填的是給出的資料數(個數，不是Byte數)
    RxTest.parts.PortDataRole = 0;
    RxTest.parts.SpecificationReversio = 0b01;
    RxTest.parts.PortPowerRole = 0;
    RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;

    // ! 清空 FIFO TX
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(0x06);
    Wire.write(0x40);
    Wire.endTransmission();
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(0x43);
    Wire.write(0x12);
    Wire.write(0x12);
    Wire.write(0x12);
    Wire.write(0x13);
    Wire.write(0x80 + (RxTest.parts.NumberOfDataObjects*4+2));
    Wire.write(RxTest.IndexSort.Index_1); //0b01 0 00010
    Wire.write(RxTest.IndexSort.Index_2); //0b0 001 010 0  0 001 000 0
    Wire.write(0x2C);
    Wire.write(0xB1);
    Wire.write(0x04);
    Wire.write(0x43);
    Wire.write(0xff);
    Wire.write(0x14);
    Wire.write(0xA1);
    Wire.endTransmission();
    FUSB302BMPX.AddMSD();
  } else if (INTERRUPTB.I_GCRCSENT & PD_STEP == 5) {
    Serial.printf("[%d]Good CRC 回應已發送\n", millis());
    PD_STEP = 2;
    Serial.printf("[%d]準備發出請求\n", millis());
    DP__HEADER_SOP RxTest;
    RxTest.parts.Extended = 0;
    RxTest.parts.MessageType = 0b00011;
    RxTest.parts.NumberOfDataObjects = 2;
    RxTest.parts.PortDataRole = 0;
    RxTest.parts.SpecificationReversio = 0b01;
    RxTest.parts.PortPowerRole = 0;
    RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;

    // ! 清空 FIFO TX
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x06);
    // Wire.write(0x40);
    // Wire.endTransmission();
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(0x43);
    Wire.write(0x12);
    Wire.write(0x12);
    Wire.write(0x12);
    Wire.write(0x13);
    Wire.write(0x80 + RxTest.parts.NumberOfDataObjects);
    Wire.write(RxTest.IndexSort.Index_1); //0b01 0 00010
    Wire.write(RxTest.IndexSort.Index_2); //0b0 001 010 0  0 001 000 0
    Wire.write(0xff);
    Wire.write(0x14);
    Wire.write(0xA1);
    Wire.endTransmission();
    FUSB302BMPX.AddMSD();
  }


  // Serial.println(info.header.parts.MessageType);
  // if (info.header.parts.MessageType == MSG_Source_Capabilities) {
  //   if (PD_STEP == 0) {
  //     Serial.println("剛啟動");
  //     PD_STEP = 1;
  //     FUSB302BMPX.MSG_ID = 0;
  //     FUSB302BMPX.ClearFIFO_Rx();
  //     // Wire.beginTransmission(FUSB302BMPX_ADDR);
  //     // Wire.write(0x0C);  
  //     // Wire.write(0x02);
  //     // Wire.endTransmission();
  //     FUSB302BMPX.SendHardReset();
  //     FUSB302BMPX.ResetI2CSetting();
  //     FUSB302BMPX.OpenAllAutoRetry();
  //     FUSB302BMPX.OpenUsefulInterrupt();
  //     //! 清空各狀態
  //     Wire.beginTransmission(FUSB302BMPX_ADDR);
  //     Wire.write(0x06);
  //     Wire.write(0x00);
  //     Wire.endTransmission();

  //     //! CC1 下拉，並開啟 ADC 量測
  //     Wire.beginTransmission(FUSB302BMPX_ADDR);
  //     Wire.write(0x02); 
  //     Wire.write(0b00100101);
  //     Wire.endTransmission();

  //     //! DP2.0打開，並且在CC1上啟用BMC，打開AutoCRC
  //     Wire.beginTransmission(FUSB302BMPX_ADDR);
  //     Wire.write(0x03);  
  //     Wire.write(0b00100101);
  //     Wire.endTransmission();
  //     FUSB302BMPX.OpenAllPower();
  //     Wire.beginTransmission(FUSB302BMPX_ADDR);
  //     Wire.write(0x0F);
  //     Wire.write(0x01);
  //     Wire.endTransmission();
  //     Wire.beginTransmission(FUSB302BMPX_ADDR);
  //     Wire.write(0x0A);
  //     Wire.write(0xEF);
  //     Wire.endTransmission();

  //     while (true) {
  //       DP__INTERRUPTB INTERRUPTB = FUSB302BMPX.GetInteruptB();
  //       if (INTERRUPTB.I_GCRCSENT) {
  //         Serial.printf("Good CRC 發送\n");
  //         Serial.println("Send");
  //         PD_STEP = 2;
  //         DP__HEADER_SOP RxTest;
  //         RxTest.parts.Extended = 0;
  //         RxTest.parts.MessageType = 0b00010;
  //         RxTest.parts.NumberOfDataObjects = 6;
  //         RxTest.parts.PortDataRole = 0;
  //         RxTest.parts.SpecificationReversio = 0b01;
  //         RxTest.parts.PortPowerRole = 0;
  //         RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;

  //         // ! 清空 FIFO TX
  //         Wire.beginTransmission(FUSB302BMPX_ADDR);
  //         Wire.write(0x06);
  //         Wire.write(0x40);
  //         Wire.endTransmission();
  //         Wire.beginTransmission(FUSB302BMPX_ADDR);
  //         Wire.write(0x43);
  //         Wire.write(0x12);
  //         Wire.write(0x12);
  //         Wire.write(0x12);
  //         Wire.write(0x13);
  //         Wire.write(0x86);
  //         Wire.write(RxTest.IndexSort.Index_1); //0b01 0 00010
  //         Wire.write(RxTest.IndexSort.Index_2); //0b0 001 010 0  0 001 000 0
  //         Wire.write(0x2C);
  //         Wire.write(0xB1);
  //         Wire.write(0x04);
  //         Wire.write(0x43);
  //         Wire.write(0xff);
  //         Wire.write(0x14);
  //         Wire.write(0xA1);
  //         Wire.endTransmission();
  //         FUSB302BMPX.AddMSD();
  //         FUSB302BMPX.CheckINTERRUPT();
  //         break;
  //       }
  //     }
  //   }
  // }
  // Serial.println(info.SOP_Type);
  // Serial.println(info.header.value);
  // for (int i = 0; i < 16; i++) {
  //   bool b = info.header.value & 0x8000;
  //   if (b) {
  //     Serial.printf("1");
  //   } else {
  //     Serial.printf("0");
  //   }
  //   info.header.value = info.header.value << 1;
  // }
  // Serial.println();
  // delay(1*1000);

  // if (digitalRead(0) == LOW) {
  //   Serial.println("Send");

  //   DP__HEADER_SOP RxTest;
  //   RxTest.parts.Extended = 0;
  //   RxTest.parts.MessageType = 0b00010;
  //   RxTest.parts.NumberOfDataObjects = 6;
  //   RxTest.parts.PortDataRole = 0;
  //   RxTest.parts.SpecificationReversio = 0b01;
  //   RxTest.parts.PortPowerRole = 0;
  //   RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;

  //   // ! 清空 FIFO TX
  //   Wire.beginTransmission(FUSB302BMPX_ADDR);
  //   Wire.write(0x06);
  //   Wire.write(0x40);
  //   Wire.endTransmission();
  //   Wire.beginTransmission(FUSB302BMPX_ADDR);
  //   Wire.write(0x43);
  //   Wire.write(0x12);
  //   Wire.write(0x12);
  //   Wire.write(0x12);
  //   Wire.write(0x13);
  //   Wire.write(0x86);
  //   Wire.write(RxTest.IndexSort.Index_1); //0b01 0 00010
  //   Wire.write(RxTest.IndexSort.Index_2); //0b0 001 010 0  0 001 000 0
  //   Wire.write(0x2C);
  //   Wire.write(0xB1);
  //   Wire.write(0x04);
  //   Wire.write(0x43);
  //   Wire.write(0xff);
  //   Wire.write(0x14);
  //   Wire.write(0xA1);
  //   Wire.endTransmission();
  //   FUSB302BMPX.AddMSD();
  //   vTaskDelay(100/portTICK_PERIOD_MS);
  // }
  
  
  //! 清空 FIFO TX
  // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // Wire.write(0x06);
  // Wire.write(0x40);
  // Wire.endTransmission();


  // DP__REQUEST__POWER data;
  // data.I_Set = 200;
  // data.V_Set = 100;

  // uint8_t *ptr = (uint8_t *)&data;
  // // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // // Wire.write(0x43);
  // Serial.println("---------------");
  // for (int i = 0; i < sizeof(DP__REQUEST__POWER); ++i) {
  //   // Wire.write(ptr[i]);
  //   Serial.printf("0x%02X\n", ptr[i]);
  // }
  // Serial.println("---------------");

  // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // Wire.write(0x43);
  // Wire.write(0x12);
  // Wire.write(0x12);
  // Wire.write(0x12);
  // Wire.write(0x13);
  // Wire.write(0x86);
  // Wire.write(0x42); //0b01 0 00010
  // Wire.write(0x10); //0b00010100  00010000
  // Wire.write(0x2C);
  // Wire.write(0xB1);
  // Wire.write(0x04);
  // Wire.write(0x43);
  // Wire.write(0xff);
  // Wire.write(0x14);
  // Wire.write(0xA1);
  // Wire.endTransmission();

  // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // Wire.write(0x43);
  // Wire.write(0x12);
  // Wire.write(0x12);
  // Wire.write(0x12);
  // Wire.write(0x13);
  // Wire.write(0x82);
  // Wire.write(0b01000001); //0b01000111
  // Wire.write(0b00000000); //0b00000000 
  // Wire.write(0xff);
  // Wire.write(0x14);
  // Wire.write(0xA1);
  // Wire.endTransmission();


  // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // Wire.write(0x06);
  // Wire.write(0x05);
  // Wire.endTransmission();

  // FUSB302BMPX.ResetI2CSetting();
  // FUSB302BMPX.MeasureCCs();
  // FUSB302BMPX.EnableAutoCRC();
  // Serial.println("===========================");
  // DP__DEVICE_ID DeviceID = FUSB302BMPX.GetDeviceID();
  // Serial.printf("DeviceID:\t%s\n", getBitString(*(u_int8_t*)&DeviceID).c_str());
  // DP__SWITCHES0 SWITCHES0 = FUSB302BMPX.GetSwitches0();
  // Serial.printf("SWITCHES0:\t%s\n", getBitString(*(u_int8_t*)&SWITCHES0).c_str());
  // DP__SWITCHES1 SWITCHES1 = FUSB302BMPX.GetSwitches1();
  // Serial.printf("SWITCHES1:\t%s\n", getBitString(*(u_int8_t*)&SWITCHES1).c_str());
  // DP__MEASURE MEASURE = FUSB302BMPX.GetMeasure();
  // double test = 0.042*MEASURE.MDAC+0.042;
  // if (MEASURE.MEAS_VBUS) {
  //   test *= 10.;
  // }
  // Serial.printf("MEASURE:\t%s : %.4f\n", getBitString(*(u_int8_t*)&MEASURE).c_str(), test);
  // MEASURE.MDAC = 0b000001;
  // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // Wire.write(ADDR_Measure);
  // Wire.write(*(u_int8_t*)&MEASURE);
  // Wire.endTransmission();



  // DP__SLICE SLICE = FUSB302BMPX.GetSlice();
  // Serial.printf("SLICE:\t%s\n", getBitString(*(u_int8_t*)&SLICE).c_str());
  // DP__CONTROL0 CONTROL0 = FUSB302BMPX.GetControl0();
  // Serial.printf("CONTROL0:\t%s\n", getBitString(*(u_int8_t*)&CONTROL0).c_str());
  // DP__CONTROL1 CONTROL1 = FUSB302BMPX.GetControl1();
  // Serial.printf("CONTROL1:\t%s\n", getBitString(*(u_int8_t*)&CONTROL1).c_str());
  // DP__CONTROL2 CONTROL2 = FUSB302BMPX.GetControl2();
  // Serial.printf("CONTROL2:\t%s\n", getBitString(*(u_int8_t*)&CONTROL2).c_str());
  // DP__CONTROL3 CONTROL3 = FUSB302BMPX.GetControl3();
  // Serial.printf("CONTROL3:\t%s\n", getBitString(*(u_int8_t*)&CONTROL3).c_str());
  // DP__MASK MASK = FUSB302BMPX.GetMask();
  // Serial.printf("MASK:\t%s\n", getBitString(*(u_int8_t*)&MASK).c_str());
  // DP__POWER POWER = FUSB302BMPX.GetPower();
  // Serial.printf("POWER:\t%s\n", getBitString(*(u_int8_t*)&POWER).c_str());
  // DP__RESET RESET = FUSB302BMPX.GetReset();
  // Serial.printf("RESET:\t%s\n", getBitString(*(u_int8_t*)&RESET).c_str());
  // DP__OCPREG OCPREG = FUSB302BMPX.GetOCPreg();
  // Serial.printf("OCPREG:\t%s\n", getBitString(*(u_int8_t*)&OCPREG).c_str());
  // DP__MASKA MASKA = FUSB302BMPX.GetMaskA();
  // Serial.printf("MASKA:\t%s\n", getBitString(*(u_int8_t*)&MASKA).c_str());
  // DP__MASKB MASKB = FUSB302BMPX.GetMaskB();
  // Serial.printf("MASKB:\t%s\n", getBitString(*(u_int8_t*)&MASKB).c_str());
  // DP__CONTROL4 CONTROL4 = FUSB302BMPX.GetControl4();
  // Serial.printf("CONTROL4:\t%s\n", getBitString(*(u_int8_t*)&CONTROL4).c_str());
  // Serial.println("=======================");
  // DP__STATUS0A STATUS0A = FUSB302BMPX.GetStatus0A();
  // Serial.printf("STATUS0A:\t%s\n", getBitString(*(u_int8_t*)&STATUS0A).c_str());
  // DP__STATUS1A STATUS1A = FUSB302BMPX.GetStatus1A();
  // Serial.printf("STATUS1A:\t%s\n", getBitString(*(u_int8_t*)&STATUS1A).c_str());
  // DP__STATUS0 STATUS0 = FUSB302BMPX.GetStatus0();
  // Serial.printf("STATUS0:\t%s\n", getBitString(*(u_int8_t*)&STATUS0).c_str());
  // DP__STATUS1 STATUS1 = FUSB302BMPX.GetStatus1();
  // Serial.printf("STATUS1:\t%s\n", getBitString(*(u_int8_t*)&STATUS1).c_str());
  // DP__INTERRUPTA INTERRUPTA = FUSB302BMPX.GetInteruptA();
  // Serial.printf("INTERRUPTA:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPTA).c_str());
  // DP__INTERRUPTB INTERRUPTB = FUSB302BMPX.GetInteruptB();
  // Serial.printf("INTERRUPTB:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPTB).c_str());
  // DP__INTERRUPT INTERRUPT_ = FUSB302BMPX.GetInterupt();
  // Serial.printf("INTERRUPT_:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPT_).c_str());
  // delay(60*1000);
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