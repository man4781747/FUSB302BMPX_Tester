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
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000);
  FUSB302BMPX.SetWire(&Wire);
  FUSB302BMPX.ResetPDAndI2C();
  FUSB302BMPX.OpenAllPower();
  Serial.println(FUSB302BMPX.CheckCC()?"有連接":"無連接");



  FUSB302BMPX.ResetPDAndI2C();
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
  pinMode(0, INPUT);
  // FUSB302BMPX.PD_Reset();
  // FUSB302BMPX.SendHardReset();
}

void loop() {
  FUSB302BMPX.CheckINTERRUPT();
  FUSB302BMPX.ReadFIFO();
  if (FUSB302BMPX.newestFIFO_HEADER_SOP.parts.MessageType != 0) {
    if (
      FUSB302BMPX.newestFIFO_HEADER_SOP.parts.MessageType == 1 & 
      FUSB302BMPX.newestFIFO_HEADER_SOP.parts.NumberOfDataObjects == 0
    ) {
      Serial.println("收到 Good CRC");
    }
    else if (
      FUSB302BMPX.newestFIFO_HEADER_SOP.parts.MessageType == MSG_Source_Capabilities & 
      FUSB302BMPX.newestFIFO_HEADER_SOP.parts.NumberOfDataObjects != 0
    ) {
      Serial.printf("[%d]獲得的是供電能力資料\n", millis());
      //! 需盡快送出Request
      if (FUSB302BMPX.SourceapAbilityChose == -1) {
        //! 到這邊代表沒有選定輸出電壓，預設要求 5V 1A 電源
        Serial.printf("[%d]準備發出Default請求\n", millis());
        DP__HEADER_SOP RxTest;
        RxTest.parts.Extended = 0;
        RxTest.parts.MessageType = 0b00010;
        RxTest.parts.NumberOfDataObjects = 1;   //! 這邊填的是給出的資料數(個數，不是Byte數)
        RxTest.parts.PortDataRole = 0;
        RxTest.parts.SpecificationReversio = 0b01;
        RxTest.parts.PortPowerRole = 0;
        RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;
        uint8_t choseIndex;
        for (int i = 0;i < FUSB302BMPX.SourceapAbilityList.size();i++) {
          if (FUSB302BMPX.SourceapAbilityList[i].Fixed.part.V == 100) {
            choseIndex = (i+1) << 4;
            break;
          }
        }

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
        Wire.write(RxTest.IndexSort.Index_1);
        Wire.write(RxTest.IndexSort.Index_2);
        Wire.write(0b00101100);
        Wire.write(0b10110001);
        Wire.write(0b00000100);
        Wire.write(choseIndex);
        Wire.write(0xff);
        Wire.write(0x14);
        Wire.write(0xA1);
        Wire.endTransmission();
        FUSB302BMPX.AddMSD();
      } else {
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
        Wire.write(RxTest.IndexSort.Index_1);
        Wire.write(RxTest.IndexSort.Index_2);
        Wire.write(0x2C);  // 00101100
        Wire.write(0xB1);  // 10110001
        Wire.write(0x04);  // 00000100
        Wire.write(0x43);  // 01000011
        Wire.write(0xff);
        Wire.write(0x14);
        Wire.write(0xA1);
        Wire.endTransmission();
        FUSB302BMPX.AddMSD();
      }
    }
  }

  // if (FUSB302BMPX.SourceapAbilityList.size() == 0) {
  //   Serial.printf("[%d]電源端供電能力無紀錄，準備重新索取\n", millis());
  //   vTaskDelay(1000/portTICK_PERIOD_MS);
  //   DP__HEADER_SOP RxTest;
  //   RxTest.parts.Extended = 0;
  //   RxTest.parts.MessageType = 0b00111;
  //   RxTest.parts.NumberOfDataObjects = 0;   //! 這邊填的是給出的資料數(個數，不是Byte數)
  //   RxTest.parts.PortDataRole = 0;
  //   RxTest.parts.SpecificationReversio = 0b01;
  //   RxTest.parts.PortPowerRole = 0;
  //   RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;
  //   FUSB302BMPX.ClearFIFO_Rx();
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
  //   Wire.write(0x80 + (RxTest.parts.NumberOfDataObjects*4+2));
  //   Wire.write(RxTest.IndexSort.Index_1); 
  //   Wire.write(RxTest.IndexSort.Index_2); 
  //   Wire.write(0xff);
  //   Wire.write(0x14);
  //   Wire.write(0xA1);
  //   Wire.endTransmission();
  //   FUSB302BMPX.AddMSD();
  //   vTaskDelay(100/portTICK_PERIOD_MS);
  // }

  if (digitalRead(0) == LOW) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Serial.printf("[%d]準備發出供電能力請求\n", millis());
    DP__HEADER_SOP RxTest;
    RxTest.parts.Extended = 0;
    RxTest.parts.MessageType = 0b00111;
    RxTest.parts.NumberOfDataObjects = 0;   //! 這邊填的是給出的資料數(個數，不是Byte數)
    RxTest.parts.PortDataRole = 0;
    RxTest.parts.SpecificationReversio = 0b01;
    RxTest.parts.PortPowerRole = 0;
    RxTest.parts.MessageID = FUSB302BMPX.MSG_ID;
    FUSB302BMPX.ClearFIFO_Rx();
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
    Wire.write(RxTest.IndexSort.Index_1); 
    Wire.write(RxTest.IndexSort.Index_2); 
    Wire.write(0xff);
    Wire.write(0x14);
    Wire.write(0xA1);
    Wire.endTransmission();
    FUSB302BMPX.AddMSD();
    // uint16_t test = RxTest.value;
    // Serial.printf("送出 Header: ");
    // for (int i = 0; i < 16; i++) {
    //   bool b = test & 0x8000;
    //   if (b) {
    //     Serial.printf("1");
    //   } else {
    //     Serial.printf("0");
    //   }
    //   test = test << 1;
    // }
    // Serial.println();

    // vTaskDelay(100/portTICK_PERIOD_MS);

  }
  // FUSB302BMPX.CheckINTERRUPT();

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