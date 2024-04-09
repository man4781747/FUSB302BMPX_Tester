//https://blog.csdn.net/qq_29246181/article/details/105636135?ops_request_misc=&request_id=&biz_id=102&utm_term=FUSB302&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-9-105636135.142^v100^pc_search_result_base6&spm=1018.2226.3001.4187
#include <Arduino.h>
#include <Wire.h>
#include "FUSB302BMPX/FUSB302BMPX.h"

void i2c_scanner();

u_int8_t FIFO_buffer[200];
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

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
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
  Wire.write(0x05);
  Wire.endTransmission();

  //! DP2.0打開，並且在CC1上啟用BMC
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(0x03);  
  Wire.write(0x41);
  Wire.endTransmission();
  FUSB302BMPX.OpenAllPower();
  
  DP__INTERRUPTA INTERRUPTA = FUSB302BMPX.GetInteruptA();
  Serial.printf("INTERRUPTA:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPTA).c_str());
  DP__INTERRUPTB INTERRUPTB = FUSB302BMPX.GetInteruptB();
  Serial.printf("INTERRUPTB:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPTB).c_str());
  DP__INTERRUPT INTERRUPT_ = FUSB302BMPX.GetInterupt();
  Serial.printf("INTERRUPT_:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPT_).c_str());


}

void loop() {
  u_int8_t FIFO;
  FUSB302BMPX.GetMemeryData(&FIFO, ADDR_FIFOs);
  Serial.printf("FIFO:\t%s\n", getBitString(FIFO).c_str());
  FIFO_buffer[0] = FIFO & 0xE0;
  if (FIFO_buffer[0] > 0x40) {
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(ADDR_FIFOs);
    Wire.endTransmission();
    Wire.requestFrom(FUSB302BMPX_ADDR,2);
    Wire.readBytes(FIFO_buffer + 2, 2);
    uint8_t i = 0;
    Serial.printf("SET:\t%s\n", getBitString(FIFO_buffer[2]).c_str());
    i = FIFO_buffer[2] % 0x70;
    i >>= 2;
    Serial.printf("獲得 %d 組電壓設定\n", i);
    i += 2;
    FIFO_Len = i + 3;
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(ADDR_FIFOs);
    Wire.endTransmission();
    Wire.requestFrom(FUSB302BMPX_ADDR, (int)i);
    Wire.readBytes(FIFO_buffer+3, i);
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(0x07);  
    Wire.write(0x04);
    Wire.endTransmission();
    
    uint16_t I_set = 0, V_set = 0;
    for (int setChose = 0;setChose<i-2;setChose++) {
      Serial.printf("A:\t%s\n", getBitString(FIFO_buffer[4*setChose + 3]).c_str());
      Serial.printf("B:\t%s\n", getBitString(FIFO_buffer[4*setChose + 4]).c_str());
      Serial.printf("C:\t%s\n", getBitString(FIFO_buffer[4*setChose + 5]).c_str());
      Serial.printf("D:\t%s\n", getBitString(FIFO_buffer[4*setChose + 6]).c_str());
      uint8_t A_set = FIFO_buffer[4*setChose + 3];
      uint8_t B_set = FIFO_buffer[4*setChose + 4];
      uint8_t C_set = FIFO_buffer[4*setChose + 5];
      *((uint8_t *)&I_set+1) = A_set;
      *((uint8_t *)&I_set) = B_set & 0x03;
      *((uint8_t *)&V_set+1) = B_set & 0xFC;
      *((uint8_t *)&V_set) = C_set & 0x0F;
      V_set >>= 2;
      V_set *= 5;
      Serial.printf(" %d - V: %d,I: %d\n", setChose, V_set, I_set);
    }





  }

  // Wire.beginTransmission(FUSB302BMPX_ADDR);
  // Wire.write(0x07);  
  // Wire.write(0x04);
  // Wire.endTransmission();

  u_int8_t test_i = 0;
  if (FIFO_Len >= 5) {
    test_i = FIFO_buffer[2] & 0x70;
    if (test_i == 0) {
      Serial.println(" - 控制類消息");
      test_i = FIFO_buffer[1] & 0x07;
      switch (test_i)
      {
      case 1:
        Serial.println("   : CRC 較驗無問題");
        break;
      case 3:
        Serial.println("   : 給電端接受了受電端的要求");
        break;
      case 4:
        Serial.println("   : 給電端拒絕了受電端的要求");
        break;
      case 6:
        Serial.println("   : 請求的電壓已準備完成");
        break;
      case 8:
        Serial.println("   : 還不知道要幹嘛");
        vTaskDelay(1/portTICK_PERIOD_MS);
        break;
      default:
        Serial.printf("   : 未知狀態: %d \n", test_i);
        break;
      }
    }
    else {
      Serial.println(" - 數據類消息");
      Serial.printf("test:\t%s\n", getBitString(FIFO_buffer[1]).c_str());
      // test_i = FIFO_buffer[1] % 0x07;
      // if (test_i == 0x01) {

      // }
    }
  }


  delay(1*1000);

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
  // Serial.printf("MEASURE:\t%s\n", getBitString(*(u_int8_t*)&MEASURE).c_str());
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
  // DP__STATUS0A STATUS0A = FUSB302BMPX.GetStatus0A();
  // Serial.printf("STATUS0A:\t%s\n", getBitString(*(u_int8_t*)&STATUS0A).c_str());
  // DP__STATUS1A STATUS1A = FUSB302BMPX.GetStatus1A();
  // Serial.printf("STATUS1A:\t%s\n", getBitString(*(u_int8_t*)&STATUS1A).c_str());
  // DP__INTERRUPTA INTERRUPTA = FUSB302BMPX.GetInteruptA();
  // Serial.printf("INTERRUPTA:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPTA).c_str());
  // DP__INTERRUPTB INTERRUPTB = FUSB302BMPX.GetInteruptB();
  // Serial.printf("INTERRUPTB:\t%s\n", getBitString(*(u_int8_t*)&INTERRUPTB).c_str());
  // DP__STATUS0 STATUS0 = FUSB302BMPX.GetStatus0();
  // Serial.printf("STATUS0:\t%s\n", getBitString(*(u_int8_t*)&STATUS0).c_str());
  // DP__STATUS1 STATUS1 = FUSB302BMPX.GetStatus1();
  // Serial.printf("STATUS1:\t%s\n", getBitString(*(u_int8_t*)&STATUS1).c_str());
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