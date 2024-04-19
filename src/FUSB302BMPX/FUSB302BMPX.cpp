#include "FUSB302BMPX.h"

void C_FUSB302BMPX::SetWire(TwoWire *_Wire)
{
  Wire_ = _Wire;
}

void C_FUSB302BMPX::GetMemeryData(u_int8_t *buffer, u_int16_t addr)
{
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(addr);
  Wire_->endTransmission();
  Wire_->requestFrom(FUSB302BMPX_ADDR, 1);
  Wire_->readBytes(buffer, 1);
}

DP__DEVICE_ID C_FUSB302BMPX::GetDeviceID()
{
  DP__DEVICE_ID data;
  GetMemeryData((u_int8_t*)&data, ADDR_DEVICE_ID);
  return data;
}

DP__SWITCHES0 C_FUSB302BMPX::GetSwitches0()
{
  DP__SWITCHES0 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Switches0);
  return data;
}

DP__SWITCHES1 C_FUSB302BMPX::GetSwitches1()
{
  DP__SWITCHES1 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Switches1);
  return data;
}

DP__MEASURE C_FUSB302BMPX::GetMeasure()
{
  DP__MEASURE data;
  GetMemeryData((u_int8_t*)&data, ADDR_Measure);
  return data;
}

DP__SLICE C_FUSB302BMPX::GetSlice()
{
  DP__SLICE data;
  GetMemeryData((u_int8_t*)&data, ADDR_Slice);
  return data;
}

DP__CONTROL0 C_FUSB302BMPX::GetControl0()
{
  DP__CONTROL0 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Control0);
  return data;
}

DP__CONTROL1 C_FUSB302BMPX::GetControl1()
{
  DP__CONTROL1 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Control1);
  return data;
}

DP__CONTROL2 C_FUSB302BMPX::GetControl2()
{
  DP__CONTROL2 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Control2);
  return data;
}

DP__CONTROL3 C_FUSB302BMPX::GetControl3()
{
  DP__CONTROL3 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Control3);
  return data;
}

DP__MASK C_FUSB302BMPX::GetMask()
{
  DP__MASK data;
  GetMemeryData((u_int8_t*)&data, ADDR_Mask);
  return data;
}

DP__POWER C_FUSB302BMPX::GetPower()
{
  DP__POWER data;
  GetMemeryData((u_int8_t*)&data, ADDR_Power);
  return data;
}

DP__RESET C_FUSB302BMPX::GetReset()
{
  DP__RESET data;
  GetMemeryData((u_int8_t*)&data, ADDR_Reset);
  return data;
}

DP__OCPREG C_FUSB302BMPX::GetOCPreg()
{
  DP__OCPREG data;
  GetMemeryData((u_int8_t*)&data, ADDR_OCPreg);
  return data;
}

DP__MASKA C_FUSB302BMPX::GetMaskA()
{
  DP__MASKA data;
  GetMemeryData((u_int8_t*)&data, ADDR_Maska);
  return data;
}

DP__MASKB C_FUSB302BMPX::GetMaskB()
{
  DP__MASKB data;
  GetMemeryData((u_int8_t*)&data, ADDR_Maskb);
  return data;
}

DP__CONTROL4 C_FUSB302BMPX::GetControl4()
{
  DP__CONTROL4 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Control4);
  return data;
}

DP__STATUS0A C_FUSB302BMPX::GetStatus0A()
{
  DP__STATUS0A data;
  GetMemeryData((u_int8_t*)&data, ADDR_Status0a);
  return data;
}

DP__STATUS1A C_FUSB302BMPX::GetStatus1A()
{
  DP__STATUS1A data;
  GetMemeryData((u_int8_t*)&data, ADDR_Status1a);
  return data;
}

DP__INTERRUPTA C_FUSB302BMPX::GetInteruptA()
{
  DP__INTERRUPTA data;
  GetMemeryData((u_int8_t*)&data, ADDR_Interrupta);
  return data;
}

DP__INTERRUPTB C_FUSB302BMPX::GetInteruptB()
{
  DP__INTERRUPTB data;
  GetMemeryData((u_int8_t*)&data, ADDR_Interruptb);
  return data;
}

DP__STATUS0 C_FUSB302BMPX::GetStatus0()
{
  DP__STATUS0 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Status0);
  return data;
}

DP__STATUS1 C_FUSB302BMPX::GetStatus1()
{
  DP__STATUS1 data;
  GetMemeryData((u_int8_t*)&data, ADDR_Status1);
  return data;
}

DP__INTERRUPT C_FUSB302BMPX::GetInterupt()
{
  DP__INTERRUPT data;
  GetMemeryData((u_int8_t*)&data, ADDR_Interrupt);
  return data;
}

void C_FUSB302BMPX::HeaderResp(DP__HEADER_SOP header)
{
  
  Serial.printf(
    "(%d)[%s][%d][%s]",
    header.parts.MessageID,
    header.parts.PortDataRole==0?"Sink":"Source",
    header.parts.SpecificationReversio,
    header.parts.PortDataRole==0?"UFP":"DFP"
  );
  if (header.parts.NumberOfDataObjects == 0) {
    //! 控制類消息
    Serial.printf("[CTRL]");
    switch (header.parts.MessageType)
    {
    case 0:
      break;
    case 1:
      Serial.printf("[GoodCRC]\t收到消息的應答");
      break;
    case 2:
      Serial.printf("[GotoMin]\t供電調整至最小");
      break;
    case 3:
      Serial.printf("[Accept]\t接收到對方的請求");
      break;
    case 4:
      Serial.printf("[Accept]\t拒絕對方的請求");
      break;
    case 5:
      Serial.printf("[Ping]\tPING");
      break;
    case 6:
      Serial.printf("[PS_RDY]\t供電已準備完成");
      break;
    case 7:
      Serial.printf("[Get_Source_Cap]\t获取对方的供电能力消息");
      break;
    case 8:
      Serial.printf("[Get_Sink_Cap]獲取对方耗电的需求");
      break;
    case 12:
      Serial.printf("[Wait] Wait ");
      break;
    case 13:
      Serial.printf("[Soft_Reset]");
      break;
    case 14:
      Serial.printf("[Data_Reset_Complete]");
      break;
    case 15:
      Serial.printf("[Not_Supported]");
      break;
    default:
      Serial.printf("[未知狀態] %d \n", header.parts.MessageType);
      break;
    }
  }
  else {
    Serial.printf("[DATA]");
    switch (header.parts.MessageType) {
    case 0:
      break;
    case 1:
      Serial.printf("[Source_Capabilities]\t供电方的供电能力信息");

      break;
    case 2:
      Serial.printf("[Request]\t请求供电");
      break;
    case 3:
      Serial.printf("[BIST]\tBIST ");
      break;
    case 4:
      Serial.printf("[Sink_Capabilities]\t耗电方的耗电需求信息");
      break;
    case 5:
      Serial.printf("[Battery_Status]");
      break;
    case 6:
      Serial.printf("[Alert]");
      break;
    default:
      Serial.printf("[未知狀態] %d \n", header.parts.MessageType);
      break;
    }
  }
  Serial.println();
}

void C_FUSB302BMPX::EnableTXCC()
{
  DP__RESET RESET = GetReset();
  RESET.SW_RES = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Reset);
  Wire_->write(*(u_int8_t*)&RESET);
  Wire_->endTransmission();

  DP__SWITCHES1 SWITCHES1_OLD = GetSwitches1();
  SWITCHES1_OLD.AUTO_CRC = 1;
  SWITCHES1_OLD.TXCC2 = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches1);
  Wire_->write(*(u_int8_t*)&SWITCHES1_OLD);
  Wire_->endTransmission();

  DP__POWER POWER = FUSB302BMPX.GetPower();
  POWER.PWR = 0b0101;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Power);
  Wire_->write(*(u_int8_t*)&POWER);
  Wire_->endTransmission();


  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_FIFOs);
  Wire_->write(0xA1);
  Wire_->endTransmission();
  DP__CONTROL0 CONTROL0 = FUSB302BMPX.GetControl0();
  CONTROL0.TX_START = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Control0);
  Wire_->write(*(u_int8_t*)&CONTROL0);
  Wire_->endTransmission();

  DP__CONTROL2 CONTROL2 = FUSB302BMPX.GetControl2();
  CONTROL2.TOGGLE = 1;
  CONTROL2.WAKE_EN = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Control2);
  Wire_->write(*(u_int8_t*)&CONTROL2);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::CheckINTERRUPT()
{
  u_int8_t buffer[2];
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Interrupta);
  Wire_->endTransmission();
  Wire_->requestFrom(FUSB302BMPX_ADDR, 1);
  Wire_->readBytes(buffer, 1);
  DP__INTERRUPTA INTERRUPTA;
  *(u_int8_t*)&INTERRUPTA = buffer[0];
  // DP__INTERRUPTB INTERRUPTB;
  // *(u_int8_t*)&INTERRUPTB = buffer[1];
  // DP__INTERRUPT INTERRUPT_;
  // *(u_int8_t*)&INTERRUPT_ = buffer[ADDR_Interrupt - ADDR_Status0a];

  // DP__INTERRUPTA INTERRUPTA = FUSB302BMPX.GetInteruptA();
  // DP__INTERRUPTB INTERRUPTB = FUSB302BMPX.GetInteruptB();
  DP__INTERRUPT INTERRUPT_ = FUSB302BMPX.GetInterupt();
  unsigned long datatime = millis();
  if (INTERRUPTA.I_SOFTRST) {
    Serial.printf("[%d][INTERRUPT]收到軟體重製需求\n", datatime);
    // ResetI2CSetting();
  }
  if (INTERRUPTA.I_TXSENT) {
    Serial.printf("[%d][INTERRUPT]發出的消息已收到GoodCRC回應\n", datatime);
  }
  if (INTERRUPTA.I_HARDSENT) {
    Serial.printf("[INTERRUPT]成功發送硬體重製\n");
  }
  if (INTERRUPTA.I_RETRYFAIL) {
    Serial.printf("[INTERRUPT]RETRY失敗\n");
  }
  if (INTERRUPTA.I_SOFTFAIL) {
    Serial.printf("[INTERRUPT]SOFT RESET 失敗\n");
  }
  // if (INTERRUPTB.I_GCRCSENT) {
  //   Serial.printf("[%d][INTERRUPT]Good CRC 回應已發送\n", datatime);
  // }
  if (INTERRUPT_.I_VBUSOK) {
    Serial.printf("[%d][INTERRUPT]偵測到VBUS電壓躍遷至 4.5V\n", datatime);
  }
  if (INTERRUPT_.I_ACTIVITY) {
    // Serial.printf("CC 總線的 ACTIVITY 值發生變化\n");
  }
  if (INTERRUPT_.I_COMP_CHNG) {
    Serial.printf("指示選定的 CC 線路已觸發 MDAC 中編程的閾值\n");
  }
  if (INTERRUPT_.I_ALERT) {
    Serial.printf("FIFO TX 或 RX 發現滿載\n");
  }
  if (INTERRUPT_.I_WAKE) {
    Serial.printf("CC 上的電壓表示設備正在嘗試連接。軟體必須為時鐘和接收器模組上電\n");
  }
  if (INTERRUPT_.I_COLLISION) {
    Serial.printf("當嘗試傳輸時，在活動 CC 線路上偵測到活動，導致傳輸未完成。資料包仍會正常接收\n");
  }
  if (INTERRUPT_.I_BC_LVL) {
    // Serial.printf("主機請求的電流等級發生變化\n");
  }
  if (INTERRUPTA.I_HARDRST) {
    Serial.printf("[%d][INTERRUPT]收到硬體重製需求\n", datatime);
    // DP__CONTROL3 CONTROL3 = FUSB302BMPX.GetControl3();
    // CONTROL3.SEND_HARD_RESET = 1;
    // Wire_->beginTransmission(FUSB302BMPX_ADDR);
    // Wire_->write(ADDR_Control3);
    // Wire_->write(*(u_int8_t*)&CONTROL3);
    // Wire_->endTransmission();


    // SendHardReset();
    // ResetI2CSetting();
    // OpenAllAutoRetry();
    // OpenUsefulInterrupt();
    // //! 清空各狀態
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x06);
    // Wire.write(0x00);
    // Wire.endTransmission();

    // //! CC1 下拉，並開啟 ADC 量測
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x02); 
    // Wire.write(0b00100101);
    // Wire.endTransmission();

    // //! DP2.0打開，並且在CC1上啟用BMC，打開AutoCRC
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x03);  
    // Wire.write(0b00100101);
    // Wire.endTransmission();
    // FUSB302BMPX.OpenAllPower();
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x0F);
    // Wire.write(0x01);
    // Wire.endTransmission();
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x0A);
    // Wire.write(0xEF);
    // Wire.endTransmission();
    // Wire.beginTransmission(FUSB302BMPX_ADDR);
    // Wire.write(0x06);
    // Wire.write(0x00);
    // Wire.endTransmission();
  }
}

DP__FIFO_Rx_Info C_FUSB302BMPX::ReadFIFO()
{
  u_int8_t FIFO_buffer[40] = {0};
  newestFIFO_Rx_Info.header = newestFIFO_HEADER_SOP;
  u_int8_t FIFO;
  GetMemeryData(&FIFO, ADDR_FIFOs);
  FIFO_Len = 0;
  FIFO_buffer[0] = FIFO & 0xE0;
  if (FIFO_buffer[0] > 0x40) {

    newestFIFO_Rx_Info.SOP_Type = FIFO_buffer[0] >> 5;
    Wire.beginTransmission(FUSB302BMPX_ADDR);
    Wire.write(ADDR_FIFOs);
    Wire.endTransmission();
    Wire.requestFrom(FUSB302BMPX_ADDR,2);
    Wire.readBytes(FIFO_buffer + 1, 2); 
    newestFIFO_HEADER_SOP.value = 0;
    newestFIFO_HEADER_SOP.IndexSort.Index_1 = FIFO_buffer[1];
    newestFIFO_HEADER_SOP.IndexSort.Index_2 = FIFO_buffer[2];;
    uint16_t test = newestFIFO_HEADER_SOP.value;
    // Serial.printf("[%d]收到 Header: ", millis());
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
    int ReadByteCount = newestFIFO_HEADER_SOP.parts.NumberOfDataObjects*4+2;
    if (newestFIFO_HEADER_SOP.parts.NumberOfDataObjects != 0) {
      Serial.printf("[%d]獲得 %d 組資料, 合計需要讀取 %d 個Byte\n", millis(), newestFIFO_HEADER_SOP.parts.NumberOfDataObjects, ReadByteCount);
      FIFO_Len = newestFIFO_HEADER_SOP.parts.NumberOfDataObjects + 5;
      Wire.beginTransmission(FUSB302BMPX_ADDR);
      Wire.write(ADDR_FIFOs);
      Wire.endTransmission();
      Wire.requestFrom(FUSB302BMPX_ADDR, ReadByteCount);
      Wire.readBytes(FIFO_buffer+3, ReadByteCount);
    }
    else {
      // Wire.beginTransmission(FUSB302BMPX_ADDR);
      // Wire.write(ADDR_FIFOs);
      // Wire.endTransmission();
      // Wire.requestFrom(FUSB302BMPX_ADDR, 2);
    }
    // if (newestFIFO_HEADER_SOP.parts.MessageType == 1 & newestFIFO_HEADER_SOP.parts.NumberOfDataObjects == 0) {
    //   //收到的是 Good CRC ， 忽略這筆
    //   Serial.printf("[%d]收到的是 Good CRC ， 忽略這筆\n", millis());
    //   GetMemeryData(&FIFO, ADDR_FIFOs);
    //   Serial.println(FIFO);
    //   GetMemeryData(&FIFO, ADDR_FIFOs);
    //   Serial.println(FIFO);
    //   GetMemeryData(&FIFO, ADDR_FIFOs);
    //   Serial.println(FIFO);
    //   GetMemeryData(&FIFO, ADDR_FIFOs);
    //   Serial.println(FIFO);
    //   GetMemeryData(&FIFO, ADDR_FIFOs);
    //   Serial.println(FIFO);
      
    //   FIFO_Len = 0;
    //   FIFO_buffer[0] = FIFO & 0xE0;
    //   newestFIFO_Rx_Info.SOP_Type = FIFO_buffer[0] >> 5;
    //   Wire.beginTransmission(FUSB302BMPX_ADDR);
    //   Wire.write(ADDR_FIFOs);
    //   Wire.endTransmission();
    //   Wire.requestFrom(FUSB302BMPX_ADDR,2);
    //   Wire.readBytes(FIFO_buffer + 1, 2); 
    //   newestFIFO_HEADER_SOP.value = 0;
    //   newestFIFO_HEADER_SOP.IndexSort.Index_1 = FIFO_buffer[1];
    //   newestFIFO_HEADER_SOP.IndexSort.Index_2 = FIFO_buffer[2];;
    //   uint16_t test = newestFIFO_HEADER_SOP.value;
    //   Serial.printf("[%d]收到 Header: ", millis());
    //   for (int i = 0; i < 16; i++) {
    //     bool b = test & 0x8000;
    //     if (b) {
    //       Serial.printf("1");
    //     } else {
    //       Serial.printf("0");
    //     }
    //     test = test << 1;
    //   }
    //   Serial.println();
    //   int ReadByteCount = newestFIFO_HEADER_SOP.parts.NumberOfDataObjects*4+2;
    //   if (newestFIFO_HEADER_SOP.parts.NumberOfDataObjects != 0) {
    //     Serial.printf("[%d]獲得 %d 組資料, 合計需要讀取 %d 個Byte\n", millis(), newestFIFO_HEADER_SOP.parts.NumberOfDataObjects, ReadByteCount);
    //     FIFO_Len = newestFIFO_HEADER_SOP.parts.NumberOfDataObjects + 5;
    //     Wire.beginTransmission(FUSB302BMPX_ADDR);
    //     Wire.write(ADDR_FIFOs);
    //     Wire.endTransmission();
    //     Wire.requestFrom(FUSB302BMPX_ADDR, ReadByteCount);
    //     Wire.readBytes(FIFO_buffer+3, ReadByteCount);
    //   }
    //   else {
    //     Wire.beginTransmission(FUSB302BMPX_ADDR);
    //     Wire.write(ADDR_FIFOs);
    //     Wire.endTransmission();
    //     Wire.requestFrom(FUSB302BMPX_ADDR, 2);
    //   }
    // }


    //! 清除FIFO Rx
    FUSB302BMPX.ClearFIFO_Rx();
    HeaderResp(newestFIFO_HEADER_SOP);
    if (newestFIFO_HEADER_SOP.parts.MessageType == MSG_Source_Capabilities) {
      newestFIFO_Rx_Info.SourceapabilityList.len = newestFIFO_HEADER_SOP.parts.NumberOfDataObjects;
      SourceapAbilityList.clear();
      for (int setChose = 0;setChose<newestFIFO_HEADER_SOP.parts.NumberOfDataObjects;setChose++) {
        SourceapAbility ThisData;
        ThisData.type = 0;
        ThisData.Fixed.bytes.index_A = FIFO_buffer[4*setChose + 3];
        ThisData.Fixed.bytes.index_B = FIFO_buffer[4*setChose + 4];
        ThisData.Fixed.bytes.index_C = FIFO_buffer[4*setChose + 5];
        ThisData.Fixed.bytes.index_D = FIFO_buffer[4*setChose + 6];
        // newestFIFO_Rx_Info.SourceapabilityList.list.push_back(ThisData);
        SourceapAbilityList.push_back(ThisData);
      }
    }
  }
  else {
    newestFIFO_HEADER_SOP.parts.MessageType = (unsigned int)0;
    // Serial.println(newestFIFO_Rx_Info.parts.MessageType);
  }
  return newestFIFO_Rx_Info;
}

// PD協議重製 (雙方皆重製) 與FUSB302中的寄存器重製為預設值
void C_FUSB302BMPX::ResetPDAndI2C()
{
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(ADDR_Reset);
  Wire.write(0b00000011);
  Wire.endTransmission();
}

// PD協議重製 (雙方皆重製)
void C_FUSB302BMPX::PD_Reset()
{
  Wire.beginTransmission(FUSB302BMPX_ADDR);
  Wire.write(ADDR_Reset);
  Wire.write(0b00000010);
  Wire.endTransmission();
}

void C_FUSB302BMPX::SendHardReset()
{
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Control3);
  Wire_->write(0b01000000);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::ResetI2CSetting()
{
  DP__RESET RESET = GetReset();
  RESET.SW_RES = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Reset);
  Wire_->write(*(u_int8_t*)&RESET);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::ClearFIFO_Rx()
{
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(0x07);  
  Wire_->write(0x04);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::OpenAllPower()
{
  DP__POWER POWER = GetPower();
  POWER.PWR = 0b1111;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Power);
  Wire_->write(*(u_int8_t*)&POWER);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::OpenAllAutoRetry()
{
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Control3);
  Wire_->write(0b00000111);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::OpenUsefulInterrupt()
{
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Maska);
  Wire_->write(0b11111100);
  Wire_->endTransmission();

  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Maskb);
  Wire_->write(0x01);
  Wire_->endTransmission();

  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Mask);
  Wire_->write(0xEF);
  Wire_->endTransmission();
}

int C_FUSB302BMPX::MeasureCC1()
{
  //! 打開 CC1 量測，同時關閉 CC2 量測
  DP__SWITCHES0 SWITCHES0_OLD = GetSwitches0();
  DP__SWITCHES0 SWITCHES0;
  SWITCHES0.PU_EN1 = 0;
  SWITCHES0.PU_EN2 = 0;
  SWITCHES0.VCONN_CC1 = 0;
  SWITCHES0.VCONN_CC2 = 0;
  SWITCHES0.MEAS_CC1 = 1;
  SWITCHES0.MEAS_CC2 = 0;
  SWITCHES0.PDWN1 = 1;
  SWITCHES0.PDWN2 = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0);
  Wire_->endTransmission();
  vTaskDelay(2/portTICK_PERIOD_MS);
  DP__STATUS0 STATUS0 = GetStatus0();
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0_OLD);
  Wire_->endTransmission();
  return STATUS0.BC_LVL;
}

int C_FUSB302BMPX::MeasureCC2()
{
  //! 打開 CC2 量測，同時關閉 CC1 量測
  DP__SWITCHES0 SWITCHES0_OLD = GetSwitches0();
  DP__SWITCHES0 SWITCHES0;
  SWITCHES0.PU_EN1 = 0;
  SWITCHES0.PU_EN2 = 0;
  SWITCHES0.VCONN_CC1 = 0;
  SWITCHES0.VCONN_CC2 = 0;
  SWITCHES0.MEAS_CC1 = 0;
  SWITCHES0.MEAS_CC2 = 1;
  SWITCHES0.PDWN1 = 1;
  SWITCHES0.PDWN2 = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0);
  Wire_->endTransmission();
  vTaskDelay(2/portTICK_PERIOD_MS);
  DP__STATUS0 STATUS0 = GetStatus0();
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0_OLD);
  Wire_->endTransmission();
  return STATUS0.BC_LVL;
}

int C_FUSB302BMPX::CheckCC()
{
  if (MeasureCC1() != 0) {
    return 1;
  }
  if (MeasureCC2() != 0) {
    return 2;
  }
  return 0;
}

void C_FUSB302BMPX::MeasureCCs()
{
  //! 先從 CC1 量起
  //! 打開 CC1 量測，同時關閉 CC2 量測
  DP__SWITCHES0 SWITCHES0 = GetSwitches0();
  uint8_t MEAS_CC1_Old = SWITCHES0.MEAS_CC1;
  uint8_t MEAS_CC2_Old = SWITCHES0.MEAS_CC2;
  SWITCHES0.MEAS_CC1 = 1;
  SWITCHES0.MEAS_CC2 = 0;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0);
  Wire_->endTransmission();
	/* CC1 is now being measured by FUSB302. */
	/* Wait on measurement */
  vTaskDelay(1/portTICK_PERIOD_MS);
  DP__STATUS0 CC1_Status = GetStatus0();
  Serial.printf("CC1 量測: %d,\t", CC1_Status.BC_LVL);
  //! 換成量測 CC2
  //! 打開 CC2 量測，同時關閉 CC1 量測
  SWITCHES0 = GetSwitches0();
  SWITCHES0.MEAS_CC1 = 0;
  SWITCHES0.MEAS_CC2 = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0);
  Wire_->endTransmission();
  vTaskDelay(1/portTICK_PERIOD_MS);
  DP__STATUS0 CC2_Status = GetStatus0();
  Serial.printf("CC2 量測: %d\n", CC2_Status.BC_LVL);

  //! 恢復量測流程前狀態
  SWITCHES0 = GetSwitches0();
  SWITCHES0.MEAS_CC1 = MEAS_CC1_Old;
  SWITCHES0.MEAS_CC2 = MEAS_CC2_Old;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches0);
  Wire_->write(*(u_int8_t*)&SWITCHES0);
  Wire_->endTransmission();
}

void C_FUSB302BMPX::EnableAutoCRC()
{
  DP__SWITCHES1 SWITCHES1 = GetSwitches1();
  SWITCHES1.AUTO_CRC = 1;
  Wire_->beginTransmission(FUSB302BMPX_ADDR);
  Wire_->write(ADDR_Switches1);
  Wire_->write(*(u_int8_t*)&SWITCHES1);
  Wire_->endTransmission();
}

C_FUSB302BMPX FUSB302BMPX;