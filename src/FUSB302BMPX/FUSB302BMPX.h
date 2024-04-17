#ifndef __FUSB302BMPX__H
#define __FUSB302BMPX__H
#include "Arduino.h"
#include "Wire.h"
#include "struct_setting.h"

class C_FUSB302BMPX 
{
  public:
    C_FUSB302BMPX(void){};
    void SetWire(TwoWire *_Wire);
    void GetMemeryData(u_int8_t *buffer, u_int16_t addr);
    DP__DEVICE_ID GetDeviceID();
    DP__SWITCHES0 GetSwitches0();
    DP__SWITCHES1 GetSwitches1();
    DP__MEASURE GetMeasure();
    DP__SLICE GetSlice();
    DP__CONTROL0 GetControl0();
    DP__CONTROL1 GetControl1();
    DP__CONTROL2 GetControl2();
    DP__CONTROL3 GetControl3();
    DP__MASK GetMask();
    DP__POWER GetPower();
    DP__RESET GetReset();
    DP__OCPREG GetOCPreg();
    DP__MASKA GetMaskA();
    DP__MASKB GetMaskB();
    DP__CONTROL4 GetControl4();
    DP__STATUS0A GetStatus0A();
    DP__STATUS1A GetStatus1A();
    DP__INTERRUPTA GetInteruptA();
    DP__INTERRUPTB GetInteruptB();
    DP__STATUS0 GetStatus0();
    DP__STATUS1 GetStatus1();
    DP__INTERRUPT GetInterupt();

    void HeaderResp(DP__HEADER_SOP header);

    void SendHardReset(); //! 最高級別重製
    void ResetI2CSetting();
    void ClearFIFO_Rx();
    void OpenAllPower();
    void OpenAllAutoRetry();
    void OpenUsefulInterrupt();
    int MeasureCC1();
    int MeasureCC2();
    int CheckCC(); //! 檢測CC腳位是否有連接 0: 無, 1: 有
    void MeasureCCs();
    void EnableAutoCRC();
    void EnableTXCC();
  private:
    TwoWire *Wire_;
};

extern C_FUSB302BMPX FUSB302BMPX;
#endif