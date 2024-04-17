//https://www.graniteriverlabs.com/zh-tw/technical-blog/usb-pd-power-delivery-spec-versions
//https://www.graniteriverlabs.com/zh-tw/technical-blog/application-notes-usbc-role-swap

#ifndef __FUSB302BMPX_struct__H
#define __FUSB302BMPX_struct__H

#define FUSB302BMPX_ADDR 0x22
#define ADDR_DEVICE_ID 0x01
#define ADDR_Switches0 0x02
#define ADDR_Switches1 0x03
#define ADDR_Measure   0x04
#define ADDR_Slice     0x05
#define ADDR_Control0  0x06
#define ADDR_Control1  0x07
#define ADDR_Control2  0x08
#define ADDR_Control3  0x09
#define ADDR_Mask      0x0A
#define ADDR_Power     0x0B
#define ADDR_Reset     0x0C
#define ADDR_OCPreg    0x0D
#define ADDR_Maska     0x0E
#define ADDR_Maskb     0x0F
#define ADDR_Control4  0x10
#define ADDR_Status0a  0x3C
#define ADDR_Status1a  0x3D
#define ADDR_Interrupta  0x3E
#define ADDR_Interruptb  0x3F
#define ADDR_Status0   0x40
#define ADDR_Status1   0x41
#define ADDR_Interrupt 0x42
#define ADDR_FIFOs     0x43

struct DP__DEVICE_ID {
  unsigned int Revision_ID : 2;
  unsigned int Product_ID : 2;
  unsigned int Version_ID : 4;
};

typedef union {
    struct {
        uint8_t low : 8;
        uint8_t high : 8;
    } parts;
    uint16_t value;
} DP__FIFO_I_SET;

typedef union {
    struct {
      // 低位
      uint8_t MessageType : 5;
      uint8_t PortDataRole : 1;
      uint8_t SpecificationReversio : 2;
      // 高位
      // 低位
      uint8_t PortPowerRole : 1;
      uint8_t MessageID : 3;
      uint8_t NumberOfDataObjects : 3;
      uint8_t Extended : 1;
      // 高位
    } parts;
    struct {
      uint8_t Index_1 : 8;
      uint8_t Index_2 : 8;
    } IndexSort;
    uint16_t value;
} DP__HEADER_SOP;

typedef union {
    struct {
        uint8_t low : 8;
        uint8_t high : 8;
    } parts;
    uint16_t value;
} DP__FIFO_V_SET;

// typedef union {
//   struct {
//     // 低位
//     uint32_t SOP : 32 = 0x12121213;
//     uint8_t DataInfo : 8 = 0x86; // 0x80 + 資料Byte數
//     uint8_t Header_Index1 : 8 = 0b01000010;
//     uint8_t Header_Index2_a : 4 = 0b0001;
//     uint8_t MessageID : 3 = 0;
//     uint8_t Header_Index2_b : 1 = 0b1;
//     uint16_t I_Set : 10;
//     uint16_t V_Set : 10;
//     uint16_t NC_1 : 12 = 0;
//     uint8_t JAM_CRC: 8 = 0xff;
//     uint8_t EOP: 8 = 0x14;
//     uint8_t TXON: 8 = 0xA1;
//     // 高位
//   } parts;
// } DP__REQUEST__POWER;

struct DP__REQUEST__POWER {
  uint32_t SOP : 32 = 0x13121212;
  uint8_t DataInfo : 8 = 0x86; // 0x80 + 資料Byte數
  uint8_t Header_Index1 : 8 = 0b01000010;
  uint8_t Header_Index2_a : 4 = 0b0001;
  uint8_t MessageID : 3 = 0;
  uint8_t Header_Index2_b : 1 = 0;
  uint16_t I_Set : 10;
  uint16_t V_Set : 10;
  uint16_t NC_1 : 12 = 0;
  uint8_t JAM_CRC: 8 = 0xff;
  uint8_t EOP: 8 = 0x14;
  uint8_t TXON: 8 = 0xA1;
};

struct DP__V_SET {
  unsigned int high : 4;
  unsigned int low : 6;
};

/**
 * (Address: 02h; Reset Value: 0x0000_0011; Type: Read/Write)
 * 
 * 電源供應端 （Source）會在CC1和CC2配置Rp拉高電阻，未接任和裝置時，為高電位。
 * 
 * 電源接受端 （Sink）則是在CC1和CC2配置Rd拉低電阻（5.1KΩ），未接任和裝置時，為低電位。
 * 
 */
struct DP__SWITCHES0 {
  unsigned int PDWN1 : 1;  // R/W - 1: Device pull down on CC1. 0: no pull down
  unsigned int PDWN2 : 1;  // R/W - 1: Device pull down on CC2. 0: no pull down
  unsigned int MEAS_CC1 : 1;  // R/W - 1: Use the measure block to monitor or measure the voltage on CC1
  unsigned int MEAS_CC2 : 1;  // R/W - 1: Use the measure block to monitor or measure the voltage on CC2
  unsigned int VCONN_CC1 : 1;  // R/W - 1: Turn on the VCONN current to CC1 pin
  unsigned int VCONN_CC2 : 1;  // R/W - 1: Turn on the VCONN current to CC2 pin
  unsigned int PU_EN1 : 1;  // R/W - 1: Apply host pull up current to CC1 pin
  unsigned int PU_EN2 : 1;  // R/W - 1: Apply host pull up current to CC2 pin
};

/**
 * (Address: 03h; Reset Value: 0x0010_0000; Type: Read/Write)
 * 
 * 
 */
struct DP__SWITCHES1 {
  unsigned int TXCC1 : 1;  // R/W - 1: Enable BMC transmit driver on CC1 pin
  unsigned int TXCC2 : 1;  // R/W - 1: Enable BMC transmit driver on CC2 pin
  /** R/W - 
    1: Starts the transmitter automatically when a message with a
      good CRC is received and automatically sends a GoodCRC
      acknowledge packet back to the relevant SOP*
    0: Feature disabled
  */
  unsigned int AUTO_CRC : 1;
  unsigned int NC : 1;
  /** R/W - 
    Bit used for constructing the GoodCRC acknowledge packet. This
    bit corresponds to the Port Data Role bit in the message header.
    For SOP:
    1: Source （SRC ) (供電端)
    0: Sink （SNK） (受電端)
  */
  unsigned int DATAROLE : 1;
  /** R/W - 
    Bit used for constructing the GoodCRC acknowledge packet.
    These bits correspond to the Specification Revision bits in the
    message header:
    00: Revision 1.0
    01: Revision 2.0
    10: Do Not Use
    11: Do Not Use
  */
  unsigned int SPECREV : 2;
  /** R/W - 
    Bit used for constructing the GoodCRC acknowledge packet. This
    bit corresponds to the Port Power Role bit in the message header if
    an SOP packet is received:
    1: Source if SOP (供電端)
    0: Sink if SOP (受電端)
  */
  unsigned int POWERROLE : 1;
};

/**
 * (Address: 04h; ·Reset Value: 0x0011_0001; Type: Read/Write)
 * 
 */
struct DP__MEASURE {
  /** R/W - 
    Measure Block DAC data input. LSB is equivalent to 42 mV of
    voltage which is compared to the measured CC voltage.
    The measured CC is selected by MEAS_CC2, or MEAS_CC1 bits.
    MDAC[5:0] MEAS_VBUS = 0 MEAS_VBUS = 1 Unit
    00_0000 0.042 0.420 V
    00_0001 0.084 0.840 V
    11_0000 2.058 20.58 V
    11_0011 2.184 21.84 V
    11_1110 2.646 26.46 V
    11_1111 > 2.688 26.88 V
  */
  unsigned int MDAC : 6;
  /** R/W - 
    0: MDAC/comparator measurement is controlled by MEAS_CC*
    bits
    1: Measure VBUS with the MDAC/comparator. This requires
    MEAS_CC* bits to be 0
  */
  unsigned int MEAS_VBUS : 1;
};

struct DP__SLICE {
  /** R/W - 
    BMC Slicer DAC data input. Allows for a programmable threshold
    so as to meet the BMC receive mask under all noise conditions.
  */
  unsigned int SDAC : 6;
  /** R/W - 
    Adds hysteresis where there are now two thresholds, the lower
    threshold which is always the value programmed by SDAC[5:0]
    and the higher threshold that is:
    11: 255 mV hysteresis: higher threshold = (SDAC value + 20hex)
    10: 170 mV hysteresis: higher threshold = (SDAC value + Ahex)
    01: 85 mV hysteresis: higher threshold = (SDAC value + 5)
    00: No hysteresis: higher threshold = SDAC value
  */
  unsigned int SDAC_HYS : 2;
};

struct DP__CONTROL0 {
  /** W/C - 
    1: Start transmitter using the data in the transmit FIFO. Preamble
    is started first. During the preamble period the transmit data
    can start to be written to the transmit FIFO. Self clearing.
  */
  unsigned int TX_START : 1;
  /** R/W - 
    1: Starts the transmitter automatically when a message with
      a good CRC is received. This allows the software to take as
      much as 300 S to respond after the I_CRC_CHK interrupt is
      received. Before starting the transmitter, an internal timer
      waits for approximately 170 S before executing the transmit
      start and preamble
    0: Feature disabled
  */
  unsigned int AUTO_PRE : 1;
  /** R/W - 
    1: Controls the host pull up current enabled by PU_EN[2:1]:
    00: No current
    01: 80 A – Default USB power
    10: 180 A – Medium Current Mode: 1.5 A
    11: 330 A – High Current Mode: 3 A
  */
  unsigned int HOST_CUR : 2;
  unsigned int NC : 1;
  /** R/W - 
    1: Mask all interrupts
    0: Interrupts to host are enabled
  */
  unsigned int INT_MASK : 1;
  unsigned int TX_FLUSH : 1; // W/C - 1: Self clearing bit to flush the content of the transmit FIFO
};

struct DP__CONTROL1 {
  /** R/W - 
    1: Enable SOP‘(SOP prime) packets
    0: Ignore SOP‘(SOP prime) packets
  */
  unsigned int ENSOP1 : 1;
  /** R/W - 
    1: Enable SOP”(SOP double prime) packets
    0: Ignore SOP”(SOP double prime) packets
  */
  unsigned int ENSOP2 : 1; 
  unsigned int RX_FLUSH : 1; // W/C - 1: Self clearing bit to flush the content of the receive FIFO
  unsigned int NC : 1;
  unsigned int BIST_MODE2 : 1; // R/W - 1: Sent BIST Mode 01s pattern for testing
  /** R/W - 
    1: Enable SOP‘_DEBUG (SOP prime debug) packets
    0: Ignore SOP‘_DEBUG (SOP prime debug) packets
  */
  unsigned int ENSOP1DB : 1;
  /** R/W - 
    1: Enable SOP”_DEBUG (SOP double prime debug) packets
    0: Ignore SOP”_DEBUG (SOP double prime debug) packets
  */
  unsigned int ENSOP2DB : 1;
};

struct DP__CONTROL2 {
  /** R/W - 
    1: Enable DRP, SNK or SRC Toggle autonomous functionality
    0: Disable DRP, SNK and SRC Toggle functionality
  */
  unsigned int TOGGLE : 1;
  /** R/W - 
    11: Enable SRC polling functionality if TOGGLE=1
    10: Enable SNK polling functionality if TOGGLE=1
    01: Enable DRP polling functionality if TOGGLE=1
    00: Do Not Use
  */
  unsigned int MODE : 2;
  /** R/W - 
    1: Enable Wake Detection functionality if the power state is
    correct
    0: Disable Wake Detection functionality
  */
  unsigned int WAKE_EN : 1;
  unsigned int NC : 1;
  /** R/W - 
    1: When TOGGLE=1 only Rd values will cause the TOGGLE
    state machine to stop toggling and trigger the I_TOGGLE
    interrupt
    0: When TOGGLE=1, Rd and Ra values will cause the TOGGLE
    state machine to stop toggling
  */
  unsigned int TOG_RD_ONLY : 1;
  /** N/A (??????????) - 
    00: Don’t go into the DISABLE state after one cycle of toggle
    01: Wait between toggle cycles for tDIS time of 40 ms
    10: Wait between toggle cycles for tDIS time of 80 ms
    11: Wait between toggle cycles for tDIS time of 160 ms
  */
  unsigned int TOG_SAVE_PWR : 2;
};

struct DP__CONTROL3 {
  /** R/W - 
    1: Enable automatic packet retries if GoodCRC is not received
    0: Disable automatic packet retries if GoodCRC not received
  */
  unsigned int AUTO_RETRY : 1;
  /** R/W - 
    11: Three retries of packet (four total packets sent)
    10: Two retries of packet (three total packets sent)
    01: One retry of packet (two total packets sent)
    00: No retries (similar to disabling auto retry)
  */
  unsigned int N_RETRIES : 2;
  /** R/W - 
    1: Enable automatic soft reset packet if retries fail
    0: Disable automatic soft reset packet if retries fail
  */
  unsigned int AUTO_SOFTRESET : 1;
  /** R/W - 
    1: Enable automatic hard reset packet if soft reset fail
    0: Disable automatic hard reset packet if soft reset fail
  */
  unsigned int AUTO_HARDRESET : 1;
  /** R/W - 
    1: BIST mode. Receive FIFO is cleared immediately after
    sending GoodCRC response
    0: Normal operation, All packets are treated as usual
  */
  unsigned int BIST_TMODE : 1;
  /** W/C - 
    1: Send a hard reset packet (highest priority)
    0: Don’t send a soft reset packet
  */
  unsigned int SEND_HARD_RESET : 1;
};

struct DP__MASK {
  /** R/W - 
1: Mask I_VBUSOK interrupt bit
0: Do not mask
  */
  unsigned int M_VBUSOK : 1;
  /** R/W - 
1: Mask interrupt for a transition in CC bus activity
0: Do not mask
  */
  unsigned int M_ACTIVITY : 1;
  /** R/W - 
1: Mask I_COMP_CHNG interrupt for change is the value of
COMP, the measure comparator
0: Do not mask
  */
  unsigned int M_COMP_CHNG : 1;
  /** R/W - 
1: Mask interrupt from CRC_CHK bit
0: Do not mask
  */
  unsigned int M_CRC_CHK : 1;
  /** R/W - 
1: Mask the I_ALERT interrupt bit
0: Do not mask
  */
  unsigned int M_ALERT : 1;
  /** R/W - 
1: Mask the I_WAKE interrupt bit
0: Do not mask
  */
  unsigned int M_WAKE : 1;
  /** R/W - 
1: Mask the I_COLLISION interrupt bit
0: Do not mask
  */
  unsigned int M_COLLISION : 1;
  /** R/W - 
1: Mask a change in host requested current level
0: Do not mask
  */
  unsigned int M_BC_LVL : 1;
};

struct DP__POWER {
  /** R/W - 
Power enables:
PWR[0]: Bandgap and wake circuit
PWR[1]: Receiver powered and current references for Measure
block
PWR[2]: Measure block powered
PWR[3]: Enable internal oscillator
  */
  unsigned int PWR : 4;
};

struct DP__RESET {
  unsigned int SW_RES : 1; // W/C - 1: Reset just the PD logic for both the PD transmitter and receiver
  unsigned int PD_RESET : 1; // W/C - 1: Reset the FUSB302B including the I2C registers to their default values
};

struct DP__OCPREG {
  /** R/W - 
111: max_range (see bit definition above for OCP_RANGE)
110: 7 × max_range / 8
101: 6 × max_range / 8
100: 5 × max_range / 8
011: 4 × max_range / 8
010: 3 × max_range / 8
001: 2 × max_range / 8
000: max_range / 8
  */
  unsigned int OCP_CUR : 3;
  /** R/W - 
1: OCP range between 100−800 mA (max_range = 800 mA)
0: OCP range between 10−80 mA (max_range = 80 mA)
  */
  unsigned int OCP_RANGE : 1;
};

struct DP__MASKA {
  unsigned int M_HARDRST : 1; // R/W - 1: Mask the I_HARDRST interrupt
  unsigned int M_SOFTRST : 1; // R/W - 1: Mask the I_SOFTRST interrupt
  unsigned int M_TXSENT : 1; // R/W - 1: Mask the I_TXSENT interrupt
  unsigned int M_HARDSENT : 1; // R/W - 1: Mask the I_HARDSENT interrupt
  unsigned int M_RETRYFAIL : 1; // R/W - 1: Mask the I_RETRYFAIL interrupt
  unsigned int M_SOFTFAIL : 1; // R/W - 1: Mask the I_SOFTFAIL interrupt
  unsigned int M_TOGDONE : 1; // R/W - 1: Mask the I_TOGDONE interrupt
  unsigned int M_OCP_TEMP : 1; // R/W - 1: Mask the I_OCP_TEMP interrupt
};

struct DP__MASKB {
  unsigned int M_GCRCSENT : 1; // R/W - 1: Mask the I_GCRCSENT interrupt
};

struct DP__CONTROL4 {
  unsigned int TOG_EXIT_AUD : 1; // R/W - 1: In auto Rd only Toggle mode, stop Toggle at Audio accessory (Ra on both CC)
};


struct DP__STATUS0A {
  unsigned int HARDRST : 1; // R - 1: Hard Reset PD ordered set has been received
  unsigned int SOFTRST : 1; // R - 1: One of the packets received was a soft reset packet
  /** R - 
  Internal power state when logic internals needs to control the
  power state. POWER3 corresponds to PWR3 bit and POWER2
  corresponds to PWR2 bit. The power state is the higher of both
  PWR[3:0] and {POWER3, POWER2, PWR[1:0]} so that if one is
  03 and the other is F then the internal power state is F
  */
  unsigned int POWER2 : 1;
  /** R - 
  Internal power state when logic internals needs to control the
  power state. POWER3 corresponds to PWR3 bit and POWER2
  corresponds to PWR2 bit. The power state is the higher of both
  PWR[3:0] and {POWER3, POWER2, PWR[1:0]} so that if one is
  03 and the other is F then the internal power state is F
  */
  unsigned int POWER3 : 1;
  /** R - 
1: All packet retries have failed to get a GoodCRC acknowledge.
This status is cleared when a START_TX, TXON or
SEND_HARD_RESET is executed
  */
  unsigned int RETRY_FAIL : 1;
  /** R - 
1: All soft reset packets with retries have failed to get
a GoodCRC acknowledge. This status is cleared when
a START_TX, TXON or SEND_HARD_RESET is executed
  */
  unsigned int SOFTFAIL : 1;
};

struct DP__STATUS1A {
  unsigned int RXSOP : 1; // R - 1: Indicates the last packet placed in the RxFIFO is type SOP
  /** R - 
1: Indicates the last packet placed in the RxFIFO is type
SOP’_DEBUG (SOP prime debug)
  */
  unsigned int RXSOP_1DB : 1;
  /** R - 
1: Indicates the last packet placed in the RxFIFO is type
SOP”_DEBUG (SOP double prime debug)
  */
  unsigned int RXSOP_2DB : 1;
  /** R - 
000: Toggle logic running (processor has previously written
TOGGLE=1)
001: Toggle functionality has settled to SRCon CC1
(STOP_SRC1 state)
010: Toggle functionality has settled to SRCon CC2
(STOP_SRC2 state)
101: Toggle functionality has settled to SNKon CC1
(STOP_SNK1 state)
110: Toggle functionality has settled to SNKon CC2
(STOP_SNK2 state)
111: Toggle functionality has detected AudioAccessory with vRa
on both CC1 and CC2 (settles to STOP_SRC1 state)
Otherwise: Not defined (do not interpret)
  */
  unsigned int TOGSS : 3;
};

struct DP__INTERRUPTA {
  unsigned int I_HARDRST : 1; // R/C - 1: Received a hard reset ordered set
  unsigned int I_SOFTRST : 1; // R/C - 1: Received a soft reset packet
  unsigned int I_TXSENT : 1; // R/C - 1: Interrupt to alert that we sent a packet that was acknowledged with a GoodCRC response packe
  unsigned int I_HARDSENT : 1; // R/C - 1: Interrupt from successfully sending a hard reset ordered set
  unsigned int I_RETRYFAIL : 1; // R/C - 1: Interrupt from automatic packet retries have failed
  unsigned int I_SOFTFAIL : 1; // R/C - 1: Interrupt from automatic soft reset packets with retries have failed
  unsigned int I_TOGDONE : 1; // R/C - 1: Interrupt indicating the TOGGLE functionality was terminated because a device was detected
  unsigned int I_OCP_TEMP : 1; // R/C - 1: Interrupt from either a OCP event on one of the VCONN switches or an over-temperature event
};

struct DP__INTERRUPTB {
  unsigned int I_GCRCSENT : 1; // R/C - 1: Sent a GoodCRC acknowledge packet in response to an incoming packet that has the correct CRC value
};

struct DP__STATUS0 {
  unsigned int BC_LVL : 2;
  unsigned int WAKE : 1;
  unsigned int ALERT : 1;
  unsigned int CRC_CHK : 1;
  unsigned int COMP : 1;
  unsigned int ACTIVITY : 1;
  unsigned int VBUSOK : 1;
};

struct DP__STATUS1 {
  unsigned int OCP : 1;
  unsigned int OVRTEMP : 1;
  unsigned int TX_FULL : 1;
  unsigned int TX_EMPTY : 1;
  unsigned int RX_FULL : 1;
  unsigned int RX_EMPTY : 1;
  unsigned int RXSOP1 : 1;
  unsigned int RXSOP2 : 1;
};

struct DP__INTERRUPT {
  unsigned int I_BC_LVL : 1;
  unsigned int I_COLLISION : 1;
  unsigned int I_WAKE : 1;
  unsigned int I_ALERT : 1;
  unsigned int I_CRC_CHK : 1;
  unsigned int I_COMP_CHNG : 1;
  unsigned int I_ACTIVITY : 1;
  unsigned int I_VBUSOK : 1;
};

#endif