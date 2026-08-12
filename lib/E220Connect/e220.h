#include <stdint.h>
#include "hardware/uart.h"
#include "type.h"

#ifndef E220_H
#define E220_H
//#include "Arduino.h"
//#include <SoftwareSerial.h>
#define SEND_MAX 255
#define GET_MAX 255

/// @brief ArduinoとLoRaモジュール間のボーレート
#define UART_RATE1200 0b000
#define UART_RATE2400 0b001
#define UART_RATE4800 0b010
#define UART_RATE9600 0b011
#define UART_RATE19200 0b100
#define UART_RATE38400 0b101
#define UART_RATE57600 0b110
#define UART_RATE115200 0b111

/// @brief 双方の伝送レートは同じでなければなりません。伝送レートが高いほど、遅延が小さくなりますが、伝送距離が短くなります。
#define AIR_RATE15625 0b00000
#define AIR_RATE9375 0b00100
#define AIR_RATE5469 0b01000
#define AIR_RATE3125 0b01100
#define AIR_RATE1758 0b10000
#define AIR_RATE31250 0b00000
#define AIR_RATE18750 0b00101
#define AIR_RATE10938 0b01001
#define AIR_RATE6250 0b01101
#define AIR_RATE3516 0b10001
#define AIR_RATE1953 0b10101
#define AIR_RATE62500 0b00010
#define AIR_RATE37500 0b00110
#define AIR_RATE21875 0b01010
#define AIR_RATE12500 0b01110
#define AIR_RATE7031 0b10010
#define AIR_RATE3906 0b10110
#define AIR_RATE2148 0b11010

/// @brief サブパケット長
#define SUB_PACKET200 0b00
#define SUB_PACKET128 0b01
#define SUB_PACKET64 0b10
#define SUB_PACKET32 0b11

/// @brief RSSI 環境ノイズの有効化
#define RSSI_NOISE_Disabled 0b0
#define RSSI_NOISE_Enabled 0b1

/// @brief 送信出力電力
#define TRANSMIT_POWER13 0b00
#define TRANSMIT_POWER12 0b01
#define TRANSMIT_POWER7 0b10
#define TRANSMIT_POWER0 0b11

/// @brief RSSIバイトの有効化
#define RSSI_BYTE_Disabled 0b0
#define RSSI_BYTE_Enabled 0b1

/// @brief 送信方法
#define TRANSMISSION_METHOD_TRANSPARENT 0b0
#define TRANSMISSION_METHOD_FIXED 0b1

/// @brief WORサイクル
#define WOR_CYCLE500 0b000
#define WOR_CYCLE1000 0b001
#define WOR_CYCLE1500 0b010
#define WOR_CYCLE2000 0b011
#define WOR_CYCLE2500 0b100
#define WOR_CYCLE3000 0b101

enum E220Enum{
  addh,
  addl,
  uart_rate,
  air_rate,
  sub_packet,
  rssi_noise,
  transmit_power,
  ch,
  rssi_byte,
  transmission_method,
  wor_cycle,
  crypt_h,
  crypt_l,
  version,
};

class Time{
  public:
  void resetTimeout(const uint32_t _timeout);
  bool isTimeOut(void);
  private:
  uint32_t timeout = 0;
  uint32_t count = 0;
};

union Reg0{
  struct{
  int air_rate : 5;
  int uart_rate : 3;
  };
  int val;
};

union Reg1{
  struct{
    int transmit_power : 2;
    int unused : 3;
    int rssi_noise : 1;
    int sub_packet : 2;
  };
  int val;
};

union Reg3{
  struct{
    int wor_cycle : 3;
    int unused : 3;
    int transmission_method : 1;
    int rssi_byte : 1;
  };
  int val;
};

union Resister{
    struct{
      int addh;
      int addl;
      union Reg0 reg0;
      union Reg1 reg1;
      int ch;
      union Reg3 reg3;
      int crypt_h;
      int crypt_l;
      int version;
    };
    int val[9];

};

extern uart_inst_t *hs;

class E220{
  public:
  E220(uart_inst_t *_serial,const unsigned long _boud_rate,const int _m0_pin,const int _m1_pin,const int _aux_pin);
  bool begin(void);
  bool is_checksum(int sum, int checksum);
  bool sendDataFixed(const int _addh,const int _addl,const int _ch,const uint32_t _timeout);
  bool sendData(const uint32_t _timeout);
  bool setData(const int *_pData,const int _size);
  void setResister(enum E220Enum _get_enum,const int _val);
  void setDefaultResister(void);
  bool sendResister(const uint32_t _timeout);
  void clearData(void);
  bool getData(const int _size,const uint32_t _timeout);
  bool getDataWithCobs(const int _size,const uint32_t _timeout);
  bool readData(int* _target_len,const int _size);
  bool readDataWithCobs(int* _target_len,const int _size);
  int readDataSendByPico(Read_ds4* output,int* inputdata);
  int readLEDData(int* LEDoutput,int* inputdata);
  bool readAux(void);
  bool isConnected(void);
  bool waitForAuxIsReady(const uint32_t _timeout);
  void initResister(void);
  bool setDataWithCobs(const int *_pData,const int _size);
  bool serialWrite(const int _data);
  bool serialWaitClear(void);
  int serialWriteAvailable(void);
  int serialReadAvailable(void);
  int serialRead(void);
  private:
  bool switchStateToNormal(const uint32_t _timeout);
  bool switchStateToWorTransmit(const uint32_t _timeout);
  bool switchStateToWorReceive(const uint32_t _timeout);
  bool switchStateToConfig(const uint32_t _timeout);
  void setM0(void);
  void resetM0(void);
  void setM1(void);
  void resetM1(void);
  void switchSerialRate(const unsigned long _boud_rate);
  int getSendSize(void);
  bool setSendSize(const int _val);
  bool addSendSize(const int _val);
  void clearGetData(void);
  int send_data[SEND_MAX] = {0};
  int get_data[GET_MAX] ={0};
  int send_size = 0;//送信データのサイズ
  int get_size = 0;
  uart_inst_t *hs;
  int m0_pin = 0;
  int m1_pin = 0;
  int aux_pin = 0;
  unsigned long boud_rate = 0;
  Resister resister;
};
#endif