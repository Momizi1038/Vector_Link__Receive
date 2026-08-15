#include "e220.h"
#include "type.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdlib.h>
#include <stdio.h>
#include <vector>

void Time::resetTimeout(const uint32_t _timeout){
  timeout = _timeout;
  //millis()
  count = to_ms_since_boot(get_absolute_time());
}
bool Time::isTimeOut(void){
  //millis()
  if(to_ms_since_boot(get_absolute_time()) - count > timeout){
    return true;
  }
  return false;
}

//E220::E220(HardwareSerial *_serial,const unsigned long _boud_rate,const int _m0_pin,const int _m1_pin,const int _aux_pin){
E220::E220(uart_inst_t *_serial,const unsigned long _boud_rate,const int _m0_pin,const int _m1_pin,const int _aux_pin){
  this->hs = _serial;
  m0_pin = _m0_pin;
  m1_pin = _m1_pin;
  aux_pin = _aux_pin;
  boud_rate = _boud_rate;
  for(int i = 0; i < 9; i++){
    resister.val[i] = 0;
  }
}

bool E220::begin(void){
  // switchSerialRate(boud_rate);
  // pinMode(m0_pin, OUTPUT);
  // pinMode(m1_pin, OUTPUT);
  // pinMode(aux_pin, INPUT);
  gpio_init(m0_pin);
  gpio_init(m1_pin);
  gpio_init(aux_pin);
  gpio_set_dir(m0_pin, GPIO_OUT);
  gpio_set_dir(m1_pin, GPIO_OUT);
  gpio_set_dir(aux_pin, GPIO_IN);  
  return switchStateToNormal(1000);
}

bool E220::is_checksum(int sum, int checksum) {
  printf("sum:%d,sum/255:%d,check:%d\n",sum,sum%255,checksum);
    return (sum % 255 == checksum -1);
}

bool E220::serialWaitClear(void){
  while(!uart_is_writable(hs)){
    tight_loop_contents();
  }
  return true;
}

int E220::serialReadAvailable(void){
  return uart_is_readable(this->hs) ? 1 : 0;
}

int E220::serialWriteAvailable(void){
  return uart_is_writable(this->hs) ? 1 : 0;
}

bool E220::sendDataFixed(const int _addh,const int _addl,const int _ch,const uint32_t _timeout){
  Time time;
  time.resetTimeout(_timeout);
  bool _flag = waitForAuxIsReady(_timeout);
  if(_flag == false)return false;
  while(serialWriteAvailable() <= 0){
    if(time.isTimeOut()) return false;
  }
  serialWrite(_addh );
  serialWrite(_addl );
  serialWrite(_ch );
  for(int i = 0; i < getSendSize(); i++){
    _flag = serialWrite(send_data[i]);
    //printf("%d,",send_data[i]);
    if(_flag == false)return false;
  }
  clearData();
  return true;
}

bool E220::sendData(const uint32_t _timeout){
  Time time;
  time.resetTimeout(_timeout);
  bool _flag = waitForAuxIsReady(_timeout);
  if(_flag == false)return false;
  while(serialWriteAvailable() <= 0){
    if(time.isTimeOut()) return false;
  }
  for(int i = 0; i < getSendSize(); i++){
    _flag = serialWrite(send_data[i]);
    if(_flag == false)return false;
  }
  clearData();
  return true;
}

// bool E220::setDataWithCobs(const int *_pData, const int _size) {
//   std::vector<int> s(_size + 2, 0); // 動的配列を使用
//   clearData();

//   // データのコピー
//   for (int i = 0; i < _size; i++) {
//     s[i + 1] = _pData[i];
//   }

//   // COBSエンコーディング
//   int i2 = 1; // 1から開始
//   int code_pos = 0;
//   s[0] = 0; // 最初のコードバイトを初期化

//   for (int i = 1; i < _size + 2; i++) {
//     if (s[i] == 0) {
//       s[code_pos] = i2; // コードバイトを更新
//       code_pos = i;      // 新しいコードバイトの位置を記録
//       i2 = 1;           // カウンタをリセット
//     } else {
//       i2++;
//       if (i2 == 256) { // 最大値は255
//         s[code_pos] = 255;
//         code_pos = i;
//         i2 = 1;
//       }
//     }
//   }
//   s[code_pos] = i2; // 最後のコードバイトを書き込む

//   setData(s.data(), _size + 2); // setDataに配列の先頭ポインタを渡す
//   return true;
// }

bool E220::setDataWithCobs(const int *_pData,const int _size){
  int s[255] = {0};
  clearData();
  for(int i = 0; i < _size; i++){
    s[i+1] = _pData[i];
  }
  int i2 = 0;
  for(int i = _size; i >= 0; i--){
    i2++;
    if(s[i] == 0){
      s[i] = i2;
      i2 = 0;
    }
  }
  setData(s,_size+2);
  return true;
}

bool E220::setData(const int *_pData,const int _size){
  int _send_size = getSendSize();
  bool _flag = addSendSize(_size);
  if(_flag == false)return false;
  for(int i = 0; i < _size; i++){
    send_data[_send_size+i] = _pData[i];
  }
  return true;
}

void E220::setResister(enum E220Enum _get_enum,const int _val){
  switch(_get_enum){
    case addh:
    resister.addh = _val;
    break;
    case addl:
    resister.addl = _val;
    break;
    case uart_rate:
    resister.reg0.uart_rate = _val & 0b111;
    break;
    case air_rate:
    resister.reg0.air_rate = _val & 0b11111;
    break;
    case sub_packet:
    resister.reg1.sub_packet = _val & 0b11;
    break;
    case rssi_noise:
    resister.reg1.rssi_noise = _val & 0b1;
    break;
    case transmit_power:
    resister.reg1.transmit_power = _val & 0b11;
    break;
    case ch:
    resister.ch = _val;
    break;
    case rssi_byte:
    resister.reg3.rssi_byte = _val & 0b1;
    break;
    case transmission_method:
    resister.reg3.transmission_method = _val & 0b1;
    break;
    case wor_cycle:
    resister.reg3.wor_cycle = _val & 0b111;
    break;
    case crypt_h:
    resister.crypt_h = _val;
    break;
    case crypt_l:
    resister.crypt_l = _val;
    break;
    case version:
    resister.version = _val;
    break;
  }
}

void E220::setDefaultResister(void){
  setResister(addh,0);

  setResister(addl,0);

  setResister(uart_rate,0b011);
  setResister(air_rate,0b00010);

  setResister(sub_packet,0b00);
  setResister(rssi_noise,0);
  setResister(transmit_power,0b01);

  setResister(ch,0);

  setResister(rssi_byte,0b0);
  setResister(transmission_method,0b0);
  setResister(wor_cycle,0b011);

  setResister(crypt_h,0);

  setResister(crypt_l,0);

  setResister(version,0);
}

bool E220::sendResister(const uint32_t _timeout){
  Time time;
  int _header[3] = {0};
  bool _flag1 = switchStateToConfig(_timeout);
  sleep_ms(100);
  //delay(100);
  clearData();
  _header[0] = 0xC0;
  _header[1] = 0x00;
  _header[2] = 0x08;
  setData(_header, 3);
  setData(resister.val, 8);
  bool _flag2 = sendData(_timeout);
  int len[3] ={0};
  getData(3, 100);
  readData(len, 3);
  bool _flag3 = true;
  if(len[0] == 0xFF && len[1] == 0xFF && len[2] == 0xFF){
    _flag3 = false;
  }
  sleep_ms(100);
  serialWaitClear();
  bool _flag4 = switchStateToNormal(_timeout);
  if(_flag1 && _flag2 && _flag3 && _flag4){
    return true;
  }
  return false;
}

void E220::clearData(void){
  for(int i = 0; i < SEND_MAX; i++){
    send_data[i] = 0;
  }
  setSendSize(0);
}

bool E220::getData(const int _size,const uint32_t _timeout){
  int _data = 0;
  int _count = 0;
  Time time;
  time.resetTimeout(_timeout);
  clearGetData();
  while(serialReadAvailable() <= 0){
    if(time.isTimeOut()) return false;
  }
  while(_count < _size){
    _data = serialRead();
    if(_data != -1){
      get_data[_count] = _data;
      _count++;
    }
    if(time.isTimeOut()){
      return false;
    }
  }
  get_size = _size;
  return true;
}

bool E220::getDataWithCobs(const int _size,const uint32_t _timeout){
  int _data = 1;
  int _count = 0;
  Time time;
  time.resetTimeout(_timeout);
  clearGetData();
 while(serialReadAvailable() <= 0){
      if(time.isTimeOut()) return false;
  }
  while(_data != 0){
    _data = serialRead();
    if(_data != -1){
      get_data[_count] = _data;
      _count++;
    }
    if(time.isTimeOut()){
      return false;
    }
  }
  // get_size = _size + 2;
  if(_count != _size + 2){
    //printf("DetaSize:%d\n", _count);
    return false;
  }
  get_size = _count;
  ////////////////////////////////
  // printf("data: ");
  // for(int i = 0; i < get_size; i++){
  //   printf("%d ",get_data[i]);
  // }
  // printf("\n");
  return true;  
}

void E220::clearGetData(void){
  for(int i = 0; i < GET_MAX; i++){
    get_data[i] = 0;
  }
  get_size = 0;
}

bool E220::readData(int* _target_len,const int _size){
  if(get_size != _size){
    return false;
  }
  for(int i = 0; i < _size; i++){
    _target_len[i] = get_data[i];
  }
  return true;
}

bool E220::readDataWithCobs(int* _target_len,const int _size){
  if(get_size != _size + 2){
    return false;
  }
  int _next_zero = get_data[0];
  for(int i = 1; i < _size + 1; i++){
    if(i == _next_zero){
      _next_zero +=get_data[i];
      _target_len[i - 1] = 0;
    }else{
      _target_len[i - 1] = get_data[i];
    }
  }  
  return true;
}

int E220::readDataSendByPico(Read_ds4* output,int* inputdata){
  int sumdata = 0;
  for(int i = 0; i < 9; i++){
    sumdata += inputdata[i];
  }

  if(inputdata[0] & (1<<0)){//非常停止信号のチェック
    return 4;
  }
  if(!(inputdata[0] & (1<<1))){//データがLEDの場合2を返す
    if(is_checksum(sumdata,inputdata[5])){
      return 2;
    }else{
      return false;
    }
  }
  if(is_checksum(sumdata ,inputdata[9])){
    output->jyoutai = inputdata[0];
    output->L_x     = inputdata[1];
    output->L_y     = inputdata[2];
    output->R_x     = inputdata[3];
    output->R_y     = inputdata[4];
    output->L2      = inputdata[5];
    output->R2      = inputdata[6];
    output->Triangle= inputdata[7] & (1<<7);
    output->Circle  = inputdata[7] & (1<<6);
    output->Cross   = inputdata[7] & (1<<5);
    output->Square  = inputdata[7] & (1<<4);
    output->up      = inputdata[7] & (1<<0);
    output->Right   = inputdata[7] & (1<<1);
    output->Down    = inputdata[7] & (1<<2);
    output->Left    = inputdata[7] & (1<<3);
    output->L1      = inputdata[8] & (1<<7);
    output->R1      = inputdata[8] & (1<<6);
    output->L3      = inputdata[8] & (1<<5);
    output->R3      = inputdata[8] & (1<<4);
    output->Share   = inputdata[8] & (1<<3);
    output->Option  = inputdata[8] & (1<<2);
    output->PS      = inputdata[8] & (1<<1);
    output->TPad    = inputdata[8] & (1<<0);
    output->checsam = inputdata[9]; 
  }else{
    return false;
  }
  return true;
}


int E220::readLEDData(int* LEDoutput,int* inputdata){
  int sumdata = 0;

  if(!(inputdata[0] & (1<<0))){//非常停止信号のチェック
    return 4;
  }
  if(inputdata[0] & (1<<1)){//LEDのデータでなければ3を返す
    return 3;
  }

  for(int i = 0; i < 4; i++){
    sumdata = inputdata[i];
  }
  if(is_checksum(sumdata,inputdata[4])){ 
    LEDoutput[0] = inputdata[1];//R
    LEDoutput[1] = inputdata[2];//G
    LEDoutput[2] = inputdata[3];//B
  }else{
    return false;
  }
  return true;
}

bool E220::readAux(void){
  //digitalRead(aux_pin)
  return gpio_get(aux_pin);
}

bool E220::isConnected(void){
  return true;
}

bool E220::waitForAuxIsReady(const uint32_t _timeout){
  Time time;
  time.resetTimeout(_timeout);
  while(readAux() == false){
    if(time.isTimeOut())
      return false;
  }
  return true;
}
bool E220::serialWrite(const int _data){
  if(_data > 255 || _data < 0){
    return false;
  }
  uart_write_blocking(this->hs, (uint8_t*)&_data, 1);
  return true;
}

bool E220::switchStateToNormal(const uint32_t _timeout){
  switchSerialRate(boud_rate);
  resetM0();
  resetM1();
  bool _flag = waitForAuxIsReady(_timeout);
  //delay(2);
  sleep_ms(2);
  return _flag;
}

bool E220::switchStateToWorTransmit(const uint32_t _timeout){
  switchSerialRate(boud_rate);
  setM0();
  resetM1();
  bool _flag = waitForAuxIsReady(_timeout);
  //delay(2);
  sleep_ms(2);
  return _flag;
}

bool E220::switchStateToWorReceive(const uint32_t _timeout){
  switchSerialRate(boud_rate);
  resetM0();
  setM1();
  bool _flag = waitForAuxIsReady(_timeout);
  //delay(2);
  sleep_ms(2);
  return _flag;
}

bool E220::switchStateToConfig(const uint32_t _timeout){
  switchSerialRate(9600);
  setM0();
  setM1();
  bool _flag = waitForAuxIsReady(_timeout);
  //delay(2);
  sleep_ms(2);
  return _flag;
}

void E220::initResister(void){
  resister.addh = 0x00;
  resister.addl = 0x00;
  resister.reg0.uart_rate = 0b011;
  resister.reg0.air_rate = 0b00010;
  resister.reg1.sub_packet = 0b00;
  resister.reg1.rssi_noise = 0b0;
  resister.reg1.unused = 0b000;
  resister.reg1.transmit_power = 0b01;
  resister.ch = 0x00;
  resister.reg3.rssi_byte = 0b0;
  resister.reg3.transmission_method = 0b0;
  resister.reg3.unused = 0b000;
  resister.reg3.wor_cycle = 0b011;
  resister.crypt_h = 0x00;
  resister.crypt_l = 0x00;
  resister.version = 0x00;
}

void E220::setM0(void){
  gpio_put(m0_pin,1);
}

void E220::resetM0(void){
  // digitalWrite(m0_pin,HIGH);
  gpio_put(m0_pin,0);
}

void E220::setM1(void){
  //digitalWrite(m0_pin,LOW);
  gpio_put(m1_pin,1);
}

void E220::resetM1(void){
  //digitalWrite(m1_pin,HIGH);
  gpio_put(m1_pin,0);
}

int E220::serialRead(void){
  //this->hs->read();
  if(uart_is_readable_within_us(this ->hs,50)){
    return uart_getc(this->hs);
  }
  return -1;
  // return 0;
}

void E220::switchSerialRate(const unsigned long _boud_rate){
  uart_init(this->hs,_boud_rate);
}

int E220::getSendSize(void){
  return send_size;
}
bool E220::setSendSize(const int _val){
  if(_val > SEND_MAX){
    send_size = 0;
    return false;
  }
  send_size = _val;
  return true;
}

bool E220::addSendSize(const int _val){
  send_size+=_val;
  if(send_size > SEND_MAX){
    send_size = 0;
    return false;
  }
  return true;
  
}