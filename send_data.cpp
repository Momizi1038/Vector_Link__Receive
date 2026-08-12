//#include "hid_app.h"
#include "send_data.h"
#include "lib/E220Connect/e220.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

 bool changeData(int* output , ds4_data rewdata){
  output[0] = int(rewdata.jyoutai);
  output[1] = int(rewdata.L_x);
  output[2] = int(rewdata.L_y);
  output[3] = int(rewdata.R_x);
  output[4] = int(rewdata.R_y);
  output[5] = int(rewdata.L2);
  output[6] = int(rewdata.R2);
  output[7] = int(rewdata.key);
  output[8] = int(rewdata.boton);
  output[9] = int(rewdata.checsam);
  return true;
 }

 //E220処理
 E220 Lora_1(uart1, BAUD_UART_RATE, E220_1_M0PIN, E220_1_M1PIN, E220_1_AUXPIN);

 void Lora1_init(void){
  gpio_set_function(E220_1_UART_TXPIN, GPIO_FUNC_UART);
  gpio_set_function(E220_1_UART_RXPIN, GPIO_FUNC_UART);
  uart_init(uart1, BAUD_UART_RATE);

  Lora_1.begin();
  Lora_1.setDefaultResister(); //デフォルトのレジスタ値を設定します(デフォルトのレジスタ値はデータシートのデフォルトのレジスタ値を基にしています)。
  Lora_1.setResister(uart_rate, UART_RATE115200);
  Lora_1.setResister(air_rate,AIR_RATE62500);
  Lora_1.setResister(addh, DEFAULT_ADDH); //E220モジュールのアドレスを設定します。引数:レジスタ名,値
  Lora_1.setResister(addl, DEFAULT_ADDL); //E220モジュールのアドレスを設定します。
  Lora_1.setResister(ch, DEFAULT_CH); //E220モジュールのチャンネルを設定します。
  Lora_1.setResister(sub_packet,SUB_PACKET32);
  Lora_1.sendResister(100);//E220モジュールにレジスタ値を送信します。引数:タイムアウト
 }

 bool Lora1_send_ds4(ds4_data input , int CH){
  int deta_len[10];
  bool check = false;

  changeData(deta_len,input);
  Lora_1.setDataWithCobs(deta_len,10);
  check = Lora_1.sendDataFixed(TARGET_ADDH, TARGET_ADDL, CH,100);

  return check;
 }

 bool Lora1_read_Aux(void){
  return Lora_1.readAux();
 }