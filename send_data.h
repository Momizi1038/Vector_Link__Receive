//#include "hid_app.h"
#include "type.h"
//#include "lib/E220Connect/e220.h"
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BAUD_UART_RATE 115200
#define DEFAULT_ADDH 0
#define DEFAULT_ADDL 0
#define DEFAULT_CH 0
#define E220_1_UART_TXPIN 8
#define E220_1_UART_RXPIN 9
#define E220_1_M0PIN 11
#define E220_1_M1PIN 10
#define E220_1_AUXPIN 7

#define TARGET_ADDH 0
#define TARGET_ADDL 0
#define TARGET_CH 0

// typedef struct {
//     uint8_t jyoutai;
    
//     uint8_t L_x;
//     uint8_t L_y;
//     uint8_t R_x;
//     uint8_t R_y;

//     uint8_t L2;
//     uint8_t R2;

//     //左から、Triangle,Circle,Cross,Square,Left,Down,Right,Up
//     uint8_t key;//方向キーと記号キー
//     //左から、L1,R1,L3,R3,Share,Option,PS,T-Pad_click
//     uint8_t boton;//その他ボタン

//     uint8_t checsam;
// }ds4_data;
bool changeData(int* output , ds4_data rewdata);

void Lora1_init(void);
bool Lora1_send_ds4(ds4_data input, int CH);

bool Lora1_read_Aux(void);

#ifdef __cplusplus
}
#endif