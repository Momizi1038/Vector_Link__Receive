#ifndef TYPE_H
#define TYPE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t jyoutai;
    
    uint8_t L_x;
    uint8_t L_y;
    uint8_t R_x;
    uint8_t R_y;

    uint8_t L2;
    uint8_t R2;

    //左から、Triangle,Circle,Cross,Square,Left,Down,Right,Up
    uint8_t key;//方向キーと記号キー
    //左から、L1,R1,L3,R3,Share,Option,PS,T-Pad_click
    uint8_t boton;//その他ボタン

    uint8_t checsam;
}ds4_data;

typedef struct {
    uint8_t jyoutai;
    
    uint8_t L_x;
    uint8_t L_y;
    uint8_t R_x;
    uint8_t R_y;

    uint8_t L2;
    uint8_t R2;

    bool Triangle;
    bool Circle;
    bool Cross;
    bool Square;
    
    bool Left;
    bool Down;
    bool Right;
    bool up;

    bool L1;
    bool R1;
    bool L3;
    bool R3;
    bool Share;
    bool Option;
    bool PS;
    bool TPad;

    uint8_t checsam;
}Read_ds4;

#ifdef __cplusplus
}
#endif

#endif // TYPE_H