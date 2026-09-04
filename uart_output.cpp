#include "uart_output.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"


// ============================================================
// UARTデータサイズ
//
// 0   jyoutai
// 1   L_x
// 2   L_y
// 3   R_x
// 4   R_y
// 5   L2
// 6   R2
// 7   key
// 8   boton
// 9   seq_H
// 10  seq_L
// 11  CRC-8
//
// ============================================================

#define UART_DATA_SIZE 12

// COBSエンコード後の最大サイズ
// データ12 byte + COBSコード + 終端0
#define UART_COBS_SIZE (UART_DATA_SIZE + 2)


// ============================================================
// コンストラクタ
// ============================================================

UartOutput::UartOutput(
    uart_inst_t* uart_id,
    uint tx_pin,
    uint rx_pin,
    uint32_t baud_rate
)
{
    _uart_id = uart_id;
    _tx_pin = tx_pin;
    _rx_pin = rx_pin;
    _baud_rate = baud_rate;

    _last_send_seq = 0;
    _last_send_seq_valid = false;
}


// ============================================================
// UART初期化
// ============================================================

void UartOutput::begin()
{
    uart_init(
        _uart_id,
        _baud_rate
    );

    gpio_set_function(
        _tx_pin,
        GPIO_FUNC_UART
    );

    gpio_set_function(
        _rx_pin,
        GPIO_FUNC_UART
    );

    _last_send_seq = 0;
    _last_send_seq_valid = false;
}


// ============================================================
// SEQ取得
//
// seq_H = 上位8 bit
// seq_L = 下位8 bit
// ============================================================

uint16_t UartOutput::getSeq(
    const ds4_data& data
)
{
    return
        ((uint16_t)data.seq_H << 8)
        | data.seq_L;
}


// ============================================================
// SEQ新旧判定
//
// 16bit SEQのオーバーフローを考慮
//
// 65535 → 0
// も新しいデータとして扱う。
//
// ============================================================

bool UartOutput::isSeqNewer(
    uint16_t new_seq,
    uint16_t old_seq
)
{
    return
        (int16_t)(new_seq - old_seq) > 0;
}


// ============================================================
// 最新SEQのデータだけ送信
//
// BLEとE220のSEQを直接比較しない。
//
// 「最後にUARTへ送信したSEQ」を基準にする。
//
// ============================================================

bool UartOutput::sendLatest(
    const ds4_data& data
)
{
    uint16_t seq = getSeq(data);


    // --------------------------------------------------------
    // 初回送信
    // --------------------------------------------------------

    if (!_last_send_seq_valid) {

        if (send(data)) {

            _last_send_seq = seq;
            _last_send_seq_valid = true;

            return true;
        }

        return false;
    }


    // --------------------------------------------------------
    // 新しいSEQなら送信
    // --------------------------------------------------------

    if (isSeqNewer(
        seq,
        _last_send_seq
    )) {

        if (send(data)) {

            // UART送信後にSEQを更新
            _last_send_seq = seq;

            return true;
        }

        return false;
    }


    // --------------------------------------------------------
    // 古いSEQ
    // --------------------------------------------------------

    return false;
}


// ============================================================
// ds4_data → UART用12 byte
//
// [0]  jyoutai
// [1]  L_x
// [2]  L_y
// [3]  R_x
// [4]  R_y
// [5]  L2
// [6]  R2
// [7]  key
// [8]  boton
// [9]  seq_H
// [10] seq_L
// [11] CRC-8
//
// ============================================================

bool UartOutput::changeData(
    uint8_t* output,
    const ds4_data& data
)
{
    output[0] = data.jyoutai;

    output[1] = data.L_x;
    output[2] = data.L_y;

    output[3] = data.R_x;
    output[4] = data.R_y;

    output[5] = data.L2;
    output[6] = data.R2;

    output[7] = data.key;
    output[8] = data.boton;


    // --------------------------------------------------------
    // SEQ
    //
    // Pico間通信と同じ
    //
    // Byte 9  = seq_H
    // Byte 10 = seq_L
    // --------------------------------------------------------

    output[9]  = data.seq_H;
    output[10] = data.seq_L;


    // --------------------------------------------------------
    // CRC-8
    //
    // Byte 0～10を対象に計算
    // --------------------------------------------------------

    output[11] = calcCRC8(
        output,
        11
    );

    return true;
}


// ============================================================
// CRC-8
//
// Polynomial : 0x07
// Initial    : 0x00
// RefIn      : false
// RefOut     : false
// XorOut     : 0x00
//
// ============================================================

uint8_t UartOutput::calcCRC8(
    const uint8_t* data,
    uint8_t size
)
{
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < size; i++) {

        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {

            if (crc & 0x80) {

                crc =
                    (uint8_t)((crc << 1) ^ 0x07);

            } else {

                crc <<= 1;
            }
        }
    }

    return crc;
}


// ============================================================
// COBS
//
// これまで使用しているCOBS方式を維持
// ============================================================

uint8_t UartOutput::cobsEncode(
    const uint8_t* input,
    uint8_t input_size,
    uint8_t* output
)
{
    uint8_t s[UART_COBS_SIZE] = {0};


    // --------------------------------------------------------
    // 元データを1 byte後ろへ配置
    // --------------------------------------------------------

    for (
        uint8_t i = 0;
        i < input_size;
        i++
    ) {

        s[i + 1] = input[i];
    }


    // --------------------------------------------------------
    // COBS処理
    // --------------------------------------------------------

    int i2 = 0;

    for (
        int i = input_size;
        i >= 0;
        i--
    ) {

        i2++;

        if (s[i] == 0) {

            s[i] = i2;
            i2 = 0;
        }
    }


    // --------------------------------------------------------
    // 出力
    // --------------------------------------------------------

    for (
        uint8_t i = 0;
        i < input_size + 2;
        i++
    ) {

        output[i] = s[i];
    }


    return input_size + 2;
}


// ============================================================
// UART送信
//
// ds4_data
//     ↓
// 12 byte生成
//     ↓
// CRC-8
//     ↓
// COBS
//     ↓
// UART
//
// ============================================================

bool UartOutput::send(const ds4_data& data)
{
    uint8_t raw_data[UART_DATA_SIZE] = {0};

    uint8_t cobs_data[UART_COBS_SIZE] = {0};


    // --------------------------------------------------------
    // 12 byteデータ生成
    // --------------------------------------------------------

    if (!changeData(
        raw_data,
        data
    )) {

        return false;
    }

    // --------------------------------------------------------
    // COBS
    // --------------------------------------------------------
    uint8_t send_size =cobsEncode(raw_data, UART_DATA_SIZE, cobs_data);

    // --------------------------------------------------------
    // UART送信
    // --------------------------------------------------------
    for (uint8_t i = 0; i < send_size; i++) {
        uart_putc_raw( _uart_id,cobs_data[i]);
    }

    return true;
}