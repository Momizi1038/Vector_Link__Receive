#ifndef UART_OUTPUT_H
#define UART_OUTPUT_H

#include <stdint.h>
#include "hardware/uart.h"

#include "type.h"


class UartOutput {
public:

    UartOutput(
        uart_inst_t* uart_id,
        uint tx_pin,
        uint rx_pin,
        uint32_t baud_rate
    );

    void begin();

    // 新しいSEQのデータだけ送信
    bool sendLatest(const ds4_data& data);

    // 実際のデータ送信
    bool send(const ds4_data& data);


private:

    // ========================================================
    // UART設定
    // ========================================================
    uart_inst_t* _uart_id;
    uint _tx_pin;
    uint _rx_pin;
    uint32_t _baud_rate;

    // ========================================================
    // 最後にUARTへ送信したSEQ
    // ========================================================
    uint16_t _last_send_seq;
    bool _last_send_seq_valid;

    // ========================================================
    // SEQ
    // ========================================================
    uint16_t getSeq(
        const ds4_data& data
    );

    bool isSeqNewer(
        uint16_t new_seq,
        uint16_t old_seq
    );

    // ========================================================
    // データ生成
    // ========================================================
    bool changeData(
        uint8_t* output,
        const ds4_data& data
    );

    // ========================================================
    // CRC-8
    // ========================================================
    uint8_t calcCRC8(
        const uint8_t* data,
        uint8_t size
    );

    // ========================================================
    // COBS
    // ========================================================
    uint8_t cobsEncode(
        const uint8_t* input,
        uint8_t input_size,
        uint8_t* output
    );
};


#endif