#ifndef UART_OUTPUT_H
#define UART_OUTPUT_H

#include <stdint.h>
#include <stddef.h>

#include "hardware/uart.h"
#include "type.h"

class UartOutput {
public:
    UartOutput(
        uart_inst_t* uart,
        uint tx_pin,
        uint rx_pin,
        uint32_t baudrate
    );

    void begin();

    // ds4_dataをUARTプロトコルへ変換して送信
    bool send(const ds4_data& data);

private:
    // CRC-8
    // Polynomial: 0x07
    // Initial:    0x00
    uint8_t crc8(const uint8_t* data, size_t length);

    // COBSエンコード
    size_t cobsEncode(
        const uint8_t* input,
        size_t input_length,
        uint8_t* output,
        size_t output_size
    );

private:
    uart_inst_t* uart_;
    uint tx_pin_;
    uint rx_pin_;
    uint32_t baudrate_;
};

#endif