#include "uart_output.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"


UartOutput::UartOutput(
    uart_inst_t* uart,
    uint tx_pin,
    uint rx_pin,
    uint32_t baudrate
)
    : uart_(uart),
      tx_pin_(tx_pin),
      rx_pin_(rx_pin),
      baudrate_(baudrate)
{
}


void UartOutput::begin()
{
    uart_init(uart_, baudrate_);

    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);

    uart_set_format(
        uart_,
        8,      // Data bits
        1,      // Stop bits
        UART_PARITY_NONE
    );

    uart_set_fifo_enabled(uart_, true);
}


// CRC-8
// Polynomial : 0x07
// Initial    : 0x00
uint8_t UartOutput::crc8(
    const uint8_t* data,
    size_t length
)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; i++) {

        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {

            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


// 戻り値：エンコード後のサイズ
// 失敗時：0
size_t UartOutput::cobsEncode(
    const uint8_t* input,
    size_t input_length,
    uint8_t* output,
    size_t output_size
)
{
    // COBSでは最悪の場合、
    // input_length + input_length / 254 + 1
    // 程度必要

    if (output_size < input_length + 2) {
        return 0;
    }

    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;

    uint8_t code = 1;

    while (read_index < input_length) {

        if (input[read_index] == 0) {

            output[code_index] = code;

            code = 1;
            code_index = write_index;

            write_index++;

            read_index++;

        } else {

            output[write_index] = input[read_index];

            write_index++;
            read_index++;

            code++;

            if (code == 0xFF) {

                output[code_index] = code;

                code = 1;
                code_index = write_index;

                write_index++;
            }
        }

        if (write_index >= output_size) {
            return 0;
        }
    }

    output[code_index] = code;

    return write_index;
}


bool UartOutput::send(const ds4_data& data)
{
    // COBSエンコード前
    //
    // Byte 0  : jyoutai
    // Byte 1  : L_x
    // Byte 2  : L_y
    // Byte 3  : R_x
    // Byte 4  : R_y
    // Byte 5  : L2
    // Byte 6  : R2
    // Byte 7  : key
    // Byte 8  : boton
    // Byte 9  : seq_L
    // Byte 10 : seq_H
    // Byte 11 : CRC-8

    uint8_t packet[12];

    packet[0]  = data.jyoutai;

    packet[1]  = data.L_x;
    packet[2]  = data.L_y;
    packet[3]  = data.R_x;
    packet[4]  = data.R_y;

    packet[5]  = data.L2;
    packet[6]  = data.R2;

    packet[7]  = data.key;
    packet[8]  = data.boton;

    packet[9]  = data.seq_L;
    packet[10] = data.seq_H;

    // Byte 0～10の11 byteからCRCを計算
    packet[11] = crc8(packet, 11);


    // 12 byteのCOBS最大サイズは14 byteあれば十分
    uint8_t encoded[14];

    size_t encoded_size = cobsEncode(
        packet,
        sizeof(packet),
        encoded,
        sizeof(encoded)
    );

    if (encoded_size == 0) {
        return false;
    }


    // UART送信
    uart_write_blocking(uart_, encoded, encoded_size);


    // COBSパケット終端
    const uint8_t delimiter = 0x00;

    uart_write_blocking(uart_, &delimiter, 1);

    return true;
}