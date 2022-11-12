#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "sx1278.h"

#define ENABLE_TRACES
#include "logger.h"

#define SX1278_SCK_PIN    18
#define SX1278_MISO_PIN   16
#define SX1278_MOSI_PIN   19
#define SX1278_CS_PIN     17
#define SX1278_RESET_PIN  20
#define SX1278_DI0_PIN    21
#define SX1278_SPI_DEV    spi0

#define NETWORK_DEFAULT_FREQUENCY           412E6
#define NETWORK_DEFAULT_HEADER_MODE         EXPLICITE_HEADER_MODE
#define NETWORK_DEFAULT_BANDWIDTH           BANDWIDTH_500_KHZ
#define NETWORK_DEFAULT_CODING_RATE         CODING_RATE_4_5
#define NETWORK_DEFAULT_SPREADING_FACTOR    SPREADING_FACTOR_7
#define NETWORK_DEFAULT_MODE                RX_CONTINUOUS_MODE
#define NETWORK_DEFAULT_OCP                 0x00

static void receive_callback(uint8_t* data,
                             uint8_t data_len,
                             uint8_t rssi,
                             uint8_t snr)
{
    LOG("Data len: [ %d ], rssi: [ %d ], snr: [ %d ]", data_len, rssi, snr);
}

static void transmit_callback(uint error_code)
{
    LOG("Return code: [ %d ]", error_code);
}

static int network_init(struct sx1278_dev_t* module)
{
    LOG("Start network initialization");
    module = sx1278_create_device(SX1278_MOSI_PIN,
                                         SX1278_MISO_PIN,
                                         SX1278_CS_PIN,
                                         SX1278_SCK_PIN,
                                         SX1278_RESET_PIN,
                                         SX1278_DI0_PIN,
                                         SX1278_SPI_DEV);
    sx1278_init(module);
    sx1278_set_frequency(module, NETWORK_DEFAULT_FREQUENCY);
    sx1278_set_header_mode(module, NETWORK_DEFAULT_HEADER_MODE);
    sx1278_set_bandwidth(module, NETWORK_DEFAULT_BANDWIDTH);
    sx1278_set_coding_rate(module, NETWORK_DEFAULT_CODING_RATE);
    sx1278_set_spreading_factor(module, NETWORK_DEFAULT_SPREADING_FACTOR);
    sx1278_set_ocp_threshold(module, NETWORK_DEFAULT_OCP);
    sx1278_register_rx_callback(module, receive_callback);
    sx1278_register_tx_callback(module, transmit_callback);
    sx1278_set_mode(module, NETWORK_DEFAULT_MODE);
}

int main()
{
    struct sx1278_dev_t* lora_module;
    const char send_buffer[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    network_init(lora_module);
    while (1) {
        sx1278_update(lora_module);
        sleep_us(1000);
        sx1278_transmit(lora_module, send_buffer, sizeof(send_buffer) / sizeof(send_buffer[0]));
    }
}
