#ifndef __SX1278__H__
#define __SX1278__H__

#define DONE 0
#define FAIL 1

#define VERSION_REG_VALUE 0x12

#define SLEEP_MODE          0
#define STDBY_MODE          1
#define FSTX_MODE           2
#define TX_MODE             3
#define FSRX_MODE           4
#define RX_CONTINUOUS_MODE  5
#define RX_SINGLE_MODE      6
#define CAD_MODE            7

#define FREQUENCY_MAX_VALUE 480
#define FREQUENCY_MIN_VALUE 400

#define PA_RAMP_MAX_VALUE 15
#define PA_RAMP_MIN_VALUE 0

#define OCP_MAX_VALUE 240
#define OCP_MIN_VALUE 45
#define OCP_DISABLE   0

#define LNA_GAIN_G1        1
#define LNA_GAIN_G2        2
#define LNA_GAIN_G3        3
#define LNA_GAIN_G4        4
#define LNA_GAIN_G5        5
#define LNA_GAIN_G6        6
#define LNA_BOOST_DEFAULT  0
#define LNA_BOOST_ON       1

#define BANDWIDTH_7_8_KHZ    0
#define BANDWIDTH_10_4_KHZ   1
#define BANDWIDTH_15_6_KHZ   2
#define BANDWIDTH_20_8_KHZ   3
#define BANDWIDTH_31_25_KHZ  4
#define BANDWIDTH_41_7_KHZ   5
#define BANDWIDTH_62_5_KHZ   6
#define BANDWIDTH_125_KHZ    7
#define BANDWIDTH_250_KHZ    8
#define BANDWIDTH_500_KHZ    9

#define CODING_RATE_4_5      1
#define CODING_RATE_4_6      2
#define CODING_RATE_4_7      3
#define CODING_RATE_4_8      4

#define EXPLICITE_HEADER_MODE  0
#define IMPLICITE_HEADER_MODE  1

#define SPREADING_FACTOR_6     6
#define SPREADING_FACTOR_7     7
#define SPREADING_FACTOR_8     8
#define SPREADING_FACTOR_9     9
#define SPREADING_FACTOR_10    10
#define SPREADING_FACTOR_11    11
#define SPREADING_FACTOR_12    12

#define CRC_DISABLE   0
#define CRC_ENABLE    1

#include "hardware/spi.h"


/**
 * @brief This callback will be used to call it in case of TX done or fail
 * @param uint Error code of tx
 *      - TX_DONE
 *      - TX_FAIL
 */
typedef void (*tx_callback)(uint);

/**
 * @brief This callback wiil be used to register it,
 * and it will be called in case of package receiving
 * @param uint8_t* pointer to allocated memory containing received data
 * @param uint8_t  received data size (bytes)
 * @param uint8_t  received package RSSI
 * @param uint8_t  received package SNR
 *
 * @note The memory which allocated for received that must be free in this callback
 */
typedef void (*rx_callback)(uint8_t*, uint8_t, uint8_t, uint8_t);

enum MODEM_STATUS {
    ERROR,
    MODEM_CLEAR,
    HEADER_INFO_VALID,
    RX_ON_GOING,
    SIGNAL_SYNCRONIZED,
    SIGNAL_DETECTED
};

struct sx1278_dev_t {
    spi_inst_t* spi_dev;
    uint8_t reset_pin;
    uint8_t dio0_pin;
    uint8_t cs_pin;
    tx_callback tx_callback_f;
    rx_callback rx_callback_f;
};

/**
 * @brief This API is responsible for hardware initalizing SX1278 device
 *
 * @param[in] mosi_pin  gpio connected to SX1278 MOSI pin
 * @param[in] miso_pin  gpio connected to SX1278 MISO pin
 * @param[in] cs_pin    gpio connected to SX1278 CS pin
 * @param[in] sck_pin   gpio connected to SX1278 SCK pin
 * @param[in] reset_pin gpio connected to SX1278 RESET pin
 * @param[in] dio0_pin  gpio connected to SX1278 DIO0 pin
 * @param[in] spi_dev   SPI device instance which will be used for communicating with SX1278
 * @return struct sx1278_dev_t* initialized sruct pointer which will be used for other device manipulating
 */
struct sx1278_dev_t* sx1278_create_device(const uint8_t mosi_pin,
                                          const uint8_t miso_pin,
                                          const uint8_t cs_pin,
                                          const uint8_t sck_pin,
                                          const uint8_t reset_pin,
                                          const uint8_t dio0_pin,
                                          spi_inst_t* spi_dev);

/**
 * @brief This API to update module state
 * Need to be called pereodicaly, to detect module updates
 *
 * @param[in] module initialized module struct pointer
 */
void sx1278_update(const struct sx1278_dev_t* module);

/**
 * @brief This API to reset SX1278 module
 *
 * @param[in] module initialized module struct pointer
 *
 * @note this API is blocking (about 5.1ms)
 */
void sx1278_reset(const struct sx1278_dev_t* module);

/**
 * @brief This API to register callback which will be called in case of update during tx
 *
 * @param[out] module initialized module struct pointer
 */
void sx1278_register_tx_callback(struct sx1278_dev_t* module, tx_callback callback);

/**
 * @brief This API to register callback which will be called in case of package receive
 *
 * @param[out] module initialized module struct pointer
 */
void sx1278_register_rx_callback(struct sx1278_dev_t* module, rx_callback callback);

/**
 * @brief This API to initilize SX1278 module (blocking about 10ms)
 *
 * @param[in] module initialized module struct pointer
 * @return uint 0 if module initialization passed, else` 1
 */
uint sx1278_init(const struct sx1278_dev_t* module);

/**
 * @brief This API to get SX1278 module version
 *
 * @param[in] module initialized module struct pointer
 * @return uint module version
 */
uint sx1278_get_version(const struct sx1278_dev_t* module);

/**
 * @brief This API to set SX1278 module mode
 *
 * @param[in] module initialized module struct pointer
 * @param[in] mode supported values are
 *      - SLEEP_MODE
 *      - STDBY_MODE
 *      - FSTX_MODE
 *      - TX_MODE
 *      - FSRX_MODE
 *      - RX_CONTINUOUS_MODE
 *      - RX_SINGLE_MODE
 *      - CAD_MODE
 * @return uint 0 if module mode set successfuly, else` 1
 *
 */
uint sx1278_set_mode(const struct sx1278_dev_t* module, uint8_t mode);

/**
 * @brief This API to set module frequency
 *
 * @param[in] module initialized module struct pointer
 * @param[in] freq See available frequency range in module documentation
 * @return uint 0 if module frequency set successfuly, else` 1
 */
uint sx1278_set_frequency(const struct sx1278_dev_t* module, const uint32_t freq);

/**
 * @brief This API to configure OCP (Over current protection) threshold
 *
 * @param[in] module initialized module struct pointer
 * @param[in] threshold supported values are:
 *      - OCP_DISABLE, OCP will be disabled
 *      - [45, 240] from 45 mA to 240 mA
 * @return uint 0 if module OCP set successfuly, else` 1
 */
uint sx1278_set_ocp_threshold(const struct sx1278_dev_t* module, const uint8_t threshold);

/**
 * @brief This API to configure LNA (Low Noise Amplifier) gain level
 *
 * @param[in] module initialized module struct pointer
 * @param[in] gain supported values are:
 *      - LNA_GAIN_G1 (maximum gain)
 *      - LNA_GAIN_G2
 *      - LNA_GAIN_G3
 *      - LNA_GAIN_G4
 *      - LNA_GAIN_G5
 *      - LNA_GAIN_G6 (minimum gain)
 * @return uint 0 if module LNA gain set successfuly, else` 1
 */
uint sx1278_set_lna_gain(const struct sx1278_dev_t* module, const uint8_t gain);

/**
 * @brief This API to configure additional LNA (Low Noise Amplifier) boost
 *
 * @param[in] module initialized module struct pointer
 * @param[in] boost supported values are:
 *      - LNA_BOOST_DEFAULT
 *      - LNA_BOOST_ON (150% LNA current)
 * @return uint 0 if module LNA Boost set successfuly, else` 1
 */
uint sx1278_set_lna_boost(const struct sx1278_dev_t* module, const uint8_t boost);

/**
 * @brief This API to configure bandwidth
 *
 * @param[in] module initialized module struct pointer
 * @param[in] bandwidth supported values are:
 *      - BANDWIDTH_7_8_KHZ
 *      - BANDWIDTH_10_4_KHZ
 *      - BANDWIDTH_15_6_KHZ
 *      - BANDWIDTH_20_8_KHZ
 *      - BANDWIDTH_31_25_KHZ
 *      - BANDWIDTH_41_7_KHZ
 *      - BANDWIDTH_62_5_KHZ
 *      - BANDWIDTH_125_KHZ
 *      - BANDWIDTH_250_KHZ
 *      - BANDWIDTH_500_KHZ
 * @return uint 0 if module bandwidth set successfuly, else` 1
 */
uint sx1278_set_bandwidth(const struct sx1278_dev_t* module, const uint bandwidth);

/**
 * @brief This API to configure condig rate (4/X)
 *
 * @param[in] module initialized module struct pointer
 * @param[in] rate supported values are:
 *      - CODING_RATE_4_5
 *      - CODING_RATE_4_6
 *      - CODING_RATE_4_7
 *      - CODING_RATE_4_8
 * @return uint 0 if module coding rate set successfuly, else` 1
 */
uint sx1278_set_coding_rate(const struct sx1278_dev_t* module, const uint8_t rate);

/**
 * @brief This API to configure header mode (implicite or explicite)
 *
 * @param[in] module initialized module struct pointer
 * @param[in] mode supported values are:
 *      - EXPLICITE_HEADER_MODE
 *      - IMPLICITE_HEADER_MODE
 * @return uint 0 if module header mode set successfuly, else` 1
 */
uint sx1278_set_header_mode(const struct sx1278_dev_t* module, const uint8_t mode);

/**
 * @brief This API to configure spreading factor
 *
 * @param[in] module initialized module struct pointer
 * @param[in] sp_factor supported values are:
 *      - SPREADING_FACTOR_6 (need spicific configuration, see SX1278 documentation)
 *      _ SPREADING_FACTOR_7
 *      _ SPREADING_FACTOR_8
 *      _ SPREADING_FACTOR_9
 *      _ SPREADING_FACTOR_10
 *      _ SPREADING_FACTOR_11
 *      _ SPREADING_FACTOR_12
 * @return uint 0 if module spreading factor set successfuly, else` 1
 */
uint sx1278_set_spreading_factor(const struct sx1278_dev_t* module, const uint8_t sp_factor);

/**
 * @brief API to dump all described registers
 *
 * @param[in] module initialized module struct pointer
 */
void sx1278_dump_all_registers(const struct sx1278_dev_t* module);

/**
 * @brief This API to get the module current mode
 *
 * @param[in] module initialized module struct pointer
 * @param[out] mode current module mode, possible values are:
 *       - SLEEP_MODE
 *       - STDBY_MODE
 *       - FSTX_MODE
 *       - TX_MODE
 *       - FSRX_MODE
 *       - RX_CONTINUOUS_MODE
 *       - RX_SINGLE_MODE
 *       - CAD_MODE
 * @return uint 0 if module current mode got successfuly, else` 1
 */
uint sx1278_get_mode(const struct sx1278_dev_t* module, uint8_t* mode);

/**
 * @brief This API to get current RSSI value
 *
 * @param[in] module initialized module struct pointer
 * @param[out] rssi current rssi value, possible values are:
 *      - in case of using HF output port: [-157; 98]
 *      - in case of using LF output port: [-164; 91]
 * @return uint
 */
uint sx1278_get_current_rssi(const struct sx1278_dev_t* module, int16_t* rssi);

/**
 * @brief This API to transmit data via modem
 * This API is not blocking, and the status of tx could be get via tx callback
 * @param[in] module initialized module struct pointer
 * @param[in] data pointer to allocated memory with data which will be sent
 * @param[in] data_len len of data buffer
 * @return uint 0 if registers configured successfuly, else` 1
 */
uint sx1278_transmit(const struct sx1278_dev_t* module, const uint8_t* data, const uint data_len);

/**
 * @brief This API to get random number which will be got from current RSSI
 *
 * @param[in] module initialized module struct pointer
 * @param[out] random_num random number
 * @return uint 0 if current RSSI got successfuly, else` 1
 */
uint sx1278_get_random_number(const struct sx1278_dev_t* module, uint8_t* random_num);

/**
 * @brief This API to get register value
 *  Mostly for unit testing
 *
 * @param[in] module initialized module struct pointer
 * @param[in] address register address
 * @param[out] reg_value register value
 * @return uint 0 if register value got successfuly, else` 1
 */
uint sx1278_get_register_value(const struct sx1278_dev_t* module, uint8_t address, uint8_t* reg_value);
#endif /* __SX1278__H__ */