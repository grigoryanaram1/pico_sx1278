#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "sx1278_registers.h"
#include "sx1278.h"

//#define ENABLE_TRACES
#include "logger.h"

#define SX1278_SPI_SPEED  10 /* 10 MHz */
#define WRITE_ACCESS(x) x | 0b10000000

static inline void cs_select(uint8_t cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint8_t cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

static uint read_register(const struct sx1278_dev_t* module,
                             const uint8_t address,
                             uint8_t* read_value)
{
    uint8_t obuf = address;
    uint8_t dummy_buffer = 0x00;
    cs_select(module->cs_pin);
    if (spi_write_blocking(module->spi_dev, &obuf, 1) != 1) {
        LOG_ERR();
        return 1;
    }
    if (spi_read_blocking(module->spi_dev, dummy_buffer, read_value, 1) != 1) {
        LOG_ERR();
        return 1;
    }
    cs_deselect(module->cs_pin);
    return 0;
}

static uint write_register(const struct sx1278_dev_t* module, const uint8_t address, const uint8_t value)
{
    uint8_t obuf[2] = { WRITE_ACCESS(address), value };
    uint8_t validate_buf;
    cs_select(module->cs_pin);
    if (spi_write_blocking(module->spi_dev, obuf, 2) != 2) {
        LOG_ERR();
        return FAIL;
    }
    cs_deselect(module->cs_pin);
    sleep_ms(1); // TODO
    if (read_register(module, address, &validate_buf) != 0) {
        LOG_ERR();
        return FAIL;
    }
    if (validate_buf != value) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

static uint reset_register(const struct sx1278_dev_t* module, const uint8_t address)
{
    uint8_t obuf[2] = { WRITE_ACCESS(address), 0xFF };
    cs_select(module->cs_pin);
    if (spi_write_blocking(module->spi_dev, obuf, 2) != 2) {
        LOG_ERR();
        return FAIL;
    }
    cs_deselect(module->cs_pin);
    sleep_ms(1); // TODO
    return DONE;
}

static uint write_to_fifo(const struct sx1278_dev_t* module, const uint8_t* data, const uint8_t data_len)
{
    uint8_t obuf = WRITE_ACCESS(REG_FIFO);
    cs_select(module->cs_pin);
    if (spi_write_blocking(module->spi_dev, &obuf, 1) == 0) {
        LOG_ERR();
        return FAIL;
    }
    if (spi_write_blocking(module->spi_dev, data, data_len) == 0) {
        LOG_ERR();
        return FAIL;
    }
    cs_deselect(module->cs_pin);
    return DONE;
}

static uint read_from_fifo(const struct sx1278_dev_t* module, uint8_t* data, uint8_t data_len)
{
    uint8_t obuf = REG_FIFO;
    uint8_t dummy_buffer = 0x00;
    cs_select(module->cs_pin);
    if (spi_write_blocking(module->spi_dev, &obuf, 1) == 0) {
        LOG_ERR();
        return FAIL;
    }
    if (spi_read_blocking(module->spi_dev, dummy_buffer, data, data_len) == 0) {
        LOG_ERR();
        return FAIL;
    }
    cs_deselect(module->cs_pin);
    return DONE;
}

static uint sx1278_enable_lora_mode(const struct sx1278_dev_t* module)
{
    uint8_t reg_value;
    read_register(module, REG_OP_MODE, &reg_value);
    reg_value |= 0b10000000;
    if (write_register(module, REG_OP_MODE, reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

static uint sx1278_enable_hf_reg_mode(const struct sx1278_dev_t* module)
{
    uint8_t reg_value;
    read_register(module, REG_OP_MODE, &reg_value);
    reg_value &= ~(0b0001000);
    if (write_register(module, REG_OP_MODE, reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

static uint receive_package(const struct sx1278_dev_t* module)
{
    uint8_t rx_bytes_count;
    uint8_t rx_rssi;
    uint8_t rx_snr;
    uint8_t rx_fifo_addr;
    uint8_t* rx_buffer = (uint8_t*) malloc(64);
    read_register(module, REG_FIFO_RX_CURRENT_ADDR, &rx_fifo_addr);
    write_register(module, REG_FIFO_ADDR_PTR, rx_fifo_addr);
    read_register(module, REG_RX_NB_BYTES, &rx_bytes_count);
    read_register(module, REG_PKT_RSSI_VALUE, &rx_rssi);
    read_register(module, REG_PKT_SNR_VALUE, &rx_snr);
    read_from_fifo(module, rx_buffer, rx_bytes_count);
    dump_buffer(rx_buffer, rx_bytes_count);
    if (module->rx_callback_f) {
        module->rx_callback_f(rx_buffer, rx_bytes_count, rx_rssi, rx_snr);
    }
}

struct sx1278_dev_t* sx1278_create_device(const uint8_t mosi_pin,
                                          const uint8_t miso_pin,
                                          const uint8_t cs_pin,
                                          const uint8_t sck_pin,
                                          const uint8_t reset_pin,
                                          const uint8_t dio0_pin,
                                          spi_inst_t* spi_dev)
{
    struct sx1278_dev_t* module = (struct sx1278_dev_t*)
                                   malloc(sizeof(struct sx1278_dev_t));
    spi_init(spi_dev, SX1278_SPI_SPEED * 1000 * 1000);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_init(cs_pin);
    gpio_init(reset_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_set_dir(reset_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
    module->spi_dev = spi_dev;
    module->reset_pin = reset_pin;
    module->dio0_pin = dio0_pin;
    module->cs_pin = cs_pin;
    module->rx_callback_f = NULL;
    module->tx_callback_f = NULL;
    return module;
}

void sx1278_update(const struct sx1278_dev_t* module)
{
    uint8_t irq_reg_value;
    read_register(module, REG_IRQ_FLAGS, &irq_reg_value);
    if (irq_reg_value != 0x00) {
        reset_register(module, REG_IRQ_FLAGS);
        LOG("SX1278 update");
    }
    if (irq_reg_value & 0b10000000) {
        LOG("RxTimeout");
    }
    if (irq_reg_value & 0b01000000) {
        LOG("RxDone");
        receive_package(module);
    }
    if (irq_reg_value & 0b00100000) {
        LOG("PayloadCrcError");
    }
    if (irq_reg_value & 0b00010000) {
        LOG("ValidHeader");
    }
    if (irq_reg_value & 0b00001000) {
        LOG("TxDone");
        if (module->tx_callback_f != NULL) {
            module->tx_callback_f(DONE);
        }
    }
    if (irq_reg_value & 0b00000100) {
        LOG("CadDone");
    }
    if (irq_reg_value & 0b00000010) {
        LOG("FhssChangeChannel");
    }
    if (irq_reg_value & 0b00000001) {
        LOG("CadDetected");
    }
}

uint sx1278_init(const struct sx1278_dev_t* module)
{
    uint8_t module_version;
    sleep_ms(10);                      /* Need for sx1278 ready state */
    sx1278_reset(module);
    if (sx1278_get_version(module, &module_version) != DONE) {
        LOG_ERR();
        return 1;
    }
    if (module_version != VERSION_REG_VALUE) {
        LOG("ERROR: module not connected");
        return FAIL;
    }
    if (sx1278_set_mode(module, SLEEP_MODE) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (sx1278_enable_lora_mode(module) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (sx1278_enable_hf_reg_mode(module) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (write_register(module, REG_FIFO_RX_BASE_ADDR, 0x00) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (write_register(module, REG_FIFO_TX_BASE_ADDR, 0x00) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (write_register(module, REG_PA_CONFIG, 0xCF) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

void sx1278_reset(const struct sx1278_dev_t* module)
{
    gpio_put(module->reset_pin, 0);
    sleep_us(100);                      /* Reset the sx1278 */
    gpio_put(module->reset_pin, 1);
    sleep_ms(5);                      /* Wait the sx1278 ready state */
}

void sx1278_register_tx_callback(struct sx1278_dev_t* module, tx_callback callback)
{
    module->tx_callback_f = callback;
}

void sx1278_register_rx_callback(struct sx1278_dev_t* module, rx_callback callback)
{
    module->rx_callback_f = callback;
}

uint sx1278_get_version(const struct sx1278_dev_t* module, uint8_t* version)
{
    if (read_register(module, REG_VERSION, version) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_mode(const struct sx1278_dev_t* module, uint8_t mode)
{
    uint8_t reg_value;
    uint8_t fifo_rx_base_addr;
    if (read_register(module, REG_OP_MODE, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    reg_value &= 0b11111000;
    reg_value |= mode;
    if (mode == RX_CONTINUOUS_MODE || mode == RX_SINGLE_MODE) {
        if (read_register(module, REG_FIFO_RX_BASE_ADDR, &fifo_rx_base_addr) != DONE) {
            LOG_ERR();
            return FAIL;
        }
        if (write_register(module, REG_FIFO_ADDR_PTR, fifo_rx_base_addr) != DONE) {
            LOG_ERR();
            return FAIL;
        }
    }
    if (write_register(module, REG_OP_MODE, reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_frequency(const struct sx1278_dev_t* module, const uint32_t freq)
{
    const uint64_t mul = (1 << 19);
    const uint64_t f_xosc = 32000000;
    uint64_t frf  = (freq * mul) / f_xosc;
    uint8_t msb_reg;
    uint8_t mid_reg;
    uint8_t lsb_reg;
    if (freq > FREQUENCY_MAX_VALUE || freq < FREQUENCY_MIN_VALUE) {
        LOG_ERR();
        return FAIL;
    }
    lsb_reg = frf;
    mid_reg = (frf >> 8);
    msb_reg = (frf >> 16);
    if (write_register(module, REG_FR_LSB, lsb_reg) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (write_register(module, REG_FR_MID, mid_reg) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (write_register(module, REG_FR_MSB, msb_reg) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_ocp_threshold(const struct sx1278_dev_t* module, const uint8_t threshold)
{
    uint8_t reg_value = 0x00;
    if ((threshold > OCP_MAX_VALUE || threshold < OCP_MIN_VALUE) && threshold != 0) {
        LOG_ERR();
        return FAIL;
    }
    if (threshold == 0) {
        reg_value &= 0b11011111;
    } else if (threshold <= 120) {
        reg_value |= 0b00100000;
        reg_value |= (((threshold - 45) / 5) & 0b00001111);
    } else {
        reg_value |= 0b00100000;
        reg_value |= (((threshold + 30) / 10) & 0b00011111);
    }
    if (write_register(module, REG_OCP, reg_value) != 0) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_lna_gain(const struct sx1278_dev_t* module, const uint8_t gain)
{
    uint8_t reg_value;
    if (read_register(module, REG_LNA, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    reg_value &= 0b00011111;
    reg_value |= (gain << 5);
    if (write_register(module, REG_LNA, reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_lna_boost(const struct sx1278_dev_t* module, const uint8_t boost)
{
    uint8_t reg_value;
    if (read_register(module, REG_LNA, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (boost == LNA_BOOST_DEFAULT) {
        reg_value &= 0b11111100;
    } else if (boost == LNA_BOOST_ON) {
        reg_value |= 0b00000011;
    }
    if (write_register(module, REG_LNA, reg_value) != 0) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_bandwidth(const struct sx1278_dev_t* module, const uint bandwidth)
{
    uint8_t reg_value;
    if (read_register(module, REG_MODEM_CONFIG_1, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    reg_value &= 0b00001111;
    reg_value |= (bandwidth << 4);
    if (write_register(module, REG_MODEM_CONFIG_1, reg_value) != 0) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_coding_rate(const struct sx1278_dev_t* module, const uint8_t rate)
{
    uint8_t reg_value;
    if (read_register(module, REG_MODEM_CONFIG_1, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    reg_value &= 0b11110001;
    reg_value |= (rate << 1);
    if (write_register(module, REG_MODEM_CONFIG_1, reg_value) != 0) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_header_mode(const struct sx1278_dev_t* module, const uint8_t mode)
{
    uint8_t reg_value;
    if (read_register(module, REG_MODEM_CONFIG_1, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (mode == IMPLICITE_HEADER_MODE) {
        reg_value |= 0b00000001;
    } else if (mode == EXPLICITE_HEADER_MODE) {
        reg_value &= 0b11111110;
    }
    if (write_register(module, REG_MODEM_CONFIG_1, reg_value) != 0) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_set_spreading_factor(const struct sx1278_dev_t* module, const uint8_t sp_factor)
{
    uint8_t reg_value;
    if (read_register(module, REG_MODEM_CONFIG_2, &reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    reg_value &= 0b00001111;
    reg_value |= (sp_factor << 4);
    if (write_register(module, REG_MODEM_CONFIG_2, reg_value) != 0) {
        LOG_ERR();
        return FAIL;
    }
    return DONE;
}

uint sx1278_get_current_rssi(const struct sx1278_dev_t* module, int16_t* rssi)
{
    uint8_t rssi_reg_value;
    uint8_t op_mode_reg_value;
    if (read_register(module, REG_RSSI_VALUE, &rssi_reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (read_register(module, REG_OP_MODE, &op_mode_reg_value) != DONE) {
        LOG_ERR();
        return FAIL;
    }
    if (op_mode_reg_value & 0b00001000) { // HF port used
        *rssi = (-157 + (int16_t)rssi_reg_value);
    } else {
        *rssi = (-164 + (int16_t)rssi_reg_value);
    }
    return DONE;
}

uint sx1278_transmit(const struct sx1278_dev_t* module, const uint8_t* data, const uint data_len)
{
    uint8_t fifo_tx_base_addr;
    read_register(module, REG_FIFO_TX_BASE_ADDR, &fifo_tx_base_addr);
    write_register(module, REG_FIFO_ADDR_PTR, fifo_tx_base_addr);
    write_to_fifo(module, data, data_len);
    write_register(module, REG_PAYLOAD_LENGTH, data_len);
    sx1278_set_mode(module, TX_MODE);
}


uint sx1278_get_random_number(const struct sx1278_dev_t* module, uint8_t* random_num)
{
    return read_register(module, REG_RSSI_VALUE, random_num);
}

uint sx1278_get_register_value(const struct sx1278_dev_t* module, uint8_t address, uint8_t* reg_value)
{
    return read_register(module, address, reg_value);
}
