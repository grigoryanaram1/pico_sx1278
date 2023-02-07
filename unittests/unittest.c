#include <pico/stdio.h>

#include "sx1278.h"
#include "sx1278_registers.h"

#define ENABLE_TRACES
#include "logger.h"

#define UNITTEST_CHECK(val, golden, conc) if ((val) != (golden)) { LOG("[ --- FAIL --- ]"); conc++;} else { LOG(" --- PASS --- "); }

#define SX1278_SCK_PIN    18
#define SX1278_MISO_PIN   16
#define SX1278_MOSI_PIN   19
#define SX1278_CS_PIN     17
#define SX1278_RESET_PIN  20
#define SX1278_DI0_PIN    21
#define SX1278_SPI_DEV    spi0

struct sx1278_dev_t* lora_module;

static uint network_init(void)
{
    LOG("Start network initialization");
    lora_module = sx1278_create_device(SX1278_MOSI_PIN,
                                         SX1278_MISO_PIN,
                                         SX1278_CS_PIN,
                                         SX1278_SCK_PIN,
                                         SX1278_RESET_PIN,
                                         SX1278_DI0_PIN,
                                         SX1278_SPI_DEV);
    if (lora_module == NULL) {
        CRITICAL("Device creating failed");
        return 1;
    }
    return sx1278_init(lora_module);
}

static uint unittest_op_mode(void)
{
    uint8_t reg_value;
    uint8_t conclusion = 0;
    LOG(" --- OP MODE TEST START --- ");
    sx1278_set_mode(lora_module, SLEEP_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 128, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, STDBY_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 129, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, FSTX_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 130, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, TX_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 131, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, FSRX_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 132, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, RX_CONTINUOUS_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 133, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, RX_SINGLE_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 134, conclusion);
    sleep_ms(50);
    sx1278_set_mode(lora_module, CAD_MODE);
    sx1278_get_register_value(lora_module, REG_OP_MODE, &reg_value);
    UNITTEST_CHECK(reg_value, 135, conclusion);
    if (conclusion != 0) {
        LOG(" --- OP MODE TEST FAIL --- ");
        return 1;
    }
    LOG(" --- OP MODE TEST PASS --- ");
    return 0;
}

static uint unittest_frequency(void)
{
    uint8_t reg_value;
    uint8_t conclusion = 0;
    LOG(" --- FREQUENCY TEST START --- ");
    sx1278_set_frequency(lora_module, 434E6);
    sx1278_get_register_value(lora_module, REG_FR_MSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x6c, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_MID, &reg_value);
    UNITTEST_CHECK(reg_value, 0x80, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_LSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x00, conclusion);
    sleep_ms(50);
    sx1278_set_frequency(lora_module, 430E6);
    sx1278_get_register_value(lora_module, REG_FR_MSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x6b, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_MID, &reg_value);
    UNITTEST_CHECK(reg_value, 0x80, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_LSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x00, conclusion);
    sleep_ms(50);
    sx1278_set_frequency(lora_module, 410E6);
    sx1278_get_register_value(lora_module, REG_FR_MSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x66, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_MID, &reg_value);
    UNITTEST_CHECK(reg_value, 0x80, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_LSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x00, conclusion);
    sleep_ms(50);
    sx1278_set_frequency(lora_module, 480E6);
    sx1278_get_register_value(lora_module, REG_FR_MSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x83, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_MID, &reg_value);
    UNITTEST_CHECK(reg_value, 0x40, conclusion);
    sx1278_get_register_value(lora_module, REG_FR_LSB, &reg_value);
    UNITTEST_CHECK(reg_value, 0x00, conclusion);
    sleep_ms(50);
    if (conclusion != 0) {
        LOG(" --- FREQUENCY TEST FAIL --- ");
        return 1;
    }
    LOG(" --- FREQUENCY TEST PASS --- ");
    return 0;
}

static void run_test(void)
{
    (void)unittest_op_mode();
    (void)unittest_frequency();
}

int main()
{
    stdio_init_all();
    if (network_init() != 0) {
        LOG("[FAIL] Unittest failed");
        //goto end;
    }
    run_test();
end:
    while (1) {
        asm("nop");
    }
    return 0;
}