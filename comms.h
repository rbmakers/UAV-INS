#pragma once
/**
 * comms/comms.h — Communication Abstraction Layer
 *
 * Wraps every physical bus used by the nav system.
 * On RP2350 these map to the pico-sdk primitives.
 * On a simulation host they can be stubbed.
 *
 * Bus assignments (RB-RP2354A board):
 *   SPI0  (GP18 CLK, GP19 MOSI, GP20 MISO)
 *         CS_GYRO   = GP17
 *         CS_ACCEL  = GP16
 *         CS_BARO   = GP15
 *         CS_FLOW   = GP14
 *   I2C1  (GP6 SDA, GP7 SCL)  — BMM350, MS4525
 *   UART0 (GP0 TX, GP1 RX)    — GNSS u-blox
 *   UART1 (GP4 TX, GP5 RX)    — LiDAR TFmini-S / FOG RS-422
 *   UART2 (GP8 TX, GP9 RX)    — Companion CPU MAVLink
 */

#include "../nav_system.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

/* =========================================================
 * BUS INSTANCES
 * ========================================================= */
#define COMMS_SPI_IMU_BARO      spi0
#define COMMS_I2C_MAG_AS        i2c1
#define COMMS_UART_GNSS         uart0
#define COMMS_UART_LIDAR_FOG    uart1
#define COMMS_UART_COMPANION    uart2   /* MAVLink to companion CPU */

/* SPI clock speeds */
#define COMMS_SPI_GYRO_HZ       8000000u    /* 8 MHz — BMI088 gyro max */
#define COMMS_SPI_ACCEL_HZ      8000000u    /* 8 MHz — BMI088 accel max */
#define COMMS_SPI_BARO_HZ       8000000u    /* 8 MHz — BMP580 */
#define COMMS_SPI_FLOW_HZ       2000000u    /* 2 MHz — PMW3901 */

/* UART baud rates */
#define COMMS_BAUD_GNSS         921600u
#define COMMS_BAUD_TFMINI       115200u
#define COMMS_BAUD_FOG          921600u     /* KVH 1775 default */
#define COMMS_BAUD_COMPANION    921600u

/* I2C speed */
#define COMMS_I2C_FAST_HZ       400000u

/* =========================================================
 * GPIO PIN ASSIGNMENTS
 * ========================================================= */
#define PIN_SPI_CLK     18
#define PIN_SPI_MOSI    19
#define PIN_SPI_MISO    20
#define PIN_CS_GYRO     17
#define PIN_CS_ACCEL    16
#define PIN_CS_BARO     15
#define PIN_CS_FLOW     14
#define PIN_I2C_SDA     6
#define PIN_I2C_SCL     7
#define PIN_UART0_TX    0
#define PIN_UART0_RX    1
#define PIN_UART1_TX    4
#define PIN_UART1_RX    5
#define PIN_UART2_TX    8
#define PIN_UART2_RX    9
#define PIN_IMU_INT     21   /* BMI088 gyro data-ready interrupt */
#define PIN_GNSS_PPS    22   /* GNSS 1PPS for time sync */

/* =========================================================
 * INITIALISATION
 * ========================================================= */
static inline void comms_init_all(void) {
    /* SPI0 — IMU, Baro, Flow */
    spi_init(COMMS_SPI_IMU_BARO, COMMS_SPI_GYRO_HZ);
    gpio_set_function(PIN_SPI_CLK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);

    /* CS lines — all start deasserted (high) */
    uint cs_pins[] = { PIN_CS_GYRO, PIN_CS_ACCEL, PIN_CS_BARO, PIN_CS_FLOW };
    for (unsigned i = 0; i < 4; i++) {
        gpio_init(cs_pins[i]);
        gpio_set_dir(cs_pins[i], GPIO_OUT);
        gpio_put(cs_pins[i], 1);
    }

    /* I2C1 — Magnetometer + Airspeed sensor */
    i2c_init(COMMS_I2C_MAG_AS, COMMS_I2C_FAST_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    /* UART0 — GNSS */
    uart_init(COMMS_UART_GNSS, COMMS_BAUD_GNSS);
    gpio_set_function(PIN_UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART0_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(COMMS_UART_GNSS, false, false);
    uart_set_format(COMMS_UART_GNSS, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(COMMS_UART_GNSS, true);

    /* UART1 — LiDAR / FOG */
    uart_init(COMMS_UART_LIDAR_FOG, COMMS_BAUD_TFMINI);
    gpio_set_function(PIN_UART1_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART1_RX, GPIO_FUNC_UART);
    uart_set_fifo_enabled(COMMS_UART_LIDAR_FOG, true);

    /* UART2 — Companion CPU MAVLink */
    uart_init(COMMS_UART_COMPANION, COMMS_BAUD_COMPANION);
    gpio_set_function(PIN_UART2_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART2_RX, GPIO_FUNC_UART);
    uart_set_fifo_enabled(COMMS_UART_COMPANION, true);

    /* IMU data-ready interrupt pin */
    gpio_init(PIN_IMU_INT);
    gpio_set_dir(PIN_IMU_INT, GPIO_IN);
    gpio_pull_down(PIN_IMU_INT);
}

/* =========================================================
 * SPI HELPERS (register-based, single-byte)
 * ========================================================= */
static inline void _cs(uint pin, bool assert) {
    gpio_put(pin, assert ? 0 : 1);
    sleep_us(1);
}

static inline void spi_write_reg(uint cs_pin, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };
    _cs(cs_pin, true);
    spi_write_blocking(COMMS_SPI_IMU_BARO, buf, 2);
    _cs(cs_pin, false);
}

static inline uint8_t spi_read_reg(uint cs_pin, uint8_t reg) {
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = { 0, 0 };
    _cs(cs_pin, true);
    spi_write_read_blocking(COMMS_SPI_IMU_BARO, tx, rx, 2);
    _cs(cs_pin, false);
    return rx[1];
}

static inline void spi_read_burst(uint cs_pin, uint8_t reg,
                                   uint8_t *buf, size_t len) {
    uint8_t tx_hdr = reg | 0x80;
    _cs(cs_pin, true);
    spi_write_blocking(COMMS_SPI_IMU_BARO, &tx_hdr, 1);
    spi_read_blocking(COMMS_SPI_IMU_BARO, 0x00, buf, len);
    _cs(cs_pin, false);
}

/* =========================================================
 * I2C HELPERS
 * ========================================================= */
static inline void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(COMMS_I2C_MAG_AS, addr, buf, 2, false);
}

static inline uint8_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    uint8_t val = 0;
    i2c_write_blocking(COMMS_I2C_MAG_AS, addr, &reg, 1, true);
    i2c_read_blocking(COMMS_I2C_MAG_AS, addr, &val, 1, false);
    return val;
}

static inline void i2c_read_burst(uint8_t addr, uint8_t reg,
                                   uint8_t *buf, size_t len) {
    i2c_write_blocking(COMMS_I2C_MAG_AS, addr, &reg, 1, true);
    i2c_read_blocking(COMMS_I2C_MAG_AS, addr, buf, len, false);
}

/* =========================================================
 * UART RING BUFFER (256-byte, lock-free single-producer/consumer)
 * Used for GNSS and LiDAR streams on Core 1
 * ========================================================= */
#define UART_RING_SIZE  256u

typedef struct {
    volatile uint8_t  buf[UART_RING_SIZE];
    volatile uint16_t head;   /* write index */
    volatile uint16_t tail;   /* read index  */
} UartRing;

static inline void uart_ring_push(UartRing *r, uint8_t byte) {
    uint16_t next = (r->head + 1) & (UART_RING_SIZE - 1);
    if (next != r->tail) {          /* drop on overflow */
        r->buf[r->head] = byte;
        r->head = next;
    }
}

static inline bool uart_ring_pop(UartRing *r, uint8_t *out) {
    if (r->head == r->tail) return false;
    *out = r->buf[r->tail];
    r->tail = (r->tail + 1) & (UART_RING_SIZE - 1);
    return true;
}

/* Global ring buffers (filled by Core 1 UART polling) */
extern UartRing g_ring_gnss;
extern UartRing g_ring_lidar;
extern UartRing g_ring_companion;
