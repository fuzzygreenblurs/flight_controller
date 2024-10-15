#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "imu.h"

/*
stdio: standard C library header file that provides functionality for input/output operations (ex: printf)
pico/stdlib.h: specific to the RPi Pico C SDK: it contains functions and macros to interact with the hardware of the Pico
*/

// TODO: do not expose these buffers as global variables. temporary solution
// TODO: ISR should push raw concat data into buffer. core should convert to fix15
// TODO: store 3 buffer pairs: noisy, complementary filtered, EKF
#define BUFFER_SIZE 64
volatile uint16_t accel_buffer[BUFFER_SIZE];
volatile uint16_t gyro_buffer[BUFFER_SIZE];

volatile int accel_write_idx = 0;
volatile int accel_read_idx = 0;
volatile int gyro_write_idx = 0;
volatile int gyro_read_idx = 0;

void init_imu() {
    // reset imu: see MPU6050 register map (reg: 0x6B) for further config details 
    const uint8_t reset_cmd[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, reset_cmd, 2, false);

    // sampling rate (SR) = gyroscope output rate / (1 + SMPLRT_DIV)
    // ex: SR = 8 khz/(1+7) => 1kHz
    const uint8_t sampling_rate[] = {0x19, 0b00000111};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, sampling_rate, 2, false);

    // set +/- 2g accelerometer range (16 bit payload resolution is adjusted based on this range)
    const uint8_t accel_config[] = {0x1C, 0x00};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, accel_config, 2, false);

    // set +/- 250deg/sec gyro range
    const uint8_t gyro_config[] = {0x1B, 0x00};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, gyro_config, 2, false);

    // enable data-ready interrupt from MPU6050
    uint8_t interrupt_config = {0x38, 0x01};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, interrupt_config, 2, false);
}

void imu_read_raw() {
    int16_t temp_accel, temp_gyro;
    uint8_t temp_buffer[6];

    /* some notes:
        - i2c_write_blocking takes a boolean param (true) to allow main to retain control of the bus
        - this allows for repeated start conditions without performing future handshakes
    
        - in this case, pass in a single register address without a second value
            - main isn't writing any actual data to the register, just telling the IMU where to start reading from
            - i.e. main (RP2040) tells the IMU, "I want to read data, starting from this register."
    */
    uint8_t accel_start_register = 0x3B
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, &accel_start_register, 1, true);
    i2c_read_blocking(I2C_CHAN, IMU_ADDRESS, temp_buffer, 6, false);

    for(int i = 0; i < 3; i++) {
        // buffer elements 0, 2, 4 are the most significant components of the reading
        // we concatenate these with their respective subsequent readings
        temp_accel = (temp_buffer[i << 1] << 8) | temp_buffer[(i << 1) + 1];
        accel_buffer[accel_write_idx++] = temp_accel;
        accel_write_idx %= BUFFER_SIZE;
    }

    uint8_t gyro_start_register = 0x43;
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, &gyro_start_register, 1, true);
    i2c_read_blocking(I2C_CHAN, IMU_ADDRESS, temp_buffer, 6, false);

    for(int i = 0; i < 3; i++) {
        temp_gyro = (temp_buffer[i << 1] << 8) | temp_buffer[(i << 1) + 1];
        gyro_buffer[gyro_write_idx++] = temp_gyro;
        gyro_write_idx %= BUFFER_SIZE;
    }
}

void init_i2c() {
    i2c_init(I2C_CHAN, IMU_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    
    // software based pull-ups to accommodate I2C standard expectations
    gpio_pull_up(SDA_PIN);     
    gpio_pull_up(SCL_PIN);

    // setup pin to monitor data-ready interrupts from IMU
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
}

void core1_entry() {
    // clear any hanging interrupts
    multicore_fifo_clear_irq(); 
    
    init_i2c();
    init_imu();
}

int main(void) {
    // initialize the standard I/O system on the RP2040 to communicate over UART/USB
    stdio_init_all();   

    // start core 1 - Do this before any interrupt configuration
    multicore_launch_core1(core1_entry);

    // step 2: CORE_1: enable the data ready interrupt on the mpu6050
    // step 3  : initialize interrupts on the rp2040
    // step 4: use a circular buffer/FIFO to store incoming data
    // step 5: transmit the data outward over USB/UART
*/

    return 0;
}