#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"          // access to spinlocks
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

volatile int corenum_0;
volatile int corenum_1;

spin_lock_t* accel_lock;
spin_lock_t* gyro_lock;

void init_locks() {
    accel_lock = spin_lock_instance(spin_lock_claim_unused(true));
    gyro_lock  = spin_lock_instance(spin_lock_claim_unused(true));
}

void init_i2c() {
    // TODO: is a state-machine style of logic more suited to the I2C pipeline?
    
    // initialize the i2c channel and 2 wire pins
    i2c_init(I2C_CHAN, IMU_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    
    // setup pin to monitor data-ready interrupts from IMU
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
    
    // software based pull-ups to accommodate I2C standard expectations
    gpio_pull_up(SDA_PIN);     
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(IMU_INT_PIN);

    /*
        note: GPIO interrupts on the RP2040 aren't enabled by default, even though individual pins can be configured for 
        interrupt generation using gpio_set_irq_enabled(). the interrupt handler for the GPIO bank itself must be enabled 
        to allow the CPU to respond to any GPIO-triggered interrupts
    */
    irq_set_enabled(IO_IRQ_BANK0, true)                          // emable the interrupt handler for the whole GPIO bank
    gpio_set_irq_enabled(IMU_INT_PIN, GPIO_IRQ_EDGE_FALL, true); // configure the INT_PIN to trigger an interrupt flag on falling edge
    gpio_set_irq_callback(&data_ready_isr);                      // bind the ISR callback to the IRQ
}

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

    // set data-ready interrupt pin to active-low
    const uint8_t interrupt_config[] = {0x37, 0x80};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, interrupt_config, 2, false);

    // enable data-ready interrupt from MPU6050
    const uint8_t enable_interrupt[] = {0x38, 0x01};
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, enable_interrupt, 2, false);
}

void data_ready_isr(uint gpio, uint32_t events) {
    /*
        why are we doing this redundant step of triggering the interrupt through the GPIO and then checking the status?
            POINT 1:
                - a GPIO is used to trigger for this ISR by pulling that line low if an interrupt occurs
                - however, the IMU can trigger an interrupt on that pin for many reasons, not just data-ready
                - thus, we need to check the data-ready bit status on the interrupt status register first

            POINT 2:
                - avoid race conditions: between the interrupt being triggered and data actually being ready on IMU

            POINT 3:
                - in some sensors, the INTERRUPT_STATUS register must be read to clear the interrupt flag on the device
                - without reading this register, the device might keep this interrupt active and not raise new flags if new data
                becomes available
    */

    uint8_t int_status_reg = 0x3A;
    uint8_t status;

    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, &int_status_reg, 1, true);
    i2c_read_blocking(I2C_CHAN, IMU_ADDRESS, &status, 1, true);

    if(status & 0x01) {
        // TODO: print which coreid we are running this on, for posterity
        read_imu();
    }
}

void read_imu() {
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

    /*
        spinlocks SDK:
            - spin_lock_blocking() / spin_unlock():
                - disables interrupts while holding the lock
                - this ensures the CPU isn't interrupted during the critical section
                - this guarantees thread safety
                - note: this prevents handling any interrupts during the lock, which may cause delays in handling other events

            - spin_lock_unsafe_blocking() / spin_unlock_unsafe():
                - does not disable interrupts
                - allows the CPU to handle interrupts while waiting for the lock or while holding it
                - this can be useful for high-priority or time-sensitive tasks
                - note: this requires more care to avoid potential issues (like race conditions)
    */

    spin_lock_blocking(accel_lock);
    for(int i = 0; i < 3; i++) {
        // buffer elements 0, 2, 4 are the most significant components of the reading
        // we concatenate these with their respective subsequent readings
        temp_accel = (temp_buffer[i << 1] << 8) | temp_buffer[(i << 1) + 1];
        accel_buffer[accel_write_idx++] = temp_accel;
        accel_write_idx %= BUFFER_SIZE;
    }
    spin_unlock(accel_lock);

    uint8_t gyro_start_register = 0x43;
    i2c_write_blocking(I2C_CHAN, IMU_ADDRESS, &gyro_start_register, 1, true);
    i2c_read_blocking(I2C_CHAN, IMU_ADDRESS, temp_buffer, 6, false);


    spin_lock_blocking(gyro_lock);
    for(int i = 0; i < 3; i++) {
        temp_gyro = (temp_buffer[i << 1] << 8) | temp_buffer[(i << 1) + 1];
        gyro_buffer[gyro_write_idx++] = temp_gyro;
        gyro_write_idx %= BUFFER_SIZE;
    }
    spin_unlock(gyro_lock);
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
    init_locks();
    multicore_launch_core1(core1_entry);

    // step 5: transmit the data outward over USB/UART
*/

    return 0;
}