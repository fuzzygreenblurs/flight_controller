#define IMU_ADDRESS 0x68            // predefined i2c address for mpu6050
#define I2C_CHAN i2c0           // rp2040 i2c channel id 
#define SDA_PIN 8
#define SCL_PIN 9
#define IMU_INT_PIN 15
#define IMU_BAUD_RATE 400000        // max frequency of i2c standard speed specification

// fixed point type
typedef signed int fix15;
#define fix2int(a)   ((int)(a >> 15))
#define int2fix(a)   ((fix15)(a << 15))
#define float2fix(a) ((fix15)(a * 32768.0))             // you cannot simply bit shift a float: shifts are not defined for a floating point object
#define fix2float(a) ((float)(a) / 32768.0)             // bit representation is not consistent with a simple shift!
                                                        // note that we must use the 32768.0 operand. these operations are SLOW

/*
    for multiplication:
        1. cast both operands to signed long long (64 bit): note long long designates a minimum of 8 bytes or 64 bits
        2. perform the multiplication: for 2^64*2^64 => 128 bit resulting value. the top 64 bits will overflow from this operation
        3. right-shifting by 16 bits will truncate the least significant 16 bits of the result, leaving 48 bits
        4. casting to a fix15 (which is really a signed int of 32 bits), will overflow the top 16 bits since signed int on pico is 32 bits)
        
        note 1: if the two operands are too large for this case, then we will lose actual data. thus the numbers must be 
*/
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 16))
#define divfix15(a,b) (fix)(((( signed long long)(a) << 15) / (b)))
#define sqrtfix15(a) (float2fix(sqrt(fix2float(a))))








void imu_reset();
void imu_read_raw(fix15 accel[3], fix15 gyro[3]);