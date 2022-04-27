/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>

static const char *TAG = "Prototype";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR         0x68        /*!< Slave address of the MPU9250 sensor */
#define AK8963_ADDRESS              0x0C        // Magnetometer address
#define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register (register 117)*/
#define AK8963_WHO_AM_I             0x00        // should return 0x48

#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT           7

#define MPU9250_ACCEL_XOUT_H        0x3B        //Accélèromètre
#define MPU9250_GYRO_XOUT_H         0x43        //Gyroscope
#define AK8963_XOUT_L               0x03        //Magnétomètre Low

#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG2       0x1D
#define MPU9250_I2C_MST_STATUS      0x36        // R
#define MPU9250_INT_PIN_CFG         0x37        // R/W

#define AK8963_CNTL1                0x0A        // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC                 0x0C        // Self test control
#define AK8963_INFO                 0x01

#define MPU9250_INT_ENABLE          0x38
#define M_PI                        3.14159265
#define DEG2RAD(deg)                (deg * M_PI / 180.0f)

float ax, ay, az, gx, gy, gz, mx, my, mz;
float heading, pitch, roll;
uint8_t data_write[20];
uint8_t data_read[20];

typedef struct
{
  float x, y, z;
} vector_t;

vector_t va, vg, vm;

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Acquisition de l'accélération
 * 
 */
static vector_t accelerometer(void)
{
    // data_write[0] = MPU9250_WHO_AM_I_REG_ADDR;
    // i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    // printf("WHO_AM_I (should return 0x71) = 0x%04X\n", data_read[0]);

    /* -------------- Accelerometer -------------- */
    data_write[0] = MPU9250_ACCEL_XOUT_H;
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, data_write, 1, data_read, 6, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    int16_t accel_xout = (int16_t)((data_read[0] << 8) | data_read[1]);
    int16_t accel_yout = (int16_t)((data_read[2] << 8) | data_read[3]);
    int16_t accel_zout = (int16_t)((data_read[4] << 8) | data_read[5]);

    float ax = (float)accel_xout/16384; // LSB/g -> g
    float ay = (float)accel_yout/16384;
    float az = (float)accel_zout/16384;

    printf("ACCELEROMETRE \n %.2f g\n %.2f g\n %.2f g\n", ax, ay, az);
    va.x = ax;
    va.y = ay;
    va.z = az;
    return va;
}

/**
 * @brief Acquision de la vitesse angulaire
 * 
 */
static vector_t gyroscope(void)
{
    data_write[0] = MPU9250_GYRO_XOUT_H;
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, data_write, 1, data_read, 6, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    
    // Registres : High puis Low
    int16_t gyro_xout = (int16_t)((data_read[0] << 8) | data_read[1]);
    int16_t gyro_yout = (int16_t)((data_read[2] << 8) | data_read[3]);
    int16_t gyro_zout = (int16_t)((data_read[4] << 8) | data_read[5]);

    float gx = (float)gyro_xout/131;
    float gy = (float)gyro_yout/131;
    float gz = (float)gyro_zout/131;

    printf("GYROSCOPE \n %.2f °/s\n %.2f °/s\n %.2f °/s\n", gx, gy, gz);
    
    vg.x = gx;
    vg.y = gy;
    vg.z = gz;
    return vg;
}

/**
 * @brief Acquisition de la direction du champ magnétique
 * 
 */
static vector_t magnetometer(void)
{
    data_write[0] = AK8963_WHO_AM_I;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("WHO_AM_I (should return 0x48) = 0x%04X\n", data_read[0]);

    data_write[0] = AK8963_XOUT_L;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 6, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    // Register : Low then High
    int16_t mag_xout = (int16_t)((data_read[1] << 8) | data_read[0]); 
    int16_t mag_yout = (int16_t)((data_read[3] << 8) | data_read[2]);
    int16_t mag_zout = (int16_t)((data_read[5] << 8) | data_read[4]);

    float mx = (float)mag_xout*0.6;
    float my = (float)mag_yout*0.6;
    float mz = (float)mag_zout*0.6;

    printf("MAGNETOMETRE \n %.2f µT\n %.2f µT\n %.2f µT\n", mx, my, mz);

    float norme = sqrtf(pow(mx, 2) + pow(my, 2) + pow(mz, 2));
    printf("Norme du vecteur : %.2f \n", norme);

    vm.x = mx;
    vm.y = my;
    vm.z = mz;
    return vm;
}

volatile float sampleFreq = 50;                            // 2 * proportional gain (Kp)
volatile float beta = 0.8;                                 // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

/**
 * @brief Fast inverse square-root
 * 
 * @param x 
 * @return float 
 */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


/**
 * @brief AHRS algorithm update
 * 
 * @param gx Axe x gyroscope
 * @param gy Axe y gyroscope
 * @param gz Axe z gyroscope
 * @param ax Axe x accéléromètre
 * @param ay Axe y accéléromètre
 * @param az Axe z Accéléromètre
 * @param mx Axe x magnétomètre
 * @param my Axe y magnétomètre
 * @param mz Axe z magnétomètre
 */
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    ESP_LOGI(TAG, "MadgwickAHRSupdate begin");
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    ESP_LOGI(TAG, "MadgwickAHRSupdate end");
}

float norm_angle_0_2pi(float a)
{
  a = fmod(a, M_PI * 2.0);
  if (a < 0)
  {
    a += M_PI * 2.0;
  }
  return a;
}

/**
 * Return an object with the Euler angles {heading; pitch, roll}, in radians.
 *
 * Where:
 *   - heading is from magnetic north, going west (about z-axis).
 *   - pitch is from vertical, going forward (about y-axis).
 *   - roll is from vertical, going right (about x-axis).
 *
 * Thanks to:
 *   https://github.com/PenguPilot/PenguPilot/blob/master/autopilot/service/util/math/quat.c#L103
 * @return {object} {heading, pitch, roll} in radians
 */
void MadgwickGetEulerAngles(float *heading, float *pitch, float *roll)
{
  float ww = q0 * q0;
  float xx = q1 * q1;
  float yy = q2 * q2;
  float zz = q3 * q3;
  *heading = norm_angle_0_2pi(atan2f(2.0 * (q1 * q2 + q3 * q0), xx - yy - zz + ww));
  *pitch = asinf(-2.0 * (q1 * q3 - q2 * q0));
  *roll = atan2(2.0 * (q2 * q3 + q1 * q0), -xx - yy + zz + ww);
}

#define RAD_2_DEG (180.0f / M_PI)


/**
 * Return an object with the Euler angles {heading, pitch, roll}, in radians.
 *
 * Where:
 *   - heading is from magnetic north, going west (about z-axis).
 *   - pitch is from vertical, going forward (about y-axis).
 *   - roll is from vertical, going right (about x-axis).
 *
 * Thanks to:
 *   https://github.com/PenguPilot/PenguPilot/blob/master/autopilot/service/util/quat.c#L103
 * @param heading 
 * @param pitch 
 * @param roll 
 * @return {object} {heading, pitch, roll} in radians
 */
void MadgwickGetEulerAnglesDegrees(float *heading, float *pitch, float *roll)
{
  MadgwickGetEulerAngles(heading, pitch, roll);

  *heading *= RAD_2_DEG;
  *pitch *= RAD_2_DEG;
  *roll *= RAD_2_DEG;
}


void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    mpu9250_register_write_byte(MPU9250_INT_PIN_CFG, 1 << 1); // Ecriture dans le registre pour activer le mode pass-through

    // 0000 : power-down mode
    // 0001 : single measurement mode
    // 0010 : continuous measurement mode 1
    // 0110 : continuous measurement mode 2
    // 0100 : external trigger measurement mode
    // 1000 : self-test mode, 1111 : fuse ROM access mode

    uint8_t write_buf[8] = {AK8963_CNTL1, 1 << 1 | 1 << 2}; // 0110 : continuous measurement mode 2
    i2c_master_write_to_device(I2C_MASTER_NUM, AK8963_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    /* Acquistitions */
    while(1)
    {
        accelerometer();
        gyroscope();
        magnetometer();
        
        MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z), va.x, va.y, va.z, vm.x, vm.y, vm.z);

        MadgwickGetEulerAnglesDegrees(&heading, &pitch, &roll);
        
        printf("heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°\n", heading, pitch, roll);

        printf("------------------------------------------\n");
        vTaskDelay(50);
    }

    /* Demonstrate writing by reseting the MPU9250 */
    ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
}