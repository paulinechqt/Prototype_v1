#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "ble.h"
#include <math.h>
#include <time.h>
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include "freertos/queue.h"

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

#define MPU9250_RA_USER_CTRL        0x6A
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
#define AK8963_ASAX                 0x10        // Magnetic sensor X-axis sensitivity adjustment value
#define AK8963_ASAY                 0x11        // Magnetic sensor Y-axis sensitivity adjustment value
#define AK8963_ASAZ                 0x12        // Magnetic sensor Z-axis sensitivity adjustment value
#define AK8963_STATUS_1             0x02
#define AK8963_STATUS_2             0x09

#define MPU9250_INT_ENABLE          0x38
#define DEG2RAD(deg)                (deg * M_PI / 180.0f)
#define RAD_2_DEG                   (180.0f / M_PI)

const char *TAG = "Prototype";

QueueHandle_t queue_ax, queue_ay, queue_az, queue_gx, queue_gy, queue_gz, queue_mx, queue_my, queue_mz;

uint8_t data_write[20];
uint8_t data_read[20];

char prof_shared_buf[22];

typedef struct
{
    float x, y, z;
} vector_t;

vector_t va, vg, vm;

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
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

void startMagnetometer()
{
    mpu9250_register_write_byte(MPU9250_RA_USER_CTRL, 0 << 5);

    data_write[0] = MPU9250_RA_USER_CTRL;
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("MPU9250_RA_USER_CTRL (should return 0): 0x%04X\n", data_read[0]);

    mpu9250_register_write_byte(MPU9250_INT_PIN_CFG, 1 << 1);

    data_write[0] = MPU9250_INT_PIN_CFG;
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("MPU9250_INT_PIN_CFG (should return 2): 0x%04X\n", data_read[0]);

    // Setup the Magnetomete to fuse ROM accesse mode to get the Sensitivity Adjustment
    uint8_t write_buf_2[8] = {AK8963_CNTL1, 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4}; // 0001 1111 : Fuse ROM access mode
    i2c_master_write_to_device(I2C_MASTER_NUM, AK8963_ADDRESS, write_buf_2, sizeof(write_buf_2), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    data_write[0] = AK8963_CNTL1;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_CNTL1 (should return 1F): 0x%04X\n\n", data_read[0]);

    // Wait for the mode changes
    vTaskDelay(100/portTICK_PERIOD_MS); 

    // Read the Sensitivity Adjustement values and calculations
    data_write[0] = AK8963_ASAX;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_ASAX : 0x%04X\n", data_read[0]);
    printf("AK8963_ASAX décimal : 0x%d\n\n", data_read[0]);
    float asax = (data_read[0]-128)*0.5/128+1;

    data_write[0] = AK8963_ASAY;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_ASAY : 0x%08X\n", data_read[0]);
    printf("AK8963_ASAY décimal : 0x%d\n\n", data_read[0]);
    float asay = (data_read[0]-128)*0.5/128+1;

    data_write[0] = AK8963_ASAZ;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_ASAZ : 0x%08X\n", data_read[0]);
    printf("AK8963_ASAZ décimal : 0x%d\n\n", data_read[0]);
    float asaz = (data_read[0]-128)*0.5/128+1;

    printf("asax = %.3f\nasay = %.3f\nasaz = %.3f\n", asax, asay, asaz);

    // Reset the magnetometer to power down mode
    uint8_t write_buf_3[8] = {AK8963_CNTL1, 0 << 0 | 0 << 1 | 0 << 2 | 0 << 3 | 0 << 4};
    i2c_master_write_to_device(I2C_MASTER_NUM, AK8963_ADDRESS, write_buf_3, sizeof(write_buf_3), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    data_write[0] = AK8963_CNTL1;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_CNTL1 power off (should return 0): 0x%04X\n\n", data_read[0]);

    //  Wait for the mode changes
    vTaskDelay(100/portTICK_PERIOD_MS);

    // Set the magnetometer to continuous mode 2 (100Hz) and 16-bit output
    uint8_t write_buf_4[8] = {AK8963_CNTL1, 1 << 1 | 1 << 2 | 1 << 4}; // 0001 1111 : Fuse ROM access mode
    i2c_master_write_to_device(I2C_MASTER_NUM, AK8963_ADDRESS, write_buf_4, sizeof(write_buf_4), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    data_write[0] = AK8963_CNTL1;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_CNTL1 continuous mode 2 (should return 16): 0x%04X\n\n", data_read[0]);

    //  Wait for the mode changes
    vTaskDelay(100/portTICK_PERIOD_MS);

    data_write[0] = AK8963_STATUS_1;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    printf("AK8963_STATUS_1 (1 : ready | 0 : not ready): %d\n\n", data_read[0]);
}

float accelerometer_scale();
float gyroscope_scale();

/**
 * @brief Acquisition de l'accélération
 * 
 */
vector_t accelerometer(void)
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

    float ax = (float)accel_xout/accelerometer_scale(); // LSB/g -> g
    float ay = (float)accel_yout/accelerometer_scale();
    float az = (float)accel_zout/accelerometer_scale();

    va.x = ax;
    va.y = ay;
    va.z = az;
    return va;
}

/**
 * 
 * @brief Acquision de la vitesse angulaire
 * 
 */
vector_t gyroscope(void)
{
    data_write[0] = MPU9250_GYRO_XOUT_H;
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, data_write, 1, data_read, 6, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    
    // Registres : High then Low
    int16_t gyro_xout = (int16_t)((data_read[0] << 8) | data_read[1]);
    int16_t gyro_yout = (int16_t)((data_read[2] << 8) | data_read[3]);
    int16_t gyro_zout = (int16_t)((data_read[4] << 8) | data_read[5]);

    float gx = (float)gyro_xout/131;
    float gy = (float)gyro_yout/131;
    float gz = (float)gyro_zout/131;
    
    vg.x = gx;
    vg.y = gy;
    vg.z = gz;
    return vg;
}

/**
 * @brief Acquisition de la direction du champ magnétique
 * 
 */
vector_t magnetometer(void)
{
    data_write[0] = AK8963_WHO_AM_I;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (data_read[0 != 0x48])
    {
        perror("  WHO_AM_I Magnetometer");
    }
    else
    {
        printf("  WHO_AM_I (should return 0x48) = 0x%04X\n", data_read[0]);  
    }


    data_write[0] = AK8963_XOUT_L;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 6, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    // Register : Low then High
    int16_t mag_xout = (int16_t)((data_read[1] << 8) | data_read[0]);
    int16_t mag_yout = (int16_t)((data_read[3] << 8) | data_read[2]);
    int16_t mag_zout = (int16_t)((data_read[5] << 8) | data_read[4]);

    float mx = (float)mag_xout*0.6;
    float my = (float)mag_yout*0.6;
    float mz = (float)mag_zout*0.6;

    vm.x = mx;
    vm.y = my;
    vm.z = mz;
    
    data_write[0] = AK8963_STATUS_2;
    i2c_master_write_read_device(I2C_MASTER_NUM, AK8963_ADDRESS, data_write, 1, data_read, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS); // Lecture du registre AK8963_STATUS_2 pour changer valeur mgnétomètre
    // printf("AK8963_STATUS_2: %d\n\n", data_read[0]);
    
    return vm;
}

int taille = 10; // Longueur de la queue

void createQueues(int taille) // Création des 9 neufs queues
{
    taille = taille*2+1;
    queue_ax = xQueueCreate(taille, sizeof(float));
    queue_ay = xQueueCreate(taille, sizeof(float));
    queue_az = xQueueCreate(taille, sizeof(float));
    queue_gx = xQueueCreate(taille, sizeof(float));
    queue_gy = xQueueCreate(taille, sizeof(float));
    queue_gz = xQueueCreate(taille, sizeof(float));
    queue_mx = xQueueCreate(taille, sizeof(float));
    queue_my = xQueueCreate(taille, sizeof(float));
    queue_mz = xQueueCreate(taille, sizeof(float));
}

void storeValues(vector_t va, vector_t vg, vector_t vm) // Envoie des valeurs acquises dans chaque queue
{
    xQueueSend(queue_ax, (void*)(&va.x), pdMS_TO_TICKS(10000));
    xQueueSend(queue_ay, (void*)(&va.y), pdMS_TO_TICKS(10000));
    xQueueSend(queue_az, (void*)(&va.z), pdMS_TO_TICKS(10000));
    xQueueSend(queue_gx, (void*)(&vg.x), pdMS_TO_TICKS(10000));
    xQueueSend(queue_gy, (void*)(&vg.y), pdMS_TO_TICKS(10000));
    xQueueSend(queue_gz, (void*)(&vg.z), pdMS_TO_TICKS(10000));
    xQueueSend(queue_mx, (void*)(&vm.x), pdMS_TO_TICKS(10000));
    xQueueSend(queue_my, (void*)(&vm.y), pdMS_TO_TICKS(10000));
    xQueueSend(queue_mz, (void*)(&vm.z), pdMS_TO_TICKS(10000));
}

void bin(int val)
{
    printf("%d en binaire : ", val);
    for (int i=31; i>=0; i--)
    {
        printf("%d", (val >> i ) & 1);
    }
}

// Récupération des valeurs dans les queues, décomposition en 2 octets et écriture dans prof_shared_buf
void sendValues(QueueHandle_t queue_ax, QueueHandle_t queue_ay, QueueHandle_t queue_az, QueueHandle_t queue_gx, QueueHandle_t queue_gy, QueueHandle_t queue_gz, QueueHandle_t queue_mx, QueueHandle_t queue_my, QueueHandle_t queue_mz)
{
    float ax_float, ay_float, az_float, gx_float, gy_float, gz_float, mx_float, my_float, mz_float;
    short ax, ay, az, gx, gy, gz, mx, my, mz;

    // --- Récupération des mesures à la sortie des queues ---
    xQueueReceive(queue_ax, (void*)(&ax_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_ay, (void*)(&ay_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_az, (void*)(&az_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_gx, (void*)(&gx_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_gy, (void*)(&gy_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_gz, (void*)(&gz_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_gx, (void*)(&mx_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_gy, (void*)(&my_float), pdMS_TO_TICKS(10000));
    xQueueReceive(queue_gz, (void*)(&mz_float), pdMS_TO_TICKS(10000));

    // --- Affichage des flottants ---
    printf("ax_float = %.3f\n", ax_float);
    printf("ay_float = %.3f\n", ay_float);
    printf("az_float = %.3f\n", az_float);
    printf("gx_float = %.3f\n", gx_float);
    printf("gy_float = %.3f\n", gy_float);
    printf("gz_float = %.3f\n", gz_float);
    printf("mx_float = %.3f\n", mx_float);
    printf("my_float = %.3f\n", my_float);
    printf("mz_float = %.3f\n\n", mz_float);

    // --- Multiplication par 1000 pour avoir des entiers ---
    ax=ax_float*1000;
    ay=ay_float*1000;
    az=az_float*1000;
    gx=gx_float*1000;
    gy=gy_float*1000;
    gz=gz_float*1000;
    mx=mx_float*1000;
    my=my_float*1000;
    mz=mz_float*1000;

    // --- Affichage des entiers ---
    // printf("ax = %d\n", ax);
    // printf("ay = %d\n", ay);
    // printf("az = %d\n", az);
    // printf("gx = %d\n", gx);
    // printf("gy = %d\n", gy);
    // printf("gz = %d\n", gz);
    // printf("mx = %d\n", mx);
    // printf("my = %d\n", my);
    // printf("mz = %d\n\n", mz);
    
    // --- Ecriture des entiers sur 2 octets chacun dans le buffer de la caractéristique--
    prof_shared_buf[0] = ax>>0;
    prof_shared_buf[1] = ax>>8;
    prof_shared_buf[2] = ay>>0;
    prof_shared_buf[3] = ay>>8;
    prof_shared_buf[4] = az>>0;
    prof_shared_buf[5] = az>>8;
    prof_shared_buf[6] = gx>>0;
    prof_shared_buf[7] = gx>>8;
    prof_shared_buf[8] = gy>>0;
    prof_shared_buf[9] = gy>>8;
    prof_shared_buf[10] = gz>>0;
    prof_shared_buf[11] = gz>>8;
    prof_shared_buf[12] = mx>>0;
    prof_shared_buf[13] = mx>>8;
    prof_shared_buf[14] = my>>0;
    prof_shared_buf[15] = my>>8;
    prof_shared_buf[16] = mz>>0;
    prof_shared_buf[17] = mz>>8;

    // --- Affichage des 2 octets pour chaque entier ---
    printf("ax = %.2x %.2x\n", prof_shared_buf[0], prof_shared_buf[1]);
    printf("ay = %.2x %.2x\n", prof_shared_buf[2], prof_shared_buf[3]);
    printf("az = %.2x %.2x\n", prof_shared_buf[4], prof_shared_buf[5]);
    printf("gx = %.2x %.2x\n", prof_shared_buf[6], prof_shared_buf[7]);
    printf("gy = %.2x %.2x\n", prof_shared_buf[8], prof_shared_buf[9]);
    printf("gz = %.2x %.2x\n", prof_shared_buf[10], prof_shared_buf[11]);
    printf("mx = %.2x %.2x\n", prof_shared_buf[12], prof_shared_buf[13]);
    printf("my = %.2x %.2x\n", prof_shared_buf[14], prof_shared_buf[15]);
    printf("mz = %.2x %.2x\n\n", prof_shared_buf[16], prof_shared_buf[17]);

    
    // --- Affichage des nombres binaires pour chaque mesures ---
    bin(ax);
    bin(ay);
    bin(az);
    bin(gx);
    bin(gy);
    bin(gz);
    bin(mx);
    bin(my);
    bin(mz);
}

void app_main(void)
{
    int taille = 10;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    startMagnetometer();

    // Création des queues
    createQueues(taille);

    /* Acquisitions */
    // bool state = true;
    // while(state)
    
    for (int i=0; i<taille; i++)
    {
        printf("i = %d\n",i);
        accelerometer(); //return vector_t va.x, va.y, va.z
        gyroscope(); //return vector_t vg.x, vg.y, vg.z
        magnetometer(); //return vector_t vm.x, vm.y, vm

        printf("  ACCELEROMETRE [g]: %.3f, %.3f, %.3f\n", va.x, va.y, va.z);
        printf("  GYROMETRE [°/s]: %.3f, %.3f, %.3f\n", vg.x, vg.y, vg.z);        
        printf("  MAGNETOMETRE [µT]: %.3f, %.3f, %.3f\n", vm.x, vm.y, vm.z);

        storeValues(va, vg, vm); // "Stockage" des données dans les queues

        printf("------------------------------------------\n");

        vTaskDelay(100/portTICK_PERIOD_MS); // Acquisition toutes les 100 ms
    }
    
    // Lancement du BLE
    ble();
    vTaskDelay(200);
    ESP_LOGI(TAG, "--- Lancer la réception sur la Raspberry Pi 3 ---");
    vTaskDelay(2000);

    for (int i=0; i<taille; i++)
    {
        sendValues(queue_ax, queue_ay, queue_az, queue_gx, queue_gy, queue_gz, queue_mx, queue_my, queue_mz);
        vTaskDelay(30);
    }
    ESP_LOGI(TAG, "--- Fin du programme ---");
}


float gyroscope_scale()
{
    float gyro_LSB;
    mpu9250_register_read(MPU9250_GYRO_CONFIG, data_read, 1);
    int FS_SEL=data_read[0];
    if (FS_SEL==0)
    {
        return gyro_LSB=131;
    }
    else if (FS_SEL==8)
    {
        return gyro_LSB=65.5;
    }
    else if (FS_SEL==16)
    {
        return gyro_LSB=32.8;
    }
    else if (FS_SEL==24)
    {
        return gyro_LSB=16.4;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

float accelerometer_scale()
{
    float accel_LSB;
    mpu9250_register_read(MPU9250_ACCEL_CONFIG, data_read, 1);
    int AFS_SEL=data_read[0];
    if (AFS_SEL==0)
    {    
        return accel_LSB=16384;
    }
    else if (AFS_SEL==8)
    {
        return accel_LSB=8192;
    }
    else if (AFS_SEL==16)
    {
        return accel_LSB=4096;
    }
    else if (AFS_SEL==24)
    {
        return accel_LSB=2048;
    }
    else
    {
        return EXIT_FAILURE;
    }
}
