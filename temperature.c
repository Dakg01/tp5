
/*
 * affichage.c
 *
 *  Nov. 25, 2020
 *  Original file : Bosch
 *  Modified by : kamel adi
 */

#include <stdio.h>
#include <math.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL27Z644.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "LiquidCrystal_I2C.h"
#include "bmp280.h"

// Définitions
#define I2C_MASTER_BASEADDR I2C0
volatile uint32_t g_systickCounter;

// Variables globales
struct bmp280_dev bmp;

// Prototypes de fonctions
void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void SysTick_Handler(void);

int main(void)
{
    int8_t rslt;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    double temperature, altitude;

    // Initialisation du matériel
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    if (SysTick_Config(SystemCoreClock / 1000U))

    {
        PRINTF("Erreur : initialisation du SysTick échouée\r\n");
        while (1);
    }

    // Initialisation de l'écran LCD
    begin();
    backlight();
    clear();
    setCursor(0, 0);

    print( "Initial...");

    // Initialisation du capteur BMP280
    bmp.dev_id = BMP280_I2C_ADDR_PRIM; // Adresse I2C
    bmp.intf = BMP280_I2C_INTF;
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;
    bmp.delay_ms = delay_ms;

    rslt = bmp280_init(&bmp);
    if (rslt != BMP280_OK)
    {
        PRINTF("Erreur initialisation BMP280 : %d\r\n", rslt);
        //clear();
        print("BMP280 Error");
        while (1);
    }

    // Configuration du BMP280
    bmp280_get_config(&conf, &bmp);
    conf.os_temp = BMP280_OS_4X;
    conf.os_pres = BMP280_OS_4X;
    conf.odr = BMP280_ODR_1000_MS;
    bmp280_set_config(&conf, &bmp);
    bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

    //clear();

    while (1)
    {
        // Lecture des données
          bmp280_get_uncomp_data(&ucomp_data, &bmp);
          bmp280_get_comp_temp_double(&temperature, ucomp_data.uncomp_temp, &bmp);
          altitude = 44330.0 * (1.0 - pow((ucomp_data.uncomp_press / 101325.0), 0.1903));
          // Formatage des données
              char tempBuffer[16];
              char altBuffer[16];
              // Affichage sur lcd
              snprintf(tempBuffer, sizeof(tempBuffer), "Temp: %.1f C", temperature);
              snprintf(altBuffer, sizeof(altBuffer), "Alt: %.1f m", altitude);

              // Mise à jour de l'affichage
                  setCursor(0, 0);  // Ligne 0
                  print(tempBuffer);
                  setCursor(0, 1);  // Ligne 1
                  print(altBuffer);
        // Affichage sur le terminal série
        PRINTF("Température : %.2f °C\r\n", temperature);
        PRINTF("Altitude : %.2f m\r\n", altitude);

        delay_ms(1000);
    }

    return 0;
}

// Implémentations des fonctions auxiliaires
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void delay_ms(uint32_t period_ms);


int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    i2c_master_transfer_t masterXfer;
    status_t status;
    uint8_t txBuff[length + 1];

    txBuff[0] = reg_addr;
    for (uint16_t i = 0; i < length; i++)
    {
        txBuff[i + 1] = reg_data[i];
    }

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = i2c_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.data = txBuff;
    masterXfer.dataSize = length + 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);
    return (status == kStatus_Success) ? BMP280_OK : BMP280_E_COMM_FAIL;
}

int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    i2c_master_transfer_t masterXfer;
    status_t status;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = i2c_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = reg_data;
    masterXfer.dataSize = length;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);
    return (status == kStatus_Success) ? BMP280_OK : BMP280_E_COMM_FAIL;
}
