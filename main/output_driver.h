#pragma once 

#include "rom/uart.h"
#include "driver/i2c.h"
#include "rom/uart.h"
#include "smbus.h"
#include "i2c-lcd1602.h"

// LCD2004
#define LCD_NUM_ROWS 4
#define LCD_NUM_COLUMNS 40
#define LCD_NUM_VISIBLE_COLUMNS 20

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0 // disabled
#define I2C_MASTER_RX_BUF_LEN 0 // disabled
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_SCL_IO 19
#define CONFIG_LCD1602_I2C_ADDRESS 0x27

void gpio_init(void);
int app_driver_set_state(bool state, unsigned short relay_no);
bool app_driver_get_state(unsigned short relay_pin);
void wifi_status(int status);
void lcd2004(void);
