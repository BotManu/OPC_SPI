#ifndef _AQM_SPI_CONTROLLER_H
#define _AQM_SPI_CONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define AQM_SCK_PIN 18
#define AQM_MISO_PIN 19
#define AQM_MOSI_PIN 23

#define TAG_SPI "SPI_OPC"

esp_err_t ret;
spi_device_handle_t spi;
spi_bus_config_t buscfg;


// OPC*************************

#define OPC_CS_PIN 5
#define OPC_HOST HSPI_HOST

spi_device_interface_config_t devcfg;
void aqm_spi_init();
extern int opc_power_ON();
void cs_high();
void cs_low();
void read_opc(uint8_t command, uint8_t *result);
void read_opc_bytes(uint8_t n_bytes, uint8_t *my_tx_buffer, uint8_t *my_rx_buffer);
int send_OPC_cmd_timeout(uint8_t cmd_byte, uint16_t cmd_timeout_ms, uint8_t expected_answer);
void timer_callback(void *args);
void read_opc_n_bytes(uint8_t n_bytes, uint8_t *my_tx_buffer, uint8_t *my_rx_buffer);

//*****************************

#endif