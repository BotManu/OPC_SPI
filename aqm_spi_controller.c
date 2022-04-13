#include "aqm_spi_controller.h"

spi_bus_config_t buscfg = {
    .miso_io_num = AQM_MISO_PIN,
    .mosi_io_num = AQM_MOSI_PIN,
    .sclk_io_num = AQM_SCK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 32};

spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .clock_speed_hz = 600 * 1000, // Clock out at 750 kHz
    .mode = 1,                    // SPI mode 1
    .spics_io_num = OPC_CS_PIN,   // CS pin
    .queue_size = 1,              // We want to be able to queue 1 transactions at a time
    .flags = SPI_DEVICE_HALFDUPLEX};

void aqm_spi_init()
{
    // initialize SPI bus
    ret = spi_bus_initialize(OPC_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    // Attach OPC to the SPI bus
    ret = spi_bus_add_device(OPC_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // initialize OPC N3
    gpio_pad_select_gpio(OPC_CS_PIN);
    gpio_set_direction(OPC_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OPC_CS_PIN, 1);

    vTaskDelay(20 / portTICK_PERIOD_MS);
}

// return 0 on success
// return <>0 on fail
extern int opc_power_ON()
{

    /****************CODE WORKS FOR DEMONSTRATION PURPOSES
    uint8_t counter = 10;
    uint8_t received_data = 0;

    cs_low();

    while ((received_data != 0xF3))
    {
        counter--;
        read_opc(0x03, &received_data);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    cs_high();

    if (received_data != 0x03)
    {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    counter = 10;

    cs_low();

    while ((received_data != 0xF3))
    {
        counter--;
        read_opc(0x3F, &received_data);
        vTaskDelay(15 / portTICK_PERIOD_MS);
    }

    uint8_t opc_tx_buffer[60];
    memset(opc_tx_buffer, 0x3F, 60);
    uint8_t opc_rx_buffer[60];

    read_opc_bytes(60, opc_tx_buffer, opc_rx_buffer);

    cs_high();

    for (int i = 0; i < 60; i++)
    {
        ESP_LOGE(TAG_SPI, "Rx buf: %x", opc_rx_buffer[i]);
    }

    ESP_LOGW(TAG_SPI, "Received byte from OPC: %x", received_data);

    return 1;

    ************************************/

    cs_low();
    // first write 0x03
    if (!send_OPC_cmd_timeout(0x03, 1000, 0xF3))
    {
        ESP_LOGI(TAG_SPI, "Begin power ON");

         //turn on fan
        if(!send_OPC_cmd_timeout(0x03,3000,0xF3)){
            ESP_LOGI(TAG_SPI, "Fan command turn on");
        }
        else{
            ESP_LOGE(TAG_SPI, "Failed fan command ON");
        }

    }
    else{
        ESP_LOGE(TAG_SPI, "OPC power error");
    }

    cs_high();

    vTaskDelay(3000/portTICK_PERIOD_MS);

    cs_low();

    if (!send_OPC_cmd_timeout(0x03, 1000, 0xF3))
    {
        ESP_LOGI(TAG_SPI, "Begin power ON");

         //turn on fan
        if(!send_OPC_cmd_timeout(0x07,3000,0xF3)){
            ESP_LOGI(TAG_SPI, "Laser command turn on");
        }
        else{
            ESP_LOGE(TAG_SPI, "Failed laser command ON");
        }

    }
    else{
        ESP_LOGE(TAG_SPI, "OPC power error");
    }

    //wait a little so that laser can be turned on
    vTaskDelay(100/portTICK_PERIOD_MS);

    if (!send_OPC_cmd_timeout(0x30, 1000, 0xF3)){
         ESP_LOGI(TAG_SPI, "Begin reading hystogram");
        uint8_t opch_tx_buffer[86];
        memset(opch_tx_buffer, 0x30, 86);
        uint8_t opch_rx_buffer[86];
        read_opc_n_bytes(86, opch_tx_buffer, opch_rx_buffer);

        uint8_t PM_A_vect[4];
        PM_A_vect[0]=opch_rx_buffer[60];
        PM_A_vect[1]=opch_rx_buffer[61];
        PM_A_vect[2]=opch_rx_buffer[62];
        PM_A_vect[3]=opch_rx_buffer[63];

        float *PM_A_value=(float *)&PM_A_vect[0];
        ESP_LOGW(TAG_SPI, "PMA read value is: %f", *PM_A_value);

        for (int i = 0; i < 86; i++)
    {
        ESP_LOGI(TAG_SPI, "Rx buf: %x", opch_rx_buffer[i]);
    }



    }

    cs_high();

    vTaskDelay(3000/portTICK_PERIOD_MS);

    cs_low();

    if(!send_OPC_cmd_timeout(0x13, 1000, 0xF3))
    {   
        ESP_LOGW(TAG_SPI, "Power state check");

        uint8_t opc_tx_buffer[6];
        memset(opc_tx_buffer, 0x13, 6);
        uint8_t opc_rx_buffer[6];
        read_opc_n_bytes(6, opc_tx_buffer, opc_rx_buffer);

        ESP_LOGI(TAG_SPI, "Fan_ON: %x", opc_rx_buffer[0]);
        ESP_LOGI(TAG_SPI, "LaserDAC_ON: %x", opc_rx_buffer[1]);
        ESP_LOGI(TAG_SPI, "FanDACval: %x", opc_rx_buffer[2]);
        ESP_LOGI(TAG_SPI, "LaserDACval: %x", opc_rx_buffer[3]);
        ESP_LOGI(TAG_SPI, "LaserSwitch: %x", opc_rx_buffer[4]);
        ESP_LOGI(TAG_SPI, "GainToggle: %x", opc_rx_buffer[5]);

    }
    else{
        ESP_LOGE(TAG_SPI, "OPC Check Failed");
    }


    cs_high();

   
    return 1;
}

// return <>0 on fail
// return 0 on success
extern int opc_power_OFF()
{
    return 0;
}

void cs_high()
{
    gpio_pad_select_gpio(OPC_CS_PIN);
    gpio_set_level(OPC_CS_PIN, 1);
}

void cs_low()
{
    gpio_pad_select_gpio(OPC_CS_PIN);
    gpio_set_level(OPC_CS_PIN, 0);
}

void read_opc(uint8_t command, uint8_t *result)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA; //|SPI_TRANS_CS_KEEP_ACTIVE;
    t.length = 16;
    t.rxlength = 8;
    t.tx_data[0] = command;

    spi_device_transmit(spi, &t);

    *result = t.rx_data[0];
    //ESP_LOGI(TAG_SPI, "OPC response: %c", t.rx_data[0]);
}

void read_opc_bytes(uint8_t n_bytes, uint8_t *my_tx_buffer, uint8_t *my_rx_buffer)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));
    t.length = 2 * n_bytes;
    t.rxlength = n_bytes;
    t.tx_buffer = my_tx_buffer;
    t.rx_buffer = my_rx_buffer;
    spi_device_polling_transmit(spi, &t);
}

// returns 0 on success
// returns 1 if command timeout
int send_OPC_cmd_timeout(uint8_t cmd_byte, uint16_t cmd_timeout_ms, uint8_t expected_answer)
{
    uint8_t real_answer = 0;

    while ((real_answer != expected_answer) && (cmd_timeout_ms > 12))
    {
        read_opc(cmd_byte, &real_answer);
        vTaskDelay(12 / portTICK_PERIOD_MS);
        cmd_timeout_ms = cmd_timeout_ms - 12;
    }

    if (real_answer != expected_answer)
    {
        return 1; // command timeout
    }
    else
    {
        return 0; // expected_answer received
    }

    return 1; // default command timeout
}

void read_opc_n_bytes(uint8_t n_bytes, uint8_t *my_tx_buffer, uint8_t *my_rx_buffer)
{
    uint8_t repetitions = n_bytes;
    uint8_t index = 0;
    uint8_t *my_params[] = {&repetitions, &index, my_tx_buffer, my_rx_buffer};

    const esp_timer_create_args_t esp_timer_create_args = {
        .callback = timer_callback,
        //.arg=(void *){repetitions,&index,&my_tx_buffer,&my_rx_buffer},
        .arg = (void **)my_params,
        .name = "SPI timer"};
    esp_timer_handle_t esp_timer_handle;
    esp_timer_create(&esp_timer_create_args, &esp_timer_handle);

            esp_timer_start_periodic(esp_timer_handle, 25);


    int max_reps=2*n_bytes*50/1000;
    int reps = 0;
    while (true)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        if (reps++ > max_reps)
        {
            esp_timer_stop(esp_timer_handle);
            esp_timer_delete(esp_timer_handle);
            break;
        }
    }
}


void timer_callback(void *args)
{
    uint8_t max_repetitions = *(((uint8_t **)args)[0]);
    uint8_t current_index = *(((uint8_t **)args)[1]);

   // ESP_LOGE(TAG_SPI, "Current repetitions and index are: %d, %d", max_repetitions, current_index);

    if (current_index < max_repetitions)
    {
        uint8_t current_index1 = *(((uint8_t **)args)[1])++;

        uint8_t current_cmd_byte = (((uint8_t **)args)[2])[current_index];

       // ESP_LOGE(TAG_SPI, "Current cmd_byte is: %d", current_cmd_byte);
        // send SPI command
        read_opc(current_cmd_byte, &(((uint8_t **)args)[3])[current_index]);

        // increase index
        //*(((uint8_t**)args)[1])++;
    }
}