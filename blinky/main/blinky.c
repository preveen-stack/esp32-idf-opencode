#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp32/rom/ets_sys.h"

#define BLINK_GPIO 2
#define TAG "BLINKY"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define BLINK_TASK_STACK 2048
#define UART_TASK_STACK 2048
#define I2S_TASK_STACK 4096

static bool blink_enabled = true;
static uint32_t blink_freq_ms = 1000;
static TaskHandle_t blink_task_handle = NULL;

static i2s_chan_handle_t i2s_tx_chan = NULL;
static bool i2s_initialized = false;
static bool i2s_running = false;
static uint32_t i2s_sample_rate = 44100;
static uint8_t i2s_bit_width = 16;
static bool i2s_stereo = true;
static TaskHandle_t i2s_tone_task = NULL;

#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
static bool i2c_initialized = false;
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static int i2c_sda_pin = I2C_MASTER_SDA_IO;
static int i2c_scl_pin = I2C_MASTER_SCL_IO;

#define ADS1115_ADDR 0x48
#define ADS1115_CONV_REG 0x00
#define ADS1115_CONFIG_REG 0x01
static uint8_t ads1115_addr = ADS1115_ADDR;

#define ADS1115_PGA_6_144V 0x0000
#define ADS1115_PGA_4_096V 0x0200
#define ADS1115_PGA_2_048V 0x0400
#define ADS1115_PGA_1_024V 0x0600
#define ADS1115_PGA_0_512V 0x0800
#define ADS1115_PGA_0_256V 0x0A00
static uint16_t ads1115_pga = ADS1115_PGA_4_096V;

void blink_task(void *arg)
{
    while (1) {
        if (blink_enabled) {
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(blink_freq_ms / portTICK_PERIOD_MS);
            gpio_set_level(BLINK_GPIO, 0);
            vTaskDelay(blink_freq_ms / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void i2s_tone_task_func(void *arg)
{
    uint32_t tone_freq = (uint32_t)arg;
    if (tone_freq < 100 || tone_freq > 15000) {
        vTaskDelete(NULL);
        return;
    }

    int sample_rate = i2s_sample_rate;
    int bit_depth = i2s_bit_width;
    int num_channels = i2s_stereo ? 2 : 1;
    int samples_per_cycle = sample_rate / tone_freq;
    int buf_size = samples_per_cycle * (bit_depth / 8) * num_channels;
    int16_t *buf = malloc(buf_size);

    while (i2s_running) {
        for (int i = 0; i < samples_per_cycle; i++) {
            int16_t sample = (int16_t)(32767 * sin(2 * M_PI * i / samples_per_cycle));
            if (num_channels == 2) {
                buf[i * 2] = sample;
                buf[i * 2 + 1] = sample;
            } else {
                buf[i] = sample;
            }
        }
        size_t bytes_written;
        i2s_channel_write(i2s_tx_chan, buf, buf_size, &bytes_written, portMAX_DELAY);
    }
    free(buf);
    i2s_tone_task = NULL;
    vTaskDelete(NULL);
}

void i2s_init_chan(int channel, int sample_rate, int bit_width, bool stereo)
{
    if (i2s_initialized && i2s_tx_chan) {
        i2s_channel_disable(i2s_tx_chan);
        i2s_del_channel(i2s_tx_chan);
        i2s_tx_chan = NULL;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0 + channel, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL);

    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width, stereo);
    i2s_std_gpio_config_t gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = GPIO_NUM_26,
        .ws = GPIO_NUM_25,
        .dout = GPIO_NUM_22,
        .din = I2S_GPIO_UNUSED,
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = gpio_cfg,
    };
    i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg);
    i2s_channel_enable(i2s_tx_chan);

    i2s_sample_rate = sample_rate;
    i2s_bit_width = bit_width;
    i2s_stereo = stereo;
    i2s_initialized = true;
}

void i2c_init_master(int sda_pin, int scl_pin)
{
    if (i2c_initialized) {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
    }
    i2c_sda_pin = sda_pin;
    i2c_scl_pin = scl_pin;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    i2c_initialized = true;
}

void i2c_detect(void)
{
    if (!i2c_initialized) {
        i2c_init_master(i2c_sda_pin, i2c_scl_pin);
    }
    uart_write_bytes(UART_NUM, "\r\nI2C Scan:\r\n", 14);
    uart_write_bytes(UART_NUM, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n", 56);
    uart_write_bytes(UART_NUM, "00:         ", 12);
    uint16_t address;
    for (address = 0x08; address <= 0x77; address++) {
        if (address % 16 == 0 && address > 0x0f) {
            char line[16];
            snprintf(line, sizeof(line), "\r\n%02x: ", address);
            uart_write_bytes(UART_NUM, line, strlen(line));
        }
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, address, 100);
        if (ret == ESP_OK) {
            char addr_str[4];
            snprintf(addr_str, sizeof(addr_str), "%02x ", address);
            uart_write_bytes(UART_NUM, addr_str, strlen(addr_str));
        } else {
            uart_write_bytes(UART_NUM, "-- ", 3);
        }
    }
    uart_write_bytes(UART_NUM, "\r\n> ", 4);
}

static i2c_master_dev_handle_t ads1115_dev = NULL;

void ads1115_write_config(uint16_t config)
{
    if (!ads1115_dev) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ads1115_addr,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };
        i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &ads1115_dev);
    }
    uint8_t data[3] = {ADS1115_CONFIG_REG, config >> 8, config & 0xFF};
    i2c_master_transmit(ads1115_dev, data, 3, 100);
}

uint16_t ads1115_read_data(void)
{
    if (!ads1115_dev) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ads1115_addr,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };
        i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &ads1115_dev);
    }
    uint8_t reg = ADS1115_CONV_REG;
    i2c_master_transmit(ads1115_dev, &reg, 1, 100);
    uint8_t data[2];
    i2c_master_receive(ads1115_dev, data, 2, 100);
    return (data[0] << 8) | data[1];
}

void ads1115_init(void)
{
    if (!i2c_initialized) {
        i2c_init_master(i2c_sda_pin, i2c_scl_pin);
    }
    uint16_t config = 0x8483;
    ads1115_write_config(config);
}

void ads1115_read_cmd(void)
{
    if (!i2c_initialized) {
        i2c_init_master(i2c_sda_pin, i2c_scl_pin);
    }
    uint16_t raw = ads1115_read_data();
    int16_t value = (int16_t)raw;
    float voltage = value * 4.096 / 32768.0;
    char buf[64];
    snprintf(buf, sizeof(buf), "\r\nADS1115: raw=%d, %.3fV\r\n> ", raw, voltage);
    uart_write_bytes(UART_NUM, buf, strlen(buf));
}

void ads1115_set_pga(uint16_t pga)
{
    ads1115_pga = pga;
    uint16_t config = 0x8403 | pga;
    ads1115_write_config(config);
}

void ads1115_read_pga(void)
{
    uint8_t reg = ADS1115_CONFIG_REG;
    i2c_master_transmit(ads1115_dev, &reg, 1, 100);
    uint8_t data[2];
    i2c_master_receive(ads1115_dev, data, 2, 100);
    uint16_t config = (data[0] << 8) | data[1];
    char buf[64];
    snprintf(buf, sizeof(buf), "\r\nADS1115 PGA: 0x%04x (current: 0x%04x)\r\n> ", config, ads1115_pga);
    uart_write_bytes(UART_NUM, buf, strlen(buf));
}

void uart_terminal_task(void *arg)
{
    uint8_t *data = malloc(BUF_SIZE);
    char cmd_buf[256];
    int cmd_idx = 0;

    uart_write_bytes(UART_NUM, "\r\n=== Blinky UART Terminal ===\r\n", 31);
    uart_write_bytes(UART_NUM, "Type 'help' for commands\r\n> ", 29);

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                uart_write_bytes(UART_NUM, &c, 1);

                if (c == '\r' || c == '\n') {
                    cmd_buf[cmd_idx] = '\0';

                    if (cmd_idx > 0) {
                        if (strncmp(cmd_buf, "blink on", 8) == 0) {
                            blink_enabled = true;
                            uart_write_bytes(UART_NUM, "\r\nBlinking enabled\r\n> ", 26);
                        } else if (strncmp(cmd_buf, "blink off", 9) == 0) {
                            blink_enabled = false;
                            gpio_set_level(BLINK_GPIO, 0);
                            uart_write_bytes(UART_NUM, "\r\nBlinking disabled\r\n> ", 27);
                        } else if (strncmp(cmd_buf, "blink freq ", 11) == 0) {
                            uint32_t freq = atoi(cmd_buf + 11);
                            if (freq > 0) {
                                blink_freq_ms = freq;
                                uart_write_bytes(UART_NUM, "\r\nBlink frequency set\r\n> ", 29);
                            } else {
                                uart_write_bytes(UART_NUM, "\r\nInvalid frequency\r\n> ", 25);
                            }
                        } else if (strncmp(cmd_buf, "i2s init ", 9) == 0) {
                            int chan, sr, width;
                            char mono_stereo[16];
                            if (sscanf(cmd_buf + 9, "%d %d %d %s", &chan, &sr, &width, mono_stereo) == 4) {
                                bool stereo = (strcmp(mono_stereo, "stereo") == 0);
                                i2s_init_chan(chan, sr, width, stereo);
                                uart_write_bytes(UART_NUM, "\r\nI2S initialized\r\n> ", 24);
                            } else {
                                uart_write_bytes(UART_NUM, "\r\nUsage: i2s init <ch> <sr> <width> <stereo/mono>\r\n> ", 62);
                            }
                        } else if (strncmp(cmd_buf, "i2s tone ", 9) == 0) {
                            uint32_t freq = atoi(cmd_buf + 9);
                            if (freq >= 100 && freq <= 15000 && i2s_initialized) {
                                if (i2s_running) {
                                    i2s_running = false;
                                    vTaskDelay(100 / portTICK_PERIOD_MS);
                                }
                                i2s_running = true;
                                xTaskCreate(i2s_tone_task_func, "i2s_tone", I2S_TASK_STACK, (void*)freq, 5, &i2s_tone_task);
                                uart_write_bytes(UART_NUM, "\r\nTone playing\r\n> ", 21);
                            } else {
                                uart_write_bytes(UART_NUM, "\r\nInvalid freq or I2S not init\r\n> ", 38);
                            }
                        } else if (strcmp(cmd_buf, "i2s start") == 0) {
                            if (i2s_initialized && !i2s_running) {
                                i2s_running = true;
                                uart_write_bytes(UART_NUM, "\r\nI2S started\r\n> ", 21);
                            } else {
                                uart_write_bytes(UART_NUM, "\r\nI2S not ready\r\n> ", 23);
                            }
                        } else if (strcmp(cmd_buf, "i2s stop") == 0) {
                            i2s_running = false;
                            uart_write_bytes(UART_NUM, "\r\nI2S stopped\r\n> ", 21);
                        } else if (strncmp(cmd_buf, "i2c init ", 9) == 0) {
                            int sda, scl;
                            if (sscanf(cmd_buf + 9, "%d %d", &sda, &scl) == 2) {
                                if (i2c_initialized) {
                                    i2c_del_master_bus(i2c_bus_handle);
                                    i2c_bus_handle = NULL;
                                    i2c_initialized = false;
                                }
                                i2c_init_master(sda, scl);
                                uart_write_bytes(UART_NUM, "\r\nI2C initialized\r\n> ", 26);
                            } else {
                                uart_write_bytes(UART_NUM, "\r\nUsage: i2c init <sda> <scl>\r\n> ", 44);
                            }
                        } else if (strcmp(cmd_buf, "i2c detect") == 0) {
                            i2c_detect();
                        } else if (strncmp(cmd_buf, "ads1115 init", 11) == 0) {
                            ads1115_init();
                            uart_write_bytes(UART_NUM, "\r\nADS1115 initialized\r\n> ", 30);
                        } else if (strcmp(cmd_buf, "ads1115 read") == 0) {
                            ads1115_read_cmd();
                        } else if (strncmp(cmd_buf, "ads1115 setpga ", 15) == 0) {
                            uint16_t pga = (uint16_t)strtol(cmd_buf + 15, NULL, 16);
                            ads1115_set_pga(pga);
                            uart_write_bytes(UART_NUM, "\r\nPGA set\r\n> ", 22);
                        } else if (strcmp(cmd_buf, "ads1115 getpga") == 0) {
                            ads1115_read_pga();
                        } else if (strcmp(cmd_buf, "clock") == 0) {
                            uint32_t cpu_freq = ets_get_cpu_frequency() * 1000000;
                            uart_write_bytes(UART_NUM, "\r\nSystem Clocks:\r\n", 20);
                            uart_write_bytes(UART_NUM, "  CPU: ", 7);
                            char freq_str[32];
                            snprintf(freq_str, sizeof(freq_str), "%lu Hz\r\n", cpu_freq);
                            uart_write_bytes(UART_NUM, freq_str, strlen(freq_str));
                            uart_write_bytes(UART_NUM, "> ", 3);
                        } else if (strcmp(cmd_buf, "reset") == 0) {
                            uart_write_bytes(UART_NUM, "\r\nResetting ESP32...\r\n", 27);
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                            esp_restart();
                        } else if (strcmp(cmd_buf, "help") == 0) {
                            uart_write_bytes(UART_NUM, "\r\nCommands:\r\n", 13);
                            uart_write_bytes(UART_NUM, "  blink on       - Enable blinking\r\n", 38);
                            uart_write_bytes(UART_NUM, "  blink off      - Disable blinking\r\n", 39);
                            uart_write_bytes(UART_NUM, "  blink freq <ms>- Set blink period in ms\r\n", 45);
                            uart_write_bytes(UART_NUM, "  i2s init <ch> <sr> <w> <s/m> - Init I2S\r\n", 51);
                            uart_write_bytes(UART_NUM, "  i2s tone <freq>- Play tone 100-15000Hz\r\n", 48);
                            uart_write_bytes(UART_NUM, "  i2s start      - Start I2S\r\n", 37);
                            uart_write_bytes(UART_NUM, "  i2s stop       - Stop I2S\r\n", 36);
                             uart_write_bytes(UART_NUM, "  i2c init <sda> <scl> - Init I2C pins\r\n", 45);
                             uart_write_bytes(UART_NUM, "  i2c detect     - Scan I2C bus\r\n", 40);
                             uart_write_bytes(UART_NUM, "  ads1115 init    - Init ADS1115\r\n", 42);
                             uart_write_bytes(UART_NUM, "  ads1115 read    - Read ADS1115\r\n", 40);
                             uart_write_bytes(UART_NUM, "  ads1115 setpga <hex> - Set PGA config\r\n", 45);
                             uart_write_bytes(UART_NUM, "  ads1115 getpga - Read PGA config\r\n", 42);
                             uart_write_bytes(UART_NUM, "  clock          - Show system clocks\r\n", 42);
                            uart_write_bytes(UART_NUM, "  reset          - Restart ESP32\r\n", 40);
                            uart_write_bytes(UART_NUM, "  help           - Show this help\r\n> ", 39);
                        } else {
                            uart_write_bytes(UART_NUM, "\r\nUnknown command. Type 'help'\r\n> ", 36);
                        }
                    } else {
                        uart_write_bytes(UART_NUM, "\r\n> ", 4);
                    }
                    cmd_idx = 0;
                } else if (c == '\b' || c == 127) {
                    if (cmd_idx > 0) {
                        cmd_idx--;
                        uart_write_bytes(UART_NUM, "\b \b", 3);
                    }
                } else if (cmd_idx < 255) {
                    cmd_buf[cmd_idx++] = c;
                }
            }
        }
    }
    free(data);
}

void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(blink_task, "blink_task", BLINK_TASK_STACK, NULL, 5, &blink_task_handle);
    xTaskCreate(uart_terminal_task, "uart_terminal", UART_TASK_STACK, NULL, 5, NULL);
}
