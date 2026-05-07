#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "esp_log.h"

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
                        } else if (strcmp(cmd_buf, "help") == 0) {
                            uart_write_bytes(UART_NUM, "\r\nCommands:\r\n", 13);
                            uart_write_bytes(UART_NUM, "  blink on       - Enable blinking\r\n", 38);
                            uart_write_bytes(UART_NUM, "  blink off      - Disable blinking\r\n", 39);
                            uart_write_bytes(UART_NUM, "  blink freq <ms>- Set blink period in ms\r\n", 45);
                            uart_write_bytes(UART_NUM, "  i2s init <ch> <sr> <w> <s/m> - Init I2S\r\n", 51);
                            uart_write_bytes(UART_NUM, "  i2s tone <freq>- Play tone 100-15000Hz\r\n", 48);
                            uart_write_bytes(UART_NUM, "  i2s start      - Start I2S\r\n", 37);
                            uart_write_bytes(UART_NUM, "  i2s stop       - Stop I2S\r\n", 36);
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
