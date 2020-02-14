/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_console.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>

#include <esp_audio_mem.h>

#include "app_defs.h"

#ifdef CTN_REV01_UART_COMM
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (1024)

static int sendData(const char* data)
{
	const int len = strlen(data);
	const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
	return txBytes;
}
#endif

static TaskHandle_t cli_task;
static int stop;
static const char *TAG = "[scli]";

static void scli_task(void *arg)
{
    int uart_num = (int) arg;
    uint8_t linebuf[256];
    int i, cmd_ret;
    esp_err_t ret;
    QueueHandle_t uart_queue;
    uart_event_t event;
#ifdef CTN_REV01_UART_COMM
	size_t buffered_size;
	uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
	char* one = "1";
#endif

    ESP_LOGI(TAG, "Initialising UART on port %d", uart_num);
    uart_driver_install(uart_num, 256, 0, 8, &uart_queue, 0);

#ifdef CTN_REV01_UART_COMM
	//Set uart pattern detect function.
	uart_enable_pattern_det_intr(uart_num, '+', PATTERN_CHR_NUM, 10000, 10, 10);
	//Reset the pattern queue length to record at most 20 pattern positions.
	uart_pattern_queue_reset(uart_num, 20);
#endif

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
    };

    esp_console_init(&console_config);
    esp_console_register_help_command();

    while (!stop) {
        uart_write_bytes(uart_num, "\n>> ", 4);
        memset(linebuf, 0, sizeof(linebuf));
        i = 0;
        do {
            ret = xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY);
#ifdef CTN_REV01_UART_COMM
			bzero(dtmp, BUF_SIZE);
#endif
            if (ret != pdPASS) {
                if(stop == 1) {
                    break;
                } else {
                    continue;
                }
            }
            if (event.type == UART_DATA) {
                while (uart_read_bytes(uart_num, (uint8_t *) &linebuf[i], 1, 0)) {
                    if (linebuf[i] == '\r') {
                        uart_write_bytes(uart_num, "\r\n", 2);
                    } else {
                        uart_write_bytes(uart_num, (char *) &linebuf[i], 1);
                    }
                    i++;
                }
            }
#ifdef CTN_REV01_UART_COMM
			else if (event.type == UART_PATTERN_DET) {
				uart_get_buffered_data_len(uart_num, &buffered_size);
				int pos = uart_pattern_pop_pos(uart_num);
				ESP_LOGE(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
				if (pos == -1) {
					// There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
					// record the position. We should set a larger queue size.
					// As an example, we directly flush the rx buffer here.
					uart_flush_input(uart_num);
				} else {
					uart_read_bytes(uart_num, dtmp, pos, 0);
					ESP_LOGE(TAG, "read data: %s", dtmp);
					if(!strcmp((char*)dtmp, one))
					{
						ESP_LOGE(TAG, "senddata");
						sendData("2---");
					}
					uart_flush_input(uart_num);
				}
			}
#endif

        } while ((i < 255) && linebuf[i-1] != '\r');
        if (stop) {
            break;
        }
        /* Remove the truncating \r\n */
        linebuf[strlen((char *)linebuf) - 1] = '\0';
        ret = esp_console_run((char *) linebuf, &cmd_ret);
        if (ret < 0) {
            printf("%s: Console dispatcher error\n", TAG);
            break;
        }
    }
    ESP_LOGE(TAG, "Stopped CLI");
    vTaskDelete(NULL);
}

int scli_init()
{
    static int cli_started;
    if (cli_started) {
        return 0;
    }
#define SCLI_STACK_SIZE 7000
    StackType_t *task_stack = (StackType_t *)esp_audio_mem_calloc(1, SCLI_STACK_SIZE);
    static StaticTask_t task_buf;
    cli_task = xTaskCreateStatic(scli_task, "scli_cli", SCLI_STACK_SIZE, (void *) 0, 3, task_stack, &task_buf);
    if (cli_task == NULL) {
        ESP_LOGE(TAG, "Couldn't create thead");
    }
    cli_started = 1;
    return 0;
}
