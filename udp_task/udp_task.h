#ifndef UDP_TASK_H
#define UDP_TASK_H

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "mongoose.h"
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

// Sensor data packet structure
typedef struct {
    char timestamp[32];     // UTC timestamp format: HH-MM-SS-DD-MM-YYYY
    char sensor_name[16];   // Sensor name (zmod4410, icp10101, etc)
    char value_str[64];     // Formatted value string with units
    uint32_t sequence;      // Sequence number for ordering
} sensor_packet_t;

// FIFO buffer for storing sensor packets when connection is lost
typedef struct {
    sensor_packet_t buffer[SENSOR_BUFFER_SIZE];
    int head;
    int tail;
    int count;
    SemaphoreHandle_t mutex;
} packet_fifo_t;

// UDP connection state
typedef struct {
    struct mg_mgr mgr;
    struct mg_connection *connection;
    volatile bool connected;
    volatile bool reconnecting;
    uint32_t last_send_time;
    uint32_t packet_sequence;
} udp_state_t;

// Task control structure
typedef struct {
    bool is_paused;
    SemaphoreHandle_t pause_mutex;
    SemaphoreHandle_t resume_sem;
} task_control_t;

// Public function prototypes
void udp_task_init(void);
void udp_task_run(void *pvParameters);
void udp_task_pause(bool pause);
void udp_task_stop(void);

// FIFO operations
bool fifo_push(packet_fifo_t *fifo, const sensor_packet_t *packet);
bool fifo_pop(packet_fifo_t *fifo, sensor_packet_t *packet);
bool fifo_is_empty(packet_fifo_t *fifo);
bool fifo_is_full(packet_fifo_t *fifo);
void fifo_clear(packet_fifo_t *fifo);

// Utility functions
void get_utc_timestamp(char *buffer, size_t buffer_size);
bool read_sensor_data(int sensor_id, sensor_packet_t *packet);
void format_sensor_message(const sensor_packet_t *packet, char *buffer, size_t buffer_size);

// Debug printf wrapper
#ifdef DEBUG
#define debug_printf(...) printf("[UDP] " __VA_ARGS__)
#else
#define debug_printf(...)
#endif

// Task priorities and stack sizes
#define UDP_TASK_PRIORITY          (tskIDLE_PRIORITY + 2)
#define UDP_TASK_STACK_SIZE        4096

#endif // UDP_TASK_H
