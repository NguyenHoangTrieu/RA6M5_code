#include "udp_task.h"
#include "hal_data.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Static variables
static udp_state_t udp_state = {0};
static packet_fifo_t sensor_fifo = {0};
static task_control_t task_control = {0};
static TaskHandle_t udp_task_handle = NULL;

// Mock sensor data for demonstration (replace with actual sensor drivers)
static float mock_zmod4410_tvoc = 220.0;    // ppb
static uint16_t mock_zmod4410_iaq = 95;      // IAQ index
static float mock_icp10101_pressure = 101650.0; // Pa
static float mock_icp10101_temp = 23.5;     // °C
static float mock_icp10101_altitude = 120.0; // m

// Forward declarations
static void udp_event_handler(struct mg_connection *c, int ev, void *ev_data);
static bool udp_connect(void);
static void udp_disconnect(void);
static bool send_buffered_packets(void);
static void process_sensor_readings(void);
static void mg_random_impl(void *buf, size_t len);

// Custom random function required by Mongoose
void mg_random(void *buf, size_t len) {
    mg_random_impl(buf, len);
}

static void mg_random_impl(void *buf, size_t len) {
    uint8_t *p = (uint8_t*)buf;
    for (size_t i = 0; i < len; i++) {
        p[i] = (uint8_t)(xTaskGetTickCount() + i);
    }
}

// Initialize UDP task and resources
void udp_task_init(void) {
    // Initialize FIFO mutex
    sensor_fifo.mutex = xSemaphoreCreateMutex();
    if (sensor_fifo.mutex == NULL) {
#ifdef DEBUG
        debug_printf("Failed to create FIFO mutex\n");
#endif
        return;
    }

    // Initialize task control
    task_control.pause_mutex = xSemaphoreCreateMutex();
    task_control.resume_sem = xSemaphoreCreateBinary();
    task_control.is_paused = false;

    if (task_control.pause_mutex == NULL || task_control.resume_sem == NULL) {
#ifdef DEBUG
        debug_printf("Failed to create task control semaphores\n");
#endif
        return;
    }

    // Initialize Mongoose manager
    mg_mgr_init(&udp_state.mgr);
    udp_state.connected = false;
    udp_state.reconnecting = false;
    udp_state.last_send_time = 0;
    udp_state.packet_sequence = 0;

    // Create UDP task
    BaseType_t result = xTaskCreate(
        udp_task_run,
        "UDP_Task",
        UDP_TASK_STACK_SIZE,
        NULL,
        UDP_TASK_PRIORITY,
        &udp_task_handle
    );

    if (result != pdPASS) {
#ifdef DEBUG
        debug_printf("Failed to create UDP task\n");
#endif
    } else {
#ifdef DEBUG
        debug_printf("UDP task initialized successfully\n");
#endif
    }
}

// Main UDP task implementation
void udp_task_run(void *pvParameters) {
    (void)pvParameters;
    
    TickType_t last_sensor_read = 0;
    TickType_t current_time;

#ifdef DEBUG
    debug_printf("UDP task started\n");
#endif

    // Initial connection attempt
    if (!udp_connect()) {
#ifdef DEBUG
        debug_printf("Initial connection failed\n");
#endif
    }

    while (1) {
        // Handle pause/resume
        if (xSemaphoreTake(task_control.pause_mutex, 0) == pdTRUE) {
            if (task_control.is_paused) {
                xSemaphoreGive(task_control.pause_mutex);
#ifdef DEBUG
                debug_printf("Task paused\n");
#endif
                xSemaphoreTake(task_control.resume_sem, portMAX_DELAY);
                continue;
            }
            xSemaphoreGive(task_control.pause_mutex);
        }

        current_time = xTaskGetTickCount();

        // Poll Mongoose events
        mg_mgr_poll(&udp_state.mgr, 50);

        // Try to reconnect if disconnected
        if (!udp_state.connected && !udp_state.reconnecting) {
            if (current_time - udp_state.last_send_time > pdMS_TO_TICKS(RECONNECT_DELAY_MS)) {
#ifdef DEBUG
                debug_printf("Attempting reconnection...\n");
#endif
                udp_connect();
            }
        }

        // Read sensors periodically
        if (current_time - last_sensor_read >= pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS)) {
            process_sensor_readings();
            last_sensor_read = current_time;
        }

        // Send buffered packets if connected
        if (udp_state.connected) {
            if (current_time - udp_state.last_send_time >= pdMS_TO_TICKS(UDP_SEND_INTERVAL_MS)) {
                send_buffered_packets();
                udp_state.last_send_time = current_time;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_LOOP_DELAY_MS));
    }
}

// Connect to UDP server
static bool udp_connect(void) {
    if (udp_state.connection != NULL) {
        mg_mgr_poll(&udp_state.mgr, 0);
        udp_state.connection = NULL;
    }

    udp_state.reconnecting = true;
    udp_state.connection = mg_connect(&udp_state.mgr, UDP_BIND_URL, udp_event_handler, NULL);

#ifdef DEBUG
    debug_printf("Connecting to %s\n", UDP_BIND_URL);
#endif

    return udp_state.connection != NULL;
}

// Disconnect from UDP server
static void udp_disconnect(void) {
    udp_state.connected = false;
    if (udp_state.connection != NULL) {
        mg_mgr_poll(&udp_state.mgr, 0);
        udp_state.connection = NULL;
    }
#ifdef DEBUG
    debug_printf("Disconnected from server\n");
#endif
}

// UDP event handler
static void udp_event_handler(struct mg_connection *c, int ev, void *ev_data) {
    switch (ev) {
        case MG_EV_CONNECT:
            udp_state.connected = true;
            udp_state.reconnecting = false;
            udp_state.connection = c;
#ifdef DEBUG
            debug_printf("Connected to server\n");
#endif
            break;

        case MG_EV_READ:
            if (c->recv.len > 0) {
                // Process server responses (optional)
#ifdef DEBUG
                debug_printf("Received %d bytes from server\n", (int)c->recv.len);
#endif
                mg_iobuf_del(&c->recv, 0, c->recv.len);
            }
            break;

        case MG_EV_CLOSE:
        case MG_EV_ERROR:
            udp_state.connected = false;
            udp_state.reconnecting = false;
            udp_state.connection = NULL;
#ifdef DEBUG
            debug_printf("Connection %s\n", ev == MG_EV_CLOSE ? "closed" : "error");
#endif
            break;
    }
}

// Read sensor data (mock implementation - replace with actual sensor drivers)
bool read_sensor_data(int sensor_id, sensor_packet_t *packet) {
    if (!packet) return false;

    // Get current timestamp
    get_utc_timestamp(packet->timestamp, sizeof(packet->timestamp));
    packet->sequence = udp_state.packet_sequence++;

    switch (sensor_id) {
        case SENSOR_ZMOD4410:
            strcpy(packet->sensor_name, "zmod4410");
            // Simulate sensor readings with slight variations
            mock_zmod4410_tvoc += (float)(rand() % 20 - 10);
            mock_zmod4410_iaq += (rand() % 10 - 5);
            snprintf(packet->value_str, sizeof(packet->value_str), 
                    "tvoc=%.0fppb iaq=%d", mock_zmod4410_tvoc, mock_zmod4410_iaq);
            break;

        case SENSOR_ICP10101:
            strcpy(packet->sensor_name, "icp10101");
            // Simulate sensor readings with slight variations  
            mock_icp10101_pressure += (float)(rand() % 100 - 50);
            mock_icp10101_temp += (float)(rand() % 4 - 2) * 0.1f;
            mock_icp10101_altitude += (float)(rand() % 10 - 5) * 0.1f;
            snprintf(packet->value_str, sizeof(packet->value_str),
                    "pressure=%.0fPa temp=%.1fC alt=%.1fm", 
                    mock_icp10101_pressure, mock_icp10101_temp, mock_icp10101_altitude);
            break;

        default:
            return false;
    }

    return true;
}

// Process sensor readings and buffer them
static void process_sensor_readings(void) {
    sensor_packet_t packet;

    // Read data from selected sensors
    int sensors[] = {SELECTED_SENSOR_1, SELECTED_SENSOR_2};
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (read_sensor_data(sensors[i], &packet)) {
            if (!fifo_push(&sensor_fifo, &packet)) {
#ifdef DEBUG
                debug_printf("FIFO full, dropping packet from sensor %d\n", sensors[i]);
#endif
            } else {
#ifdef DEBUG
                debug_printf("Buffered packet from %s: %s\n", 
                           packet.sensor_name, packet.value_str);
#endif
            }
        }
    }
}

// Send buffered packets to server
static bool send_buffered_packets(void) {
    sensor_packet_t packet;
    char message_buffer[PACKET_MAX_SIZE];
    bool sent_any = false;

    while (!fifo_is_empty(&sensor_fifo) && udp_state.connected) {
        if (fifo_pop(&sensor_fifo, &packet)) {
            format_sensor_message(&packet, message_buffer, sizeof(message_buffer));
            
            size_t msg_len = strlen(message_buffer);
            if (mg_send(udp_state.connection, message_buffer, msg_len) > 0) {
#ifdef DEBUG
                debug_printf("Sent: %s", message_buffer);
#endif
                sent_any = true;
            } else {
                // Put packet back if send failed
                if (!fifo_push(&sensor_fifo, &packet)) {
#ifdef DEBUG
                    debug_printf("Failed to put packet back in FIFO\n");
#endif
                }
                break;
            }
        }
    }

    return sent_any;
}

// Format sensor message in key=value format
void format_sensor_message(const sensor_packet_t *packet, char *buffer, size_t buffer_size) {
    if (!packet || !buffer) return;

#if USE_KEY_VALUE_FORMAT
    snprintf(buffer, buffer_size, 
            "dev=%s sensor=%s %s timestamp=%s seq=%lu%s",
            DEVICE_NAME,
            packet->sensor_name,
            packet->value_str,
            packet->timestamp,
            (unsigned long)packet->sequence,
            MESSAGE_DELIMITER);
#else
    // JSON format alternative
    snprintf(buffer, buffer_size,
            "{\"dev\":\"%s\",\"sensor\":\"%s\",\"data\":\"%s\",\"timestamp\":\"%s\",\"seq\":%lu}%s",
            DEVICE_NAME,
            packet->sensor_name, 
            packet->value_str,
            packet->timestamp,
            (unsigned long)packet->sequence,
            MESSAGE_DELIMITER);
#endif
}

// Get UTC timestamp string
void get_utc_timestamp(char *buffer, size_t buffer_size) {
    if (!buffer) return;
    
    // Get current tick count and convert to seconds since boot
    TickType_t ticks = xTaskGetTickCount();
    uint32_t seconds = ticks / configTICK_RATE_HZ;
    
    // For demonstration, create a mock UTC timestamp
    // In real implementation, use RTC or NTP to get actual UTC time
    uint32_t hour = (seconds / 3600) % 24;
    uint32_t minute = (seconds / 60) % 60;
    uint32_t sec = seconds % 60;
    
    // Mock date - replace with actual date from RTC
    snprintf(buffer, buffer_size, "%02lu-%02lu-%02lu-24-09-2025", 
            (unsigned long)hour, (unsigned long)minute, (unsigned long)sec);
}

// FIFO operations implementation
bool fifo_push(packet_fifo_t *fifo, const sensor_packet_t *packet) {
    if (!fifo || !packet || !fifo->mutex) return false;

    if (xSemaphoreTake(fifo->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    bool success = false;
    if (fifo->count < SENSOR_BUFFER_SIZE) {
        fifo->buffer[fifo->head] = *packet;
        fifo->head = (fifo->head + 1) % SENSOR_BUFFER_SIZE;
        fifo->count++;
        success = true;
    }

    xSemaphoreGive(fifo->mutex);
    return success;
}

bool fifo_pop(packet_fifo_t *fifo, sensor_packet_t *packet) {
    if (!fifo || !packet || !fifo->mutex) return false;

    if (xSemaphoreTake(fifo->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    bool success = false;
    if (fifo->count > 0) {
        *packet = fifo->buffer[fifo->tail];
        fifo->tail = (fifo->tail + 1) % SENSOR_BUFFER_SIZE;
        fifo->count--;
        success = true;
    }

    xSemaphoreGive(fifo->mutex);
    return success;
}

bool fifo_is_empty(packet_fifo_t *fifo) {
    if (!fifo || !fifo->mutex) return true;

    bool empty = true;
    if (xSemaphoreTake(fifo->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        empty = (fifo->count == 0);
        xSemaphoreGive(fifo->mutex);
    }
    return empty;
}

bool fifo_is_full(packet_fifo_t *fifo) {
    if (!fifo || !fifo->mutex) return true;

    bool full = true;
    if (xSemaphoreTake(fifo->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        full = (fifo->count >= SENSOR_BUFFER_SIZE);
        xSemaphoreGive(fifo->mutex);
    }
    return full;
}

void fifo_clear(packet_fifo_t *fifo) {
    if (!fifo || !fifo->mutex) return;

    if (xSemaphoreTake(fifo->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        fifo->head = 0;
        fifo->tail = 0;
        fifo->count = 0;
        xSemaphoreGive(fifo->mutex);
    }
}

// Task control functions
void udp_task_pause(bool pause) {
    if (xSemaphoreTake(task_control.pause_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        task_control.is_paused = pause;
        if (!pause) {
            xSemaphoreGive(task_control.resume_sem);
        }
        xSemaphoreGive(task_control.pause_mutex);
    }
}

void udp_task_stop(void) {
    if (udp_task_handle != NULL) {
        vTaskDelete(udp_task_handle);
        udp_task_handle = NULL;
    }
    
    udp_disconnect();
    mg_mgr_free(&udp_state.mgr);
    
    if (sensor_fifo.mutex != NULL) {
        vSemaphoreDelete(sensor_fifo.mutex);
    }
    if (task_control.pause_mutex != NULL) {
        vSemaphoreDelete(task_control.pause_mutex);
    }
    if (task_control.resume_sem != NULL) {
        vSemaphoreDelete(task_control.resume_sem);
    }

#ifdef DEBUG
    debug_printf("UDP task stopped and resources cleaned up\n");
#endif
}
