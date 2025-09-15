#ifndef TCP_TASK_H
#define TCP_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

// TCP socket options for sensor data transmission
typedef struct {
    uint8_t keepalive;              // Enable TCP keepalive
    uint8_t tcp_nodelay;            // Disable Nagle algorithm for real-time
    uint16_t keepalive_idle;        // Keepalive idle time (seconds)
    uint16_t keepalive_interval;    // Keepalive probe interval
    uint8_t keepalive_count;        // Max keepalive probes
} tcp_socket_options_t;

// System fields included in sensor data packets
typedef struct {
    char data_source[32];           // Device identifier
    uint8_t include_timestamp : 1;  // Include UTC timestamp
    uint8_t include_gateway_ip : 1; // Include device IP
    uint8_t include_node_count : 1; // Include sensor count
    uint8_t include_system_info : 1;// Include device info
    uint8_t reserved : 4;
} tcp_system_fields_t;

// Protocol settings for sensor data transmission
typedef struct {
    char message_delimiter[4];      // Message delimiter ("\n")
    uint16_t max_message_size;      // Max packet size (bytes)
    uint8_t compression_enabled : 1;// Enable compression (unused)
    uint8_t encryption_enabled : 1; // Enable encryption (unused)
    uint8_t reserved : 6;
} tcp_protocol_settings_t;

// Feature flags for IoT system
typedef struct {
    uint8_t telemetry_upload : 1;   // Enable data upload
    uint8_t status_reporting : 1;   // Enable status reports
    uint8_t reserved : 6;
} tcp_features_t;

// Main TCP configuration for CK-RA6M5 sensor data upload
typedef struct {
    char server_host[64];           // Linux server IP/hostname
    uint16_t server_port;           // Server port
    char client_id[32];             // Unique device ID
    char protocol_version[8];       // Protocol version
    char data_format[8];            // Data format (json)
    uint16_t send_interval;         // Send interval (seconds)
    uint16_t connection_timeout;    // Connection timeout
    uint16_t reconnect_delay_ms;    // Reconnection delay
    uint16_t loop_interval_ms;      // Main loop interval
    uint16_t payload_buffer_size;   // Buffer size for packets
    tcp_socket_options_t socket_options;
    tcp_system_fields_t system_fields;
    tcp_protocol_settings_t protocol_settings;
    tcp_features_t features;
} tcp_config_t;

// CK-RA6M5 system information
typedef struct {
    char firmware_version[16];      // Firmware version
    char device_type[32];          // Device type
    char manufacturer[16];         // Manufacturer
    char model[16];               // Device model
} system_info_t;

// Configuration getter functions
const tcp_config_t* get_tcp_config(void);
const system_info_t* get_system_info(void);
int get_node_count(void);
char *tcp_get_local_ip(void);

// Sensor data transmission API (upload only)
int tcp_client_init(void);          // Initialize sensor data task
void tcp_client_deinit(void);       // Cleanup resources
void tcp_client_pause(void);        // Pause transmission
void tcp_client_resume(void);       // Resume transmission

#endif // TCP_TASK_H
