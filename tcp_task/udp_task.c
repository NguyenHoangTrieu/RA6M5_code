#include "udp_task.h"
#include "ethernet_init.h"

// Task configuration
#define UDP_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define UDP_TASK_STACK_SIZE (3072)

// Static variables for sensor data transmission
static TaskHandle_t udp_task_handle = NULL;
static Socket_t udp_socket = FREERTOS_INVALID_SOCKET;
static struct freertos_sockaddr udp_server_addr;
static uint8_t udp_task_should_run = 1;

// Pause/resume control
typedef struct {
    uint8_t is_paused;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t resume_sem;
} thread_pause_t;

static thread_pause_t udp_pause = {0};

// Forward declarations
static void sensor_data_transmission_task(void *pvParameters);

// Initialize UDP configuration with hardcoded values
static void init_udp_config(udp_config_t *config)
{
    if (!config) return;
    
    // Server connection settings
    strcpy(config->server_host, "192.168.1.11");
    config->server_port = 8765;
    config->local_port = 0; // Any available port
    strcpy(config->client_id, "ck-ra6m5-01");
    strcpy(config->protocol_version, "1.0");
    strcpy(config->data_format, "json");
    
    // Timing settings
    config->send_interval = 5;
    config->socket_timeout = 5000;
    config->retry_delay_ms = 1000;
    config->loop_interval_ms = 100;
    config->payload_buffer_size = 1024;
    
    // Socket options
    config->socket_options.broadcast_enabled = 0;
    config->socket_options.reuse_addr = 1;
    config->socket_options.send_buffer_size = 2048;
    config->socket_options.recv_buffer_size = 1024;
    config->socket_options.send_timeout_ms = 5000;
    
    // System fields
    strcpy(config->system_fields.data_source, "ck-ra6m5-01");
    config->system_fields.include_timestamp = 1;
    config->system_fields.include_gateway_ip = 1;
    config->system_fields.include_node_count = 1;
    config->system_fields.include_system_info = 1;
    
    // Protocol settings
    strcpy(config->protocol_settings.message_delimiter, "\n");
    config->protocol_settings.max_message_size = 1024;
    config->protocol_settings.compression_enabled = 0;
    config->protocol_settings.encryption_enabled = 0;
    
    // Features
    config->features.telemetry_upload = 1;
    config->features.status_reporting = 1;
    
    printf("UDP config initialized\n");
}

// Get UDP configuration (singleton pattern)
const udp_config_t* get_udp_config(void)
{
    static udp_config_t udp_config;
    static uint8_t initialized = 0;
    
    if (!initialized) {
        init_udp_config(&udp_config);
        initialized = 1;
    }
    
    return &udp_config;
}

// Get CK-RA6M5 system information
const system_info_t* get_system_info(void)
{
    static system_info_t sys_info;
    static uint8_t initialized = 0;
    
    if (!initialized) {
        strcpy(sys_info.firmware_version, "1.0.0");
        strcpy(sys_info.device_type, "CK-RA6M5 IoT Gateway");
        strcpy(sys_info.manufacturer, "Renesas");
        strcpy(sys_info.model, "CK-RA6M5");
        initialized = 1;
    }
    
    return &sys_info;
}

// Get number of connected sensors
int get_node_count(void) { return 2; }

// Get device IP address
char *udp_get_local_ip(void)
{
    static char ip_str[16];
    uint32_t ip = ethernet_get_ip_address();
    
    sprintf(ip_str, "%d.%d.%d.%d", 
           (int)(ip & 0xFF),
           (int)((ip >> 8) & 0xFF),
           (int)((ip >> 16) & 0xFF),
           (int)((ip >> 24) & 0xFF));
    
    return ip_str;
}

// Initialize UDP socket
static int udp_init(void)
{
    const udp_config_t *config = get_udp_config();
    if (!config || !strlen(config->server_host) || !config->server_port) {
        printf("ERROR: Invalid UDP configuration\n");
        return -1;
    }
    
    // Create UDP socket
    udp_socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
    if (udp_socket == FREERTOS_INVALID_SOCKET) {
        printf("ERROR: Failed to create UDP socket\n");
        return -1;
    }
    
    // Configure socket options
    TickType_t send_timeout = pdMS_TO_TICKS(config->socket_options.send_timeout_ms);
    FreeRTOS_setsockopt(udp_socket, 0, FREERTOS_SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    
    // Setup server address
    memset(&udp_server_addr, 0, sizeof(udp_server_addr));
    udp_server_addr.sin_family = FREERTOS_AF_INET;
    udp_server_addr.sin_port = FreeRTOS_htons(config->server_port);
    udp_server_addr.sin_addr = FreeRTOS_inet_addr(config->server_host);
    
    printf("UDP initialized: %s:%d\n", config->server_host, config->server_port);
    return 0;
}

// Send sensor data packet to server
static int udp_send_sensor_data(const char *data, size_t len)
{
    if (!data || len == 0 || udp_socket == FREERTOS_INVALID_SOCKET) {
        return -1;
    }
    
    const udp_config_t *config = get_udp_config();
    if (!config || len > config->protocol_settings.max_message_size) {
        return -1;
    }
    
    // Append delimiter
    size_t delimiter_len = strlen(config->protocol_settings.message_delimiter);
    size_t total_len = len + delimiter_len;
    
    char *send_buffer = pvPortMalloc(total_len);
    if (!send_buffer) return -1;
    
    memcpy(send_buffer, data, len);
    memcpy(send_buffer + len, config->protocol_settings.message_delimiter, delimiter_len);
    
    // Send UDP packet
    int32_t sent = FreeRTOS_sendto(udp_socket, send_buffer, total_len, 0, 
                                   &udp_server_addr, sizeof(udp_server_addr));
    
    vPortFree(send_buffer);
    
    if (sent != (int32_t)total_len) {
        printf("UDP send failed: %d/%zu bytes\n", sent, total_len);
        return -1;
    }
    
    return 0;
}

// Build sensor data packet for transmission
static void build_sensor_data_packet(char *payload, size_t payload_size, TickType_t timestamp)
{
    if (!payload || payload_size == 0) return;
    
    const udp_config_t *config = get_udp_config();
    if (!config) {
        payload[0] = '\0';
        return;
    }
    
    // Build JSON sensor data packet
    snprintf(payload, payload_size,
             "{\"timestamp\":%lu,\"dev\":\"%s\",\"sensor\":\"ZMOD4510\","
             "\"o3\":45,\"no2\":12,\"aqi\":85,\"status\":\"active\"}",
             timestamp, config->client_id);
}

// Main sensor data transmission task
static void sensor_data_transmission_task(void *pvParameters)
{
    const udp_config_t *config = get_udp_config();
    if (!config) {
        printf("UDP Task: No config, exiting\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Wait for network to be ready
    while (FreeRTOS_GetIPAddress() == 0) {
        printf("Waiting for IP address...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    printf("Network ready, IP: %s\n", udp_get_local_ip());
    
    // Initialize UDP
    if (udp_init() != 0) {
        printf("UDP Task: Init failed, exiting\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Allocate sensor data buffer
    char *sensor_data_buffer = pvPortMalloc(config->payload_buffer_size);
    if (!sensor_data_buffer) {
        printf("UDP Task: Buffer allocation failed\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("UDP Task: Started, buffer size: %d\n", config->payload_buffer_size);
    
    TickType_t last_sensor_reading = 0;
    
    // Send initial device info
    const system_info_t *sys_info = get_system_info();
    char device_info[256];
    snprintf(device_info, sizeof(device_info),
             "{\"type\":\"device_info\",\"dev\":\"%s\",\"firmware\":\"%s\","
             "\"sensors\":[\"ZMOD4510\",\"ICP-10101\"],\"ip\":\"%s\"}",
             config->client_id, sys_info->firmware_version, udp_get_local_ip());
    
    udp_send_sensor_data(device_info, strlen(device_info));
    
    // Main transmission loop
    while (udp_task_should_run) {
        // Handle pause/resume
        if (udp_pause.mutex && xSemaphoreTake(udp_pause.mutex, 0) == pdTRUE) {
            if (udp_pause.is_paused) {
                xSemaphoreGive(udp_pause.mutex);
                xSemaphoreTake(udp_pause.resume_sem, portMAX_DELAY);
            } else {
                xSemaphoreGive(udp_pause.mutex);
            }
        }
        
        TickType_t current_time = xTaskGetTickCount();
        
        // Send sensor data at configured interval
        if ((current_time - last_sensor_reading) >= pdMS_TO_TICKS(config->send_interval * 1000)) {
            build_sensor_data_packet(sensor_data_buffer, config->payload_buffer_size, current_time);
            
            if (udp_send_sensor_data(sensor_data_buffer, strlen(sensor_data_buffer)) == 0) {
                printf("Sensor data sent: %s\n", sensor_data_buffer);
                last_sensor_reading = current_time;
            } else {
                printf("Failed to send sensor data\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(config->loop_interval_ms));
    }
    
    // Cleanup
    if (udp_socket != FREERTOS_INVALID_SOCKET) {
        FreeRTOS_closesocket(udp_socket);
        udp_socket = FREERTOS_INVALID_SOCKET;
    }
    
    vPortFree(sensor_data_buffer);
    printf("UDP Task: Exiting\n");
    vTaskDelete(NULL);
}

// Initialize sensor data transmission system
int udp_client_init(void)
{
    // Initialize Ethernet first
    if (ethernet_init() != 0) {
        printf("ERROR: Ethernet initialization failed\n");
        return pdFAIL;
    }
    
    if (ethernet_start() != 0) {
        printf("ERROR: Ethernet start failed\n");
        return pdFAIL;
    }
    
    // Create synchronization objects
    udp_pause.mutex = xSemaphoreCreateMutex();
    if (!udp_pause.mutex) return pdFAIL;
    
    udp_pause.resume_sem = xSemaphoreCreateBinary();
    if (!udp_pause.resume_sem) {
        vSemaphoreDelete(udp_pause.mutex);
        return pdFAIL;
    }
    
    // Create sensor data transmission task
    BaseType_t result = xTaskCreate(sensor_data_transmission_task, "SensorDataUDP",
                                   UDP_TASK_STACK_SIZE, NULL, UDP_TASK_PRIORITY, &udp_task_handle);
    
    if (result != pdPASS) {
        vSemaphoreDelete(udp_pause.mutex);
        vSemaphoreDelete(udp_pause.resume_sem);
        printf("UDP: Task creation failed\n");
        return pdFAIL;
    }
    
    printf("UDP: Client initialized successfully\n");
    return pdPASS;
}

// Pause sensor data transmission
void udp_client_pause(void)
{
    if (udp_pause.mutex && xSemaphoreTake(udp_pause.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        udp_pause.is_paused = 1;
        xSemaphoreGive(udp_pause.mutex);
        printf("UDP: Client paused\n");
    }
}

// Resume sensor data transmission
void udp_client_resume(void)
{
    if (udp_pause.mutex && xSemaphoreTake(udp_pause.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        udp_pause.is_paused = 0;
        xSemaphoreGive(udp_pause.mutex);
        
        if (udp_pause.resume_sem) {
            xSemaphoreGive(udp_pause.resume_sem);
        }
        
        printf("UDP: Client resumed\n");
    }
}

// Cleanup sensor data transmission system
void udp_client_deinit(void)
{
    udp_task_should_run = 0;
    
    if (udp_task_handle) {
        vTaskDelete(udp_task_handle);
        udp_task_handle = NULL;
    }
    
    if (udp_socket != FREERTOS_INVALID_SOCKET) {
        FreeRTOS_closesocket(udp_socket);
        udp_socket = FREERTOS_INVALID_SOCKET;
    }
    
    if (udp_pause.mutex) {
        vSemaphoreDelete(udp_pause.mutex);
        udp_pause.mutex = NULL;
    }
    
    if (udp_pause.resume_sem) {
        vSemaphoreDelete(udp_pause.resume_sem);
        udp_pause.resume_sem = NULL;
    }
    
    ethernet_stop();
    printf("UDP: Client deinitialized\n");
}