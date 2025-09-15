#include "tcp_task.h"
#include "mongoose.h"

// Task configuration
#define TCP_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define TCP_TASK_STACK_SIZE (3072)

// Static variables for sensor data transmission
static TaskHandle_t tcp_task_handle = NULL;
static struct mg_mgr tcp_mgr;
static struct mg_connection *tcp_connection = NULL;
static uint8_t tcp_connected = 0;
static uint8_t tcp_task_should_run = 1;

// Pause/resume control
typedef struct {
    uint8_t is_paused;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t resume_sem;
} thread_pause_t;

static thread_pause_t tcp_pause = {0};

// Forward declarations
static void tcp_event_handler(struct mg_connection *c, int ev, void *ev_data);
static void sensor_data_transmission_task(void *pvParameters);

// Initialize TCP configuration with hardcoded values
static void init_tcp_config(tcp_config_t *config)
{
    if (!config) return;
    
    // Server connection settings
    strcpy(config->server_host, "192.168.1.11");
    config->server_port = 8765;
    strcpy(config->client_id, "ck-ra6m5-01");
    strcpy(config->protocol_version, "1.0");
    strcpy(config->data_format, "json");
    
    // Timing settings
    config->send_interval = 5;
    config->connection_timeout = 30;
    config->reconnect_delay_ms = 5000;
    config->loop_interval_ms = 100;
    config->payload_buffer_size = 1024;
    
    // Socket options
    config->socket_options.keepalive = 1;
    config->socket_options.tcp_nodelay = 1;
    config->socket_options.keepalive_idle = 60;
    config->socket_options.keepalive_interval = 10;
    config->socket_options.keepalive_count = 3;
    
    // System fields
    strcpy(config->system_fields.data_source, "ck-ra6m5-01");
    config->system_fields.include_timestamp = 1;
    config->system_fields.include_gateway_ip = 1;
    config->system_fields.include_node_count = 1;
    config->system_fields.include_system_info = 1;
    
    // Protocol settings
    strcpy(config->protocol_settings.message_delimiter, "\n");
    config->protocol_settings.max_message_size = 2048;
    config->protocol_settings.compression_enabled = 0;
    config->protocol_settings.encryption_enabled = 0;
    
    // Features
    config->features.telemetry_upload = 1;
    config->features.status_reporting = 1;
    
#ifdef DEBUG
    printf("TCP config initialized\n");
#endif
}

// Get TCP configuration (singleton pattern)
const tcp_config_t* get_tcp_config(void)
{
    static tcp_config_t tcp_config;
    static uint8_t initialized = 0;
    
    if (!initialized) {
        init_tcp_config(&tcp_config);
        initialized = 1;
    }
    return &tcp_config;
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
char *tcp_get_local_ip(void)
{
    static char ip_str[16];
    strcpy(ip_str, "192.168.1.100");
    return ip_str;
}

// Initialize TCP client for sensor data transmission
static int tcp_init(void)
{
    mg_mgr_init(&tcp_mgr);
    tcp_connection = NULL;
    tcp_connected = 0;
    
    const tcp_config_t *config = get_tcp_config();
    if (!config || !strlen(config->server_host) || !config->server_port) {
#ifdef DEBUG
        printf("ERROR: Invalid TCP configuration\n");
#endif
        return -1;
    }
    
#ifdef DEBUG
    printf("TCP initialized: %s:%d\n", config->server_host, config->server_port);
#endif
    return 0;
}

// Establish TCP connection to server
static int tcp_connect(const char *server_url)
{
    const tcp_config_t *config = get_tcp_config();
    if (!config) return -1;
    
    char url[128];
    if (server_url && strlen(server_url) > 0) {
        snprintf(url, sizeof(url), "%s", server_url);
    } else {
        snprintf(url, sizeof(url), "tcp://%s:%d", config->server_host, config->server_port);
    }
    
    tcp_connected = 0;
    tcp_connection = mg_connect(&tcp_mgr, url, tcp_event_handler, NULL);
    
#ifdef DEBUG
    printf("TCP: %s to %s\n", tcp_connection ? "Connecting" : "Failed to connect", url);
#endif
    
    return tcp_connection ? 0 : -1;
}

// Send sensor data packet to Linux server
static int tcp_send_sensor_data(const char *data, size_t len)
{
    if (!data || len == 0 || !tcp_connection || !tcp_connected)
        return -1;
    
    const tcp_config_t *config = get_tcp_config();
    if (!config || len > config->protocol_settings.max_message_size)
        return -1;
    
    // Append delimiter
    size_t delimiter_len = strlen(config->protocol_settings.message_delimiter);
    size_t total_len = len + delimiter_len;
    
    char *send_buffer = pvPortMalloc(total_len);
    if (!send_buffer) return -1;
    
    memcpy(send_buffer, data, len);
    memcpy(send_buffer + len, config->protocol_settings.message_delimiter, delimiter_len);
    
    size_t sent = mg_send(tcp_connection, send_buffer, total_len);
    vPortFree(send_buffer);
    
#ifdef DEBUG
    if (sent != total_len) {
        printf("TCP send failed: %zu/%zu bytes\n", sent, total_len);
    }
#endif
    
    return (sent == total_len) ? 0 : -1;
}

// TCP event handler for sensor data communication
void tcp_event_handler(struct mg_connection *c, int ev, void *ev_data)
{
    switch (ev) {
        case MG_EV_CONNECT:
            tcp_connected = 1;
            tcp_connection = c;
            
#ifdef DEBUG
            printf("TCP: Connected to server\n");
#endif
            
            // Send device identification
            const tcp_config_t *config = get_tcp_config();
            if (config && config->features.status_reporting) {
                const system_info_t *sys_info = get_system_info();
                char handshake[256];
                snprintf(handshake, sizeof(handshake),
                         "{\"type\":\"device_info\",\"dev\":\"%s\",\"firmware\":\"%s\","
                         "\"sensors\":[\"ZMOD4510\",\"ICP-10101\"],\"ip\":\"%s\"}\n",
                         config->client_id, sys_info->firmware_version, tcp_get_local_ip());
                mg_send(c, handshake, strlen(handshake));
            }
            break;
            
        case MG_EV_READ:
            // Ignore all incoming data (one-way communication only)
            if (c->recv.len > 0) {
#ifdef DEBUG
                printf("TCP: Ignoring %zu bytes from server\n", c->recv.len);
#endif
                mg_iobuf_del(&c->recv, 0, c->recv.len);
            }
            break;
            
        case MG_EV_CLOSE:
        case MG_EV_ERROR:
            tcp_connected = 0;
            tcp_connection = NULL;
            
#ifdef DEBUG
            printf("TCP: Connection %s\n", ev == MG_EV_CLOSE ? "closed" : "error");
#endif
            break;
    }
}

// Build sensor data packet for transmission
static void build_sensor_data_packet(char *payload, size_t payload_size, TickType_t timestamp)
{
    if (!payload || payload_size == 0)
        return;
    
    const tcp_config_t *config = get_tcp_config();
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
    const tcp_config_t *config = get_tcp_config();
    if (!config) {
#ifdef DEBUG
        printf("TCP Task: No config, exiting\n");
#endif
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize TCP
    if (tcp_init() != 0) {
#ifdef DEBUG
        printf("TCP Task: Init failed, exiting\n");
#endif
        vTaskDelete(NULL);
        return;
    }
    
    // Connect to server
    char server_url[64];
    snprintf(server_url, sizeof(server_url), "tcp://%s:%d",
             config->server_host, config->server_port);
    tcp_connect(server_url);
    
    // Allocate sensor data buffer
    char *sensor_data_buffer = pvPortMalloc(config->payload_buffer_size);
    if (!sensor_data_buffer) {
#ifdef DEBUG
        printf("TCP Task: Buffer allocation failed\n");
#endif
        vTaskDelete(NULL);
        return;
    }
    
#ifdef DEBUG
    printf("TCP Task: Started, buffer size: %d\n", config->payload_buffer_size);
#endif
    
    TickType_t last_sensor_reading = 0;
    
    // Main transmission loop
    while (tcp_task_should_run) {
        // Handle pause/resume
        if (tcp_pause.mutex && xSemaphoreTake(tcp_pause.mutex, 0) == pdTRUE) {
            if (tcp_pause.is_paused) {
                xSemaphoreGive(tcp_pause.mutex);
                xSemaphoreTake(tcp_pause.resume_sem, portMAX_DELAY);
            } else {
                xSemaphoreGive(tcp_pause.mutex);
            }
        }
        
        // Process network events
        mg_mgr_poll(&tcp_mgr, 100);
        
        TickType_t current_time = xTaskGetTickCount();
        
        // Send sensor data at configured interval
        if (tcp_connected && 
            (current_time - last_sensor_reading) >= pdMS_TO_TICKS(config->send_interval * 1000)) {
            
            build_sensor_data_packet(sensor_data_buffer, config->payload_buffer_size, current_time);
            
            if (tcp_send_sensor_data(sensor_data_buffer, strlen(sensor_data_buffer)) == 0) {
#ifdef DEBUG
                printf("Sensor data sent: %s\n", sensor_data_buffer);
#endif
                last_sensor_reading = current_time;
            }
        }
        
        // Handle reconnection
        if (!tcp_connected) {
#ifdef DEBUG
            printf("TCP: Reconnecting...\n");
#endif
            tcp_connect(server_url);
            vTaskDelay(pdMS_TO_TICKS(config->reconnect_delay_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(config->loop_interval_ms));
        }
    }
    
    // Cleanup
    vPortFree(sensor_data_buffer);
    mg_mgr_free(&tcp_mgr);
    
#ifdef DEBUG
    printf("TCP Task: Exiting\n");
#endif
    
    vTaskDelete(NULL);
}

// Initialize sensor data transmission system
int tcp_client_init(void)
{
    // Create synchronization objects
    tcp_pause.mutex = xSemaphoreCreateMutex();
    if (!tcp_pause.mutex) return pdFAIL;
    
    tcp_pause.resume_sem = xSemaphoreCreateBinary();
    if (!tcp_pause.resume_sem) {
        vSemaphoreDelete(tcp_pause.mutex);
        return pdFAIL;
    }
    
    // Create sensor data transmission task
    BaseType_t result = xTaskCreate(sensor_data_transmission_task, "SensorData", 
                                   TCP_TASK_STACK_SIZE, NULL, TCP_TASK_PRIORITY, &tcp_task_handle);
    
    if (result != pdPASS) {
        vSemaphoreDelete(tcp_pause.mutex);
        vSemaphoreDelete(tcp_pause.resume_sem);
#ifdef DEBUG
        printf("TCP: Task creation failed\n");
#endif
        return pdFAIL;
    }
    
#ifdef DEBUG
    printf("TCP: Client initialized successfully\n");
    tcp_debug_print_config();
#endif
    
    return pdPASS;
}

// Pause sensor data transmission
void tcp_client_pause(void)
{
    if (tcp_pause.mutex && xSemaphoreTake(tcp_pause.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        tcp_pause.is_paused = 1;
        xSemaphoreGive(tcp_pause.mutex);
        
#ifdef DEBUG
        printf("TCP: Client paused\n");
#endif
    }
}

// Resume sensor data transmission
void tcp_client_resume(void)
{
    if (tcp_pause.mutex && xSemaphoreTake(tcp_pause.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        tcp_pause.is_paused = 0;
        xSemaphoreGive(tcp_pause.mutex);
        if (tcp_pause.resume_sem) {
            xSemaphoreGive(tcp_pause.resume_sem);
        }
        
#ifdef DEBUG
        printf("TCP: Client resumed\n");
#endif
    }
}

// Cleanup sensor data transmission system
void tcp_client_deinit(void)
{
    tcp_task_should_run = 0;
    
    if (tcp_task_handle) {
        vTaskDelete(tcp_task_handle);
        tcp_task_handle = NULL;
    }
    
    if (tcp_pause.mutex) {
        vSemaphoreDelete(tcp_pause.mutex);
        tcp_pause.mutex = NULL;
    }
    
    if (tcp_pause.resume_sem) {
        vSemaphoreDelete(tcp_pause.resume_sem);
        tcp_pause.resume_sem = NULL;
    }
    
#ifdef DEBUG
    printf("TCP: Client deinitialized\n");
#endif
}
