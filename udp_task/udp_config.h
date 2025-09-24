#ifndef CONFIG_H
#define CONFIG_H

// Network Configuration
#define UDP_SERVER_HOST    "192.168.1.11"
#define UDP_SERVER_PORT    8765
#define UDP_LOCAL_PORT     4000
#define UDP_BIND_URL       "udp://" UDP_SERVER_HOST ":8765"

// Device Configuration
#define DEVICE_NAME        "ck-ra6m5-01"
#define FIRMWARE_VERSION   "1.0.0"

// Sensor Configuration - Select 2 sensors from available options
#define SENSOR_ZMOD4410    0  // Indoor air quality (TVOC, IAQ)
#define SENSOR_ZMOD4510    1  // Outdoor air quality (O3, NO2, AQI) 
#define SENSOR_ICP10101    2  // Pressure, Temperature, Altitude
#define SENSOR_BIOMETRIC   3  // Heart rate, SpO2

#define SELECTED_SENSOR_1  SENSOR_ZMOD4410
#define SELECTED_SENSOR_2  SENSOR_ICP10101
#define NUM_SENSORS        2

// Timing Configuration
#define SENSOR_READ_INTERVAL_MS    5000   // Read sensors every 5 seconds
#define UDP_SEND_INTERVAL_MS       5000   // Send data every 5 seconds
#define TASK_LOOP_DELAY_MS         100    // Main loop delay
#define CONNECTION_TIMEOUT_MS      30000  // Connection timeout
#define RECONNECT_DELAY_MS         2000   // Reconnection delay

// Buffer Configuration
#define SENSOR_BUFFER_SIZE         50     // Number of sensor packets to buffer
#define PACKET_MAX_SIZE            128    // Maximum UDP packet size
#define UDP_RECEIVE_BUFFER_SIZE    512    // Receive buffer size

// Message Format Configuration
#define MESSAGE_DELIMITER          "\n"
#define USE_KEY_VALUE_FORMAT       1      // Use key=value format instead of JSON

// Debug Configuration (comment out to disable debug prints)
#define DEBUG

#endif // CONFIG_H
