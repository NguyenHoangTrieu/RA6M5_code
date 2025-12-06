#ifndef SENSOR_DATA_QUEUE_H_
#define SENSOR_DATA_QUEUE_H_

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

/* Sensor data structure */
typedef struct {
    char sensor_name[32];    // e.g., "zmod4410", "icp10101"
    float value1;            // e.g., tvoc for ZMOD
    float value2;            // e.g., iaq for ZMOD, temp for ICP
    char unit1[16];          // e.g., "ppb"
    char unit2[16];          // e.g., "iaq", "C"
} sensor_data_t;

/* Queue handle - extern to be used in both threads */
extern QueueHandle_t g_sensor_data_queue;

/* Queue parameters */
#define SENSOR_QUEUE_LENGTH     20
#define SENSOR_QUEUE_ITEM_SIZE  sizeof(sensor_data_t)

/* Function to create queue */
void sensor_queue_create(void);
void uart_print(const char *msg);
#endif /* SENSOR_DATA_QUEUE_H_ */
