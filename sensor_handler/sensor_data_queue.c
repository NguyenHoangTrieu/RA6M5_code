#include "sensor_data_queue.h"

/* Queue handle */
QueueHandle_t g_sensor_data_queue = NULL;

/* Static memory for queue */
static StaticQueue_t xQueueBuffer;
static uint8_t ucQueueStorageArea[SENSOR_QUEUE_LENGTH * SENSOR_QUEUE_ITEM_SIZE];

/* Create sensor data queue */
void sensor_queue_create(void) {
    g_sensor_data_queue = xQueueCreateStatic(
        SENSOR_QUEUE_LENGTH,
        SENSOR_QUEUE_ITEM_SIZE,
        ucQueueStorageArea,
        &xQueueBuffer
    );
    
    configASSERT(g_sensor_data_queue != NULL);
}