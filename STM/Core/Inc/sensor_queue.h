#ifndef __SENSOR_QUEUE_H
#define __SENSOR_QUEUE_H

#include "cmsis_os.h"
#include <stdint.h>

typedef enum {
  EVT_HALL,
  EVT_FLAME
} SensorEventType;

typedef struct {
  SensorEventType type;
  uint8_t slot;    /* EVT_HALL: 0-3 */
  uint8_t state;   /* EVT_HALL: occupied(1)/vacant(0), EVT_FLAME: verdict(1)/clear(0) */
  float energy;    /* EVT_FLAME only */
} SensorEvent_t;

extern osMessageQueueId_t sensorEventQueueHandle;
extern osSemaphoreId_t adcBufReadySemHandle;
extern osSemaphoreId_t hallEdgeSemHandle;

#endif /* __SENSOR_QUEUE_H */
