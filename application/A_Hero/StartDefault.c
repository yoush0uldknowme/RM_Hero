#include "Hero.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern osThreadId calibrateTaskHandle;
extern osThreadId chassisTaskHandle;
extern osThreadId gimbalTaskHandle;
extern osThreadId usbtaskHandle;
extern osThreadId decodetaskHandle;
extern osThreadId uipaintTaskHandle;
extern osThreadId ledTaskHandle;
extern osThreadId capTaskHandle;

void StartDefaultTask(void const * argument) {
#ifdef GIMBAL
    vTaskDelete(chassisTaskHandle);
    vTaskDelete(uipaintTaskHandle);
    vTaskDelete(capTaskHandle);

    // vTaskDelete(calibrateTaskHandle);
    // vTaskDelete(gimbalTaskHandle);
    vTaskDelete(NULL);
#endif //!GIMBAL

#ifdef CHASSIS
    vTaskDelete(calibrateTaskHandle);
    vTaskDelete(gimbalTaskHandle);
    vTaskDelete(usbtaskHandle);
    vTaskDelete(decodetaskHandle);
    vTaskDelete(ledTaskHandle);
    vTaskDelete(NULL);
#endif //!CHASSIS
}