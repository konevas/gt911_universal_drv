#pragma once

/**
 * @brief Touch panel driver header
 * 
 * @author Konev A.
 */

#include <stdint.h>
#include <stdbool.h>

#define TOUCHPANEL_ADDR 0xBA
#define TOUCHPANEL_POINTS_MAX 6

typedef enum touchPanelEventIdT
{
    TOUCHPANEL_EVENTID_TOUCH_DETECTED,
    TOUCHPANEL_EVENTID_TOUCH_DEACTIVATED,
} touchPanelEventIdT;

typedef struct touchPanelPointT
{
    uint16_t x;
    uint16_t y;
} touchPanelPointT;

typedef struct touchPanelTouchT
{
    size_t count;
    touchPanelPointT point[TOUCHPANEL_POINTS_MAX];
} touchPanelTouchT; 

typedef union touchPanelEventDataT
{
    touchPanelTouchT touch;
} touchPanelEventDataT;

typedef struct touchPanelT * touchPanelH;

typedef void (* touchPanelSleepMsT)(size_t ms);

typedef int (* touchPanelReadT)(void * hndl, 
                                uint8_t addr,
                                const uint8_t * bufTx, int countTx,
                                uint8_t * bufRx, int countRx);

typedef int (* touchPanelWriteT)(void * hndl, uint8_t addr,
                                 const uint8_t * buf, int count);

typedef void (* touchPanelDispatchT)(touchPanelH tp, touchPanelEventIdT id,
                                     const touchPanelEventDataT * eventData,
                                     void * userData);

typedef struct touchPanelIOT
{
    void * hndl;
    touchPanelReadT read;
    touchPanelWriteT write;
    touchPanelSleepMsT sleepMs;
} touchPanelIOT;

typedef struct touchPanelCfgT
{
    touchPanelIOT io;
    touchPanelDispatchT dispatch;
    void * userData;
} touchPanelCfgT;

unsigned long touchPanelESDProcessing(touchPanelH tp);
unsigned long touchPanelProcessing(touchPanelH tp);

touchPanelH touchPanelOpen(touchPanelCfgT * cfg);
void touchPanelClose(touchPanelH tp);
