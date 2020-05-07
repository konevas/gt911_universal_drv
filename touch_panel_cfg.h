#pragma once

/**
 * @brief Touch panle config
 * 
 * @author Konev A.
 */
 
#define TOUCHPANEL_ADDR 0xBA

#define MALLOC(len__) malloc(len__)
#define FREE(ptr__) free(ptr__)

#define DEBUG_LEVEL 1

#ifndef DEBUG_ID
    #define DEBUG_ID "[touch_panel] "
#endif

#if( DEBUG_LEVEL >= 1 )
    #define DBGPRINT(level, ...)\
        do{\
            if( DEBUG_LEVEL >= level )\
            {\
                printf(DEBUG_ID __VA_ARGS__);\
            }\
        }while(0)
#else
    #define DBGPRINT(level, ...)
#endif
