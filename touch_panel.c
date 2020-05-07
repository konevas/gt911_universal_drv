/**
 * @brief Touch panel driver
 *
 * @author Konev A.
 */

#ifndef DEBUG_LEVEL
    #define DEBUG_LEVEL 1
#endif

#include <string.h>
#include <stdint.h>
#include <stdchar.h>
#include <stdlib.h>
#include <stdio.h>
#include "macro_checkers.h"
#include "touch_panel_cfg.h"
#include "touch_panel_reg_cfg.h"
#include "touch_panel.h"


#define GTP_DEFAULT_MAX_X 720    /* default coordinate max values */
#define GTP_DEFAULT_MAX_Y 1080
#define GTP_DEFAULT_MAX_WIDTH 1024
#define GTP_DEFAULT_MAX_PRESSURE 1024
#define GTP_MAX_TOUCH_ID 6

#define GOODIX_COORDS_ARR_SIZE  4
#define PROP_NAME_SIZE      24
#define I2C_MAX_TRANSFER_SIZE   255
#define GTP_PEN_BUTTON1     BTN_STYLUS
#define GTP_PEN_BUTTON2     BTN_STYLUS2
#define RETRY_MAX_TIMES     5
#define MASK_BIT_8      0x80
#define GTP_CONFIG_MIN_LENGTH   186
#define GTP_ESD_CHECK_VALUE 0xAA

/* Registers define */
#define GTP_REG_COMMAND     0x8040
#define GTP_REG_ESD_CHECK   0x8041
#define GTP_REG_COMMAND_CHECK   0x8046
#define GTP_REG_CONFIG_DATA 0x8047
#define GTP_REG_VERSION     0x8140
#define GTP_REG_SENSOR_ID   0x814A
#define GTP_REG_DOZE_BUF    0x814B
#define GTP_READ_COOR_ADDR  0x814E
#define GTP_READ_STATUS     0x81A8

#define ENOMEM 1
#define EAGAIN 2
#define EINVAL 3
#define EPERM 4

#define FAIL            0
#define SUCCESS         1

#define dev_err(...) DBGPRINT(1, __VA_ARGS__)
#define dev_warn(...) DBGPRINT(2, __VA_ARGS__)
#define dev_info(...) DBGPRINT(3, __VA_ARGS__)
#define dev_dbg(...) DBGPRINT(4, __VA_ARGS__)

#define u8 uint8_t
#define u32 uint32_t
#define s32 int32_t
#define u16 uint16_t

#define GTP_TOOL_PEN    1
#define GTP_TOOL_FINGER 2

#define MAX_KEY_NUMS 4
#define GTP_CONFIG_MAX_LENGTH 240
#define GTP_ADDR_LENGTH       2

#define CELLSOF(array__) (sizeof(array__) / sizeof(array__[0]))

typedef touchPanelIOT * IO_HandleT;

enum {
    WORK_THREAD_ENABLED = 0,
    HRTIMER_USED,
    FW_ERROR,

    DOZE_MODE,
    SLEEP_MODE,
    POWER_OFF_MODE,
    RAW_DATA_MODE,

    FW_UPDATE_RUNNING,
    PANEL_RESETTING
};

struct goodix_point_t {
    int id;
    int x;
    int y;
    int w;
    int p;
    int tool_type;
};

struct goodix_config_data {
    int length;
    u8 data[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH];
};

struct goodix_ts_platform_data {
    u32 irq_flags;
    u32 abs_size_x;
    u32 abs_size_y;
    u32 max_touch_id;
    u32 max_touch_width;
    u32 max_touch_pressure;
    u32 key_map[MAX_KEY_NUMS];
    u32 key_nums;
    u32 int_sync;
    u32 driver_send_cfg;
    u32 swap_x2y;
    u32 slide_wakeup;
    u32 auto_update;
    u32 auto_update_cfg;
    u32 esd_protect;
    u32 type_a_report;
    u32 power_off_sleep;
    u32 resume_in_workqueue;
    u32 pen_suppress_finger;
    struct goodix_config_data config;
};

struct goodix_fw_info {
    u8 pid[6];
    u16 version;
    u8 sensor_id;
};

typedef enum stateT
{
    TOUCH_PRESENTED,
    CLEAR
} stateT;

typedef struct touchPanelT
{
    unsigned long flags;
    touchPanelCfgT cfg;
    struct goodix_ts_platform_data data;
    struct goodix_fw_info fw_info;
    stateT state;
} touchPanelT;

static s32 gtp_send_cfg(IO_HandleT io,
                        struct goodix_ts_platform_data *data);
static void init_tp_data(struct goodix_ts_platform_data *data);
static int gtp_io_test(IO_HandleT io);
static s32 gtp_get_fw_info(IO_HandleT io,
                           struct goodix_fw_info *fw_info);
static inline void set_bit(long pos, volatile unsigned long *addr);
static inline void clear_bit(long pos, volatile unsigned long *addr);
static inline bool test_bit(long pos, volatile unsigned long *addr);
static int gtp_enter_doze(touchPanelH tp);
static void gtp_show_touches(touchPanelH tp, u8 touch_num,
                             struct goodix_point_t points[]);
static s32 gtp_init_panel(struct goodix_ts_platform_data *data,
                          IO_HandleT io);
static int gtp_io_write(IO_HandleT io, const u8 *buf, int len);
static int gtp_io_read(IO_HandleT io, u8 *buf, int len);
static int gtp_gesture_handler(touchPanelH tp);
static u8 gtp_get_points(touchPanelH tp, struct goodix_point_t *points,
                         u8 *key_value);
static void clear_touch_points_status(IO_HandleT io);
static int gtp_init_esd_reg(IO_HandleT io);
static bool check_io_instance(IO_HandleT io);
static int gtp_find_valid_cfg_data(struct goodix_config_data *cfg,
                                   IO_HandleT io);
static inline size_t get_touch_count(uint32_t state);

/**
 * @brief Open touch panel driver
 * @return descriptor
 */
touchPanelH touchPanelOpen(touchPanelCfgT *cfg)
{
    bool success;

    IF_NULL_RETURN_EXPR(cfg, NULL);
    check_io_instance(&cfg->io);

    touchPanelH tp = MALLOC(sizeof(*tp));
    IF_NULL_SETSTATUS_GOTO(tp, success, false, EXIT_POINT);

    memset(tp, 0, sizeof(*tp));

    init_tp_data(&tp->data);

    tp->cfg = *cfg;
    tp->state = CLEAR;

    success = !gtp_io_test(&tp->cfg.io);
    if (!success) {
        dev_err("Failed communicate with IC use I2C\n");
        goto EXIT_POINT;
    }

    success = gtp_get_fw_info(&tp->cfg.io, &tp->fw_info) >= 0;
    if (!success) {
        dev_err("Failed read FW version\n");
        goto EXIT_POINT;
    }

    success = !gtp_init_panel(&tp->data, &tp->cfg.io);
    if (!success)
    {
        dev_info("Panel un-initialize\n");
        goto EXIT_POINT;
    }

#ifdef CONFIG_TOUCHSCREEN_GT9XX_UPDATE
    if (ts->pdata->auto_update) {
        ret = gup_init_update_proc(ts);
        if (ret < 0)
            dev_err("Failed create update thread\n");
    }
#endif

    set_bit(WORK_THREAD_ENABLED, &tp->flags);

#ifdef CONFIG_TOUCHSCREEN_GT9XX_TOOL
    init_wr_node(client);/*TODO judge return value */
#endif

    EXIT_POINT:
    if (!success)
    {
        touchPanelClose(tp);
        tp = NULL;
    }
    return tp;
}

/**
 * @brief Close touch panel driver
 *
 * @param tp descriptor
 */
void touchPanelClose(touchPanelH tp)
{
    IF_NULL_RETURN(tp);

    free(tp);
}

/**
 * @brief Touchpanel ESD protection routine
 * @details Should be called periodically
 *
 * @param tp touch panel descriptor
 * @return period in ms to reenter
 */
unsigned long touchPanelESDProcessing(touchPanelH tp)
{
    enum {WAIT_TIME_MS = 500};
    s32 i;
    s32 ret = -1;
    u8 esd_buf[5] = { (u8)(GTP_REG_COMMAND >> 8), (u8)GTP_REG_COMMAND };

    if (test_bit(SLEEP_MODE, &tp->flags) ||
        test_bit(FW_UPDATE_RUNNING, &tp->flags)) {
        dev_dbg("Esd cancled by power_suspend or fw_update!\n");
       return WAIT_TIME_MS;
    }

    for (i = 0; i < 3; i++) {
       ret = gtp_io_read(&tp->cfg.io, esd_buf, 4);
       if (ret < 0)
           continue;

       dev_dbg("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X\n",
               esd_buf[2], esd_buf[3]);
       if (esd_buf[2] == (u8)GTP_ESD_CHECK_VALUE ||
           esd_buf[3] != (u8)GTP_ESD_CHECK_VALUE) {
           gtp_io_read(&tp->cfg.io, esd_buf, 4);
           if (ret < 0)
               continue;

           if (esd_buf[2] == (u8)GTP_ESD_CHECK_VALUE ||
               esd_buf[3] != (u8)GTP_ESD_CHECK_VALUE) {
               i = 3;
               break;
            }
        } else {
                       /* IC works normally, Write 0x8040 0xAA, feed the dog */
           esd_buf[2] = (u8)GTP_ESD_CHECK_VALUE;
           gtp_io_write(&tp->cfg.io, esd_buf, 3);
           break;
        }
    }
    if (i >= 3) {
       dev_err("IC working abnormally! Reset IC\n");
       esd_buf[0] = 0x42;
       esd_buf[1] = 0x26;
       esd_buf[2] = 0x01;
       esd_buf[3] = 0x01;
       esd_buf[4] = 0x01;
       gtp_io_write(&tp->cfg.io, esd_buf, 5);
               /** @TODO power off (may be not needed) */
               /** @TODO reset */
    }

    return WAIT_TIME_MS;
}

/**
 * @brief Touch panel trace routine
 * @details Should be called periodically
 *
 * @param tp touch panel descriptor
 * @return period in ms to reenter
 */
unsigned long touchPanelProcessing(touchPanelH tp)
{
    enum {POLL_TIME_MS = 20, WAIT_TIME_MS = 100};

    u8 point_state = 0;
    u8 key_value = 0;
    s32 i = 0;
    s32 ret = -1;
    static u8 pre_key = 0;
    struct goodix_point_t points[GTP_MAX_TOUCH_ID];

    if (test_bit(PANEL_RESETTING, &tp->flags))
        return WAIT_TIME_MS;
    if (!test_bit(WORK_THREAD_ENABLED, &tp->flags))
        return WAIT_TIME_MS;

    /* gesture event */
    if (tp->data.slide_wakeup && test_bit(DOZE_MODE, &tp->flags)) {
        ret =  gtp_gesture_handler(tp);
        if (ret)
            dev_err(                    "Failed handler gesture event %d\n", (int)ret);
        goto proc_exit;
    }

    point_state = gtp_get_points(tp, points, &key_value);

    if( point_state != 0 )
    {
        tp->state = TOUCH_PRESENTED;

        touchPanelEventDataT event = {.touch.count = get_touch_count(point_state)};
        for (int i = 0; i < event.touch.count; ++i)
        {
            event.touch.point[i].x = points[i].x;
            event.touch.point[i].y = points[i].y;
        }

        tp->cfg.dispatch(tp, TOUCHPANEL_EVENTID_TOUCH_DETECTED, &event, 
                         tp->cfg.userData);
    }
    else if(tp->state != CLEAR)
    {
        tp->state = CLEAR;

        tp->cfg.dispatch(tp, TOUCHPANEL_EVENTID_TOUCH_DEACTIVATED, NULL, 
                         tp->cfg.userData);
    }

    if (!point_state) {
        goto proc_exit;
    }

    /* touch key event */
    if (key_value & 0xf0 || pre_key & 0xf0) {
        dev_dbg("touch key event %d \n", key_value);
        pre_key = key_value;
    } else if (key_value & 0x0f || pre_key & 0x0f) {
        /* panel key */
        for (i = 0; i < tp->data.key_nums; i++) {
            if ((pre_key | key_value) & (0x01 << i)){
                dev_dbg("panel key event %d : %u\n",
                        (unsigned int)tp->data.key_map[i],
                        key_value & (0x01 << i));
            }
        }
        pre_key = key_value;
    }

    gtp_show_touches(tp, point_state & 0x0f, points);

    proc_exit:
    return POLL_TIME_MS;
}

/*
 * return: 2 - ok, < 0 - io transfer error
 */
static int gtp_io_read(IO_HandleT io, u8 *buf, int len)
{
    unsigned int payload_length = 0;
    unsigned int pos = 0, address = (buf[0] << 8) + buf[1];
    unsigned char buf_tmp[256], addr_buf[2];
    int retry, result = 2;
    unsigned char *buf_;
    bool heap_used = false;
    int err;

    len -= GTP_ADDR_LENGTH;
    if (len < sizeof(buf_tmp)) {
        /* code optimize, use stack memory */
        buf_ = buf_tmp;
    } else {
        buf_ = MALLOC(I2C_MAX_TRANSFER_SIZE);
        if (buf_ == NULL)
            return -ENOMEM;
        heap_used = true;
    }

    while (pos != len) {
        if (len - pos > I2C_MAX_TRANSFER_SIZE)
            payload_length = I2C_MAX_TRANSFER_SIZE;
        else
            payload_length = len - pos;

        addr_buf[0] = (address >> 8) & 0xFF;
        addr_buf[1] = address & 0xFF;
        
        for (retry = 0; retry < RETRY_MAX_TIMES; retry++) {
            err = io->read(io->hndl, TOUCHPANEL_ADDR, addr_buf, 2, buf_,
                            payload_length);
            if (err == 0) {
                memcpy(&buf[2 + pos], buf_, payload_length);
               pos += payload_length;
               address += payload_length;
               break;
           }
           dev_dbg("I2c read retry[%d]:0x%x\n",
                   retry + 1, address);
           io->sleepMs(21);
       }
       if (retry == RETRY_MAX_TIMES) {
           dev_err(                   "I2c read failed,dev:%02x,reg:%04x,size:%u\n",
                   TOUCHPANEL_ADDR, address, len);
           result = -EAGAIN;
           goto read_exit;
       }
    }

    read_exit:
    if (heap_used)
       FREE(buf_);
    return result;
}

/*******************************************************
 * Function:
 *  io read twice, compare the results
 * Input:
 *  client: io device
 *  addr: operate address
 *  rxbuf: read data to store, if compare successful
 *  len: bytes to read
 * Output:
 *  FAIL: read failed
 *  SUCCESS: read successful
 *********************************************************/
static s32 gtp_io_read_dbl_check(IO_HandleT io, u16 addr, u8 *rxbuf,
                                  int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;

    if (len + 2 > sizeof(buf)) {
        dev_warn("%s, only support length less then %zu\n",
                 __func__, sizeof(buf) - 2);
        return FAIL;
    }
    while (retry++ < 3) {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_io_read(io, buf, len + 2);

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_io_read(io, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len + 2)) {
            memcpy(rxbuf, confirm_buf + 2, len);
            return SUCCESS;
        }
    }
    dev_err(            "I2C read 0x%04X, %d bytes, double check failed!\n",
            addr, len);

    return FAIL;
}

/*******************************************************
 * Function:
 *  Write data to the io slave device.
 * Input:
 *  client: io device.
 *  buf[0~1]: write start address.
 *  buf[2~len-1]: data buffer
 *  len: GTP_ADDR_LENGTH + write bytes count
 * Output:
 *  numbers of io_msgs to transfer:
 *      1: succeed, otherwise: failed
 *********************************************************/
static int gtp_io_write(IO_HandleT io, const u8 *buf, int len)
{
    unsigned int pos = 0, payload_length = 0;
    unsigned int address = (buf[0] << 8) + buf[1];
    unsigned char buf_tmp[64];
    unsigned char *buf_;
    int retry, result = 1;
    bool heap_used = false;
    int err;

    if (len < sizeof(buf_tmp)) {
        /* code optimize,use stack memory*/
        buf_ = &buf_tmp[0];
    } else {
        buf_ = MALLOC(I2C_MAX_TRANSFER_SIZE);
        if (buf_ == NULL)
            return -ENOMEM;
        heap_used = true;
    }

    len -= GTP_ADDR_LENGTH;
    while (pos != len) {
        if (len - pos > I2C_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH)
            payload_length = I2C_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH;
        else
            payload_length = len - pos;

        buf_[0] = (unsigned char)((address >> 8) & 0xFF);
        buf_[1] = (unsigned char)(address & 0xFF);
        memcpy(&buf_[2], &buf[2 + pos], payload_length);

        for (retry = 0; retry < RETRY_MAX_TIMES; retry++) {
            err = io->write(io->hndl, TOUCHPANEL_ADDR, buf_,
                             payload_length + GTP_ADDR_LENGTH);
            if (err == 0) {
                pos += payload_length;
                address += payload_length;
                break;
            }
            dev_dbg("I2C write retry[%d]\n", retry + 1);
            io->sleepMs(2100);
        }
        if (retry == RETRY_MAX_TIMES) {
            dev_err(                    "I2c write failed,dev:%02x,reg:%04x,size:%u\n",
                    TOUCHPANEL_ADDR, address, len);
            result = -EAGAIN;
            goto write_exit;
        }
    }
    
    write_exit:
    if (heap_used)
        FREE(buf_);

    return result;
}


/*
 * return touch state register value
 * pen event id fixed with 9 and set tool type TOOL_PEN
 *
 */
static u8 gtp_get_points(touchPanelH tp, struct goodix_point_t *points,
                         u8 *key_value)
{
    int ret;
    int i;
    u8 *coor_data = NULL;
    u8 finger_state = 0;
    u8 touch_num = 0;
    u8 point_data[2 + 1 + 8 * GTP_MAX_TOUCH_ID + 1] = {
        GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF
    };

    ret = gtp_io_read(&tp->cfg.io, point_data, CELLSOF(point_data));
    if (ret < 0) {
        dev_err("I2C transfer error. errno:%d\n ", ret);
        return 0;
    }

    finger_state = point_data[GTP_ADDR_LENGTH];
    if (finger_state == 0x00)
        return 0;

    touch_num = finger_state & 0x0f;
    if ((finger_state & MASK_BIT_8) == 0) {
        dev_err(                "Invalid touch state: 0x%x\n", finger_state);
        finger_state = 0;
        goto exit_get_point;
    }

    if (touch_num > 1) {
        u8 buf[8 * GTP_MAX_TOUCH_ID] = {
            (GTP_READ_COOR_ADDR + 10) >> 8,
            (GTP_READ_COOR_ADDR + 10) & 0xff
        };

        ret = gtp_io_read(&tp->cfg.io, buf, 2 + 8 * (touch_num - 1));
        if (ret < 0) {
            dev_err("I2C error. %d\n", ret);
            finger_state = 0;
            goto exit_get_point;
        }
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

    /* panel have touch key */
    /* 0x20_UPKEY 0X10_DOWNKEY 0X40_ALLKEYDOWN */
    *key_value = point_data[3 + 8 * touch_num];

    memset(points, 0, sizeof(*points) * GTP_MAX_TOUCH_ID);
    for (i = 0; i < touch_num; i++) {
        coor_data = &point_data[i * 8 + 3];
        points[i].id = coor_data[0];
        points[i].x = coor_data[1] | (coor_data[2] << 8);
        points[i].y = coor_data[3] | (coor_data[4] << 8);
        points[i].w = coor_data[5] | (coor_data[6] << 8);
            /* if pen hover points[].p must set to zero */
        points[i].p = coor_data[5] | (coor_data[6] << 8);

        dev_dbg("[%d][%d %d %d]\n",
                points[i].id, points[i].x, points[i].y, points[i].p);

            /* pen device coordinate */
        if (points[i].id & 0x80) {
            points[i].tool_type = GTP_TOOL_PEN;
            points[i].id = 10;
        } else {
            points[i].tool_type = GTP_TOOL_FINGER;
        }
    }

exit_get_point:
    clear_touch_points_status(&tp->cfg.io);
    return finger_state;
}

/**
 * @brief Fill valid touch panel configuration 
 * 
 * @param goodix_config_data config to fill
 * @param io descriptor
 * @return 0 - success, < 0 - error
 */
static int gtp_find_valid_cfg_data(struct goodix_config_data *cfg,
                                   IO_HandleT io)
{
    #define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

    int ret = -1;
    u8 sensor_id = 0;

    /* if defined CONFIG_OF, parse config data from dtsi
     * else parse config data form header file.
     */
     cfg->length = 0;

#ifndef CONFIG_OF
     u8 cfg_info_group0[] = CTP_CFG_GROUP0;
     u8 cfg_info_group1[] = CTP_CFG_GROUP1;
     u8 cfg_info_group2[] = CTP_CFG_GROUP2;
     u8 cfg_info_group3[] = CTP_CFG_GROUP3;
     u8 cfg_info_group4[] = CTP_CFG_GROUP4;
     u8 cfg_info_group5[] = CTP_CFG_GROUP5;

     u8 *send_cfg_buf[] = {
        cfg_info_group0, cfg_info_group1,
        cfg_info_group2, cfg_info_group3,
        cfg_info_group4, cfg_info_group5
     };

     u8 cfg_info_len[] = {
        CFG_GROUP_LEN(cfg_info_group0),
        CFG_GROUP_LEN(cfg_info_group1),
        CFG_GROUP_LEN(cfg_info_group2),
        CFG_GROUP_LEN(cfg_info_group3),
        CFG_GROUP_LEN(cfg_info_group4),
        CFG_GROUP_LEN(cfg_info_group5)
     };

     dev_dbg("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d\n",
             cfg_info_len[0], cfg_info_len[1], cfg_info_len[2],
             cfg_info_len[3], cfg_info_len[4], cfg_info_len[5]);
#endif

    /* read sensor id */
     ret = gtp_io_read_dbl_check(io, GTP_REG_SENSOR_ID, &sensor_id, 1);

     /** @TODO debug */
     sensor_id = 0;


     if (SUCCESS != ret || sensor_id >= 0x06) {
        dev_err("Failed get valid sensor_id(0x%02X), No Config Sent\n",
                sensor_id);
        return -EINVAL;
     }

     dev_dbg("Sensor_ID: %d\n", sensor_id);
    /* parse config data */
#ifdef CONFIG_OF
     dev_dbg("Get config data from device tree\n");
     ret = gtp_parse_dt_cfg(0,
                            &cfg->data[GTP_ADDR_LENGTH],
                            &cfg->length, sensor_id);
     if (ret < 0) {
        dev_err(                "Failed to parse config data form device tree\n");
        cfg->length = 0;
        return -EPERM;
     }
#else
     dev_dbg("Get config data from header file\n");
     if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
         (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
         (!cfg_info_len[5])) {
        sensor_id = 0;
    }
    cfg->length = cfg_info_len[sensor_id];
    memset(&cfg->data[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&cfg->data[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id],
           cfg->length);
#endif

    if (cfg->length < GTP_CONFIG_MIN_LENGTH) {
        dev_err(               "Failed get valid config data with sensor id %d\n",
               sensor_id);
        cfg->length = 0;
        return -EPERM;
    }

    dev_info("Config group %d used,length: %d\n",
             sensor_id, cfg->length);

    return 0;
}

/*******************************************************
 * Function:
 *  Get valid config data from dts or .h file.
 *  Read firmware version info and judge firmware
 *  working state
 * Input:
 *  ts: goodix private data
 * Output:
 *  Executive outcomes.
 *      0: succeed, otherwise: failed
 *******************************************************/
static s32 gtp_init_panel(struct goodix_ts_platform_data *data,
                          IO_HandleT io)
{
    s32 ret = -1;
    u8 opr_buf[16] = {0};
    u8 drv_cfg_version = 0;
    u8 flash_cfg_version = 0;
    struct goodix_config_data *cfg = &data->config;

    cfg->data[0] = GTP_REG_CONFIG_DATA >> 8;
    cfg->data[1] = GTP_REG_CONFIG_DATA & 0xff;

    /** @TODO debug */
    data->driver_send_cfg = 1;

    if (!data->driver_send_cfg) {
        dev_info("Driver set not send config\n");
        cfg->length = GTP_CONFIG_MAX_LENGTH;
        ret = gtp_io_read(io, cfg->data, cfg->length + GTP_ADDR_LENGTH);
        if (ret < 0)
            dev_err("Read origin Config Failed\n");

        return 0;
    }

    gtp_find_valid_cfg_data(&data->config, io);

    /* check firmware */
    ret = gtp_io_read_dbl_check(io, 0x41E4, opr_buf, 1);
    if (SUCCESS == ret) {
        if (opr_buf[0] != 0xBE) {
            dev_err(                    "Firmware error, no config sent!\n");
            return -EINVAL;
        }
    }

    ret = gtp_io_read_dbl_check(io, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
    if (ret == SUCCESS) {
        dev_dbg("Config Version: %d; IC Config Version: %d\n",
                cfg->data[GTP_ADDR_LENGTH], opr_buf[0]);
        flash_cfg_version = opr_buf[0];
        drv_cfg_version = cfg->data[GTP_ADDR_LENGTH];

        if (flash_cfg_version < 90 &&
            flash_cfg_version > drv_cfg_version)
            cfg->data[GTP_ADDR_LENGTH] = 0x00;
    } else {
        dev_err(                "Failed to get ic config version!No config sent\n");
        return -EPERM;
    }

    ret = gtp_send_cfg(io, data);
    if (ret < 0){
        dev_err("Send config error\n");
    }
    else{
        io->sleepMs(11); /* 10 ms */
        data->driver_send_cfg = 0;
    }

    ret = gtp_init_esd_reg(io);
    if( ret != 0)
    {
        dev_err("ESD register init error\n");
    }

    /* restore config version */
    cfg->data[GTP_ADDR_LENGTH] = drv_cfg_version;

    return 0;
}

/*******************************************************
 * Function:
 *      Send config.
 * Input:
 *      client: io device.
 * Output:
 *      result of io write operation.
 *              1: succeed, otherwise
 *              0: Not executed
 *      < 0: failed
 *********************************************************/
static s32 gtp_send_cfg(IO_HandleT io,
                        struct goodix_ts_platform_data *data)
{
    s32 ret, i;
    u8 check_sum;
    s32 retry = 0;
    struct goodix_config_data *cfg = &data->config;

    if (!cfg->length || !data->driver_send_cfg) {
        dev_info("No config data or error occurred in panel_init\n");
        return 0;
    }

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < cfg->length; i++)
        check_sum += cfg->data[i];
    cfg->data[cfg->length] = (~check_sum) + 1;

    dev_info("Driver send config\n");
    for (retry = 0; retry < RETRY_MAX_TIMES; retry++) {
        ret = gtp_io_write(io, cfg->data,
                            GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
            break;
    }

    return ret;
}

/**
 * @brief Initialize touch panel configurtion data
 *
 * @param goodix_ts_platform_data descriptor
 */
static void init_tp_data(struct goodix_ts_platform_data *data)
{
    data->slide_wakeup = false;
    data->auto_update = true;
    data->auto_update_cfg = false;
    data->type_a_report = false;
    data->esd_protect = false;
    data->max_touch_id = GTP_MAX_TOUCH_ID;
    data->abs_size_x = GTP_DEFAULT_MAX_X;
    data->abs_size_y = GTP_DEFAULT_MAX_Y;
    data->max_touch_width = GTP_DEFAULT_MAX_WIDTH;
    data->max_touch_pressure = GTP_DEFAULT_MAX_PRESSURE;
}

/**
 * @brief Test touch panel connection
 *
 * @param io descriptor
 * @return error
 */
static int gtp_io_test(IO_HandleT io)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    int ret = -1;

    while (retry++ < 3) {
        ret = gtp_io_read(io, test, 3);
        if (ret == 2)
            return 0;

        dev_err("GTP io test failed time %d\n", retry);
        io->sleepMs(10); /* 10 ms */
    }

    return -EAGAIN;
}

/**
 * @brief Get firmware information
 *
 * @param io_client descriptor
 * @param goodix_fw_info descriptor to fill
 *
 * @return error
 */
static s32 gtp_get_fw_info(IO_HandleT io,
                           struct goodix_fw_info *fw_info)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    ret = gtp_io_read(io, buf, sizeof(buf));
    if (ret < 0) {
        dev_err("Failed read fw_info\n");
        return ret;
    }
    /* product id */
    memset(fw_info, 0, sizeof(*fw_info));

    if (buf[5] == 0x00) {
        memcpy(fw_info->pid, buf + GTP_ADDR_LENGTH, 3);
        dev_info("IC Version: %c%c%c_%02X%02X\n",
                 buf[2], buf[3], buf[4], buf[7], buf[6]);
    } else {
        memcpy(fw_info->pid, buf + GTP_ADDR_LENGTH, 4);
        dev_info("IC Version: %c%c%c%c_%02X%02X\n",
                 buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }

    /* current firmware version */
    fw_info->version = (buf[7] << 8) | buf[6];

    /* read sensor id */
    fw_info->sensor_id = 0xff;
    ret = gtp_io_read_dbl_check(io, GTP_REG_SENSOR_ID,
                                 &fw_info->sensor_id, 1);
    if (SUCCESS != ret || fw_info->sensor_id >= 0x06) {
        dev_err(                "Failed get valid sensor_id(0x%02X), No Config Sent\n",
                fw_info->sensor_id);

        fw_info->sensor_id = 0xff;
    }

    return ret;
}

/**
 * @brief Set bit
 * @details @TODO non-atomic
 *
 * @param pos bit position
 * @param addr address
 */
static inline void set_bit(long pos, volatile unsigned long *addr) {
    *addr |= 1 << pos;
}

/**
 * @brief Clear bit
 * @details @TODO non-atomic
 *
 * @param pos bit position
 * @param addr address
 */
static inline void clear_bit(long pos, volatile unsigned long *addr)
{
    *addr &= ~(1 << pos);
}

/**
 * @brief Test bit
 * @details @TODO non-atomic
 *
 * @param pos bit position
 * @param addr address
 */
static inline bool test_bit(long pos, volatile unsigned long *addr) {
    return *addr & (1 << pos);
}

/**
 * @brief Touch panel gesture handle
 * 
 * @param tp touch panel descriptor
 * @return error
 */
static int gtp_gesture_handler(touchPanelH tp)
{
    u8 doze_buf[3] = {GTP_REG_DOZE_BUF >> 8, GTP_REG_DOZE_BUF & 0xFF};
    int ret;

    ret = gtp_io_read(&tp->cfg.io, doze_buf, 3);
    if (ret < 0) {
        dev_err("Failed read doze buf\n");
        return -EINVAL;
    }

    dev_dbg("0x814B = 0x%02X\n", doze_buf[2]);
    if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') ||
        (doze_buf[2] == 'c') || (doze_buf[2] == 'd') ||
        (doze_buf[2] == 'e') || (doze_buf[2] == 'g') ||
        (doze_buf[2] == 'h') || (doze_buf[2] == 'm') ||
        (doze_buf[2] == 'o') || (doze_buf[2] == 'q') ||
        (doze_buf[2] == 's') || (doze_buf[2] == 'v') ||
        (doze_buf[2] == 'w') || (doze_buf[2] == 'y') ||
        (doze_buf[2] == 'z') || (doze_buf[2] == 0x5E) ||
        (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xAB) ||
        (doze_buf[2] == 0xBA) || (doze_buf[2] == 0xBB) ||
        (doze_buf[2] == 0xCC)) {
        /** @TODO report power up */
        /*  clear 0x814B */
        doze_buf[2] = 0x00;
    gtp_io_write(&tp->cfg.io, doze_buf, 3);
} else {
        /*  clear 0x814B */
    doze_buf[2] = 0x00;
    gtp_io_write(&tp->cfg.io, doze_buf, 3);
    gtp_enter_doze(tp);
}
return 0;
}

/*******************************************************
 * Function:
 *  Enter doze mode for sliding wakeup.
 * Input:
 *  ts: goodix tp private data
 * Output:
 *  1: succeed, otherwise failed
 *******************************************************/
static int gtp_enter_doze(touchPanelH tp)
{
    int ret = -1;
    int retry = 0;
    u8 io_control_buf[3] = { (u8)(GTP_REG_COMMAND >> 8),
      (u8)GTP_REG_COMMAND, 8 };

      set_bit(DOZE_MODE, &tp->flags);
      dev_dbg("Entering gesture mode.\n");
      while (retry++ < 5) {
        io_control_buf[0] = (u8)(GTP_REG_COMMAND_CHECK >> 8);
        io_control_buf[1] = (u8)GTP_REG_COMMAND_CHECK;
        ret = gtp_io_write(&tp->cfg.io, io_control_buf, 3);
        if (ret < 0) {
            dev_dbg("failed to set doze flag into 0x8046, %d\n",
                    retry);
            continue;
        }
        io_control_buf[0] = (u8)(GTP_REG_COMMAND >> 8);
        io_control_buf[1] = (u8)GTP_REG_COMMAND;
        ret = gtp_io_write(&tp->cfg.io, io_control_buf, 3);
        if (ret > 0) {
            dev_dbg("Gesture mode enabled\n");
            return ret;
        }
        tp->cfg.io.sleepMs(10);
    }

    dev_err("Failed enter doze mode\n");
    clear_bit(DOZE_MODE, &tp->flags);
    return ret;
}

/**
 * @brief Touch panel state report
 * 
 * @param goodix_ts_data [description]
 * @param touch_num [description]
 * @param goodix_point_t [description]
 */
static void gtp_show_touches(touchPanelH tp, u8 touch_num,
                             struct goodix_point_t points[])
{
    int i;
    u16 cur_touch = 0;
    static u16 pre_touch;
    static u8 pre_pen_id;

    for (i = 0; i < tp->data.max_touch_id; i++) {
        if (touch_num && i == points->id) {

            if (points->tool_type == GTP_TOOL_PEN) {
                dev_dbg("Pen is activated\n");
                pre_pen_id = points->id;
            } else {
                dev_dbg("Finger is activated\n");
            }

            dev_dbg("x = %d, y = %d\nw = %d, p = %d\n",
                    points->x, points->y, points->w, points->p);

            cur_touch |= 0x01 << points->id;
            points++;
        } else if (pre_touch & 0x01 << i) {
            if (pre_pen_id == i) {
                dev_dbg("Pen is deactivated\n");
                /* valid id will < 10, so id to 0xff to indicate a invalid state */
                pre_pen_id = 0xff;
            } else {
                dev_dbg("Finger is deactivated\n");
            }
        }
    }

    pre_touch = cur_touch;
    if (!pre_touch) {
        dev_dbg("Button is deactivated\n");
    }
}

/**
 * @brief Clear read status
 * 
 * @param io descriptor
 */
static void clear_touch_points_status(IO_HandleT io){
    int ret = gtp_io_write(io, 
                            (uint8_t [3]){ 
                                GTP_READ_COOR_ADDR >> 8,
                                GTP_READ_COOR_ADDR & 0xFF, 0
                            },
                            3);
    if (ret < 0)
    {
        dev_dbg("I2C write end_cmd error!\n");
    }
}

/*******************************************************
 * Function:
 *  Initialize external watchdog for esd protect
 * Input:
 *  client: io device.
 * Output:
 *  result of io write operation.
 *      0: succeed, otherwise: failed
 ********************************************************/
static int gtp_init_esd_reg(IO_HandleT io)
{
    int ret;
    const u8 opr_buffer[3] = { (u8)(GTP_REG_ESD_CHECK >> 8),
                               (u8)GTP_REG_ESD_CHECK,
                               (u8)GTP_ESD_CHECK_VALUE };

    dev_dbg("[Esd]Init external watchdog\n");
    ret = gtp_io_write(io, opr_buffer, 3);
    if (ret == 1)
        return 0;

    dev_err("Failed init ext watchdog\n");
    return -EINVAL;
}

/**
 * @brief Check io instance
 * 
 * @param io descriptor
 * @return true - success, false - fail
 */
static bool check_io_instance(IO_HandleT io)
{
    IF_FAILED_RETURN_EXPR(io->hndl != NULL, false);
    IF_FAILED_RETURN_EXPR(io->read != NULL, false);
    IF_FAILED_RETURN_EXPR(io->write != NULL, false);
    
    return true;
}

/**
 * @brief Get touch count
 * 
 * @param state touch panel state
 */
static inline size_t get_touch_count(uint32_t state)
{
    return state & 0x0F;
}
