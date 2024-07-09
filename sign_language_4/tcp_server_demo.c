#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"
#include "ohos_init.h"

#include "lwip/sockets.h"
#include "wifi_connect.h"

#include <dtls_al.h>
#include <mqtt_al.h>
#include <oc_mqtt_al.h>
#include <oc_mqtt_profile.h>
#include "E53_SC2.h"

//ADC的头文件
#include "iot_adc.h"
#include "iot_errno.h"
#include <math.h>
#include "iot_gpio_ex.h"

#include "iot_watchdog.h"
#include "hi_io.h"   //上拉、复�?
#include "hi_gpio.h" //hi_gpio_set_dir()、hi_gpio_set(get)_output(input)_val()
#include "iot_gpio.h"//gpioInit
#include "hi_time.h"
#include "hi_i2c.h"

#include "uart/uart_control.h"

#define CONFIG_WIFI_SSID "yuizhu" // 修改为自己的WiFi 热点账号

#define CONFIG_WIFI_PWD "20040906" // 修改为自己的WiFi 热点密码

#define CONFIG_APP_SERVERIP "117.78.5.125"

#define CONFIG_APP_SERVERPORT "1883"

#define CONFIG_APP_DEVICEID "65d7ed4071d845632afd3d84_20240316" // 替换为注册设备后生成的deviceid

#define CONFIG_APP_DEVICEPWD "zhangran20040906" // 替换为注册设备后生成的密�?

#define CONFIG_APP_LIFETIME 60 // < seconds

#define CONFIG_QUEUE_TIMEOUT (5 * 1000)

#define MSGQUEUE_COUNT 16
#define MSGQUEUE_SIZE 10
#define CLOUD_TASK_STACK_SIZE (1024 * 10)
#define CLOUD_TASK_PRIO 24
#define SENSOR_TASK_STACK_SIZE (1024 * 4)
#define SENSOR_TASK_PRIO 25
#define TASK_DELAY 3
#define FLIP_THRESHOLD 100

#define Flex_1_ADC_CHANNEL_1 1
#define Flex_2_ADC_CHANNEL_0 0
#define Flex_3_ADC_CHANNEL_3 3
#define Flex_4_ADC_CHANNEL_4 4
#define Flex_5_ADC_CHANNEL_5 5

#define Flex_1_GPIO 4
#define Flex_2_GPIO 12
#define Flex_3_GPIO 7
#define Flex_4_GPIO 9
#define Flex_5_GPIO 11

#define ADC_TASK_DELAY_1S 1000000

#define ADC_VREF_VOL 1.8
#define ADC_COEFFICIENT 4
#define ADC_RATIO 4096

#define TASK_STACK_SIZE (1024 * 10)
#define TASK_DELAY_2S 2
#define CONFIG_WIFI_SSID "yuizhu"    // 要连接的WiFi 热点账号
#define CONFIG_WIFI_PWD "20040906"  // 要连接的WiFi 热点密码
#define CONFIG_CLIENT_PORT 8888      // 要连接的服务器端�?
 #define TCP_BACKLOG 10

c har recvbuf[256];
char *buf_ni = "ni";
char *buf_hao = "hao";
char *buf_xiexie = "xiexie";
char *buf_lihai = "lihai";
char *buf_baibai = "baibai";
char *buf_haode = "haode";
char *buf_bushi = "bushi";
char *buf_wocuole = "wocuole";
int new_fd;

    int flex_1;
    int flex_2;
    int flex_3;
    int flex_4;
    int flex_5;

    int ACCEL_x[100],
        ACCEL_y[100],
        ACCEL_z[100],
        GYRO_x[100],
        GYRO_y[100],
        GYRO_z[100];  //����mpu6050�������飬��Ϊ�˶����ݵĻ�����

osMessageQueueId_t mid_MsgQueue;

osMutexId_t muxe_id;     //创建互斥锁ID

osSemaphoreId_t ID;

//这里是定义枚举，在CloudMainTaskEntry线程函数中做switch的case选项
typedef enum {
    en_msg_cmd = 0,
    en_msg_mpu6050_report,
    en_msg_flex_report,
    en_msg_conn,
    en_msg_disconn,
} en_msg_type_t;

//云平台的请求参数，在msg_rcv_callback函数中使�?
typedef struct {
    char *request_id;
    char *payload;
} cmd_t;

//这里定义mpu6050消息上报的结构体
typedef struct {
    int temp;
    int acce_x;
    int acce_y;
    int acce_z;
    int gyro_x;
    int gyro_y;
    int gyro_z;
    char band_1[10000];
} report_mpu6050_t;

//这里定义max30102消息上报的结构体
typedef struct {
    int flex_1;
    int flex_2;
    int flex_3;
    int flex_4;
    int flex_5;
}report_flex_t;

//这里是定义消息打包的ID
typedef struct {
    en_msg_type_t msg_type;
    union {
        cmd_t cmd;
        report_mpu6050_t report_mpu6050;
        report_flex_t report_flex;
    } msg;
} app_msg_t;

//定义程序需要的参数变量
typedef struct {
    osMessageQueueId_t app_msg;
    int connected;
    int mpu6050;       //mpu6050的命令下发状态参数，类似于mpu6050.value = g_app_cb.mpu6050 ? "ON" : "OFF";
    int mpu6050_state;   //mpu6050传感器的状态参�?
    int max30102;      //max30102的命令下发状态参数，类似于max30102.value = g_app_cb.max30102 ? "ON" : "OFF";
    int max30102_state;  //max30102传感器的状态参�?

    int flag_timer_1;     //判断运动检测时间是否到�?
    int flag_timer_2;     //判断心率检测时间是否到�?
} app_cb_t;

static app_cb_t g_app_cb;
int g_coverStatus = 1;
int state = 0;
osTimerId_t ID1,ID2;

static void flex_report_msg(report_flex_t *report_flex)
{
    oc_mqtt_profile_service_t service;
    oc_mqtt_profile_kv_t Flex_1;
    oc_mqtt_profile_kv_t Flex_2;
    oc_mqtt_profile_kv_t Flex_3;
    oc_mqtt_profile_kv_t Flex_4;
    oc_mqtt_profile_kv_t Flex_5;
    oc_mqtt_profile_kv_t Flex;

    if (g_app_cb.connected != 1) {
        return;
    }

    service.event_time = NULL;
    service.service_id = "sign_language";
    service.service_property = &Flex_1;
    service.nxt = NULL;

    Flex_1.key = "Flex_1";
    Flex_1.value = &report_flex->flex_1;
    Flex_1.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Flex_1.nxt = &Flex_2;

    Flex_2.key = "Flex_2";
    Flex_2.value = &report_flex->flex_2;
    Flex_2.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Flex_2.nxt = &Flex_3;

    Flex_3.key = "Flex_3";
    Flex_3.value = &report_flex->flex_1;
    Flex_3.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Flex_3.nxt = &Flex_4;

    Flex_4.key = "Flex_4";
    Flex_4.value = &report_flex->flex_1;
    Flex_4.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Flex_4.nxt = &Flex_5;

    Flex_5.key = "Flex_5";
    Flex_5.value = &report_flex->flex_1;
    Flex_5.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Flex_5.nxt = &Flex;

    Flex.key = "FlexStatus";
    Flex.value = g_app_cb.max30102 ? "ON" : "OFF";
    Flex.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Flex.nxt = NULL;

    oc_mqtt_profile_propertyreport(NULL, &service);
    return;
}

static void mpu6050_report_msg(report_mpu6050_t *report_mpu6050)
{
    oc_mqtt_profile_service_t service;
    oc_mqtt_profile_kv_t temperature;
    oc_mqtt_profile_kv_t Accel_x;
    oc_mqtt_profile_kv_t Accel_y;
    oc_mqtt_profile_kv_t Accel_z;
    oc_mqtt_profile_kv_t Gyro_x;
    oc_mqtt_profile_kv_t Gyro_y;
    oc_mqtt_profile_kv_t Gyro_z;
    oc_mqtt_profile_kv_t voltage_1;
    oc_mqtt_profile_kv_t mpu6050;
    

    if (g_app_cb.connected != 1) {
        return;
    }

    service.event_time = NULL;
    service.service_id = "sign_language";
    service.service_property = &temperature;
    service.nxt = NULL;

    temperature.key = "Temperature";
    temperature.value = &report_mpu6050->temp;
    temperature.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    temperature.nxt = &Accel_x;

    Accel_x.key = "Accel_x";
    Accel_x.value = &report_mpu6050->acce_x;
    Accel_x.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Accel_x.nxt = &Accel_y;

    Accel_y.key = "Accel_y";
    Accel_y.value = &report_mpu6050->acce_y;
    Accel_y.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Accel_y.nxt = &Accel_z;

    Accel_z.key = "Accel_z";
    Accel_z.value = &report_mpu6050->acce_z;
    Accel_z.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Accel_z.nxt = &Gyro_x;

    Gyro_x.key = "Gyro_x";
    Gyro_x.value = &report_mpu6050->gyro_x;
    Gyro_x.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Gyro_x.nxt = &Gyro_y;

    Gyro_y.key = "Gyro_y";
    Gyro_y.value = &report_mpu6050->gyro_y;
    Gyro_y.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Gyro_y.nxt = &Gyro_z;

    Gyro_z.key = "Gyro_z";
    Gyro_z.value = &report_mpu6050->gyro_z;
    Gyro_z.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    Gyro_z.nxt = &voltage_1;

    voltage_1.key = "Angle_1";
    voltage_1.value = &report_mpu6050->band_1;
    voltage_1.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    voltage_1.nxt = &mpu6050;

    mpu6050.key = "MPU6050Status";
    mpu6050.value = g_app_cb.mpu6050 ? "ON" : "OFF";
    mpu6050.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    mpu6050.nxt = NULL;

    oc_mqtt_profile_propertyreport(NULL, &service);
    return;
}


// use this function to push all the message to the buffer
static int msg_rcv_callback(oc_mqtt_profile_msgrcv_t *msg)
{
    int ret = 0;
    char *buf;
    int buf_len;
    app_msg_t *app_msg;

    if ((msg == NULL) || (msg->request_id == NULL) || (msg->type != EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_COMMANDS)) {
        return ret;
    }

    buf_len = sizeof(app_msg_t) + strlen(msg->request_id) + 1 + msg->msg_len + 1;
    buf = malloc(buf_len);
    if (buf == NULL) {
        return ret;
    }
    app_msg = (app_msg_t *)buf;
    buf += sizeof(app_msg_t);

    app_msg->msg_type = en_msg_cmd;
    app_msg->msg.cmd.request_id = buf;
    buf_len = strlen(msg->request_id);
    buf += buf_len + 1;
    memcpy_s(app_msg->msg.cmd.request_id, buf_len, msg->request_id, buf_len);
    app_msg->msg.cmd.request_id[buf_len] = '\0';

    buf_len = msg->msg_len;
    app_msg->msg.cmd.payload = buf;
    memcpy_s(app_msg->msg.cmd.payload, buf_len, msg->msg, buf_len);
    app_msg->msg.cmd.payload[buf_len] = '\0';

    ret = osMessageQueuePut(g_app_cb.app_msg, &app_msg, 0U, CONFIG_QUEUE_TIMEOUT);
    if (ret != 0) {
        free(app_msg);
    }

    return ret;
}

static void oc_cmdresp(cmd_t *cmd, int cmdret)
{
    oc_mqtt_profile_cmdresp_t cmdresp;
    ///< do the response
    cmdresp.paras = NULL;
    cmdresp.request_id = cmd->request_id;
    cmdresp.ret_code = cmdret;
    cmdresp.ret_name = NULL;
    (void)oc_mqtt_profile_cmdresp(NULL, &cmdresp);
}

///< COMMAND DEAL
#include <cJSON.h>

/*
    函数名：deal_mpu6050_cmd
    参数：cmd_t *cmd, cJSON *obj_root
    作用：用来开启或关闭mpu6050的传感器，本来的mpu6050传感器线程函数是开启的，只不过我用if+状态参数跳过了数据采集与上�?
*/
static void deal_mpu6050_cmd(cmd_t *cmd, cJSON *obj_root)
{
    cJSON *obj_paras;
    cJSON *obj_para;
    int cmdret;

    obj_paras = cJSON_GetObjectItem(obj_root, "paras");
    if (obj_paras == NULL) {
        cJSON_Delete(obj_root);
    }
    obj_para = cJSON_GetObjectItem(obj_paras, "kneading_control");      //对应华为云IoTDA的产品—�?>模型的命令名称—�?>下发参数
    if (obj_paras == NULL) {
        cJSON_Delete(obj_root);
    }
    ///< operate the LED here
    if (strcmp(cJSON_GetStringValue(obj_para), "ON") == 0) {
        g_app_cb.mpu6050 = 1;           //对应消息队列里定义的ON
            g_app_cb.mpu6050_state = 1; //打开mpu6050传感�?
            printf("mpu6050传感�? On!");
    } else {
        g_app_cb.mpu6050 = 0;           //对应消息队列里定义的OFF
            g_app_cb.mpu6050_state = 0; //打开mpu6050传感�?
            printf("mpu6050传感�? Off!");
    }
    cmdret = 0;
    oc_cmdresp(cmd, cmdret);

    cJSON_Delete(obj_root);
    return;
}

/*
    函数名：deal_max30102_cmd
    参数：cmd_t *cmd, cJSON *obj_root
    作用：用来开启或关闭max30102的传感器，本来的max30102传感器线程函数是开启的，只不过我用if+状态参数跳过了数据采集与上�?
*/
static void deal_flex_cmd(cmd_t *cmd, cJSON *obj_root)
{
    cJSON *obj_paras;
    cJSON *obj_para;
    int cmdret;

    obj_paras = cJSON_GetObjectItem(obj_root, "Paras");
    if (obj_paras == NULL) {
        cJSON_Delete(obj_root);
    }
    obj_para = cJSON_GetObjectItem(obj_paras, "Max30102_Begin_control");    //对应华为云IoTDA的产品—�?>模型的命令名称—�?>下发参数
    if (obj_para == NULL) {
        cJSON_Delete(obj_root);
    }
    ///< operate the Motor here
    if (strcmp(cJSON_GetStringValue(obj_para), "ON") == 0) {
        g_app_cb.max30102 = 1;
            g_app_cb.max30102_state = 1;
            printf("max30102传感�? On!");
    } else {
        g_app_cb.max30102 = 0;
            g_app_cb.max30102_state = 0;
            printf("max30102传感�? Off!");
    }
    cmdret = 0;
    oc_cmdresp(cmd, cmdret);

    cJSON_Delete(obj_root);
    return;
}

/*
    函数名：deal_cmd_msg
    参数：cmd_t *cmd
    作用：定义命令下发的命令名称，同时可以调用对应的命令执行函数
*/
static void deal_cmd_msg(cmd_t *cmd)
{
    cJSON *obj_root;
    cJSON *obj_cmdname;

    int cmdret = 1;
    obj_root = cJSON_Parse(cmd->payload);
    if (obj_root == NULL) {
        oc_cmdresp(cmd, cmdret);
    }
    obj_cmdname = cJSON_GetObjectItem(obj_root, "command_name");
    if (obj_cmdname == NULL) {
        cJSON_Delete(obj_root);
    }
    if (strcmp(cJSON_GetStringValue(obj_cmdname), "Kneading") == 0) {       //对应华为云IoTDA的产品—�?>模型的命令名�?
        deal_mpu6050_cmd(cmd, obj_root);
    } else if (strcmp(cJSON_GetStringValue(obj_cmdname), "Max30102_Begin") == 0) {  //对应华为云IoTDA的产品—�?>模型的命令名�?
        deal_flex_cmd(cmd, obj_root);
    }
    return;
} 

static int CloudMainTaskEntry(void)
{
    app_msg_t *app_msg;
    uint32_t ret;
    WifiConnect(CONFIG_WIFI_SSID, CONFIG_WIFI_PWD);
    dtls_al_init();
    mqtt_al_init();
    oc_mqtt_init();

    g_app_cb.app_msg = osMessageQueueNew(MSGQUEUE_COUNT, MSGQUEUE_SIZE, NULL);
    if (g_app_cb.app_msg == NULL) {
        printf("Create receive msg queue failed");
    }
    oc_mqtt_profile_connect_t connect_para;
    (void)memset_s(&connect_para, sizeof(connect_para), 0, sizeof(connect_para));

    connect_para.boostrap = 0;
    connect_para.device_id = CONFIG_APP_DEVICEID;
    connect_para.device_passwd = CONFIG_APP_DEVICEPWD;
    connect_para.server_addr = CONFIG_APP_SERVERIP;
    connect_para.server_port = CONFIG_APP_SERVERPORT;
    connect_para.life_time = CONFIG_APP_LIFETIME;
    connect_para.rcvfunc = msg_rcv_callback;
    connect_para.security.type = EN_DTLS_AL_SECURITY_TYPE_NONE;
    ret = oc_mqtt_profile_connect(&connect_para);
    if ((ret == (int)en_oc_mqtt_err_ok)) {
        g_app_cb.connected = 1;
        printf("oc_mqtt_profile_connect succed!\r\n");
    } else {
        printf("oc_mqtt_profile_connect faild!\r\n");
    }

    //检测命令下�?
    while (1) {
        app_msg = NULL;
        (void)osMessageQueueGet(g_app_cb.app_msg, (void **)&app_msg, NULL, 0xFFFFFFFF);
        if (app_msg != NULL) {
            switch (app_msg->msg_type) {
                case en_msg_cmd:
                    deal_cmd_msg(&app_msg->msg.cmd);  //主的命令名称
                    break;
                case en_msg_mpu6050_report:
                    mpu6050_report_msg(&app_msg->msg.report_mpu6050);  //副的下发参数
                    break;
                case en_msg_flex_report:
                    flex_report_msg(&app_msg->msg.report_flex); //副的下发参数
                    break;
                default:
                    break;
            }
            free(app_msg); //释放堆空�?
        }
    }
    
    return 0;
}

float map(float val, float I_Min, float I_Max, float O_Min, float O_Max)
{
    return(((val-I_Min)*((O_Max-O_Min)/(I_Max-I_Min)))+O_Min);
}

static float GetFlex_1(void)
{
    unsigned int ret;
    unsigned short data;
    unsigned short * return_flex;

    ret = IoTAdcRead(Flex_1_ADC_CHANNEL_1, &data, IOT_ADC_EQU_MODEL_8, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        printf("ADC Read Fail\n");
    }
    printf("-----:%d\n",data);
    return_flex = &data;
    map(*return_flex,300,820,0,180);
    return *return_flex;
}

static float GetFlex_2(void)
{
    unsigned int ret;
    unsigned short data;
    unsigned short * return_flex;

    ret = IoTAdcRead(Flex_2_ADC_CHANNEL_0, &data, IOT_ADC_EQU_MODEL_8, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        printf("ADC Read Fail\n");
    }
    printf("-----:%d\n",data);
    return_flex = &data;
    map(*return_flex,300,820,0,180);
    return *return_flex;
}

static float GetFlex_3(void)
{
    unsigned int ret;
    unsigned short data;
    unsigned short * return_flex;

    ret = IoTAdcRead(Flex_3_ADC_CHANNEL_3, &data, IOT_ADC_EQU_MODEL_8, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        printf("ADC Read Fail\n");
    }
    printf("-----:%d\n",data);
    return_flex = &data;
    map(*return_flex,300,820,0,180);
    return *return_flex;
}

static float GetFlex_4(void)
{
    unsigned int ret;
    unsigned short data;
    unsigned short * return_flex;

    ret = IoTAdcRead(Flex_4_ADC_CHANNEL_4, &data, IOT_ADC_EQU_MODEL_8, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        printf("ADC Read Fail\n");
    }
    printf("-----:%d\n",data);
    return_flex = &data;
    map(*return_flex,300,820,0,180);
    return *return_flex;
}

static float GetFlex_5(void)
{
    unsigned int ret;
    unsigned short data;

    unsigned short * return_flex;

    ret = IoTAdcRead(Flex_5_ADC_CHANNEL_5, &data, IOT_ADC_EQU_MODEL_8, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        printf("ADC Read Fail\n");
    }
    printf("-----:%d\n",data);
    return_flex = &data;
    map(*return_flex,300,820,0,180);
    return *return_flex;
}

static void OLED_Init()
{
    IoTGpioInit(13);
    IoTGpioInit(14);
    hi_io_set_func(13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(14, HI_IO_FUNC_GPIO_14_I2C0_SCL);

    my_oled_demo();
}

static void flex_SensorTask()
{
    osDelay(500U);
    app_msg_t *app_msg;
    uint8_t ret;

    IoTGpioSetPull(Flex_1_GPIO, IOT_GPIO_PULL_UP);
    IoTGpioSetPull(Flex_2_GPIO, IOT_GPIO_PULL_UP);
    IoTGpioSetPull(Flex_3_GPIO, IOT_GPIO_PULL_UP);
    IoTGpioSetPull(Flex_4_GPIO, IOT_GPIO_PULL_UP);
    IoTGpioSetPull(Flex_5_GPIO, IOT_GPIO_PULL_UP);
    OLED_Init();
    OLED_ni();
    UartTask("[v8][t5]��ӭʹ������֮������");
        while (1) {
            flex_1 = GetFlex_1();
            flex_2 = GetFlex_2();
            flex_3 = GetFlex_3();
            flex_4 = GetFlex_4();
            flex_5 = GetFlex_5();

            usleep(1000000);

            if (flex_1 < 900 && (flex_2 > 1200 && flex_2 < 1500) && flex_3 < 800 && flex_4 < 1200 && flex_5 < 1100)
                {
                    OLED_ni();
                    UartTask("[v13][t5]��");
                } 

                else if ((flex_1 > 1000 && flex_1 < 1200) && flex_2 < 1100 && flex_3 < 800 && flex_4 < 1200 && flex_5 < 1000)
                {
                    OLED_hao();
                    UartTask("[v10][t5]��");
                } 

                else if (flex_1 < 610 && flex_2 < 760 && flex_3 < 780 && flex_4 < 720 && flex_5 < 644)
                {
                    OLED_xiexie();
                    UartTask("[v8][t5]лл");
                } 

                else if ((flex_1>1000 && flex_1 < 1200) && flex_2 < 1100 && flex_3 < 780 && flex_4 < 1200 && (flex_5 > 1000 && flex_5 < 1100))
                {
                    OLED_lihai();
                    UartTask("[v8][t5]����");
                }

                else if ((flex_1 > 1000 && flex_1 < 1080) && (flex_2 > 1100 && flex_2 < 1270) && (flex_3 > 400 && flex_3 < 1050) && (flex_4 > 1150 && flex_4 < 1270) && (flex_5 > 1050 && flex_5 < 1170))
                {
                    OLED_baibai();
                    UartTask("[v8][t5]�ݰ�");
                }

                else if (flex_1 < 1000 && flex_2 < 1250 && (flex_3 > 400 && flex_3 < 1126) && (flex_4 > 1200 && flex_4 < 1400) && (flex_5 > 1050 && flex_5 < 1250))
                {
                    OLED_haode();
                    UartTask("[v8][t5]�õ�");
                }

                else if (flex_1 < 970 && (flex_2 > 1200 && flex_2 < 1400) && (flex_3 > 400 && flex_3 < 1126) && flex_4 < 1130 && flex_5 < 1100)
                {
                    OLED_bushi();
                    UartTask("[v10][t5]����");
                }

                else if (flex_1 < 970 && flex_2 < 1200 && flex_3 < 500 && flex_4 < 1100 && (flex_5 > 1050 && flex_5 < 1200))
                {
                    OLED_wocuole();
                    UartTask("[v11][t5]�Ҵ���");
                }

            app_msg = malloc(sizeof(app_msg_t));
            if (app_msg != NULL) {
                printf("wwww");
                app_msg->msg_type = en_msg_flex_report;
                app_msg->msg.report_flex.flex_1 = (int)flex_1;
                app_msg->msg.report_flex.flex_2 = (int)flex_2;
                app_msg->msg.report_flex.flex_3 = (int)flex_3;
                app_msg->msg.report_flex.flex_4 = (int)flex_4;
                app_msg->msg.report_flex.flex_5 = (int)flex_5;
                if (osMessageQueuePut(g_app_cb.app_msg, &app_msg, 0U, CONFIG_QUEUE_TIMEOUT) != 0) {
                free(app_msg);
                }
                
            }
    }
}

static int SensorTaskEntry(void)
{
    app_msg_t *app_msg;
    uint8_t ret;
    E53SC2Data data;
    int X = 0, Y = 0, Z = 0;
    ret = E53SC2Init();
    if (ret != 0) {
        printf("E53_SC2 Init failed!\r\n");
        return;
    }

    while (1)
    {
        ret = E53SC2ReadData(&data);
        if (ret != 0) {
            printf("E53_SC2 Read Data!\r\n");
            return;
        }
        int i = 0,j;
        printf("\r\n**************Temperature      is  %d\r\n", (int)data.Temperature);
        ACCEL_x[i]=data.Accel[ACCEL_X_AXIS];
        printf("\r\n**************Accel[0]         is  %lf\r\n", data.Accel[ACCEL_X_AXIS]);
        ACCEL_y[i]=data.Accel[ACCEL_Y_AXIS];
        printf("\r\n**************Accel[1]         is  %lf\r\n", data.Accel[ACCEL_Y_AXIS]);
        ACCEL_z[i]=data.Accel[ACCEL_Z_AXIS];
        printf("\r\n**************Accel[2]         is  %lf\r\n", data.Accel[ACCEL_Z_AXIS]);
        GYRO_x[i]=data.Gyro[ACCEL_X_AXIS];
        printf("\r\n**************Gyro[0]         is  %lf\r\n", data.Gyro[ACCEL_X_AXIS]);
        GYRO_y[i]=data.Gyro[ACCEL_Y_AXIS];
        printf("\r\n**************Gyro[1]         is  %lf\r\n", data.Gyro[ACCEL_Y_AXIS]);
        GYRO_z[i]=data.Gyro[ACCEL_Z_AXIS];
        printf("\r\n**************Gyro[2]         is  %lf\r\n", data.Gyro[ACCEL_Z_AXIS]);

        if (X == 0 && Y == 0 && Z == 0) {
            X = (int)data.Accel[ACCEL_X_AXIS];
            Y = (int)data.Accel[ACCEL_Y_AXIS];
            Z = (int)data.Accel[ACCEL_Z_AXIS];
        } else {
            if (X + FLIP_THRESHOLD < data.Accel[ACCEL_X_AXIS] || X - FLIP_THRESHOLD > data.Accel[ACCEL_X_AXIS] ||
                Y + FLIP_THRESHOLD < data.Accel[ACCEL_Y_AXIS] || Y - FLIP_THRESHOLD > data.Accel[ACCEL_Y_AXIS] ||
                Z + FLIP_THRESHOLD < data.Accel[ACCEL_Z_AXIS] || Z - FLIP_THRESHOLD > data.Accel[ACCEL_Z_AXIS]) {
                LedD1StatusSet(OFF);
                LedD2StatusSet(ON);
                g_coverStatus = 1;
            } else {
                LedD1StatusSet(ON);
                LedD2StatusSet(OFF);
                g_coverStatus = 0;
            }
        }

        app_msg = malloc(sizeof(app_msg_t));
        if (app_msg != NULL) {
            for (j=0; j<=i; j++)
            {
                printf("\nj:%d",j);
                printf("\nAccel_x:%d",ACCEL_x[j]);
                app_msg->msg_type = en_msg_mpu6050_report;
            app_msg->msg.report_mpu6050.temp = (int)data.Temperature;
            app_msg->msg.report_mpu6050.acce_x = ACCEL_x[j];
            app_msg->msg.report_mpu6050.acce_y = ACCEL_y[j];
            app_msg->msg.report_mpu6050.acce_z = ACCEL_z[j];
            app_msg->msg.report_mpu6050.gyro_x = GYRO_x[j];
            app_msg->msg.report_mpu6050.gyro_y = GYRO_y[j];
            app_msg->msg.report_mpu6050.gyro_z = GYRO_z[j];
            if (osMessageQueuePut(g_app_cb.app_msg, &app_msg, 0U, CONFIG_QUEUE_TIMEOUT) != 0) {
                free(app_msg);
            }
            osDelay(50U);
            } 
        }
    }
    sleep(TASK_DELAY);   
    return 0;
}


static void TCPServerDemo(void)
{
    osThreadAttr_t attr;

    attr.name = "ServerTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = TASK_STACK_SIZE;
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)SensorTaskEntry, NULL, &attr) == NULL) {
        printf("Failed to create SensorTaskEntry!\n");
    }

    attr.stack_size = CLOUD_TASK_STACK_SIZE;
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)CloudMainTaskEntry, NULL, &attr) == NULL) {
        printf("Failed to create CloudMainTaskEntry!\n");
    }

    attr.stack_size = SENSOR_TASK_STACK_SIZE;
    attr.priority = 24;
    attr.name = "SensorTaskEntry";
    if (osThreadNew((osThreadFunc_t)flex_SensorTask, NULL, &attr) == NULL) {
        printf("Failed to create SensorTaskEntry!\n");
    }

}
APP_FEATURE_INIT(TCPServerDemo);
