#ifndef __NRF_H
#define __NRF_H
#include "main.h"

/*����֡ͷ*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*����֡ͷ*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 30

#define ASSERT(e)  if (e) ; else assertFail( #e, __FILE__, __LINE__ )

#ifdef DEBUG
#define IF_DEBUG_ASSERT(e)  if (e) ; else assertFail( #e, __FILE__, __LINE__ )
#else
#define IF_DEBUG_ASSERT(e)
#endif

#define ASSERT_FAILED() assertFail( "", __FILE__, __LINE__ )

/*ͨѶ���ݽṹ*/
typedef struct
{
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[ATKP_MAX_DATA_SIZE];
} atkp_t;

/*����ָ��ID*/
typedef enum
{
    UP_VERSION		= 0x00,
    UP_STATUS		= 0x01,
    UP_SENSER		= 0x02,
    UP_RCDATA		= 0x03,
    UP_GPSDATA		= 0x04,
    UP_POWER		= 0x05,
    UP_MOTOR		= 0x06,
    UP_SENSER2		= 0x07,
    UP_FLYMODE		= 0x0A,
    UP_SPEED 		= 0x0B,
    UP_PID1			= 0x10,
    UP_PID2			= 0x11,
    UP_PID3			= 0x12,
    UP_PID4			= 0x13,
    UP_PID5			= 0x14,
    UP_PID6			= 0x15,
    UP_RADIO		= 0x40,
    UP_MSG			= 0xEE,
    UP_CHECK		= 0xEF,

    UP_REMOTER		= 0x50,
    UP_PRINTF		= 0x51,

    UP_USER_DATA1	= 0xF1,
    UP_USER_DATA2	= 0xF2,
    UP_USER_DATA3	= 0xF3,
    UP_USER_DATA4	= 0xF4,
    UP_USER_DATA5	= 0xF5,
    UP_USER_DATA6	= 0xF6,
    UP_USER_DATA7	= 0xF7,
    UP_USER_DATA8	= 0xF8,
    UP_USER_DATA9	= 0xF9,
    UP_USER_DATA10	= 0xFA,
} upmsgID_e;


/*����ָ��*/
#define  D_COMMAND_ACC_CALIB		0x01
#define  D_COMMAND_GYRO_CALIB		0x02
#define  D_COMMAND_MAG_CALIB		0x04
#define  D_COMMAND_BARO_CALIB		0x05
#define  D_COMMAND_ACC_CALIB_EXIT	0x20
#define  D_COMMAND_ACC_CALIB_STEP1	0x21
#define  D_COMMAND_ACC_CALIB_STEP2	0x22
#define  D_COMMAND_ACC_CALIB_STEP3	0x23
#define  D_COMMAND_ACC_CALIB_STEP4	0x24
#define  D_COMMAND_ACC_CALIB_STEP5	0x25
#define  D_COMMAND_ACC_CALIB_STEP6	0x26
#define  D_COMMAND_FLIGHT_LOCK		0xA0
#define  D_COMMAND_FLIGHT_ULOCK		0xA1

#define  D_ACK_READ_PID				0x01
#define  D_ACK_READ_VERSION			0xA0
#define  D_ACK_RESET_PARAM			0xA1
/*����ָ��ID*/
typedef enum
{
    DOWN_COMMAND	= 0x01,
    DOWN_ACK		= 0x02,
    DOWN_RCDATA		= 0x03,
    DOWN_POWER		= 0x05,
    DOWN_FLYMODE	= 0x0A,
    DOWN_PID1		= 0x10,
    DOWN_PID2		= 0x11,
    DOWN_PID3		= 0x12,
    DOWN_PID4		= 0x13,
    DOWN_PID5		= 0x14,
    DOWN_PID6		= 0x15,
    DOWN_RADIO		= 0x40,

    DOWN_REMOTER	= 0x50,
} downmsgID_e;

typedef struct
{
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t thrust;
} joystickFlyui16_t;

/*ң���������*/
typedef enum
{
    REMOTER_CMD,
    REMOTER_DATA,
} remoterType_e;

/*��������*/
#define  CMD_GET_MSG		0x01	/*��ȡ������Ϣ���Լ죩*/
#define  CMD_GET_CANFLY		0x02	/*��ȡ�����Ƿ��ܷ�*/
#define  CMD_FLIGHT_LAND	0x03	/*��ɡ�����*/
#define  CMD_EMER_STOP		0x04	/*����ͣ��*/
#define  CMD_FLIP			0x05	/*4D����*/
#define  CMD_POWER_MODULE	0x06	/*�򿪹ر���չģ���Դ*/
#define  CMD_LEDRING_EFFECT	0x07	/*����RGB�ƻ�Ч��*/
#define  CMD_POWER_VL53LXX	0x08	/*�򿪹رռ���*/

/*���б���*/
#define  ACK_MSG    0x01

/*ң�����ݽṹ*/
typedef __packed struct
{
    float   roll;
    float   pitch;
    float   yaw;
    float   thrust;
    float   trimPitch;
    float   trimRoll;
    uint8_t ctrlMode;
    bool    flightMode;
    bool    RCLock;
} remoterData_t;

typedef __packed struct
{
    uint8_t version;
    bool mpu_selfTest;
    bool baro_slfTest;
    bool isCanFly;
    bool isLowpower;

    float trimRoll;     /*roll΢��*/
    float trimPitch;    /*pitch΢��*/
} MiniFlyMsg_t;

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

//���ݲ�ֺ궨��
#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)    ) )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

//���ݷ�������ʱ�䣨��λms��
#define  PERIOD_STATUS      30
#define  PERIOD_SENSOR      10
#define  PERIOD_RCDATA      40
#define  PERIOD_POWER       100
#define  PERIOD_MOTOR       40
#define  PERIOD_SENSOR2     40
#define  PERIOD_SPEED       50
#define  PERIOD_USERDATA    20

typedef struct
{
    uint8_t ctrlMode    : 2;    /*bit0  1=����ģʽ 0=�ֶ�ģʽ bit1 1=����ģʽ*/
    uint8_t keyFlight   : 1;    /*bit2 һ�����*/
    uint8_t keyLand     : 1;    /*bit3 һ������*/
    uint8_t emerStop    : 1;    /*bit4 ����ͣ��*/
    uint8_t flightMode  : 1;    /*bit5 ����ģʽ 1=��ͷ 0=��ͷ*/
    uint8_t reserved    : 2;    /*bit6~7 ����*/
} commanderBits_t;

/*�������ݽṹ��*/
typedef __packed struct
{
    float roll;       // deg
    float pitch;      // deg
    float yaw;        // deg
    float trimPitch;
    float trimRoll;
    uint16_t thrust;
} ctrlVal_t;

/*���ݻ���ṹ��*/
typedef struct
{
    ctrlVal_t  tarVal[2];
    bool activeSide;
    uint32_t timestamp;  /* FreeRTOS ʱ�ӽ���*/
} ctrlValCache_t;

typedef enum
{
    RATE    = 0,
    ANGLE   = 1,
} RPYType;

typedef enum
{
    XMODE     = 0, /*Xģʽ*/
    CAREFREE  = 1, /*��ͷģʽ*/
} YawModeType;

typedef enum
{
    ATK_REMOTER = 0,
    WIFI        = 1,
} ctrlSrc_e;

void uartSendPacket(atkp_t *p);
void sendPower(uint16_t votage, uint16_t current);
void uart_rev_init(void);
void uart_rev_handle(uint8_t *para);
void sendSenser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z);
void sendStatus(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void sendCheck(uint8_t head, uint8_t check_sum);


#endif
