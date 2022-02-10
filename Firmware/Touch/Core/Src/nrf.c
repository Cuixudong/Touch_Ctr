#include "nrf.h"
#include "usart.h"

static enum
{
    waitForStartByte1,
    waitForStartByte2,
    waitForMsgID,
    waitForDataLength,
    waitForData,
    waitForChksum1,
} rxState;

static uint8_t cksum = 0;
static uint8_t dataIndex = 0;
static atkp_t txPacket;
static atkp_t rxPacket;
static joystickFlyui16_t rcdata;
static ctrlVal_t remoterCtrl;/*发送到commander姿态控制数据*/
static MiniFlyMsg_t msg;


/*打包ATKPPacket数据通过串口DMA发送*/
void uartSendPacket(atkp_t *p)
{
    int dataSize;
    uint8_t cksum = 0;
    uint8_t sendBuffer[36];
    //ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
    sendBuffer[0] = UP_BYTE1;
    sendBuffer[1] = UP_BYTE2;
    sendBuffer[2] = p->msgID;
    sendBuffer[3] = p->dataLen;
    memcpy(&sendBuffer[4], p->data, p->dataLen);
    dataSize = p->dataLen + 5;//加上cksum
    /*计算校验和*/
    for (int i=0; i<dataSize-1; i++)
    {
        cksum += sendBuffer[i];
    }
    sendBuffer[dataSize-1] = cksum;
    /*串口DMA发送*/
    //uartslkSendDataDmaBlocking(dataSize, sendBuffer);
    HAL_UART_Transmit(&huart1,sendBuffer,dataSize,0xffff);
}

void sendSenser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z)
{
    uint8_t _cnt=0;
    atkp_t p;
    __IO int16_t _temp;

    p.msgID = UP_SENSER;

    _temp = a_x;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);

    _temp = g_x;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);

    _temp = m_x;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = 0;
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);

    p.dataLen = _cnt;
    uartSendPacket(&p);
}

void sendStatus(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
    uint8_t _cnt=0;
    atkp_t p;
    __IO int16_t _temp;
    __IO int32_t _temp2 = alt;

    p.msgID = UP_STATUS;

    _temp = (int)(roll*100);
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = (int)(pitch*100);
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);
    _temp = (int)(yaw*100);
    p.data[_cnt++]=BYTE1(_temp);
    p.data[_cnt++]=BYTE0(_temp);

    p.data[_cnt++]=BYTE3(_temp2);
    p.data[_cnt++]=BYTE2(_temp2);
    p.data[_cnt++]=BYTE1(_temp2);
    p.data[_cnt++]=BYTE0(_temp2);

    p.data[_cnt++] = fly_model;
    p.data[_cnt++] = armed;

    p.dataLen = _cnt;
    uartSendPacket(&p);
}

void sendPower(uint16_t votage, uint16_t current)
{
    uint8_t _cnt=0;
    atkp_t p;

    p.msgID = UP_POWER;

    p.data[_cnt++]=BYTE1(votage);
    p.data[_cnt++]=BYTE0(votage);
    p.data[_cnt++]=BYTE1(current);
    p.data[_cnt++]=BYTE0(current);

    p.dataLen = _cnt;
    uartSendPacket(&p);
}

uint8_t atkpCheckSum(atkp_t *packet)
{
    uint8_t sum;
    sum = DOWN_BYTE1;
    sum += DOWN_BYTE2;
    sum += packet->msgID;
    sum += packet->dataLen;
    for(int i=0; i<packet->dataLen; i++)
    {
        sum += packet->data[i];
    }
    return sum;
}

void sendCheck(uint8_t head, uint8_t check_sum)
{
    atkp_t p;

    p.msgID = UP_CHECK;
    p.dataLen = 2;
    p.data[0] = head;
    p.data[1] = check_sum;
    uartSendPacket(&p);
}

void uart_rev_init(void)
{
    rxState = waitForStartByte1;
}

void uart_rev_handle(uint8_t *para)
{
    uint8_t c = *(para);
    switch(rxState)
    {
        case waitForStartByte1:
            rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
            cksum = c;
            break;
        case waitForStartByte2:
            rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
            cksum += c;
            break;
        case waitForMsgID:
            rxPacket.msgID = c;
            rxState = waitForDataLength;
            cksum += c;
            break;
        case waitForDataLength:
            if (c <= ATKP_MAX_DATA_SIZE)
            {
                rxPacket.dataLen = c;
                dataIndex = 0;
                rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
                cksum += c;
            } else
            {
                rxState = waitForStartByte1;
            }
            break;
        case waitForData:
            rxPacket.data[dataIndex] = c;
            dataIndex++;
            cksum += c;
            if (dataIndex == rxPacket.dataLen)
            {
                rxState = waitForChksum1;
            }
            break;
        case waitForChksum1:
            if (cksum == c)	/*所有校验正确*/
            {
                if( rxPacket.msgID == DOWN_POWER)
                {
                    ;/*do noting*/
                }
                else
                {
                    uartSendPacket(&txPacket);
                }
            }
            else    /*校验错误*/
            {
                rxState = waitForStartByte1;
                IF_DEBUG_ASSERT(1);
            }
            rxState = waitForStartByte1;
            break;
        default:
            //ASSERT(0);
            break;
    }
}

/*遥控数据接收处理*/
void remoterCtrlProcess(atkp_t* pk)
{
    if(pk->data[0] == REMOTER_CMD)
    {
        switch(pk->data[1])
        {
        case CMD_FLIGHT_LAND:
            break;
        case CMD_EMER_STOP:
            break;
        case CMD_FLIP:
            break;
        case CMD_GET_MSG:
            break;
        case CMD_POWER_MODULE:
            break;
        case CMD_LEDRING_EFFECT:
            break;
        case CMD_POWER_VL53LXX:
            break;
        }
    }
    else if(pk->data[0] == REMOTER_DATA)
    {
        remoterData_t remoterData = *(remoterData_t*)(pk->data+1);
        remoterCtrl.roll = remoterData.roll;
        remoterCtrl.pitch = remoterData.pitch;
        remoterCtrl.yaw = remoterData.yaw;
        remoterCtrl.thrust = remoterData.thrust * 655.35f;
        remoterCtrl.trimPitch = remoterData.trimPitch;
        remoterCtrl.trimRoll = remoterData.trimRoll;
    }
}

void atkpReceiveAnl(atkp_t *anlPacket)
{
    if(anlPacket->msgID == DOWN_COMMAND)
    {
        switch(anlPacket->data[0])
        {
        case D_COMMAND_ACC_CALIB:
            break;
        case D_COMMAND_GYRO_CALIB:
            break;
        case D_COMMAND_MAG_CALIB:
            break;
        case D_COMMAND_BARO_CALIB:
            break;
        case D_COMMAND_ACC_CALIB_STEP1:
            break;
        case D_COMMAND_ACC_CALIB_STEP2:
            break;
        case D_COMMAND_ACC_CALIB_STEP3:
            break;
        case D_COMMAND_ACC_CALIB_STEP4:
            break;
        case D_COMMAND_ACC_CALIB_STEP5:
            break;
        case D_COMMAND_ACC_CALIB_STEP6:
            break;
        case D_COMMAND_FLIGHT_LOCK:
            break;
        case D_COMMAND_FLIGHT_ULOCK:
            break;
        }
    }
    else if(anlPacket->msgID == DOWN_ACK)
    {
        if(anlPacket->data[0] == D_ACK_READ_PID)/*读取PID参数*/
        {
            //sendPid(1);
        }
        if(anlPacket->data[0] == D_ACK_RESET_PARAM)/*恢复默认参数*/
        {
            //sendPid(1);
        }
    }
    else if(anlPacket->msgID == DOWN_RCDATA)
    {
        rcdata = *((joystickFlyui16_t*)anlPacket->data);
    }
    else if(anlPacket->msgID == DOWN_POWER)/*nrf51822*/
    {
    }
    else if(anlPacket->msgID == DOWN_REMOTER)/*遥控器*/
    {
        remoterCtrlProcess(anlPacket);
    }
    else if(anlPacket->msgID == DOWN_PID1)
    {
    }
    else if(anlPacket->msgID == DOWN_PID2)
    {
    }
    else if(anlPacket->msgID == DOWN_PID3)
    {
    }
    else if(anlPacket->msgID == DOWN_PID4)
    {
    }
    else if(anlPacket->msgID == DOWN_PID5)
    {
    }
    else if(anlPacket->msgID == DOWN_PID6)
    {
    }
}
