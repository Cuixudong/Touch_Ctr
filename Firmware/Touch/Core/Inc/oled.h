#ifndef __OLED_H
#define __OLED_H
#include "main.h"

#define OLED_CMD    0       //写命令
#define OLED_DATA   1       //写数据

#define OLED_W  128
#define OLED_H   64
#define OLED_PAGE_V 12      //页数
#define OLED_PAGE_H 2
#define USB_HW_SPI  1       //硬SPI

#ifndef OLED_CS_Pin
#define OLED_CS_Pin GPIO_PIN_3
#endif
#ifndef OLED_CS_GPIO_Port
#define OLED_CS_GPIO_Port GPIOB
#endif
#ifndef OLED_RST_Pin
#define OLED_RST_Pin GPIO_PIN_4
#endif
#ifndef OLED_RST_GPIO_Port
#define OLED_RST_GPIO_Port GPIOB
#endif
#ifndef OLED_DC_Pin
#define OLED_DC_Pin GPIO_PIN_5
#endif
#ifndef OLED_DC_GPIO_Port
#define OLED_DC_GPIO_Port GPIOB
#endif

#if (!USB_HW_SPI)

#ifndef OLED_D0_Pin
#define OLED_D0_Pin GPIO_PIN_13
#endif
#ifndef OLED_D0_GPIO_Port
#define OLED_D0_GPIO_Port GPIOB
#endif
#ifndef OLED_D1_Pin
#define OLED_D1_Pin GPIO_PIN_15
#endif
#ifndef OLED_D1_GPIO_Port
#define OLED_D1_GPIO_Port GPIOB
#endif

#define OLED_D0_H   HAL_GPIO_WritePin(OLED_D0_GPIO_Port,OLED_D0_Pin,GPIO_PIN_SET)
#define OLED_D0_L   HAL_GPIO_WritePin(OLED_D0_GPIO_Port,OLED_D0_Pin,GPIO_PIN_RESET)
#define OLED_D1_H   HAL_GPIO_WritePin(OLED_D1_GPIO_Port,OLED_D1_Pin,GPIO_PIN_SET)
#define OLED_D1_L   HAL_GPIO_WritePin(OLED_D1_GPIO_Port,OLED_D1_Pin,GPIO_PIN_RESET)
#endif

//OLED操作脚
#define OLED_DC_H   HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_SET)
#define OLED_DC_L   HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_RESET)
#define OLED_CS_H   HAL_GPIO_WritePin(OLED_CS_GPIO_Port,OLED_CS_Pin,GPIO_PIN_SET)
#define OLED_CS_L   HAL_GPIO_WritePin(OLED_CS_GPIO_Port,OLED_CS_Pin,GPIO_PIN_RESET)
#define OLED_RST_H  HAL_GPIO_WritePin(OLED_RST_GPIO_Port,OLED_RST_Pin,GPIO_PIN_SET)
#define OLED_RST_L  HAL_GPIO_WritePin(OLED_RST_GPIO_Port,OLED_RST_Pin,GPIO_PIN_RESET)

typedef struct
{
    __IO uint16_t   widget;         //OLED宽度
    __IO uint16_t   height;         //OLED高度
    __IO uint8_t    oled_page_h;    //OLED水平页宽
    __IO uint8_t    oled_page_v;    //OLED垂直页高
    __IO uint8_t    oled_move_dir;  //OLED移动方向
    
    __IO uint16_t   oled_cur_pos_x; //OLED当前水平显示位置
    __IO uint16_t   oled_cur_pos_y; //OLED当前显示垂直位置
    
    __IO uint16_t   oled_offset_x;  //OLED写水平偏移
    __IO uint16_t   oled_offset_y;  //OLED写垂直偏移
}OLED_Display_Ctr;

extern OLED_Display_Ctr oled_ctr;

#define OLED_MOVE_DEMO_EN 0
#define OLED_CUR_POS_X  oled_ctr.oled_cur_pos_x
#define OLED_CUR_POS_Y  oled_ctr.oled_cur_pos_y
#define OLED_OFFSET_X   oled_ctr.oled_offset_x
#define OLED_OFFSET_Y   oled_ctr.oled_offset_y
#define OLED_MOVE_DIR   oled_ctr.oled_move_dir

#define EX_MOVE_POS(x,y)    do\
                            {\
                                if(x<0)\
                                {\
                                    if((OLED_CUR_POS_X + x) >= 0)\
                                    {\
                                        OLED_CUR_POS_X += x;\
                                    }\
                                    else\
                                    {\
                                        OLED_CUR_POS_X = 0;\
                                    }\
                                }\
                                else\
                                {\
                                    OLED_CUR_POS_X += x;\
                                    if(OLED_CUR_POS_X >= (OLED_W * (OLED_PAGE_H - 1)))\
                                    {\
                                        OLED_CUR_POS_X = (OLED_W * (OLED_PAGE_H - 1));\
                                    }\
                                }\
                                if(y<0)\
                                {\
                                    if((OLED_CUR_POS_Y + y) >= 0)\
                                    {\
                                        OLED_CUR_POS_Y += y;\
                                    }\
                                    else\
                                    {\
                                        OLED_CUR_POS_Y = 0;\
                                    }\
                                }\
                                else\
                                {\
                                    OLED_CUR_POS_Y += y;\
                                    if(OLED_CUR_POS_Y >= (OLED_H * (OLED_PAGE_V - 1)))\
                                    {\
                                        OLED_CUR_POS_Y = (OLED_H * (OLED_PAGE_V - 1));\
                                    }\
                                }\
                            }\
                            while(0)

extern uint8_t OLED_GRAM[OLED_W * OLED_PAGE_H][OLED_PAGE_V * OLED_H / 8];
extern const unsigned char bmp_battery[13][20];

extern const char * articl[46];

//OLED控制用函数
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);

void OLED_Init(void);
void OLED_Clear(uint8_t para);
void OLED_DrawPoint(uint16_t x,uint16_t y,uint8_t t);
void OLED_Fill(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint8_t dot);
void OLED_ShowChar(uint16_t x,uint16_t y,char chr,uint8_t size,uint8_t mode);
void OLED_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode);
void OLED_ShowString(uint16_t x,uint16_t y,char *p,uint8_t size);

void OLED_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint8_t dot);
void OLED_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint8_t dot);
void OLED_Draw_Circle(uint16_t x0,uint16_t y0,uint16_t r,uint8_t dot);
uint8_t OLED_ReadPoint(uint16_t x,uint16_t y);

void OLED_Show_Font(uint16_t x,uint16_t y,char  *font,uint8_t f_w,uint8_t f_h,uint8_t mode);
void OLED_Show_Str(uint16_t x,uint16_t y,char *str,uint8_t f_h,uint8_t mode);
void OLED_Show_Str_Mid(uint16_t x,uint16_t y,char *str,uint8_t f_h,uint8_t mode,uint16_t len);
void OLED_Show_Picture(uint16_t x,uint16_t y,uint8_t *p,uint16_t p_w,uint16_t p_h);

void show_start_ui(void);
void OLED_SetDir(uint8_t para);
void OLED_SetContrast(uint8_t para);





#define MG_W 32
#define MG_H 64
typedef struct
{
    uint8_t msg[5];
}mg_gird;

extern __IO mg_gird MapMain[MG_W][MG_H];

void Recursive_division(int left, int right, int top, int bottom);
void test_mg(uint16_t x,uint16_t y,uint16_t wid,uint16_t size_x,uint16_t size_y);

#endif
