/**********************************************************
filename: board.h
**********************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BOARD_H
#define _BOARD_H
#include "misc.h"
#include "uartdev.h"
#include "rs485dev.h"
#include "net_device.h"
#include "spiRsrc.h"

//#include "w25qxx.h"

/* output variables for extern function --------------------------------------*/

#if defined(F_APP_FTP)
#define MAX_HTTPSOCK	4
#else
#define MAX_HTTPSOCK	6
#endif

extern const char ABOUT[];
extern const char COMMON_HELP[];

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern CAN_HandleTypeDef hcan;

extern CRC_HandleTypeDef hcrc;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern SD_HandleTypeDef hsd;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// gloable var
extern char addrPre[4];
extern u8 boardAddr;
extern u32 errorCode;

extern UartDev_t console;
extern Rs485Dev_t rs485;
extern WizDevDev netDev;
extern TcpSeverDev_t* handler_tcps;
//extern TcpClientDev_t* handler_tcpc;
//extern w25qxxDev_t flshRom;

//extern const PIN_T FROM_CS;
extern const PIN_T LAN_CS1, LAN_CS2;
extern const PIN_T LAN_IRQ1, LAN_IRQ2;
extern const PIN_T RUNNING;
extern u8 initalDone;

#define DEV_PORT_SUM	2
extern DEV_PORT DevPort[DEV_PORT_SUM];

// gloable method
void boardInit(void);
void print(const char* FORMAT_ORG, ...);
void printS(const char* MSG);
void print485(const char* FORMAT_ORG, ...);
void printS485(const char* STRING);

s8 psramWrite(u32 addr, const u8 *pDat, u16 nBytes);
s8 psramRead(u32 addr, u8 *pDat, u16 nBytes);
s8 ioWrite(u16 addr, const u8 *pDat, u16 nBytes);
s8 ioRead(u16 addr, u8 *pDat, u16 nBytes);
u8 brdCmd(const char* CMD, u8 brdAddr, void (*xprintS)(const char* MSG), void (*xprint)(const char* FORMAT_ORG, ...));

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
