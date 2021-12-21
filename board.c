/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : board.c
* Author             : Tiko Zhong
* Date First Issued  : 11/18/2021
* Description        : 
*                      
********************************************************************************
* History:
* Apr22,2021: V0.2
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "main.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "at24cxx.h"
#include "iic_io.h"
#include "ConfigData.h"
#include "LY68L6400.h"
//#include "w25qxx.h"
#include "wlun.h"

//#include "cjson.h"
//#include "fatfs.h"
//#include "w25qxx.h"

#include "usbd_msc.h"

//#include "httpServer.h"

#define NOUSED_PIN_INDX 255

/* import handle from main.c variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//////////////////////////////
// Shared Buffer Definition //
//////////////////////////////
#define DATA_BUF_SIZE   1024

uint8_t RX_BUF[DATA_BUF_SIZE];
uint8_t TX_BUF[DATA_BUF_SIZE];
#if defined(F_APP_FTP)
uint8_t FTP_DBUF[_MAX_SS];
#endif

////////////////////////////////
// W5500 HW Socket Definition //
////////////////////////////////
#define SOCK_CONFIG		0
#define SOCK_DHCP		1
#if defined(F_APP_FTP)
uint8_t socknumlist[] = {4, 5, 6, 7};
#else
uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7};
#endif
//////////////////////////////////////////
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const char ABOUT[] = {"MB103VEMN-1.0.0"};
const char COMMON_HELP[] = {
	"Commands:"
    "\n help()"
    "\n about()"
	"\n iic.write 0x%x 0x%x 0x%x"
	"\n iic.read 0x%x 0x%x"
	"\n"
};
char addrPre[4] = {0};	//addr precode
u8 boardAddr = 0;
/**********************************************
*  PINs Define
**********************************************/
const PIN_T NULL_PIN = {NULL, NULL};
const PIN_T RUNNING = {RUNNING_GPIO_Port, RUNNING_Pin};
//EEPROM
const PIN_T SCL = {EROM_SCL_GPIO_Port, EROM_SCL_Pin};
const PIN_T SDA = {EROM_SDA_GPIO_Port, EROM_SDA_Pin};
//RS485
const PIN_T DE = {DE_GPIO_Port, DE_Pin};
const PIN_T DET = {DET1_GPIO_Port, DET1_Pin};

//const PIN_T IRQ = {INPUT_IRQ_GPIO_Port, INPUT_IRQ_Pin};
// ethenet
const PIN_T LAN_CS1 = {LAN_CS1_GPIO_Port, LAN_CS1_Pin};
const PIN_T LAN_IRQ1 = {LAN_IRQ1_GPIO_Port, LAN_IRQ1_Pin};

/**********************************************
*  Devices PORTS
**********************************************/
DEV_PORT DevPort[DEV_PORT_SUM] = {
	//dev0
	{
		//	PIN_T PA0,PA1,PA2;
		{GPIOD, GPIO_PIN_7},
		{GPIOB, GPIO_PIN_6},
		{GPIOB, GPIO_PIN_7},
		//	PIN_T PB0,PB1,PB2,PB3;
		{GPIOB, GPIO_PIN_3},
		{GPIOA, GPIO_PIN_15},
		{GPIOB, GPIO_PIN_5},
		{GPIOB, GPIO_PIN_4},
		//	PIN_T PC0,PC1,PC2,PC3;
		{GPIOC, GPIO_PIN_0},
		{GPIOC, GPIO_PIN_1},
		{GPIOC, GPIO_PIN_2},
		{GPIOC, GPIO_PIN_3},
		//	PIN_T PD0,PD1,PD2,PD3;
		{GPIOE, GPIO_PIN_1},
		{GPIOE, GPIO_PIN_0},
		{GPIOB, GPIO_PIN_9},
		{GPIOB, GPIO_PIN_8},
		//	SPI_HandleTypeDef* SPI_HANDLE;
		&hspi3,
		//	I2C_HandleTypeDef* IIC_HANDLE;
		&hi2c1,
		//	ADC_HandleTypeDef* ADC_HANDLE;
		0,	//hadc1,
		//	TIM_HandleTypeDef* TIM_HANDLE;
		&htim1,
		//	void (*Setup)(u8 argc, void* argp);
		NULL
	},
	//dev1
	{
		//	PIN_T PA0,PA1,PA2;
		{GPIOE, GPIO_PIN_15},
		{GPIOB, GPIO_PIN_10},
		{GPIOB, GPIO_PIN_11},
		//	PIN_T PB0,PB1,PB2,PB3;
		{GPIOB, GPIO_PIN_13},
		{GPIOB, GPIO_PIN_12},
		{GPIOB, GPIO_PIN_15},
		{GPIOB, GPIO_PIN_14},
		//	PIN_T PC0,PC1,PC2,PC3;
		{GPIOD, GPIO_PIN_12},		//A1+D12
		{GPIOD, GPIO_PIN_13},		//C4+D13	
		{GPIOB, GPIO_PIN_0},
		{GPIOB, GPIO_PIN_1},
		//	PIN_T PD0,PD1,PD2,PD3;
		{GPIOE, GPIO_PIN_4},
		{GPIOE, GPIO_PIN_5},
		{GPIOE, GPIO_PIN_6},
		{GPIOC, GPIO_PIN_13},
		//	SPI_HandleTypeDef* SPI_HANDLE;
		&hspi2,
		//	I2C_HandleTypeDef* IIC_HANDLE;
		0,  //&hi2c2,
		//	ADC_HandleTypeDef* ADC_HANDLE;
		0,	//&hadc1,
		//	TIM_HandleTypeDef* TIM_HANDLE;
		&htim4,
		//	void (*Setup)(u8 argc, void* argp);
		NULL
	}
};

u32 errorCode = 0;

/**********************************************
*  static Devices
**********************************************/
// console uart device
#define RX_POOL_LEN	(512)
#define	RX_BUF_LEN	(128)	//115200bps, can receive 13bytes/ms, (13*8) = 104 < 128
#define	TX_POOL_LEN	(512)
static u8 uartMem[RX_POOL_LEN+2*RX_BUF_LEN+TX_POOL_LEN] = {0};
UartDev_t console;

static u8 rs485Mem[RX_POOL_LEN+2*RX_BUF_LEN+TX_POOL_LEN] = {0};
Rs485Dev_t rs485;
static s8 rs485AfterSend_1(UART_HandleTypeDef *huart);
static s8 rs485BeforeSend_1(void);

#define MONITOR_RX_POOL_LEN	(256)
#define	MONITOR_RX_BUF_LEN	(128)	//115200bps, can receive 13bytes/ms, (13*8) = 104 < 128
#define	MONITOR_TX_BUF_LEN	(256)
static u8 monitorMem[MONITOR_RX_POOL_LEN+2*MONITOR_RX_BUF_LEN+MONITOR_TX_BUF_LEN] = {0};

u8 initalDone = 0;

// eeprom 
IIC_IO_Dev_T iic400k;
AT24CXX_Dev_T erom = {0};		// AT24C256, 32K BYTES
#define EEPROM_SIZE_USR			4096
#define EEPROM_SIZE_REG			1024
#define EEPROM_SIZE_DEV0		256
#define EEPROM_SIZE_DEV1		256
#define EEPROM_SIZE_DEV2		256
#define EEPROM_SIZE_DEV3		256
#define EEPROM_SIZE_DEV4		256
#define EEPROM_SIZE_DEV5		256
#define EEPROM_SIZE_DEV6		256
#define EEPROM_SIZE_DEV7		256
#define EEPROM_SIZE_NET			(1024*8)

#define EEPROM_BASE_USER		0
#define EEPROM_BASE_REG			(EEPROM_BASE_USER + EEPROM_SIZE_USR)
#define EEPROM_BASE_DEV0		(EEPROM_BASE_REG + EEPROM_SIZE_REG)
#define EEPROM_BASE_DEV1		(EEPROM_BASE_DEV0 + EEPROM_SIZE_DEV0)
#define EEPROM_BASE_DEV2		(EEPROM_BASE_DEV1 + EEPROM_SIZE_DEV1)
#define EEPROM_BASE_DEV3		(EEPROM_BASE_DEV2 + EEPROM_SIZE_DEV2)
#define EEPROM_BASE_DEV4		(EEPROM_BASE_DEV3 + EEPROM_SIZE_DEV3)
#define EEPROM_BASE_DEV5		(EEPROM_BASE_DEV4 + EEPROM_SIZE_DEV4)
#define EEPROM_BASE_DEV6		(EEPROM_BASE_DEV5 + EEPROM_SIZE_DEV5)
#define EEPROM_BASE_DEV7		(EEPROM_BASE_DEV6 + EEPROM_SIZE_DEV6)
#define EEPROM_BASE_NET			(EEPROM_BASE_DEV7 + EEPROM_SIZE_DEV7)

//w25qxxDev_t flshRom;
#define	SERVER_PORT 8080
WizDevDev netDev = {0};
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x11, 0x11, 0xaa},
                            .ip = {169, 254, 1, 92},
                            .sn = {255,255,255,0},
                            .gw = {169, 254, 1, 1},
                            .dns = {88,88,98,98},
                            .dhcp = NETINFO_STATIC };
/**********************************************
*  dymatic Devices
**********************************************/
static void network_init(void);

static void cb_newRcv(u16 rcbBytes);	// callback while there are receive data
static void cb_connected(u8* destip, u16 destport);
static void cb_closed();
static void cb_listen(u16 port);
					
static void cb_newRcvClient(u16 rcvBytes);
static void cb_connectedClient(u8* ip, u16 port);
static void cb_closedClient();
static void cb_listenClient(u16 port);
		
static void cb_newRcvUdp(u16 rcvBytes);
static void cb_closedUdp();
							
TcpSeverDev_t* handler_tcps = NULL;
TcpClientDev_t* handler_tcpc = NULL;
UdpDev_t* handler_udp = NULL;


LY68L6400_Dev_T psram;
const PIN_T PSRAM_CS = {PSRAM_CS_GPIO_Port, PSRAM_CS_Pin};

/* Private function prototypes -----------------------------------------------*/
//TokenDev_t token;
// 169.254.1.92:8080
void boardInit(void){
//	u8 destip[4] = {169, 254, 144, 20};
//	u16 destport = 8080;
	char testbuf[20] = {0}; 
	u32 counter;
	S2E_Packet *value;
	void* ram;

	for(counter = 0; counter < 20; counter++){
		HAL_GPIO_TogglePin(RUNNING.GPIOx, RUNNING.GPIO_Pin);
		HAL_Delay(30);
	}
	
	//read board addr
	setupUartDev(&console, &huart1, uartMem, RX_POOL_LEN, RX_BUF_LEN, TX_POOL_LEN);
	print("%s\r\n", ABOUT);

	//read board addr
	strFormat(addrPre, 4, "%d.", boardAddr);
	print("BoardAddr: %d\r\n", boardAddr);	
	
	printS("setup eeprom...");
	AT24CXX_Setup(&erom, SCL, SDA, AT24C256, 0x01);
	printS("ok\r\n");
	
	printS("setup psram...");
	LY68L6400_Setup(
		&psram, 
		&hspi1, 
		PSRAM_CS
	);
	printS("ok\r\n");

	psramWrite(123, (const u8*)"hello", strlen("hello"));

	memset(testbuf,0,20);
	psramRead(123, (u8*)testbuf, strlen("hello"));
	print("psram test:%s\n", testbuf);
	
	ioRead(4*0, (u8*)&counter, 4);
	gWIZNETINFO.ip[0] = counter;
	ioRead(4*1, (u8*)&counter, 4);
	gWIZNETINFO.ip[1] = counter;
	ioRead(4*2, (u8*)&counter, 4);
	gWIZNETINFO.ip[2] = counter;
	ioRead(4*3, (u8*)&counter, 4);
	gWIZNETINFO.ip[3] = counter;
	wlunUsing = gWIZNETINFO.ip[3];
	
	ioRead(4*4, (u8*)&counter, 4);
	gWIZNETINFO.gw[0] = counter;
	ioRead(4*5, (u8*)&counter, 4);
	gWIZNETINFO.gw[1] = counter;
	ioRead(4*6, (u8*)&counter, 4);
	gWIZNETINFO.gw[2] = counter;
	ioRead(4*7, (u8*)&counter, 4);
	gWIZNETINFO.gw[3] = counter;
	
	printS("setup rs485_1...");
	setupRs485Dev(&rs485,&huart2, rs485Mem, RX_POOL_LEN, RX_BUF_LEN, TX_POOL_LEN, 
		DE, 
		DET,
		rs485BeforeSend_1,
		rs485AfterSend_1
	);
	printS("ok\r\n");

//	printS("setup flashrom w25q32...");HAL_Delay(10);
//	setup_w25qxx(&flshRom, FROM_CS, &hspi1);
////	flshRom.EraseChip(&flshRom.rsrc);
//	printS("ok\r\n");

//	printS("f_mount\n");	HAL_Delay(5);
//	memset(USERPath, 0, 4);
//	strncpy(USERPath, "0:/", 3);
//	retUSER = f_mount(&USERFatFS, USERPath, 0);
//	if(retUSER != FR_OK){	print("f_mount:%d\r\n", retUSER);	HAL_Delay(2);	}

//	printS("formatt\n");	HAL_Delay(5);
//	retUSER = f_mkfs("", 0, 0);
//	if(retUSER != FR_OK){	print("xxxxxxxxxxxxxxxxxxxf_mkfs:%d\r\n", retUSER);	HAL_Delay(2);}
//	HAL_Delay(500);

//	printS("open file\n");	HAL_Delay(5);
//    retUSER = f_open(&USERFile, "0:/wr_test.txt", FA_READ | FA_WRITE);
////	retUSER = f_open(&USERFile, "0:/wr_test.txt", FA_READ | FA_CREATE_NEW | FA_WRITE);
//    if(retUSER != FR_OK){	print("f_open fail err%d\r\n", retUSER);	}

//	printS("write file\n");	HAL_Delay(5);
//	memset(testbuf,0,20);
//    strncpy((char*)testbuf, "WIZnet WiKi", 19);
//    retUSER = f_write(&USERFile, testbuf, 11, &counter); 
//    if(retUSER != FR_OK){	print("f_write fail err%d\r\n", retUSER);	}

//	printS("seek file\n");	HAL_Delay(5);
//	retUSER = f_lseek(&USERFile, 0); 
//	if(retUSER != FR_OK){	print("f_lseek fail err%d\r\n", retUSER);	}
//	
//	printS("read file\n");	HAL_Delay(5);
//	memset(testbuf,0,20);
//	retUSER = f_read(&USERFile, testbuf, 19, &counter);
//	if(retUSER != FR_OK){	print("f_read fail err%d\r\n", retUSER);	}
//	else{	print("read file:%s\n", testbuf);	}
//	
//	printS("close file\n");	HAL_Delay(5);
//    f_close(&USERFile);
	
//	//json test
//	cJSON* root = cJSON_CreateObject();
//	cJSON* item = cJSON_CreateObject();
//	cJSON* next = cJSON_CreateObject();
//	cJSON* obj = cJSON_CreateObject();

//	//?root?????
//	cJSON_AddItemToObject(root,"wb",cJSON_CreateNumber(3));
//	cJSON_AddItemToObject(root,"book",cJSON_CreateString("dddd"));
//	cJSON_AddItemToObject(root,"author",cJSON_CreateString("wanger"));

//	//????
//	cJSON* array = NULL;
//	cJSON_AddItemToObject(root,"books",array=cJSON_CreateArray());

//	//??????
//	cJSON_AddItemToArray(array,obj);
//	cJSON_AddItemToObject(obj,"name",cJSON_CreateString("www"));
//	cJSON_AddStringToObject(obj,"type","123");

//	cJSON_AddItemToArray(array,obj=cJSON_CreateObject());
//	cJSON_AddItemToObject(obj,"name",cJSON_CreateString("eee"));
//	cJSON_AddStringToObject(obj,"type","456");

//	cJSON_AddItemToObject(root, "sem", item);//root?????sem??
//	cJSON_AddItemToObject(item, "slots", next);//se?????item??
//	cJSON_AddItemToObject(next, "name", cJSON_CreateString("wangsan"));//??name??

//	if(root == NULL){	printS("root is null\n");	}

//	print("%s\n", cJSON_Print(root));
//	cJSON_Delete(root);
//	HAL_Delay(100);

	setup_S2E_Packet(EEPROM_BASE_NET, ioWrite, ioRead);
	value = get_S2E_Packet_pointer();

	printS("setup wizchip...");	
	wizDev_setup(&netDev, gWIZNETINFO, print, printS);
	printS("ok\n");
	
	printS("new tcp server...");	
	handler_tcps = 
	netDev.newTcpServer(&netDev.rsrc, 4, 4, SERVER_PORT, cb_newRcv, cb_connected, cb_closed, cb_listen);
	if(handler_tcps == NULL)	printS(" fail\n");
	else printS("ok\n");

//	printS("new tcp client...");	HAL_Delay(10);
//	handler_tcpc = 
//	netDev.newTcpClient(&netDev.rsrc, 2, 2, 60000, destip, 4000, cb_newRcvClient, cb_connectedClient, cb_closedClient);
//	if(handler_tcpc == NULL)	printS(" fail\n");
//	else 	printS(" ok\n");	HAL_Delay(10);
//	handler_tcpc->openSession(&handler_tcpc->rsrc);

//	printS("new udp...");	HAL_Delay(10);
//	handler_udp = netDev.newUdp(&netDev.rsrc, 2, 2, 8848, cb_newRcvUdp, cb_closedUdp);
//	if(handler_udp == NULL)	printS(" fail\n");
//	else 	printS(" ok\n");	HAL_Delay(10);

//	httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);
//	netDev.reLink(&netDev.rsrc);

	
	printS(COMMON_HELP);
	print("initial complete 0x%08x\r\n", errorCode);
	initalDone = 1;
}

void printS(const char* STRING){
	console.Send(&console.rsrc, (const u8*)STRING, strlen(STRING));
}

void print(const char* FORMAT_ORG, ...){
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	s16 bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
	//send out
	if(bytes>0)	console.Send(&console.rsrc, (u8*)buf, bytes);
}

void printS485(const char* STRING){
	rs485.Send(&rs485.rsrc, (const u8*)STRING, strlen(STRING));
}

void print485(const char* FORMAT_ORG, ...){
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	s16 bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
	//send out
	if(bytes>0)	rs485.Send(&rs485.rsrc, (u8*)buf, bytes);
}

s8 psramWrite(u32 addr, const u8 *pDat, u16 nBytes){
	return(psram.Write(&psram.rsrc, addr, pDat, nBytes));
}

s8 psramRead(u32 addr, u8 *pDat, u16 nBytes){
	return(psram.Read(&psram.rsrc, addr, pDat, nBytes));
}

s8 ioWrite(u16 addr, const u8 *pDat, u16 nBytes){
	erom.Write(&erom.rsrc, addr, pDat, nBytes);
	HAL_Delay(5);
	return 0;
}

s8 ioRead(u16 addr, u8 *pDat, u16 nBytes){
  erom.Read(&erom.rsrc, addr, pDat, nBytes);
  return 0;
}

u8 brdCmd(const char* CMD, u8 brdAddr, void (*xprintS)(const char* MSG), void (*xprint)(const char* FORMAT_ORG, ...)){
	u32 i,j,k;
	u8 buf[4];
	s8 rslt;
	// common
	if(strncmp(CMD, "about", strlen("about")) == 0){
		xprint("+ok@%d.about(\"%s\")\r\n",brdAddr,ABOUT);
		return 1;
	}
	else if(strncmp(CMD, "help", strlen("help")) == 0){
		xprintS(COMMON_HELP);
		xprint("+ok@%d.help()\r\n",brdAddr);
		return 1;
	}
	else if(sscanf(CMD, "reg.write %d %d ", &i, &j)==2){
		if(i>=32)	{
			xprint("+err@%d.reg.write(%d,%d)\r\n", brdAddr, i, j);
			return 1;
		}
		xprint("+ok@%d.reg.write(%d,%d)\r\n", brdAddr, i, j);
		buf[0] = j & 0xff;	j>>=8;
		buf[1] = j & 0xff;	j>>=8;
		buf[2] = j & 0xff;	j>>=8;
		buf[3] = j & 0xff;	
		ioWrite(i*4, buf, 4);
		return 1;
	}
	else if(sscanf(CMD, "reg.read %d ", &i)==1){
		if(i>=32){	
			xprint("+err@%d.reg.read(%d)\r\n", brdAddr, i);
			return 1;
		}
		memset(buf,0,4);
		ioRead(i*4, buf, 4);
		j = buf[3];	j<<=8;
		j|=buf[2];	j<<=8;
		j|=buf[1];	j<<=8;
		j|=buf[0];
		xprint("+ok@%d.reg.read(%d,%d)\r\n", brdAddr, i, j);
		return 1;
	}
	return 0;
}

static void cb_newRcv(u16 rcvBytes){	// callback while there are receive data
	s32 i,j,k;
	s32 t0,t1;
	u8 buff[800] = {0};
	char* p;
	s32 *tmpBuf;

	handler_tcps->take_rcv(&handler_tcps->rsrc, buff, (rcvBytes>800?800:rcvBytes));
	// print("tcps rcv:%s", buff);
	if(rcvBytes > 800){
		handler_tcps->printS(&handler_tcps->rsrc, "+err@NET_PAYLOAD_LEN_OVERFLOAT\n");
		printS("+err@NET_PAYLOAD_LEN_OVERFLOAT\n");
		return;
	}
		
	for(i=0;i<800;i++){
		if(buff[i] == 0)	break;
		if(buff[i] == '(' || buff[i] == ')' || buff[i] == ',')	buff[i] = ' ';
		if(buff[i] >= 'A' && buff[i] <= 'Z')	buff[i] += 32;
	}
	// common command
	if(brdCmd((char*)buff, boardAddr, printS, print)){
		handler_tcps->send(&handler_tcps->rsrc, buff, strlen((char*)buff));
	}
	// response format: F4666:T01:23.4567;T02:14.4567;T03:25.4567;T04:26.4567;...T64:26.1234;
	else if(strncmp((char*)buff, "f4666\r\n", strlen("f4666\r\n")) == 0){
		tmpBuf = wlunTRemap(0);
		memset(buff,0,800);
		strcpy((char*)buff, "F4666:");
		p = (char*)&buff[6];
		for(i=0;i<WLUN_BRD_CNT*8;i++){
			strFormat(p, 800-(p-(char*)buff), "T%02d:%02d.%04d;", i+1, tmpBuf[i]/10000, tmpBuf[i]%10000);
			p = (char*)&buff[strlen((char*)buff)];
		}
		strcat((char*)buff, "\r\n");
		handler_tcps->printS(&handler_tcps->rsrc, (char*)buff);
	}
	// response temperature for wlun app
	else if(strncmp((char*)buff, "a4666\r\n", strlen("a4666\r\n")) == 0){
		tmpBuf = wlunTRemap(1);
		memset(buff,0,800);
		strcpy((char*)buff, "A4666:");
		p = (char*)&buff[6];
		for(i=0;i<WLUN_BRD_CNT*8;i++){
			strFormat(p, 800-(p-(char*)buff), "T%02d:%02d.%04d;", i+1, tmpBuf[i]/10000, tmpBuf[i]%10000);
			p = (char*)&buff[strlen((char*)buff)];
		}
		strcat((char*)buff, "\r\n");
		handler_tcps->printS(&handler_tcps->rsrc, (char*)buff);
	}
	// response heater volt for wlun app
	else if(strncmp((char*)buff, "a4000\r\n", strlen("a4000\r\n")) == 0){
		tmpBuf = wlunVRemap();
		memset(buff,0,800);
		strcpy((char*)buff, "A4000:");
		p = (char*)&buff[6];
		for(i=0;i<WLUN_BRD_CNT/2*8;i++){
			strFormat(p, 800-(p-(char*)buff), "V%02d:%02d.%04d;", i+1, tmpBuf[i]/10000, tmpBuf[i]%10000);
			p = (char*)&buff[strlen((char*)buff)];
		}
		strcat((char*)buff, "\r\n");
		handler_tcps->printS(&handler_tcps->rsrc, (char*)buff);
	}	
	else{	//forward to rs485
		i = strlen((char*)buff);
		rs485.Send(&rs485.rsrc, (u8*)buff, i);
	}
	
}
static void cb_connected(u8* ip, u16 port){
	char testStr[20] = "hello, world\n";
	print("sever connected to %d.%d.%d.%d, port[%d]\n", ip[0], ip[1], ip[2], ip[3], port);
	// handler_tcpc->send(&handler_tcpc->rsrc, (u8*)testStr, 20);
}
static void cb_closed(){
	printS("sever closed\n");
}

static void cb_listen(u16 port){
	print("server listen on port[%d]\n", port);
}

static void cb_newRcvClient(u16 rcvBytes){	// callback while there are receive data
	u8 rcvBuf[200] = {0};
	s32 rtn;
	memset(rcvBuf,0,200);
	memcpy(rcvBuf, "+ok@", 4);
	rtn = handler_tcpc->take_rcv(&handler_tcpc->rsrc, &rcvBuf[4], rcvBytes);
	print("client receive %d bytes:%s", rtn, &rcvBuf[4]);
	//handler_tcpc->closeSession(&handler_tcpc->rsrc);
	handler_tcpc->send(&handler_tcpc->rsrc, rcvBuf, rcvBytes+4);
}

static void cb_connectedClient(u8* ip, u16 port){
	char testStr[20] = "hello, world\n";
	print("client connected to %d.%d.%d.%d, port[%d]\n", ip[0], ip[1], ip[2], ip[3], port);
	// handler_tcpc->send(&handler_tcpc->rsrc, (u8*)testStr, 20);
}

static void cb_closedClient(){
	// printS("client closed\n");
}

static void cb_listenClient(u16 port){
	print("client listen on port[%d]\n", port);
}

static void cb_newRcvUdp(u16 rcvBytes){
	s32 rtn;
	u8 rcvBuf[200] = {0};
	UdpRsrc_t *pRsrc = &handler_udp->rsrc;
	memset(rcvBuf,0,200);
	memcpy(rcvBuf, "+ok@", 4);
	rtn = handler_udp->take_rcv(pRsrc, &rcvBuf[4], 200-4);
	print("UDP[%d.%d.%d.%d:%d] receive %d bytes:%s", 
	pRsrc->destNetInfo.ip[0],
	pRsrc->destNetInfo.ip[1],
	pRsrc->destNetInfo.ip[2],
	pRsrc->destNetInfo.ip[3],
	pRsrc->destNetInfo.port,
	rtn, &rcvBuf[4]);
	handler_udp->send(&handler_udp->rsrc, rcvBuf, rtn+4);
}
static void cb_closedUdp(){
	printS("UDP closed\n");
}

static s8 rs485BeforeSend_1(void){
	u16 x;
	if(HAL_GPIO_ReadPin(rs485.rsrc.DET.GPIOx, rs485.rsrc.DET.GPIO_Pin)==GPIO_PIN_SET){
		return -1;
	}
	HAL_GPIO_WritePin(rs485.rsrc.DE.GPIOx, rs485.rsrc.DE.GPIO_Pin, GPIO_PIN_SET);
	for(x=0;x<100;x++)	{NOP();}
	return 0;
}

static s8 rs485AfterSend_1(UART_HandleTypeDef *huart){
	if(huart->Instance == rs485.rsrc.uartdev.rsrc.huart->Instance){
		HAL_GPIO_WritePin(rs485.rsrc.DE.GPIOx, rs485.rsrc.DE.GPIO_Pin, GPIO_PIN_RESET);
		rs485.rsrc.uartdev.rsrc.flag |= BIT(0);
	}
	return 0;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle){}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	rs485.rsrc.uartdev.rsrc.afterSend(huart);
	if(huart->Instance == console.rsrc.huart->Instance){
		console.rsrc.flag |= BIT(0);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	psram.TxRxCpltCB(&psram.rsrc, hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	psram.TxRxCpltCB(&psram.rsrc, hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	psram.TxRxCpltCB(&psram.rsrc, hspi);
}


/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	psram.ErrorCB(&psram.rsrc, hspi);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
