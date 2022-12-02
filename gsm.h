/*
 * gsm.h
 *
 *  Created on: Apr 12, 2021
 *      Author: EngJosiah
 */

#ifndef INC_GSM_H_
#define INC_GSM_H_


#include "main.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../FLASH/flash.h"

#define BUFFER_SIZE 256

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define GSM_PORT &huart2

typedef struct {
	uint8_t          rx_buffer[BUFFER_SIZE];
	uint8_t          rx_data[BUFFER_SIZE];
	uint8_t          received_bytes;
	uint8_t          stored_bytes;
	uint8_t          expected_reply_1[20];
	uint8_t          expected_reply_2[20];
	uint8_t          expected_reply_3[20];
	uint8_t          found_replies;
} serial_port_t;

typedef struct{
	float lat;
	float lon;
	char buffer[100];
}gsm_gps_t;

typedef struct {
	uint8_t          received_msg[BUFFER_SIZE];
    uint8_t          sender_number[16];
	uint8_t          outgoing_msg[BUFFER_SIZE];
	uint8_t          recipient_number[16];
	uint8_t          sender_name[20];
	uint8_t          msg_time_str[23];
	uint8_t          slot;
	uint8_t          send;
}msg_t;

typedef struct {
	  uint8_t       year;
	  uint8_t       month;
	  uint8_t       day;
	  uint8_t       hour;
	  uint8_t       minute;
	  uint8_t       second;
	  uint8_t       set;
}network_time_t;

typedef struct
{
  bool            on;
  bool            onLast;
  char            ip[32];
  uint16_t        port;
  uint16_t        index;
  uint32_t        dataLen;
  uint32_t        dataCurrent;
  int16_t         code;
  uint8_t         buff[BUFFER_SIZE];
  uint8_t         post_data[150];
  uint8_t         post;
  uint8_t         tcpConnection;
  uint8_t         gotData;

}gprs_t;

typedef struct {
	uint8_t           powered;
	uint8_t           registered;
	uint8_t           signal;
	uint8_t           IMEI[16];
	uint8_t           imei;
	serial_port_t     port;
	msg_t             msg;
	network_time_t    time;
	gprs_t            gprs;
	gsm_gps_t         gps;
}gsm_t;



#define _SERIAL_PORT_EVENT 0x01

extern osThreadId gsmTaskHandle;
extern osThreadId serialTaskHandle;

uint8_t sendGsmCmd(uint8_t *cmd, uint32_t timeout, uint8_t *response, uint8_t *reply_1, uint8_t *reply_2);
void checkGsmPort();
void gsmInterrupt();
void sendGsmData(uint8_t *data);
void sendGsmBytes(uint8_t *data, uint16_t len);
void gsmInit();
void gsmSendSMS();
void gsmReadSMS();
void gsmDeleteSMS();
void gsmCallbackNewMsg(char *number, char *msg);
void getNetworkTime();
void gsmNetworkStatus();
void waitForRegister(uint16_t sec);

bool gsmSetApn(uint8_t *apn_name);
bool gsmGprsOn();
bool gsmGprsOff();

bool gsmGetImei();

bool gsmTcpConnect(uint8_t *address, uint16_t port, bool ssl);
bool gsmTcpSend(uint8_t *data, uint16_t len);
void gsmTcpRead();

void gsmHTTPPost(uint8_t * url, uint8_t *data);


#endif /* INC_GSM_H_ */
