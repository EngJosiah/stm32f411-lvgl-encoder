/*
 * gsm.c
 *
 *  Created on: Apr 12, 2021
 *      Author: EngJosiah
        a modified version if library by Nimaltd
 */
#include "gsm.h"
extern settings_t settings;
gsm_t gsm;
const char            GSM_ALWAYS_SEARCH[10][25] =
{
  "\r\n+CLIP:",               //  0
  "POWER DOWN\r\n",           //  1
  "\r\n+CMTI:",               //  2
  "\r\nNO CARRIER\r\n",       //  3
  "\r\n+DTMF:",               //  4
  "\r\n+CREG:",               //  5
  "\r\nCLOSED\r\n",           //  6
  "\r\n+CIPRXGET: 1\r\n",     //  7
  "\r\n+CCLK:"               //  8
};
//extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
uint8_t sendGsmCmd(uint8_t *cmd, uint32_t timeout, uint8_t *response, uint8_t *reply_1, uint8_t *reply_2){
	memset((char*)gsm.port.expected_reply_1,0,sizeof(gsm.port.expected_reply_1));
	memset((char*)gsm.port.expected_reply_2,0,sizeof(gsm.port.expected_reply_2));

	if(reply_1 != NULL) strcpy((char *) gsm.port.expected_reply_1, (char *)reply_1);
	if(reply_2 != NULL) strcpy((char *) gsm.port.expected_reply_2, (char *)reply_2);

	gsm.port.found_replies = 0;
	sendGsmData(cmd);
	uint32_t startTime = HAL_GetTick();
	  while (HAL_GetTick() - startTime < timeout)
	  {
	    if (gsm.port.found_replies != 0)
	      break;
	    osDelay(10);
	  }
	  if (response != NULL)
	  {
	    if (gsm.port.found_replies > 0)
	      strcpy((char *)response, (char *)gsm.port.rx_data);
	  }
	  return gsm.port.found_replies;

}
void sendGsmData(uint8_t *data){
	HAL_UART_Transmit_DMA(GSM_PORT, data, strlen((char *)data));
	rs485((char*)data);
}
void sendGsmBytes(uint8_t *data, uint16_t len){
	HAL_UART_Transmit_DMA(GSM_PORT, data, len);
	rs485((char*)data);
}

void gsmInterrupt(){

	if(RESET != __HAL_UART_GET_FLAG(GSM_PORT, UART_FLAG_IDLE))   //Judging whether it is idle interruption
	        {
	            __HAL_UART_CLEAR_IDLEFLAG(GSM_PORT);

	            HAL_UART_DMAStop(GSM_PORT);

	            //Calculate the length of the received data
	            gsm.port.received_bytes  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	            gsm.port.rx_buffer[gsm.port.received_bytes] = 0;
	            memset(gsm.port.rx_data, 0, BUFFER_SIZE);
	            memcpy(gsm.port.rx_data, gsm.port.rx_buffer, gsm.port.received_bytes);
	            gsm.port.stored_bytes = gsm.port.received_bytes;
	            gsm.port.received_bytes = 0;
	            osThreadFlagsSet(serialTaskHandle, _SERIAL_PORT_EVENT);
		  	    HAL_UART_Receive_DMA(&huart2, gsm.port.rx_buffer, BUFFER_SIZE);//start DMA Receiver
		  		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	        }
}

void checkGsmPort(){
	if(gsm.port.stored_bytes > 0){
		gsm.port.stored_bytes = 0;
		//search if there are replies
		char *found;
		if(gsm.port.expected_reply_1 != 0){
			found = strstr((char *)gsm.port.rx_data, (char *)gsm.port.expected_reply_1);
			if(found != NULL){
				gsm.port.found_replies = 1;
			}
		}
		if(gsm.port.expected_reply_2 != 0){
			found = strstr((char *)gsm.port.rx_data, (char *)gsm.port.expected_reply_2);
			if(found != NULL){
				gsm.port.found_replies = 2;
			}
		}

		//the usual searches now
		for (uint8_t i = 0; i < 10; i++)
		    {
		      char *str = strstr((char*)gsm.port.rx_data, GSM_ALWAYS_SEARCH[i]);
		      if(str != NULL)
		      {
		        switch (i)
		        {
		          case 0:   //  found   "\r\n+CLIP:"
		            #if (_GSM_CALL_ENABLE == 1)
		            if (sscanf(str,"\r\n+CLIP: \"%[^\"]\"", gsm.call_number) == 1)
		              gsm.call.ringing = 1;
		            #endif
		          break;
		          case 1:   //  found   "POWER DOWN\r\n"
		            gsm.powered = 0;
		            //gsm.started = 0;
		          break;
		          case 2:   //  found   "\r\n+CMT:" //\r\n+CMTI: \"SM\",1
					//sscanf(str, "\r\n+CMTI: \"SM\",%hhd",&gsm.msg.slot);//AT+CMGR=%d\r\n
					strtok(str, ",");
					sscanf(strtok(NULL,"\r\n"),"%d", &gsm.msg.slot);

		          break;
		          case 3:   //  found   "\r\nNO CARRIER\r\n"
		            #if (_GSM_CALL_ENABLE == 1)
		            gsm.call.callbackEndCall = 1;
		            #endif
		          break;
		          case 4:   //  found   "\r\n+DTMF:"
		            #if ((_GSM_DTMF_DETECT_ENABLE == 1) && (_GSM_CALL_ENABLE == 1))
		            if (sscanf(str, "\r\n+DTMF: %c\r\n", &gsm.call.dtmf) == 1)
		              osMessagePut(gsmDtmfQueueHandle, gsm.call.dtmf, 10);
		            #endif
		          break;
		          case 5:   //  found   "\r\n+CREG:"
		            if (strstr(str, "\r\n+CREG: 1\r\n") != NULL)
		              gsm.registered = 1;
		            else
		              gsm.registered = 0;
		          break;
		          case 6:   //  found   "\r\nCLOSED\r\n"
		            gsm.gprs.tcpConnection = 0;
		          break;
		          case 7:   //  found   "\r\n+CIPRXGET: 1\r\n"
		            gsm.gprs.gotData = 1;
		          break;
		          case 8:   //found "\r\n+CMGR:"
		        	  ;//CDC_Transmit_FS(str, strlen(str));
		          break;
		        }
		      }
		    //  --- search always
		    }
	}
}

void gsmInit(){
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	  HAL_UART_Receive_DMA(&huart2, gsm.port.rx_buffer, BUFFER_SIZE);//start DMA Receiver
	  gsm.port.received_bytes= 0;
	  for(int i = 0; i <4; i++){
		  sendGsmCmd((uint8_t *)"AT\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
		  osDelay(500);
	  }
	  sendGsmCmd((uint8_t *)"ATE0\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+COLP=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+CLIP=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+CREG=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+FSHEX=0\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+CLTS=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+CNMI=2,1,0,0,0\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+CTZU=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT&W\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sendGsmCmd((uint8_t *)"AT+CMGDA=\"DEL ALL\"\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
}

void gsmGpsPwr(bool on_off){
	  gsmSendCmd("AT+CGNSPWR=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  //gsmSendCmd("AT+CGPSPWR=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  //gsmSendCmd("AT+CGPS=1\r\n", 2000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  //gsmSendCmd("AT+CGNSSEQ=\"RMC\"\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);//AT+CGPSRST=0
}
void gsmSendSMS(){
	uint8_t buffer[257];
	  sendGsmCmd((uint8_t *)"AT+CMGF=1\r\n", 1000, NULL, (uint8_t *)"\r\nOK\r\n", NULL);
	  sprintf((char *)buffer, "AT+CMGS=\"%s\"\r\n", gsm.msg.recipient_number);
	  sendGsmCmd((uint8_t *)buffer, 1000, NULL, (uint8_t *)"\r\n>", NULL);
	  sprintf((char *)buffer, "%s%c", gsm.msg.outgoing_msg,26);
	  sendGsmCmd((uint8_t *)buffer, 1000, NULL, (uint8_t *)"\r\n+CMGS:", (uint8_t *)"\r\nERROR\r\n");
}
void gsmReadSMS(){
	//AT+CMGR=%d\r\n
	uint8_t buffer[255];
	sprintf((char *)buffer, "AT+CMGR=%03d\r\n", gsm.msg.slot);
	if(sendGsmCmd(buffer, 1000, NULL, (uint8_t *)"\r\n+CMGR:", (uint8_t *)"\r\nERROR\r\n") == 1){
		//+CMGR: "REC READ","+85291234567",,"07/02/18,00:12:05+32"
		strcpy((char *)buffer, strtok((char *)gsm.port.rx_data,"\r\n"));
		strcpy((char *)gsm.msg.received_msg, strtok(NULL, "\r\n"));
		strtok((char *)buffer, ",");
		strcpy((char *) gsm.msg.sender_number, strtok(NULL, "\""));
		gsmCallbackNewMsg(gsm.msg.sender_number, gsm.msg.received_msg);
		gsmDeleteSMS();
	}
}

void gsmCallbackNewMsg(char *number, char *msg)
{
  //CDC_Transmit_FS("NEW MESSAGE\r\n", 13);
	char out_msg[200];

	if(strstr(msg,"ON") != NULL){
		//HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_RESET);
		settings.state = 1;
		FlashWrite();
		sprintf(out_msg,"turned on http://maps.google.com/maps?q=%f,%f",gsm.gps.lat, gsm.gps.lon);
		gsmSendSMS(number, out_msg);
	}
	else if(strstr(msg,"OFF") != NULL){
		//HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_SET);
		settings.state = 0;
		FlashWrite();
		sprintf(out_msg,"turned off http://maps.google.com/maps?q=%f,%f",gsm.gps.lat, gsm.gps.lon);
		gsmSendSMS(number, out_msg);
	}
	else {
		gsmSendSMS(number, "The love of Jesus Christ is so deep, so sweet, so beautiful! It the most passionate love in the whole universe");
	}
}




void gsmDeleteSMS(){
	uint8_t buffer[64];
	  sprintf((char *)buffer, "AT+CMGD=%03d\r\n", gsm.msg.slot);
	  gsm.msg.slot = 0;
	  sendGsmCmd(buffer, 1000, NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n");
}

void getNetworkTime(){
	  uint8_t buffer[64];
	  if(sendGsmCmd((uint8_t *)"AT+CCLK?\r\n", 1000, buffer, (uint8_t *)"\r\n+CCLK:", (uint8_t *)"\r\nERROR\r\n") == 1){
		int yr, mon,day,hr,min,sec;
		int x=0;
		if (7 == sscanf((char *)buffer, "\r\n+CCLK: \"%d/%d/%d,%d:%d:%d+%d\"", &yr, &mon, &day, &hr, &min, &sec,&x)) {
			if (yr > 20) {
				gsm.time.hour = hr; gsm.time.minute = min; gsm.time.second = sec;
				gsm.time.day = day; gsm.time.month = mon; gsm.time.year = yr;
				if(gsm.time.year > 20)gsm.time.set = 1;
			}
		}
	  }
}

void waitForRegister(uint16_t sec){
	uint32_t start = HAL_GetTick();
	while(HAL_GetTick() - start < (sec * 1000)){
		gsmNetworkStatus();
		if(gsm.registered == 1) break;
		osDelay(3000);
	}
}

void gsmNetworkStatus(){
	if(sendGsmCmd((uint8_t *)"AT+CREG?\r\n", 1000, NULL, (uint8_t *)"\r\n+CREG: 1,1\r\n", (uint8_t *)"\r\nERROR\r\n") ==1){
		gsm.registered = 1;
	}
}

bool gsmGetImei(){
	  char str[32];
	  if (sendGsmCmd((uint8_t *)"AT+GSN\r\n", 1000 , (uint8_t *)str,  NULL, (uint8_t *)"\r\nERROR\r\n") == 2)
	    return false;
	  if (sscanf(str,"\r\n%[^\r\n]", gsm.IMEI) != 1)
	    return false;
	  if(gsm.IMEI != NULL) gsm.imei = 1;
	  return true;
}

bool gsmSetApn(uint8_t *apn_name){
	char str[128];
	  if (apn_name == NULL)
	    return false;
	  if (sendGsmCmd((uint8_t *)"AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") != 1)
	      return false;
	  sprintf(str, "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", apn_name);
	  if (sendGsmCmd((uint8_t *)str, 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") == 1)
	    return true;
	  return false;
}

bool gsmGprsOn(){
	  char str[64];
	  sendGsmCmd((uint8_t *)"AT+CGATT=1\r\n", 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n");
	  //("AT+SAPBR=0,1\r\n", 1000 , NULL, 0, 2, "\r\nOK\r\n", "\r\nERROR\r\n");
	  if (sendGsmCmd((uint8_t *)"AT+SAPBR=1,1\r\n", 85000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") != 1)
	    return false;
	  if (sendGsmCmd((uint8_t *)"AT+CGATT?\r\n", 1000 , (uint8_t *)str, (uint8_t *)"\r\n+CGATT: 1", (uint8_t *)"\r\nERROR\r\n") != 1)
	    {
	      gsm.gprs.on = false;
	      return false;
	    }
	  if (sendGsmCmd((uint8_t *)"AT+SAPBR=2,1\r\n", 1000 , (uint8_t *)str, (uint8_t *)"\r\n+SAPBR: 1,1,", (uint8_t *)"\r\nERROR\r\n") != 1)
	  {
	    //gsm.gprs.on = false;
	    return false;
	  }
	  sscanf(str, "\r\n+SAPBR: 1,1,\"%[^\"\r\n]", gsm.gprs.ip);
	  gsm.gprs.on = true;
	  gsm.gprs.onLast = true;
	  return true;
}

bool gsmGprsOff(){
	  if (sendGsmCmd((uint8_t *)"AT+SAPBR=0,1\r\n", 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") == 1)
	  {
	    gsm.gprs.on = false;
	    gsm.gprs.onLast = false;
	    return true;
	  }
	  return false;
}

bool gsmTcpConnect(uint8_t *address, uint16_t port, bool ssl){
	  if (gsm.gprs.on == 0)
	    return false;
	  if (ssl)
	  {
	    if (sendGsmCmd((uint8_t *)"AT+SSLOPT=1,1\r\n", 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") != 1)
	      return false;
	    if (sendGsmCmd((uint8_t *)"AT+CIPSSL=1\r\n", 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") != 1)
	      return false;
	  }
	  else
	  {
	    if (sendGsmCmd((uint8_t *)"AT+CIPSSL=0\r\n", 1000 , NULL, (uint8_t *)"\r\nOK\r\n", (uint8_t *)"\r\nERROR\r\n") != 1)
	      return false;
	  }
	  char str[128];
	  sprintf(str, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", address, port);
	  uint8_t answer = sendGsmCmd((uint8_t *)str, 10000 , NULL, (uint8_t *)"\r\nCONNECT OK\r\n", (uint8_t *)"\r\nALREADY CONNECT\r\n");
	  if ((answer == 1) || (answer == 2))
	  {
	    gsm.gprs.tcpConnection = 1;
	    return true;
	  }
	  return false;
}

bool gsmTcpSend(uint8_t *data, uint16_t len){
	  if (gsm.gprs.on == false)
	    return false;
	  if (gsm.gprs.tcpConnection == 0)
	    return false;
	  char str[32], str2[32];
	  bool err = false;
	sprintf(str, "AT+CIPSEND=%d\r\n", len);
	if (sendGsmCmd((uint8_t *)str, 10000 , (uint8_t *)str2, (uint8_t *)"\r\n>", (uint8_t *)"\r\nERROR\r\n") != 1){
		if(strstr(str2, "ERROR") != NULL){
			gsm.gprs.tcpConnection = 0;
		}
	}
    sendGsmBytes((uint8_t *)data, len);
    if (sendGsmCmd((uint8_t *)"", 10000 , (uint8_t *)str2, (uint8_t *)"\r\nSEND OK\r\n", (uint8_t *)"\r\nERROR\r\n") != 1){
    	if(strstr(str2, "ERROR") != NULL){
    		gsm.gprs.tcpConnection = 0;
    	}
    	return err;
    }
    return true;

}

void gsmTcpRead(){
		char str[64];
		memset(gsm.gprs.buff, 0, sizeof(gsm.gprs.buff));
		sprintf(str, "AT+CIPRXGET=2,%d\r\n", sizeof(gsm.gprs.buff) - 16);
		if (sendGsmCmd((uint8_t *)str, 1000, (uint8_t *)gsm.gprs.buff, (uint8_t *)"\r\n+CIPRXGET: 2", (uint8_t *)"\r\nERROR\r\n") == 1)
		{
		  uint16_t len = 0, read = 0;
		  if (sscanf((char*)gsm.gprs.buff, "\r\n+CIPRXGET: 2,%hd,%hd\r\n", &len, &read) == 2)
		  {

			//gsm_callback_gprsGotData(gsm.gprs.buff, len);
		  }
		}
}

void gsmHTTPPost(uint8_t * url, uint8_t *data){
	char buffer[64], buffer_2[200];
	int action,code,len=0;
	sendGsmCmd((uint8_t *)"AT+HTTPINIT\r\n", 12000, (uint8_t *)buffer, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");

	sendGsmCmd((uint8_t *)"AT+HTTPPARA=\"CID\",1\r\n", 1000, (uint8_t *)buffer, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");

	sprintf(buffer_2, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", (char *)url);
	sendGsmCmd((uint8_t *)buffer_2, 1000, (uint8_t *)buffer, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");


	sendGsmCmd((uint8_t *)"AT+HTTPPARA=\"Content-Type\",\"application/json\"\r\n", 1000, (uint8_t *)buffer, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");

	sprintf(buffer_2,"AT+HTTPDATA=\"%d\",2000\r\n", strlen((char *)data));
	sendGsmCmd((uint8_t *)buffer_2, 1000, (uint8_t *)buffer, (uint8_t *)"\r\nDOWNLOAD", (uint8_t *)"\r\nERROR\r\n");

	sendGsmCmd(data, 1000, (uint8_t *)buffer, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");

	if(sendGsmCmd((uint8_t *)"AT+HTTPACTION=1\r\n", 10000, (uint8_t *)buffer, (uint8_t *)"\r\n+HTTPACTION:", (uint8_t *)"\r\nERROR\r\n") == 1){
		sscanf(buffer,"\r\n+HTTPACTION: %d,%d,%d",&action,&code,&len);
	}
	if(len > 0){
		sendGsmCmd((uint8_t *)"AT+HTTPREAD\r\n", 1000, gsm.gprs.buff, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");
	}
	sendGsmCmd((uint8_t *)"AT+HTTPTERM\r\n", 1000, (uint8_t *)buffer, (uint8_t *)"\r\nOK", (uint8_t *)"\r\nERROR\r\n");
}
