/*
 * iot_connect.c
 *
 *  Created on: Dec 7, 2023
 *      Author:
 */

#include "string.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>
#include "lwipopts.h"
#include "lwip.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/debug.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "iot_connect.h"
#include "cmsis_os.h"
#include "rfid.h"
#include "core_http_client.h"

osThreadId_t HTTPTaskHandle;
const osThreadAttr_t httpHandle_attributes = {
  .name = "HTTPTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};

extern struct netif gnetif;
extern QueueHandle_t   RFIDdataQueue;
extern QueueHandle_t   ledStatusQueue;
int lwsock = -1;
uint8_t http_rx_buffer[2048];
uint8_t http_tx_buffer[2048];

uint8_t responseBuff[512];


uint8_t *pResponse;
size_t responseSize;

NetworkContext_t nwContext;

int platform_connect(char *host, int port) 
{
	if (host == NULL) {
		return -1; // Invalid host name
	}

	// Create a TCP socket
	lwsock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (lwsock == -1) {
		return lwsock; // Socket creation failed
	}
#if 0
	// Resolve the host name to an IP address
	struct hostent *host_entry = lwip_gethostbyname(host);
	if (host_entry == NULL) {
		// DNS resolution failed
		lwip_close(lwsock);
		return -1;
	}

	// Declare a variable to socket address
	ip_addr_t resolved_ip = *(ip_addr_t*) (host_entry->h_addr_list[0]);
	//Set up the socket address structure
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_len = sizeof(addr);
	addr.sin_family = AF_INET;
	addr.sin_port = PP_HTONS(port);
	addr.sin_addr.s_addr = resolved_ip.addr;
#else
	//Set up the socket address structure
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_len = sizeof(addr);
	addr.sin_family = AF_INET;
	addr.sin_port = PP_HTONS(port);
	addr.sin_addr.s_addr = inet_addr(host);

#endif


	// Connect to the server
	int ret = lwip_connect(lwsock, (struct sockaddr*) &addr, sizeof(addr));
	if (ret == -1) {
		lwip_close(lwsock);  // Close the socket on failure
	}

	return ret;
}

int platform_disconnect(void) 
{
	int _s = lwsock;
	lwsock = -1;
	if (_s < 0)
		return _s;
	return lwip_close(_s);
}

int32_t platform_receive( NetworkContext_t * pNetworkContext, void * pBuffer, size_t bytesToRecv )
{
    if (pNetworkContext == NULL || pBuffer == NULL) {
        return -1;
    }
    int receivedBytes = lwip_read(*(pNetworkContext->socket) , pBuffer, bytesToRecv);
    if (receivedBytes < 0) {
        SEGGER_RTT_printf(0, ("Receive message failed: %d\n"), receivedBytes);
        return -1;
    }else{
        SEGGER_RTT_printf(0, "Receive message success, raw data: \n\n%s\n\n", (char *)pBuffer);
    }
    return receivedBytes;
}

int32_t platform_send( NetworkContext_t * pNetworkContext, const void * pBuffer, size_t bytesToSend )
{
	if (pNetworkContext == NULL || pBuffer == NULL) {
		return -1;
	}
	int sentBytes = lwip_write(*(pNetworkContext->socket), pBuffer, bytesToSend);
	if (sentBytes < 0) {
		SEGGER_RTT_printf(0, ("Send message %d fail: %d\n"), sentBytes);
		return -1;
	}else
	SEGGER_RTT_printf(0, "Send message success\n");
	return sentBytes;
}

uint32_t GetCurrentTimeStamp ( void )
{
	return (TickType_t)xTaskGetTickCount();
}

void http_init(void)
{

}

void http_get(char *host, int port, char *path, char *payload, uint32_t length) 
{
	size_t reqBodyLen = 0;
	uint32_t sendFlags = 0;
	TransportInterface_t mTransport;
	//---------------------------------------------------------------//
	HTTPRequestHeaders_t mRequestHeaders;
	HTTPRequestInfo_t mRequestInfo;
	HTTPResponse_t mResponse;
	//---------------------------------------------------------------//
	mResponse.pBuffer = http_rx_buffer;
	mResponse.bufferLen = 2048;
	mResponse.getTime = GetCurrentTimeStamp;
	mRequestHeaders.pBuffer = http_tx_buffer;
	mRequestHeaders.bufferLen = 1024;
	//---------------------------------------------------------------//
	mTransport.recv = platform_receive;
	mTransport.pNetworkContext = &nwContext;
	mTransport.send = platform_send;
	//---------------------------------------------------------------//
	nwContext.host = host;
	nwContext.port = port;
	nwContext.socket = &lwsock;
	//---------------------------------------------------------------//
	mRequestInfo.pHost = host;
	mRequestInfo.hostLen = strlen(host);
	mRequestInfo.pPath = path;
	mRequestInfo.pathLen = strlen(path);
	mRequestInfo.pMethod = HTTP_METHOD_GET;
	mRequestInfo.methodLen = strlen((char*) HTTP_METHOD_GET);
	//---------------------------------------------------------------//

	if (host == NULL || path == NULL) {
		return;
	}
	HTTPClient_InitializeRequestHeaders(&mRequestHeaders, &mRequestInfo);

	// Connect
	if (platform_connect(host, port) != 0) {
		SEGGER_RTT_printf(0, "Failed to connect\n");
		if (lwsock != -1) {
			platform_disconnect();
		}
		return;
	}else
	SEGGER_RTT_printf(0, "Connection successful\n");

	if (payload != NULL) {
		SEGGER_RTT_printf(0, "pay load != NULL");
		reqBodyLen = strlen(payload);
	}else
		SEGGER_RTT_printf(0, "pay load = NULL %s\n");

	if (HTTPClient_SendHttpHeaders(&mTransport, GetCurrentTimeStamp,
			&mRequestHeaders, reqBodyLen, sendFlags) != HTTPSuccess) {
		// Error
		SEGGER_RTT_printf(0, "Send header Fail\n");
		platform_disconnect();
		return;
	}else
	SEGGER_RTT_printf(0, "Send header success\n");
	// Send header successful
	if (payload != NULL) {
		//TO-DO
//		 sendHttpBody( const TransportInterface_t * pTransport,
//		                                  HTTPClient_GetCurrentTimeFunc_t getTimestampMs,
//		                                  const uint8_t * pRequestBodyBuf,
//		                                  size_t reqBodyBufLen )
	}


	if (HTTPClient_ReceiveAndParseHttpResponse(&mTransport, &mResponse,
			&mRequestHeaders) != HTTPSuccess) {
		SEGGER_RTT_printf(0, "Receive And Parse Http Response Fail\n");
		// Error
	}

	SEGGER_RTT_printf(0, "Receive And Parse Http Response success\n");
	SEGGER_RTT_printf(0, "Response Header: %s \n", mResponse.pHeaders);
	SEGGER_RTT_printf(0, "Response Code: %d \n", mResponse.statusCode);

	// Disconnect - TO-DO
	platform_disconnect();
}

int http_post(char *host, int port, char *path, char *payload, uint32_t length) 
{
	size_t reqBodyLen = 0;
	uint32_t sendFlags = 0;
	int ans = -1;

	char fullHost[32];
	TransportInterface_t mTransport;
	//---------------------------------------------------------------//
	HTTPRequestHeaders_t mRequestHeaders;
	HTTPRequestInfo_t mRequestInfo;
	HTTPResponse_t mResponse;
	//---------------------------------------------------------------//
	memset(http_rx_buffer, 0, 2048);
	memset(http_tx_buffer, 0, 2048);
	mResponse.pBuffer = http_rx_buffer;
	mResponse.bufferLen = 2048;
	mResponse.getTime = GetCurrentTimeStamp;
	mRequestHeaders.pBuffer = http_tx_buffer;
	mRequestHeaders.bufferLen = 2048;
	//---------------------------------------------------------------//
	mTransport.recv = platform_receive;
	mTransport.pNetworkContext = &nwContext;
	mTransport.send = platform_send;
	//---------------------------------------------------------------//
	nwContext.host = host;
	nwContext.port = port;
	nwContext.socket = &lwsock;
	//---------------------------------------------------------------//

	snprintf(fullHost, 32, "%s:%d", host, port);
	mRequestInfo.pHost = fullHost;
	mRequestInfo.hostLen = strlen(fullHost);
	mRequestInfo.pPath = path;
	mRequestInfo.pathLen = strlen(path);
	mRequestInfo.pMethod = HTTP_METHOD_POST;
	mRequestInfo.methodLen = strlen((char*) HTTP_METHOD_POST);
	//---------------------------------------------------------------//

	if (host == NULL || path == NULL) {
		return -1;
	}
	HTTPClient_InitializeRequestHeaders(&mRequestHeaders, &mRequestInfo);

	HTTPClient_AddHeader( &mRequestHeaders,
	                                   "Content-Type",
	                                   strlen("Content-Type"),
	                                   "application/json",
	                                   strlen("application/json") );
	HTTPClient_AddHeader( &mRequestHeaders,
	                                   "Accept",
	                                   strlen("Accept"),
	                                   "*/*",
	                                   strlen("*/*") );
	SEGGER_RTT_printf(0, "%s\n", mRequestHeaders.pBuffer);
	// Connect
	if (platform_connect(host, port) != 0) {
		SEGGER_RTT_printf(0, "Failed to connect\n");
		if (lwsock != -1) {
			platform_disconnect();
		}
		return -1;
	}else{
		SEGGER_RTT_printf(0, "Connection successful\n");
	}


	if (payload != NULL) {
		SEGGER_RTT_printf(0, "pay load != NULL\n");
		reqBodyLen = strlen(payload);
	}else
	{
		SEGGER_RTT_printf(0, "pay load = NULL\n");
	}

	if(HTTPClient_Send((const TransportInterface_t *)&mTransport,
	                              (HTTPRequestHeaders_t *)&mRequestHeaders,
	                              (const uint8_t *)payload,
	                              (size_t)reqBodyLen,
	                              (HTTPResponse_t *)&mResponse,
								  0) == HTTPSuccess)
	{
		SEGGER_RTT_printf(0, "HTTPClient_Send success\n");
	}else{
		SEGGER_RTT_printf(0, "HTTPClient_Send fail\n");
	}

	if(mResponse.statusCode == 201)
	{

		ans = 0;
	}
	memset(&responseBuff, 0, 512);
	memcpy(responseBuff, mResponse.pBody, mResponse.bodyLen);
	pResponse = mResponse.pBody;
	responseSize = mResponse.bodyLen;

	SEGGER_RTT_printf(0, "Receive And Parse Http Response success\n");
	SEGGER_RTT_printf(0, "Response Header: %s \n", mResponse.pHeaders);
	SEGGER_RTT_printf(0, "Response Code: %d \n", mResponse.statusCode);

	// Disconnect - TO-DO
	platform_disconnect();
	return ans;
}


void httpHandleTask(void * argument)
{
	SEGGER_RTT_printf(0, "Begin httpHandleTask() \r\n");
	data_rfid rcvData;
	char payload[256] = {0};
	int ledStatus = 0;
	while (1) 
	{
		memset(&rcvData, 0, sizeof(data_rfid));
		if (xQueueReceive(RFIDdataQueue, &rcvData, portMAX_DELAY) == pdPASS)
		{
			/* Prepare the payload */
			memset(payload, 0, sizeof(payload));
			snprintf(payload, 256, "{\"name\":\"%s\",\"age\":%d,\"uid\":\"%02X %02X %02X %02X\"}\r\n",
			rcvData.Name, rcvData.age, rcvData.ID[0], rcvData.ID[1], rcvData.ID[2], rcvData.ID[3]);
			SEGGER_RTT_printf(0, "Payload: %s\r\n", payload);

			if(http_post("146.190.83.57", 3000,"/api/test", payload, strlen(payload)) == 0)
			{
				//"{"access": "allowed"}"
				//"{"access": "denied"}"
				// HTTP request sent successfully.

				if(strstr((const char *)responseBuff, "\"Access Allow\"") != NULL  && strstr((const char*)responseBuff, "\"success\"") != NULL)
				{
					// Open the door (Turn green led on)
					ledStatus = 1;
					if(xQueueSend(ledStatusQueue, &ledStatus, 10000) == pdPASS)
					{
						SEGGER_RTT_printf(0, "Successfully send signal turn on green led \r\n");
					}
				}

			}else{
				if(strstr((const char*)responseBuff, "Not Acceptable") != NULL  && strstr((const char*)responseBuff, "not allow to access!") != NULL)
				{
					// Close the door (Turn red led on)
					ledStatus = 0;
					if(xQueueSend(ledStatusQueue, &ledStatus, 10000) == pdPASS)
					{
						SEGGER_RTT_printf(0, "Successfully send signal turn on red led \r\n");
					}
				}else{
					;
				}

			}
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}	
}

void LWIPTask(void *argument)
{
	SEGGER_RTT_printf(0, "Begin LWIPTask() \r\n");
	MX_LWIP_Init();
	while (1)
	{
		if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0
				|| gnetif.gw.addr == 0)
		{
			osDelay(200);
			continue;
		}
		else
		{

			SEGGER_RTT_printf(0, "\r\n\nDHCP/Static IP: %s\r\n\n",
					ip4addr_ntoa(netif_ip4_addr(&gnetif)));
			break;
		}
	}
	HTTPTaskHandle = osThreadNew(httpHandleTask, NULL, &httpHandle_attributes);

	for (;;)
	{
		osDelay(1);
	}
}

