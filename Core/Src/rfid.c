/*
 * rfid.c
 *
 *  Created on: Dec 7, 2023
 *      Author: xgene
 *
 */
#include "rfid.h"


data_rfid rfidData;

QueueHandle_t xQueue_st;
//write infomation to card
uint8_t buffer[20];
uint8_t block;
StatusCode status;
uint8_t len;
extern QueueHandle_t RFIDdataQueue;

void blockRead(int block, char *data, uint8_t length) {
	MIFARE_Key key;
	for (uint8_t i = 0; i < 6; i++)
		key.keyByte[i] = 0xFF;
	StatusCode status;
	uint8_t rcvBuf[20] = { 0 };
	memset(rcvBuf, 0, 20);
	status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(rfidReader.uid));
	if (status != STATUS_OK) {
		SEGGER_RTT_printf(0, "Authentication failed: ");
		SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
		return;
	}

	status = MIFARE_Read(&rfidReader, block, rcvBuf, &length);
	if (status != STATUS_OK) {
		SEGGER_RTT_printf(0, "Reading failed: ");
		SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
		return;
	}
	memcpy(data, rcvBuf, length);

}

void readData(data_rfid *data)
{
	uint8_t fname[20] = {0};
	uint8_t tuoi[20] = {0};

	/* Read name */
	blockRead(1, (char *)fname, 20); 	// First name
	memcpy(data->Name, fname, 16);
	osDelay(100);

	/* Read age */
	blockRead(2, (char *)tuoi, 20); 	// Age
	uint8_t age = 0;
	sscanf(tuoi, "%d", &age);
	data->age = age;
	osDelay(100);

	PICC_HaltA(&rfidReader);
	PCD_StopCrypto1(&rfidReader);
}

void blockWrite(int block, char *data)
{
	  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
	  MIFARE_Key key;
	  for (uint8_t i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
	  StatusCode status;
	  if(data == NULL)
	  {
		  return;
	  }
	  memcpy(buffer, data, strlen(data));
	  buffer[strlen(data)] = '\0';

	  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfidReader.uid));
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "PCD_Authenticate() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "PCD_Authenticate() success: ");

	  // Write block
	  status = MIFARE_Write(&rfidReader, block, buffer, 16);
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "MIFARE_Write() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "MIFARE_Write() success: ");
}

void writeData(char *Name, uint8_t age) {

	SEGGER_RTT_printf(0, "Card UID:");    //Dump UID
	for (uint8_t i = 0; i < rfidReader.uid.size; i++) {
		SEGGER_RTT_printf(0, rfidReader.uid.uidByte[i] < 0x10 ? " 0" : " ");
		SEGGER_RTT_printf(0, "%02X", rfidReader.uid.uidByte[i]);
	}
	SEGGER_RTT_printf(0, " PICC type: ");   // Dump PICC type
	PICC_Type piccType = PICC_GetType(&rfidReader, rfidReader.uid.sak);
	SEGGER_RTT_printf(0, "%s", PICC_GetTypeName(&rfidReader, piccType));

	blockWrite(1, Name);

	char temp_buf[20] = {0};
	sprintf(temp_buf, "%d", age);
	blockWrite(2, temp_buf);

	PICC_HaltA(&rfidReader); // Halt PICC
	PCD_StopCrypto1(&rfidReader);  // Stop encryption on PCD
}


void RFIDTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for (;;) {
		if (PICC_IsNewCardPresent(&rfidReader) == false) {
//			SEGGER_RTT_printf(0, "Card not found!!!!!\r\n");
			osDelay(1000);
			continue;
		}
		SEGGER_RTT_printf(0, "Card found!\r\n");
		if (PICC_ReadCardSerial(&rfidReader) == true) {
			//PICC_DumpToSerial(&rfidReader, &rfidReader.uid);
		}
#ifdef WRITE_DATA
//		writeData("NGOC HOA", 18); 		// CARD 0
//		writeData("TUAN NGOC",	23);		// CARD 1
//		writeData("MAI HOA",	25);		// CARD 2
//		writeData("HONG PHUC",	26);		// CARD 3
//		writeData("VAN TEO",	20);		// CARD 4
//		writeData("NHAT MINH",	24);		// CARD 5
#else
		memset(&rfidData, 0u, sizeof(rfidData));
		readData(&rfidData);
		for (uint8_t i = 0; i < rfidReader.uid.size; i++) {
			rfidData.ID[i] = rfidReader.uid.uidByte[i];
		}
		if (xQueueSend(RFIDdataQueue, &rfidData , 10000) == pdPASS) {
			SEGGER_RTT_printf(0, "Send queue successfully\r\n");
		} else {
			SEGGER_RTT_printf(0, "Send queue unsuccessfully\r\n");
		}
#endif
		osDelay(3000);
	}







void readNameFromRFIDCard(void)
{

  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  MIFARE_Key key;
  for (uint8_t i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  //some variables we need
  uint8_t block;
  uint8_t len;
  StatusCode status;

  //-------------------------------------------

//  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
//  if ( ! PICC_IsNewCardPresent(&rfidReader)) {
//    return;
//  }
//
//  // Select one of the cards
//  if ( ! PICC_ReadCardSerial(&rfidReader)) {
//    return;
//  }

  SEGGER_RTT_printf(0, "**Card Detected:**");

  //-------------------------------------------

  //PICC_DumpDetailsToSerial(&rfidReader, &(rfidReader.uid)); //dump some details about the card

  //PICC_DumpToSerial(&rfidReader, &(rfidReader.uid));      //uncomment this to see all blocks in hex

  //-------------------------------------------

  SEGGER_RTT_printf(0, "Name: \n");

  uint8_t buffer1[18];
  len = 18;

//  block = 4;
//  len = 18;
//
//  //------------------------------------------- GET FIRST NAME
//  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(rfidReader.uid)); //line 834 of MFRC522.cpp file
//  if (status != STATUS_OK) {
//    SEGGER_RTT_printf(0, "Authentication failed: ");
//    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
//    return;
//  }
//
//  status = MIFARE_Read(&rfidReader, block, buffer1, &len);
//  if (status != STATUS_OK) {
//    SEGGER_RTT_printf(0, "Reading failed: ");
//    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
//    return;
//  }
//
//  //PRINT FIRST NAME
//
//    buffer1[16] = 0;
//
//    SEGGER_RTT_printf(0, "%s", buffer1);
//
//
//  SEGGER_RTT_printf(" ");

  //---------------------------------------- GET LAST NAME

  uint8_t buffer2[18];
  block = 1;

  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(rfidReader.uid)); //line 834
  if (status != STATUS_OK) {
    SEGGER_RTT_printf(0, "Authentication failed: ");
    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
    return;
  }

  status = MIFARE_Read(&rfidReader, block, buffer2, &len);
  if (status != STATUS_OK) {
    SEGGER_RTT_printf(0, "Reading failed: ");
    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
    return;
  }

  //PRINT LAST NAME
  buffer2[16] = 0;
  SEGGER_RTT_printf(0, "%s", buffer2 );



  //----------------------------------------

  SEGGER_RTT_printf(0, "\n**End Reading**\n");

  PICC_HaltA(&rfidReader);
  PCD_StopCrypto1(&rfidReader);
}
}











































