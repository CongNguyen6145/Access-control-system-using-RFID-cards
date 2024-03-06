#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "stdbool.h"
#include "MFRC522.h"
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"


//#define WRITE_DATA

typedef struct Data_RFID
{
  uint8_t age;
  char Name[16];
  uint32_t ID[16];
}data_rfid;

extern MFRC rfidReader;
extern data_rfid rfidData;

void readCard (data_rfid *infor_rfid);
void writeData(char *Name, uint8_t age);
void RFIDTask(void *argument);
