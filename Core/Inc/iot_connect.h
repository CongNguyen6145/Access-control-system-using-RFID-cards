#ifndef IOT_CONNECT_H_
#define IOT_CONNECT_H_



void LWIPTask(void *argument);

int http_post(char *host, int port, char *path, char *payload, uint32_t length);
#endif
