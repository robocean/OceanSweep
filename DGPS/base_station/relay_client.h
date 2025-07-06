#ifndef RELAY_CLIENT_H
#define RELAY_CLIENT_H

#include <winsock2.h>

int init_relay_client(const char* host, int port);  // 서버 연결, 성공 시 1
void send_to_server(const char* message);           // 서버로 문자열 전송
void close_relay_client();                          // 연결 종료

#endif