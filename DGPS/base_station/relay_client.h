#ifndef RELAY_CLIENT_H
#define RELAY_CLIENT_H

#include <winsock2.h>

int init_relay_client(const char* host, int port);  // ���� ����, ���� �� 1
void send_to_server(const char* message);           // ������ ���ڿ� ����
void close_relay_client();                          // ���� ����

#endif