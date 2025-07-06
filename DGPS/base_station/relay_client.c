#include "relay_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

static SOCKET sock;

int init_relay_client(const char* host, int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup 실패\n");
        return 0;
    }

    struct addrinfo hints = { 0 }, * res;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", port);

    if (getaddrinfo(host, port_str, &hints, &res) != 0) {
        fprintf(stderr, "getaddrinfo 실패\n");
        WSACleanup();
        return 0;
    }

    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "소켓 생성 실패: %d\n", WSAGetLastError());
        freeaddrinfo(res);
        WSACleanup();
        return 0;
    }

    if (connect(sock, res->ai_addr, (int)res->ai_addrlen) == SOCKET_ERROR) {
        fprintf(stderr, "서버 연결 실패: %d\n", WSAGetLastError());
        closesocket(sock);
        freeaddrinfo(res);
        WSACleanup();
        return 0;
    }

    freeaddrinfo(res);
    return 1;
}

void send_to_server(const char* message) {
    if (send(sock, message, (int)strlen(message), 0) == SOCKET_ERROR) {
        fprintf(stderr, "데이터 전송 실패: %d\n", WSAGetLastError());
    }
}

void close_relay_client() {
    closesocket(sock);
    WSACleanup();
}
