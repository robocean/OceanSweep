#include "serial_read.h"
#include "relay_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SERVER_IP	"YOUR_SERVER_IP"
#define SERVER_PORT 58080

#define BUFFER_SIZE 1024
#define SERIAL_PORT L"\\\\.\\COM11"		// 유니코드 문자열로 COM 포트 지정, 포트는 사용자 환경에 따라 달라짐

int main() {

    char buffer[BUFFER_SIZE];
    char message[BUFFER_SIZE];

    if (!init_serial(SERIAL_PORT)) {
        return 1;
    }

    if (!init_relay_client(SERVER_IP, SERVER_PORT)) {
        close_serial();
        return 1;
    }

    printf("[시작] 시리얼 수신 및 서버 전송 루프 시작...\n");

    while (1) {
        int len = read_line_from_serial(buffer, sizeof(buffer));

        if (len > 0) {
            // \r 제거
            if (buffer[len - 1] == '\r') {
                buffer[len - 1] = '\0';     //null 종료문자 추가
                len--;
            }

            // \n 추가한 새 문자열 준비
            snprintf(message, sizeof(message), "%s\n", buffer); //안전하게 포맷팅 하기위해 원본데이터를 복사하여 개행문자 추가

            printf("[수신] %s\n", buffer);
            send_to_server(message);
        }
    }

    close_serial();
    close_relay_client();

    return 0;
}
