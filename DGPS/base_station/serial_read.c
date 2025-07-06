#include "serial_read.h"

#include <stdio.h>
#include <stdlib.h>

static HANDLE hSerial = NULL;

int init_serial(const wchar_t* port_name) {
    hSerial = CreateFileW(port_name, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "시리얼 포트 열기 실패: %lu\n", GetLastError());
        return 0;
    }

    DCB dcb = { .DCBlength = sizeof(DCB) };
    if (!GetCommState(hSerial, &dcb)) {
        fprintf(stderr, "시리얼 설정 읽기 실패: %lu\n", GetLastError());
        CloseHandle(hSerial);
        return 0;
    }

    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcb)) {
        fprintf(stderr, "시리얼 설정 실패: %lu\n", GetLastError());
        CloseHandle(hSerial);
        return 0;
    }

    return 1;
}

void close_serial() {
    if (hSerial != NULL) {
        CloseHandle(hSerial);
        hSerial = NULL;
    }
}

int read_line_from_serial(char* out_buffer, size_t buffer_size) {
    if (hSerial == NULL) {
        fprintf(stderr, "시리얼 포트가 초기화되지 않았습니다.\n");
        return 0;
    }

    char ch;
    DWORD bytesRead;
    int index = 0;

    while (index < (int)(buffer_size - 1)) {
        if (ReadFile(hSerial, &ch, 1, &bytesRead, NULL) && bytesRead > 0) {
            if (ch == '\n') {
                break;
            }
            out_buffer[index++] = ch;
        }
    }

    out_buffer[index] = '\0';
    return index;
}
