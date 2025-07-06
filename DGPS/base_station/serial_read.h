#ifndef SERIAL_READ_H
#define SERIAL_READ_H

#define WIN32_LEAN_AND_MEAN				// winsock 관련 헤더들을 생략하고 필요한 최소 헤더만 포함
#include <windows.h>

// 시리얼 초기화 및 종료
int init_serial(const wchar_t* port_name);
void close_serial();

// 한 줄 읽기 (\n 기준)
int read_line_from_serial(char* out_buffer, size_t buffer_size);

#endif