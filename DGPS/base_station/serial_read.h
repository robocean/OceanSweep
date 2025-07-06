#ifndef SERIAL_READ_H
#define SERIAL_READ_H

#define WIN32_LEAN_AND_MEAN				// winsock ���� ������� �����ϰ� �ʿ��� �ּ� ����� ����
#include <windows.h>

// �ø��� �ʱ�ȭ �� ����
int init_serial(const wchar_t* port_name);
void close_serial();

// �� �� �б� (\n ����)
int read_line_from_serial(char* out_buffer, size_t buffer_size);

#endif