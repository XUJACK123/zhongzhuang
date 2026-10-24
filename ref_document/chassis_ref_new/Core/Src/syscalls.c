#include <stddef.h>

/*
 * 固件不把标准输入输出重定向到电机或调试串口。显式提供 newlib 的最小
 * 系统调用，避免 nosys 库悄悄引入半主机依赖；调试数据统一走 UART7 遥测。
 */
int _close(int file)
{
    (void)file;
    return -1;
}

int _lseek(int file, int offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    return -1;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    return -1;
}

int _write(int file, const char *buffer, int length)
{
    (void)file;
    (void)buffer;
    return length;
}
