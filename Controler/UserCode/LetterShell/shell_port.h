/**
 * @file shell_port.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/6
  */
#ifndef CONTROLER_SHELL_PORT_H
#define CONTROLER_SHELL_PORT_H

#include "usart.h"

short uart_charPut(char *data, unsigned short len);

#endif //CONTROLER_SHELL_PORT_H
