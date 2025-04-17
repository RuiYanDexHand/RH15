/*********************************************************************
File name:      can_socket.h
Author:         zhanglf
Version:        v1.0
Date:           2025.03.11
Description:    can_socket 驱动接口
								
Others:         

History:        
*********************************************************************/


#ifndef __CAN_SOCKET_H_
#define __CAN_SOCKET_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include "stdbool.h"
#include "ryhandlib.h"


extern int sock;                          // CAN 套接字
extern struct sockaddr_can addr;          // CAN 地址
extern struct ifreq ifr;                  // 网络接口请求

extern u8_t open_can_socket(int *sock, struct sockaddr_can *addr, struct ifreq *ifr, const char *can_interface);
extern u8_t send_can_message(int sock, u8_t id, u8_t *data, u8_t len );
extern u8_t receive_can_message(int sock, struct can_frame *frame );
extern void close_can_socket(int sock);




#endif
