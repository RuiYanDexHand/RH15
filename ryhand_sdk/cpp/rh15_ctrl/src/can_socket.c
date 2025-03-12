#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "can_socket.h"

#define CAN_INTERFACE "can0"      // 替换为实际的 CAN 接口名称


int sock = 0;                      // CAN 套接字
struct sockaddr_can addr;          // CAN 地址
struct ifreq ifr;                  // 网络接口请求


// 请先在终端完成接口的配置
u8_t open_can_socket(int *psock, struct sockaddr_can *addr, struct ifreq *ifr) 
{

    // char command[128];

    // // 构建设置波特率的命令
    // snprintf(command, sizeof(command), "sudo ip link set %s type can bitrate 1000000", CAN_INTERFACE);

    // // 执行命令
    // if (system(command) != 0) 
    // {
    //     perror("Failed to set CAN bitrate");
    // }


    // 创建 SocketCAN 套接字
    if ((*psock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
    {
        perror("Socket creation failed");
        return 0;
    }

    // 获取接口索引
    strcpy(ifr->ifr_name, CAN_INTERFACE);
    if (ioctl(*psock, SIOCGIFINDEX, ifr) < 0) 
    {
        perror("ioctl: Failed to get interface index");
        return 0;
    }

    addr->can_family = AF_CAN;
    addr->can_ifindex = ifr->ifr_ifindex;

    // 绑定套接字到 CAN 接口
    if (bind(*psock, (struct sockaddr *)addr, sizeof(*addr)) < 0) 
    {
        perror("Bind failed");
        return 0;
    }
    printf("CAN socket opened successfully on interface: %s\n", CAN_INTERFACE);


    int flags = fcntl(*psock, F_GETFL, 0);
    fcntl(*psock, F_SETFL, flags | O_NONBLOCK);


    int buf_size;
    socklen_t len = sizeof(buf_size);

    // 获取接收缓冲区大小
    getsockopt(*psock, SOL_SOCKET, SO_RCVBUF, &buf_size, &len);
    printf("Receive buffer size: %d bytes\n", buf_size);

    // 获取发送缓冲区大小
    getsockopt(*psock, SOL_SOCKET, SO_SNDBUF, &buf_size, &len);
    printf("Send buffer size: %d bytes\n",  buf_size);

    return 1;
}



u8_t send_can_message(int sock, u8_t id, u8_t *data, u8_t len ) 
{
    struct can_frame frame;

    // 设置 CAN 帧内容
    frame.can_id = id;                  // 帧 ID
    frame.can_dlc = len;                // 数据长度
    for (int i = 0; i < len; i++)       // 数据内容
    {    
        frame.data[i] = data[i];
    }

    // 发送 CAN 帧
    if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) 
    {
        perror("Write failed");
        return 0;
    }

    return 1;
    
}




u8_t receive_can_message(int sock, struct can_frame *frame ) 
{
    // 接收 CAN 帧
    if ( read(sock, frame, sizeof(struct can_frame)) < 0 ) 
    {
        // perror("Read failed");
        return 0;
    }

    return 1;
}



void close_can_socket(int sock) 
{
    // 关闭 CAN 套接字
    if (close(sock) < 0) 
    {
        perror("Close failed");
        exit(EXIT_FAILURE);
    }
    printf("CAN socket closed\n");
}



