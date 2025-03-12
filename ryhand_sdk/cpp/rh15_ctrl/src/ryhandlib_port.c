/*********************************************************************
File name:      ryhandlib.c
Author:         zhanglf
Version:        v1.0
Date:           2024.11.07
Description:    ryhandlib库对外接口实现
								
Others:         

History:        
*********************************************************************/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ryhandlib.h"
#include <can_socket.h>



RyCanServoBus_t stuServoCan;
CanMsg_t stuListenMsg[40];
ServoData_t sutServoDataW[15];
ServoData_t sutServoDataR[15];
volatile s16_t  uwTick = 0;



//****************************************************************************** 
// 函数名称:  DelayInit
// 函数功能:  Delay功能初始化
// 输入参数:  无
// 返回值:    无
//****************************************************************************** 
void MyHoockCallBck(CanMsg_t stuMsg, void * para)
{
	(void)stuMsg;
    (void)para;
	// 在这里完成用消息的异步解析，
	
}

	
//****************************************************************************** 
// 函数名称:  BusWrite
// 函数功能:  ryhandlib 库操作用的 消息发送 硬件实现
// 输入参数:  stuMsg - 要发送的消息对象
// 返回值:    OK - 成功 ;  ERROR - 发送失败
//****************************************************************************** 
s8_t BusWrite(CanMsg_t stuMsg)
{
		s8_t ret = OK;
#if 0
	
		// 如果用户想实现异步操作( 在使用库接口是，超时参数给的是0)，可以由用户自行添加钩子，操作方式如下，
		// 这种方式式，如果用户关系返回数据内容，则可以在钩子的回调函数中调用对应的异步解析结果，如果用户
		// 不关心设备返回的消息内容，则可以不添加钩子！！
	
		// 如果用户自行添加过钩子，且钩子成功匹配，那么该钩子会自动删除，如果钩子未成功匹配，且程序确认已
		// 超时，那么需用户自行删除该钩子，否则，后续的接口可能会无法分配到钩子，而导致执行失败。
			stuListenMsg[i].ulId = stuMsg.ulId + 256;
			stuListenMsg[i].pucDat[0] = stuMsg.pucDat[0];
			AddHook( &stuServoCan,stuListenMsg, MyHoockCallBck );
	
#endif	


		// printf("<T> %ld : ", stuMsg.ulId);
		// for (int i = 0; i < stuMsg.ucLen; i++)
		// {
		// 	printf("%02x ", stuMsg.pucDat[i]);
		// }
		// printf("\n");


		// 通过 CAN 总线发送消息
		if( sock > 0 )
		{
			if( send_can_message(sock, stuMsg.ulId, stuMsg.pucDat, stuMsg.ucLen ) ) 
				ret = OK;
			else ret = ERROR;
		}
		else
		{
			ret = ERROR;
		}
	
		return ret;
}



//****************************************************************************** 
// 函数名称:  CallBck0
// 函数功能:  监听事件回调函数0，用于一个监听对象成功监听后的回调
// 输入参数:  
//     stuMsg - 成功监听到的消息对象
//     para   - 监听事件传入的参数，一般是监听对象自身地址
// 返回值:    无
//****************************************************************************** 
void CallBck0(CanMsg_t stuMsg, void * para)
{
    u8_t id = stuMsg.ulId;
	(void)para;
	// ServoData_t *psutServoData = (ServoData_t *)stuMsg.pucDat;

#if 1
		
	// 测试用，如果发现有电机处于错误状态，可以在这里进行处理
	if( stuMsg.pucDat[1] == enServo_CurrentOverE )
	{
		// 处理错误
		RyParam_ClearFault( &stuServoCan, id, 1 );
	}
		
#endif

	// 收集电机数据
	switch( stuMsg.pucDat[0] )
	{
		case 0xa0:
		case 0xa1:
		case 0xa6:
		case 0xa9:
		case 0xaa:
			if( id && (id < 0x10) )
				sutServoDataR[ id-1 ] = *(ServoData_t *)stuMsg.pucDat;
			break;

		default:
			break;
	}

}






//******************************************************************************
// 函数名称:  rad_to_deg
// 函数功能:  弧度转角度
// 输入参数:  rad - 弧度
// 返回值:    角度
//******************************************************************************
float rad_to_deg(float rad) 
{
	return rad * 180 / M_PI;
}

//******************************************************************************
// 函数名称:  deg_to_rad
// 函数功能:  角度转弧度
// 输入参数:  deg - 角度
// 返回值:    弧度
//******************************************************************************
float deg_to_rad(float deg) 
{
	return deg * M_PI / 180;
}




//******************************************************************************
// 函数名称:  map_rad90_to_value
// 函数功能:  将弧度映射到0-4095的值
// 输入参数:  rad - 弧度值
// 返回值:    映射后的值
//******************************************************************************
int map_rad90_to_value(float rad) 
{
    if (rad < 0) 
	{
        rad = 0;
    } 
	else if (rad > M_PI / 2) 
	{
        rad = M_PI / 2;
    }
    return (int)(4095 - (rad / (M_PI / 2)) * 4095);
}


//******************************************************************************
// 函数名称:  map_rad75_to_value
// 函数功能:  将弧度映射到0-4095的值
// 输入参数:  rad - 弧度值
// 返回值:    映射后的值
//******************************************************************************
int map_rad75_to_value(float rad) 
{
    if (rad < 0) 
	{
        rad = 0;
    } 
	else if (rad > M_PI * 75 / 180) 
	{
        rad = M_PI * 75 / 180;
    }
    return (int)(4095 - (rad / (M_PI * 75 / 180)) * 4095);
}





//******************************************************************************  
// 函数名称:  value_to_rad90  
// 函数功能:  将0-4095的值映射回弧度(0到π/2)  
// 输入参数:  value - 数值  
// 返回值:    映射后的弧度  
//******************************************************************************  
float value_to_rad90(int value) 
{
    if (value < 0) 
	{
        value = 0;
    } 
	else if (value > 4095) 
	{
        value = 4095;
    }
    return (M_PI / 2) * (4095 - value) / 4095;
}


//******************************************************************************  
// 函数名称:  value_to_rad75  
// 函数功能:  将0-4095的值映射回弧度(0到75度, 即0到M_PI*75/180)  
// 输入参数:  value - 数值  
// 返回值:    映射后的弧度  
//******************************************************************************  
float value_to_rad75(int value) 
{
    if (value < 0) 
	{
        value = 0;
    } 
	else if (value > 4095) 
	{
        value = 4095;
    }
    return (M_PI * 75 / 180) * (4095 - value) / 4095;
}


//******************************************************************************  
// 函数名称:  value_to_rad_full_range  
// 函数功能:  将-4095到4095的值映射回弧度(-π/2到π/2)    
// 输入参数:  value - 数值  
// 返回值:    映射后的弧度 
//******************************************************************************  
float value_to_rad_full_range(int value) 
{
    if (value < -4095) 
	{
        value = -4095;
    } 
	else if (value > 4095) 
	{
        value = 4095;
    }
    return (float)value / 4095 * (M_PI / 2);
}




//******************************************************************************
// 函数名称:  evaluatePolynomial
// 函数功能:  计算多项式的值
// 输入参数:  coefficients - 多项式系数
//            degree - 多项式的次数 p = 0.000504 -0.043255 2.561668 0.148494
//            x - 自变量, 单位是度
// 返回值:    多项式的值，单 位时度
//******************************************************************************
double evaluatePolynomial(double coefficients[], int degree, double x)
{
	double result = 0;

    for (int i = 0; i <= degree; ++i) 
	{
        result += coefficients[degree - i] * pow(x, i);
    }

    return result;
}








//******************************************************************************
// 函数名称:  map_rad_to_value_full_range
// 函数功能:  将弧度映射到0-4095的值
// 输入参数:  rad - 弧度值
// 返回值:    映射后的值
//******************************************************************************
int map_rad_to_value_full_range(float rad)
 {
    if (rad < -M_PI / 2) 
	{
        rad = -M_PI / 2;
    } 
	else if (rad > M_PI / 2) 
	{
        rad = M_PI / 2;
    }
    return (int)((rad / (M_PI / 2)) * 4095);
}




//******************************************************************************
// 函数名称:  update_motor_positions
// 函数功能:  更新电机位置
// 输入参数:  rads - 电机角度，15个电机角度 0-14
//            hand_lr - 左右手 ： 0 - 左手； 1 - 右手
// 返回值:    无
//******************************************************************************
void update_motor_positions( float *rads, int hand_lr ) 
{

    for (int i = 0; i < 15; i++) 
	{
        if (i % 3 != 0) 
		{
            if (rads[i] < 0) 
			{
                rads[i] = 0;
            } 
			else if (rads[i] > M_PI * 90 / 180) 
			{
                rads[i] = M_PI * 90 / 180;
            }
        } 
		else 
		{
            if (rads[i] < M_PI * -30 / 180) 
			{
                rads[i] = M_PI * -30 / 180;
            } 
			else if (rads[i] > M_PI * 30 / 180) 
			{
                rads[i] = M_PI * 30 / 180;
            }
        }
    }

    int positions[15] = {0};
    for (int i = 0; i < 15; i++) 
	{
        if (i % 3 == 1) 
		{
            positions[i] = map_rad90_to_value(rads[i]);
        } 
		else if (i % 3 == 2) 
		{
            positions[i] = map_rad75_to_value(rads[i]);
        } 
		else 
		{
            positions[i] = map_rad_to_value_full_range(rads[i]);
        }
    }

    int cmds[15] = {0};
    for (int i = 0; i < 15; i++) 
	{
        if (i % 3 == 2) 
		{
            cmds[i] = positions[i];
        } 
		else if (i % 3 == 0) 
		{
            cmds[i] = (hand_lr == 0) ? (positions[i + 1] - positions[i]) : (positions[i + 1] + positions[i]);
        } 
		else if (i % 3 == 1) 
		{
            cmds[i] = (hand_lr == 0) ? (positions[i] + positions[i - 1]) : (positions[i] - positions[i - 1]);
        }
    }

    for (int i = 0; i < 15; i++) 
	{
        if (cmds[i] < 0) 
		{
            cmds[i] = 0;
        } 
		else if (cmds[i] > 4095) 
		{
            cmds[i] = 4095;
        }
    }

    // 发送指令
    for (int i = 0; i < 15; i++) 
	{
		RyMotion_ServoMove_Mix( &stuServoCan, i+1, cmds[i], 1000, 80, &sutServoDataR[i], 1 );
        // printf("Servo %d: Command = %d\n", i + 1, cmds[i]);
    }
	
}
