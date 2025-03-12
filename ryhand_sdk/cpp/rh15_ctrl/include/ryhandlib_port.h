/*********************************************************************
File name:      ryhandlib.h
Author:         zhanglf
Version:        v1.0
Date:           2024.11.07
Description:    ryhandlib库对外接口实现
								
Others:         

History:        
*********************************************************************/


#ifndef __RYHANDLIB_H_
#define __RYHANDLIB_H_

#include "stdbool.h"
#include "ryhandlib.h"




extern RyCanServoBus_t stuServoCan;
extern CanMsg_t stuListenMsg[40];
extern ServoData_t sutServoDataW[15];
extern ServoData_t sutServoDataR[15];
extern volatile s16_t  uwTick;

extern void MyHoockCallBck(CanMsg_t stuMsg, void * para);
extern s8_t BusWrite(CanMsg_t stuMsg);
extern void CallBck0(CanMsg_t stuMsg, void * para);


extern float rad_to_deg(float rad);
extern float deg_to_rad(float deg);
extern int map_rad90_to_value(float rad);
extern int map_rad75_to_value(float rad);
extern int map_rad_to_value_full_range(float rad);
extern float value_to_rad90(int value); 
extern float value_to_rad75(int value);  
extern float value_to_rad_full_range(int value);

extern double evaluatePolynomial(double coefficients[], int degree, double x);

extern void update_motor_positions( float *rads, int hand_lr );



#endif
