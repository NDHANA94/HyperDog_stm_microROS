/*
********************************************************************************
*
* Created on: March 15, 2022
*     Author: Nipun Dhananjaya Weerakkodi -> nipun.dhananjaya@gmail.com
********************************************************************************
*/

#include "servo_controller.h"
#include "BaseIMU.h"
// temp
void set_servoAng_FR(){
	double th_temp[3];
	th_temp[0] = joint_ang[0];
	th_temp[1] = joint_ang[1];
	th_temp[2] = joint_ang[2];

	th_temp[0] = - th_temp[0];

	if (th_temp[0] > TH1_MAX)
		th_temp[0] = TH1_MAX;
	else if (th_temp[0] < TH1_MIN)
		th_temp[0] = TH1_MIN;

	if (th_temp[1] < TH2_MIN)
		th_temp[1] = TH2_MIN;
	else if (th_temp[1] > TH2_MAX)
		th_temp[1] = TH2_MAX;

	if (th_temp[2] < MIN_ANG_L2_L3 )
		th_temp[2] =  MIN_ANG_L2_L3;
	else if (th_temp[2] > MAX_ANG_L2_L3)
		th_temp[2] = MAX_ANG_L2_L3;

	servo_ang[0] = FR_HIP_OFFSET + th_temp[0];
	servo_ang[1] = FR_ULEG_OFFSET + th_temp[1];
	servo_ang[2] = FR_LLEG_OFFSET + th_temp[2];
}

void set_servoAng_FL(){
	double th_temp[3];
	th_temp[0] = joint_ang[3];
	th_temp[1] = joint_ang[4];
	th_temp[2] = joint_ang[5];

	th_temp[0] = - th_temp[0];

	if (th_temp[0] > TH1_MAX)
		th_temp[0] = TH1_MAX;
	else if (th_temp[0] < TH1_MIN)
		th_temp[0] = TH1_MIN;

	if (th_temp[1] < TH2_MIN)
		th_temp[1] = TH2_MIN;
	else if (th_temp[1] > TH2_MAX)
		th_temp[1] = TH2_MAX;

	if (th_temp[2] < MIN_ANG_L2_L3 )
		th_temp[2] = MIN_ANG_L2_L3;
	else if (th_temp[2] > MAX_ANG_L2_L3)
		th_temp[2] = MAX_ANG_L2_L3;

	servo_ang[3] = FL_HIP_OFFSET - th_temp[0];
	servo_ang[4] = FL_ULEG_OFFSET - th_temp[1];
	servo_ang[5] = FL_LLEG_OFFSET - ( th_temp[2]);

}

void set_servoAng_BR(){
	double th_temp[3];
	th_temp[0] = joint_ang[6];
	th_temp[1] = joint_ang[7];
	th_temp[2] = joint_ang[8];

	th_temp[0] = - th_temp[0];

	if (th_temp[0] > TH1_MAX)
		th_temp[0] = TH1_MAX;
	else if (th_temp[0] < TH1_MIN)
		th_temp[0] = TH1_MIN;

	if (th_temp[1] < TH2_MIN)
		th_temp[1] = TH2_MIN;
	else if (th_temp[1] > TH2_MAX)
		th_temp[1] = TH2_MAX;

	if (th_temp[2] < MIN_ANG_L2_L3 )
		th_temp[2] =  MIN_ANG_L2_L3;
	else if (th_temp[2] > MAX_ANG_L2_L3)
		th_temp[2] = MAX_ANG_L2_L3;

	servo_ang[6] = BR_HIP_OFFSET - th_temp[0];
	servo_ang[7] = BR_ULEG_OFFSET + th_temp[1];
	servo_ang[8] = BR_LLEG_OFFSET + th_temp[2];
}

void set_servoAng_BL(){
	double th_temp[3];
	th_temp[0] = joint_ang[9];
	th_temp[1] = joint_ang[10];
	th_temp[2] = joint_ang[11];

	th_temp[0] = - th_temp[0];

	if (th_temp[0] > TH1_MAX)
		th_temp[0] = TH1_MAX;
	else if (th_temp[0] < TH1_MIN)
		th_temp[0] = TH1_MIN;

	if (th_temp[1] < TH2_MIN)
		th_temp[1] = TH2_MIN;
	else if (th_temp[1] > TH2_MAX)
		th_temp[1] = TH2_MAX;

	if (th_temp[2] < MIN_ANG_L2_L3 )
		th_temp[2] = MIN_ANG_L2_L3;
	else if (th_temp[2] > MAX_ANG_L2_L3)
		th_temp[2] = MAX_ANG_L2_L3;

	servo_ang[9] = BL_HIP_OFFSET + th_temp[0];
	servo_ang[10] = BL_ULEG_OFFSET - th_temp[1];
	servo_ang[11] = BL_LLEG_OFFSET - ( th_temp[2]);

}




void drive_servos(){
    set_servoAng_FR();
    set_servoAng_FL();
    set_servoAng_BR();
    set_servoAng_BL();

    uint8_t srv_ch = 0;
    for (int i = 0; i<12; i++){
        if (i<3){srv_ch = i;}
        else if (i>=3 && i < 6){ srv_ch = i + 1;}
        else if (i>=6 && i<9){srv_ch = i + 2;}
        else{srv_ch = i+3;}
        bool err = PCA9685_SetServoAngle(srv_ch, servo_ang[i]);
        if(err){
            while (1){
                bool err = PCA9685_Init(&hi2c3);
                if(err){
                    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET); // Green LED off
                    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET); // red LED on
                    osDelay(100);
                    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET); // red LED off
                    osDelay(1000);
                }
                else{
                    i = 0;
                    break;
                }
            }
        }
    }
    
    
}


