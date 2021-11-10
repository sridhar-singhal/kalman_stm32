#include "stm32f303x8.h"
#include "arm_math.h"

using namespace std;
//Matrix Initializations
//https://stackoverflow.com/questions/58645727/arm-math-matrix-multiplication-hardfault 
int main()
{	
	uint16_t rows = 4;
	uint16_t cols = 4;
	float dt = 0.1;
	float v = 20; //m/s
	float A_Data[4*4] = {1,0,dt,0, 0,1,0,dt, 0,0,1,0, 0,0,0,0};
	float X0[4*1] = {0,0,0,v}; //Initial value of state
	
	arm_matrix_instance_f32 A = {rows, cols, (float32_t *) A_Data};
	arm_matrix_instance_f32 X = {rows, 1, (float32_t *) X0};
	
	uint16_t sig_x=5;
	uint16_t sig_y=5;
	uint16_t sig_vx=2;
	uint16_t sig_vy=2;
	uint16_t sig_ax=20;
	uint16_t sig_ay=20;
	
	uint16_t sig_senx=2;
	uint16_t sig_seny=2;
	
	float P_data[4*4] = {(float32_t)sig_x,0,0,0, 0,(float32_t)sig_y,0,0, 0,0,(float32_t)sig_vx,0, 0,0,0,(float32_t)sig_vy};
	arm_matrix_instance_f32 P = {rows, cols, (float32_t *) P_data};	
	
	float H_data[2*4] = {1,0,0,0,0,1,0,0};
	arm_matrix_instance_f32 H = {2, cols, (float32_t *)H_data};
   
	
	while(1);
	return 0;
}