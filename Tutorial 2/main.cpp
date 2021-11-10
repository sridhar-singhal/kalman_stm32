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
   
	
	while(1);
}