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
	
	uint16_t sig_x=5; //Model X cost IMU
	uint16_t sig_y=5; //Model Y cost IMU
	uint16_t sig_vx=2;//Model Vx cost IMU
	uint16_t sig_vy=2;//Model Vy cost IMU
	//uint16_t sig_ax=20;
	//uint16_t sig_ay=20;
	//Adjust those costs according to sensor noise in IMU
	
	uint16_t sig_senx=2; //GPS X cost
	uint16_t sig_seny=2; //GPS Y cost
	
	
	
	float A_Data[4*4] = {1,0,dt,0, 0,1,0,dt, 0,0,1,0, 0,0,0,1};
	arm_matrix_instance_f32 A = {rows, cols, (float32_t *) A_Data};
	//arm_mat_init_f32 (&A,rows, cols, (float32_t *) A_Data);

	float B_Data[4*2] = {float(0.5)*dt*dt,0, 0,float(0.5)*dt*dt, dt,0, 0,dt};
	arm_matrix_instance_f32 B = {4, 2, (float32_t *) B_Data};

	
	float X_data[4*1] = {0,0,0,v}; //Initial value of state
	arm_matrix_instance_f32 X = {rows, 1, (float32_t *) X_data};

	float Xt_data[4*1] = {0,0,0,v}; //Initial value of state
	arm_matrix_instance_f32 Xt = {rows, 1, (float32_t *) Xt_data};
	
	//This method is not required, as given in https://www.eevblog.com/forum/microcontrollers/using-matrix-multiplication-function-of-cmsis-dsp-library-for-stm32discovery/
	//arm_matrix_instance_f32 X;
	//arm_mat_init_f32(&X, rows, 1, (float32_t *) X0);
	//Just rememeber to send &X in the matrix multiplication and addition functions

	
	float P_data[4*4] = {(float32_t)sig_x,0,0,0, 0,(float32_t)sig_y,0,0, 0,0,(float32_t)sig_vx,0, 0,0,0,(float32_t)sig_vy};
	arm_matrix_instance_f32 P = {rows, cols, (float32_t *) P_data};	
	
	float H_data[2*4] = {1,0,0,0,0,1,0,0};
	arm_matrix_instance_f32 H = {2, cols, (float32_t *)H_data};
	
	float R_data[2*2] ={(float)sig_senx, 0, 0, (float)sig_seny} ;
	arm_matrix_instance_f32 R = {2, 2, (float32_t *)R_data};
		
	float I_data[4*4] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
	arm_matrix_instance_f32 I = {4, 4, (float32_t *)I_data};
	
	float Y_data[2*1] = {0, 0};
	arm_matrix_instance_f32 Y = {2, 1, (float32_t *)Y_data};
	
	//Initialize first values of U (acc), Z (GPS) here.
	float U_data[2*1] = {0, 0};
	arm_matrix_instance_f32 U = {2, 1, (float32_t *)U_data};
	
	float Z_data[2*1] = {0, 0};
	arm_matrix_instance_f32 Z = {2, 1, (float32_t *)Z_data};
	
	uint32_t i = 1;
	while(1)
	{
		arm_mat_mult_f32(&A,&X,&X); //Dest = AB. -> (A,B,Dest)  
		//Have to verify if this works
		arm_mat_mult_f32(&B,&U,&Xt); //X0 is used as a temp vector here
		
		arm_mat_add_f32(&X,&Xt,&X);
		
	};
	
	return 0;
}