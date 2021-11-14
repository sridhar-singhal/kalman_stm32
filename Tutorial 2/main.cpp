#include "stm32f303x8.h"
#include "stm32f3xx.h" //This contains all the NVIC stuff. Remeber
#include "arm_math.h"
//#include "stm32f3xx_ll_utils.h"
//#include "misc.h"
#include "core_cm4.h"

using namespace std;
//Matrix Initializations
//https://stackoverflow.com/questions/58645727/arm-math-matrix-multiplication-hardfault 
//https://www.eevblog.com/forum/microcontrollers/using-matrix-multiplication-function-of-cmsis-dsp-library-for-stm32discovery/

//Timer
//https://deepbluembedded.com/stm32-timer-interrupt-hal-example-timer-mode-lab/
//Timer calculations: https://www.emcu-homeautomation.org/stm32-basic-timer-in-interrupt-pwm-mode/
//Using Timer: https://visualgdb.com/tutorials/arm/stm32/timers/ 
//A problem in timer might occur:
//https://developer.arm.com/documentation/ka003795/latest
//https://electronics.stackexchange.com/questions/439873/timer-interrupt-on-stm32f303
//A link which uses only CMSIS and not ST's libraries: 
	//https://www.eng.auburn.edu/~nelsovp/courses/elec3040_3050/LabLectures/ELEC30x0%20Lab4%20Interrupts.pdf
typedef struct
{
  uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be a value of @ref IRQn_Type 
                                                   (For the complete STM32 Devices IRQ Channels list, please
                                                    refer to stm32f37x.h file) */

  uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */   
} NVIC_InitTypeDef;


//Name of the Timer 2 Interrupt handler as defined in the Vector interrup table in startup_stm32f303x8.s
//void TIM2_IRQHandler()
//{
	//get_vals();
	
//}


void EnableTimerInterrupt()
{
    //NVIC_InitTypeDef nvicStructure;
    //nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    //nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //nvicStructure.NVIC_IRQChannelSubPriority = 1;
    //nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&nvicStructure);
	

	NVIC_EnableIRQ(TIM2_IRQn); //Timer 2 is enabled
	NVIC_SetPriority(TIM2_IRQn, 1); //Timer 2 priority is set to 1, highest is 0. 
	
	
}


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
	float Atrans_Data[4*4] = {1,0,0,0, 0,1,0,0, dt,0,1,0, 0,dt,0,1};
	arm_matrix_instance_f32 Atrans = {rows, cols, (float32_t *) Atrans_Data};	
	
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

	float Q_data[4*4] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
	arm_matrix_instance_f32 Q = {rows, cols, (float32_t *) Q_data};	

	float H_data[2*4] = {1,0,0,0,0,1,0,0};
	arm_matrix_instance_f32 H = {2, cols, (float32_t *)H_data};
	
	float Htrans_data[4*2] = {1,0, 0,1, 0,0, 0,0};
	arm_matrix_instance_f32 Htrans = {4, 2, (float32_t *)Htrans_data};
	
	float S_data[2*2] ={0, 0, 0, 0} ;
	arm_matrix_instance_f32 S = {2, 2, (float32_t *)S_data};	
	
	float Sinv_data[2*2] ={0, 0, 0, 0} ;
	arm_matrix_instance_f32 Sinv = {2, 2, (float32_t *)Sinv_data};
	
	float R_data[2*2] ={(float)sig_senx, 0, 0, (float)sig_seny} ;
	arm_matrix_instance_f32 R = {2, 2, (float32_t *)R_data};
		
	float I_data[4*4] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
	arm_matrix_instance_f32 I = {4, 4, (float32_t *)I_data};
	
	float Pt_data[4*4] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
	arm_matrix_instance_f32 Pt = {4, 4, (float32_t *)Pt_data};
	
	float Y_data[2*1] = {0, 0};
	arm_matrix_instance_f32 Y = {2, 1, (float32_t *)Y_data};
	
	float Yt_data[2*1] = {0, 0};
	arm_matrix_instance_f32 Yt = {2, 1, (float32_t *)Yt_data};
	
	//Initialize first values of U (acc), Z (GPS) here.
	float U_data[2*1] = {0, 0};
	arm_matrix_instance_f32 U = {2, 1, (float32_t *)U_data};
	
	float Z_data[2*1] = {0, 0};
	arm_matrix_instance_f32 Z = {2, 1, (float32_t *)Z_data};
	
	float K_data[4*2];
	arm_matrix_instance_f32 K = {4,2, (float32_t *)K_data};
	
	float Kt_data[4*2];
	arm_matrix_instance_f32 Kt = {4,2, (float32_t *)Kt_data};
	
	
	uint32_t i = 1;
	while(1)
	{
		//Get z(k) and u(k) over here
		
		//Xk = AX(k-1) + BU(k);
		arm_mat_mult_f32(&A,&X,&X); //Dest = AB. -> (A,B,Dest)  
		//Have to verify if this works
		arm_mat_mult_f32(&B,&U,&Xt); //X0 is used as a temp vector here
		arm_mat_add_f32(&X,&Xt,&X);
		
		arm_mat_mult_f32(&A,&P,&P); //P = APA' + Q
		arm_mat_mult_f32(&P,&Atrans,&P); //P = APA' + Q
		arm_mat_add_f32(&P,&Q,&P);
		
		//Y(k) = Z(k) - HX(k)
		arm_mat_mult_f32(&H,&X,&Yt); 
		arm_mat_sub_f32(&Z,&Yt,&Y);
		
		//S(k) = H*P(k)*H' + R 
		arm_mat_mult_f32(&H,&P,&S);
		arm_mat_mult_f32(&S,&Htrans,&S);
		arm_mat_add_f32(&S,&R,&S);
		
		//Sinv(k) = inverse(S)
		arm_mat_inverse_f32(&S,&Sinv);
	
		//K(k) = P(k)*H'*Sinv(k) 
		arm_mat_mult_f32(&P,&Htrans,&Kt);
		arm_mat_mult_f32(&Kt,&Sinv,&K);
		
		//X(k) = X(k) + K(k)*Y(k) Final Update
		arm_mat_mult_f32(&K,&Y,&Xt); //Xt is already used at this point, and is recalculated everytime before use
		arm_mat_add_f32(&Xt,&X,&X);
		
		//P(k) = (I - K(k)*H)*P(k);
		arm_mat_mult_f32(&K,&H,&Pt);
		arm_mat_sub_f32(&I,&Pt,&Pt);
		arm_mat_mult_f32(&Pt,&P,&P);
		
		//Values from P can now be copied to the output states
		
	};
	
	return 0;
}