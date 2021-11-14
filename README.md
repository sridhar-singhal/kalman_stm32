# kalman_stm32

KEIL and STM32 do not have all libraries included. 

For Timers in STM32, functions are already present in std_peripherel library like NVIC_init etc which make things very easy. 
But std_peripheral is not available in KEIL. 

https://community.arm.com/support-forums/f/keil-forum/28379/stm32-using-the-st-standard-peripheral-library-with-keil 

The counterpart in KEIL is Keil\ARM\RVxx\LIB\ST, which upon searching the internet, is found in the I2C or SPI library. 

The required functions should be found here ideally.

http://www.disca.upv.es/aperles/arm_cortex_m3/curset/STM32F4xx_DSP_StdPeriph_Lib_V1.0.1/html/struct_n_v_i_c___init_type_def.html 

http://www.disca.upv.es/aperles/arm_cortex_m3/curset/STM32F4xx_DSP_StdPeriph_Lib_V1.0.1/html/main.html



But this library is not available. http://www.compel.ru/wordpress/wp-content/uploads/2012/07/STM32F3-Standard-Firmware-Library.pdf 





