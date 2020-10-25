# cubeide-timers-demo

This is the matching project for the blog at https://01001000.xyz/2020-10-24-Tutorial-STM32CubeIDE-Timers-PWM-AM-Radio/

In this project four timers are used on the NUCLEO-F303RE STM32 development kit. 
* TIM2 generates a 1kHz PWM signal for the on-board LED, and TIM1 changes the duty cycle to fade the LED on and off using an interrupt.
* TIM3 generates a 1000kHz PWM signal for an antenna at the arduino pin PWM/D9
* TIM4 is used for a custom `delay_us` function.
Then, the main loop of the program generates an AM radio signal to play a few bars of Fur Elise in the AM frequency spectrum.

For more details, do check out the blog post at https://01001000.xyz/2020-10-24-Tutorial-STM32CubeIDE-Timers-PWM-AM-Radio/
