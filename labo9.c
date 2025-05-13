#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.c"
#include <string.h>
#include <stdlib.h>
#include "driverlib/pwm.h"

#ifdef DEBUG
void
_error_(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
uint32_t FS=120000000*1.5; //120 MHz
void timer0A_handler(void);
uint8_t switch_state=0;
char  msg[]="h";
char data[100];
char c[100];
uint32_t aux=0;
volatile uint32_t width;

int distancia=0;
uint32_t reloj;
#define TIMEOUT 1000000 
int
main(void)
{
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x07); // PN2 (trigger) y PN3 (echo)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x1c); // PF0 y PF4
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, 0x30);
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, 0x08); // PN3 (echo)

    GPIOPadConfigSet(GPIO_PORTJ_BASE, 0x03, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinWrite(GPIO_PORTN_BASE, 0x0f, 0x00); // Apaga PN0 y PN1
    GPIOPinWrite(GPIO_PORTF_BASE, 0x1c, 0x00); // Apaga PF0 y PF4            
    // //TIMER
    // //enable the timer peripheral
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // // Set timer
    // TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // // Set the count time for Timer
    // TimerLoadSet(TIMER0_BASE, TIMER_A, FS);
    // //Enable processor interrupts
    // IntMasterEnable();
    // // Enable Interrupt
    // IntEnable(35);//INT_TIMER0A_TM4C129
    // // Enable timer A interrupt
    // TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // // Enable the timer
    // TimerEnable(TIMER0_BASE, TIMER_A);
  //UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, 0x03);

    UARTStdioConfig(0, 9600, 120000000);
    //Hablitar el contador del timer para el diferencia de tiempo de pulsos
    SysTickPeriodSet(0xFFFFFF);  // Máximo valor (24 bits)
    SysTickEnable(); 
    //PWM
    // Enable the PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);


    // Configure the PWM function for this pin.
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    // wait for the PWM0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }
    // Configure the PWM generator for count down mode with immediate updates to the parameters.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20 microseconds.
    // For a 20 MHz clock, this translates to 400 clock ticks. Use this value to set the period.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 400);

    // Set the pulse width of PWM1 for a 75% duty cycle (width = 300).
    width = 300;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width);

    // Start the timers in generator 0.
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, (PWM_OUT_1_BIT), true);
    //PWMMMMMMMMMMMMM
    // Enable the PWM0 peripheral

    // Configure the PWM function for this pin.
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);

    // wait for the PWM0 module to be ready.
    // Configure the PWM generator for count down mode with immediate updates to the parameters.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20 microseconds.
    // For a 20 MHz clock, this translates to 400 clock ticks. Use this value to set the period.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 400);

    // Set the pulse width of PWM1 for a 75% duty cycle (width = 300).
    width = 300;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, width);

    // Start the timers in generator 0.
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT), true);
    
    while(1)
    {   
        while (UARTCharsAvail(UART0_BASE)) {
            UARTgets(data, 100);
            if((strcmp(data, "uno") == 0)){
                GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x03); // PN1 = 1, PN0 = 0
                GPIOPinWrite(GPIO_PORTF_BASE, 0x11, 0x11); // PN1 = 1, PN0 = 0
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);//adelante
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);//adelante
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 400*0.25);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_PIN_4);//adelante
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, 0x00);//adelante
                GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 400*0.40);

            }
            if((strcmp(data, "objetos") == 0)){
                GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x03); // PN1 = 1, PN0 = 0
                GPIOPinWrite(GPIO_PORTF_BASE, 0x11, 0x00); // PN1 = 1, PN0 = 0
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, 0x00);
                GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0x00);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1);
            }
            if((strcmp(data, "nada") == 0)){
                GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00); // PN1 = 1, PN0 = 0
                GPIOPinWrite(GPIO_PORTF_BASE, 0x11, 0x00); // PN1 = 1, PN0 = 0
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 400*0.21);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, GPIO_PIN_5);
                GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0x00);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 400*0.02);
            }
        }
                    
        SysCtlDelay(800000); // ~1 segundo (depende de clock)
    }
}
void timer0A_handler(void)
{
    // switch_state++;
    // // Clear timer
    // TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // if (switch_state == 1)  
    // {
    //     GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00);
    //     GPIOPinWrite(GPIO_PORTF_BASE, 0x13, 0x00);
    // }
    // else if (switch_state == 2)  
    // {
    //     GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00);
    //     GPIOPinWrite(GPIO_PORTF_BASE, 0x13, 0x01);
    // }
    // else if (switch_state == 3)  
    // {
    //     GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00);
    //     GPIOPinWrite(GPIO_PORTF_BASE, 0x13, 0x11);
    // }
    // else if (switch_state == 4)  
    // {
    //     GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x01);
    //     GPIOPinWrite(GPIO_PORTF_BASE, 0x13, 0x11);
    //     switch_state=0;
    // }
}