// Carlos Verduzco
// CECS 447
// April 11 2022
// RobotCar.c
// Main program for robot car
// Uses UART to recrive input from bluetooth terminal
// PWM used to control duty cycle of DC motors

// Header files 
#include "tm4c123gh6pm.h"
#include "UART.h"
#include "string.h"

#define	LEDS 		(*((volatile unsigned long *)0x40025038)) // PF3-1
#define MOTOR1	(*((volatile unsigned long *)0x4002400C)) // PE1-0
#define MOTOR2	(*((volatile unsigned long *)0x4000500C)) // PB1-0
	
#define Off 0x00
#define Forward 0x02
#define Reverse 0x01

#define R 0x02
#define G 0x08
#define B 0x04
#define P 0x06
#define W 0x0E
#define Y 0x0A
#define D 0x00

#define NVIC_EN0_PORTF 0x40000000

#define PERIOD 			16000       	  // max. count, frequency of 10ms, value is based on 16 MHz system clock
#define STARTING_PERIOD 6000
#define DUTY_STEP   1000
#define Duty0				0

uint32_t Duty = Duty0; 


char button_response;

void Delay(void);

void PortF_Init(void);
void Motor1_Init(void);
void Motor2_Init(void);
void PWM1A_Init(uint32_t period);
void PWM1B_Init(uint32_t period);
void SetUp(void);


extern void DisableInterrupts(void);
extern void EnableInterrupts(void);  

void OutCRLF(void){
  UART0_OutChar(CR);
  UART0_OutChar(LF);
}

void PortF_Init(void){ 
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // (a) activate clock for port F
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x11;           // allow changes to PF4-0
    
  GPIO_PORTF_AMSEL_R &= ~0x11;        // 3) disable analog function
  GPIO_PORTF_PCTL_R &= 0x000F000F;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R &= ~0x11;          // 5) PF4,PF0 input   
  GPIO_PORTF_AFSEL_R &= ~0x11;        // 6) no alternate function
  GPIO_PORTF_PUR_R |= 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R |= 0x11;          // 7) enable digital pins PF4-PF0        
	GPIO_PORTF_DIR_R |= 0x0E;    // (c) make PF3-1 output (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x0E;  //     disable alt funct on PF3-1
	GPIO_PORTF_DEN_R |= 0x0E;     //     enable digital I/O on PF3-1
}

void Motor1_Init(void) { //Port C
	SYSCTL_RCGC2_R |= 0x00000002;	// (a) activate clock for port B
	while ((SYSCTL_RCGC2_R & 0x00000002) != 0x00000002){}
	GPIO_PORTB_DIR_R |= 0x03;			// set PB1 - 0 as output 
	GPIO_PORTB_AFSEL_R &= ~0x03;  //     disable alt funct on PB1-0
	GPIO_PORTB_DEN_R |= 0x03;     //     enable digital I/O on PB1-0
	GPIO_PORTB_PCTL_R &= ~0x000000FF; //  configure PB1-0 as GPIO
	GPIO_PORTB_AMSEL_R &= ~0x03;  //     disable analog functionality on PB1-0

}

void Motor2_Init(void) { //Port E
	
	SYSCTL_RCGC2_R |= 0x00000010;	// (a) activate clock for port E
	while ((SYSCTL_RCGC2_R & 0x00000010) != 0x00000010){}
	GPIO_PORTE_DIR_R |= 0x03;			// set PE1 - 0 as output 
	GPIO_PORTE_AFSEL_R &= ~0x03;  //     disable alt funct on PE1-0
	GPIO_PORTE_DEN_R |= 0x03;     //     enable digital I/O on PE1-0
	GPIO_PORTE_PCTL_R &= ~0x000000FF; //  configure PE1-0 as GPIO
	GPIO_PORTE_AMSEL_R &= ~0x03;  //     disable analog functionality on PE1-0

}

// PWM Output on PD0/M1PWM0
void PWM1A_Init(uint32_t period)
{
  SYSCTL_RCGCPWM_R |= 0x02;             // activate PWM1 clock
  SYSCTL_RCGCGPIO_R |= 0x08;            // activate port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};
	GPIO_PORTD_AFSEL_R |= 0x01;           // enable alt funct on PD0
  GPIO_PORTD_PCTL_R &= ~0x0000000F;     // configure PD0 as PWM1
  GPIO_PORTD_PCTL_R |=  0x00000005;
  GPIO_PORTD_AMSEL_R &= ~0x01;          // disable analog functionality on PD0
  GPIO_PORTD_DEN_R |= 0x01;             // enable digital I/O on PD0
  SYSCTL_RCC_R &= ~0x00100000;          // System clock
  PWM1_0_CTL_R = 0;                     // re-loading down-counting mode
  PWM1_0_GENA_R = 0xC8;                 //  output 0 when counter=Load, output 1 when counter=Compare A
  PWM1_0_LOAD_R = period - 1;           // cycles needed to count down to 0
  PWM1_0_CMPA_R = 0;             				// Outputs single clock cycle high pulse when equals value in load 
  PWM1_0_CTL_R |= 0x00000001;           // start PWM1
  PWM1_ENABLE_R |= 0x00000001;          // enable PD0/M1PWM0
}

// PWM Output on PD1/M1PWM0
void PWM1B_Init(uint32_t period)
{
  while((SYSCTL_PRGPIO_R&0x08) == 0){};
	GPIO_PORTD_AFSEL_R |= 0x02;           // enable alt funct on PD1
  GPIO_PORTD_PCTL_R &= ~0x000000F0;     // configure PD1 as PWM1
  GPIO_PORTD_PCTL_R |=  0x00000050;
  GPIO_PORTD_AMSEL_R &= ~0x02;          // disable analog functionality on PD1
  GPIO_PORTD_DEN_R |= 0x02;             // enable digital I/O on PD1
  SYSCTL_RCC_R &= ~0x00100000;          // Use system clock
  PWM1_0_CTL_R = 0;                     // re-loading down-counting mode
  PWM1_0_GENB_R = 0xC08;                //  output 0 when counter=Load, output 1 when counter=Compare A
  PWM1_0_LOAD_R = period - 1;           // cycles needed to count down to 0
  PWM1_0_CMPB_R = 0;                    // Outputs single clock cycle high pulse when equals value in load
  PWM1_0_CTL_R |= 0x00000001;           // start PWM1
  PWM1_ENABLE_R |= 0x00000002;          // enable PD1/M1PWM0
}

void SetUp(void) {
	MOTOR1 = Off; 
	MOTOR2 = Off;
	Duty = STARTING_PERIOD;
	PWM1_0_CMPA_R = Duty - 1;             // count value when output rises
	PWM1_0_CMPB_R = Duty - 1;             // count value when output rises
	LEDS = D;
}


int main(void) {
  UART_Init();        //initialize UART
  OutCRLF();
	PortF_Init();
	Motor1_Init();
	Motor2_Init();
	PWM1A_Init(PERIOD); //load max duty cycle frequency
	PWM1B_Init(PERIOD);
	SetUp();

		while(1) {
			button_response = UART1_InChar(); //Input received from Bluetooth Serial Terminal;
			
			switch (button_response) {
				case 'W': //Forward
				case 'w':
					MOTOR1 = Forward;
					MOTOR2 = Forward; 
					LEDS = G;

					break;
				case 'S': //Reverse
					case 's':
					MOTOR1 = Reverse; 
					MOTOR2 = Reverse; 
					LEDS = B;

					break;
				case 'A': //Left Turn
					case 'a':
					MOTOR1 = Reverse;
					MOTOR2 = Forward;
					LEDS = Y;

					break;
				case 'D': //Right Turn
					case 'd':
					MOTOR1 = Forward;
					MOTOR2 = Reverse;
					LEDS = P;
					break;
					
				case 'T': //Stop
					case 't':
					MOTOR1 = Off;
					MOTOR2 = Off;
					LEDS = D;
					break;
					
				case 'U': //Speed Up
					case 'u':
					if(Duty < PERIOD) {
					Duty += DUTY_STEP;
					PWM1_0_CMPA_R = Duty - 1;             // count value when output rises
					PWM1_0_CMPB_R = Duty - 1;             // count value when output rises
					}
					break;
					
				case 'L'://Slow Down
					case 'l':
					if(Duty > Duty0) {
					Duty -= DUTY_STEP;
					PWM1_0_CMPA_R = Duty - 1;             // count value when output rises
					PWM1_0_CMPB_R = Duty - 1;             // count value when output rises
					}
					break;
					
				default:
					break;
		}
	}
}
			
		
