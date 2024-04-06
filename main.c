void SystemInit(){}
#include <stdio.h>

#define 		SYSCTL_RCGCGPIO_R		(*((volatile unsigned long*)0x400FE608))
//-------------button----------------------------------------------------------------
//IRQ0to31SetEnableRegister
#define 		NVIC_EN0_R					(*((volatile unsigned long*)0xE000E100))
//IRQ28to31PriorityRegister
#define 	NVIC_PRI7_R					(*((volatile unsigned long*)0xE000E41C))
#define 	GPIO_PORTF_DATA_R			(*((volatile unsigned long*)0x400253FC))
#define 	GPIO_PORTF_DIR_R			(*((volatile unsigned long*)0x40025400))
#define 	GPIO_PORTF_DEN_R			(*((volatile unsigned long*)0x4002551C))
#define 	GPIO_PORTF_PUR_R			(*((volatile unsigned long*)0x40025510))
#define 	GPIO_PORTF_IS_R				(*((volatile unsigned long*)0x40025404))
#define 	GPIO_PORTF_IBE_R			(*((volatile unsigned long*)0x40025408))
#define 	GPIO_PORTF_IEV_R			(*((volatile unsigned long*)0x4002540C))
#define 	GPIO_PORTF_IM_R				(*((volatile unsigned long*)0x40025410))
#define 	GPIO_PORTF_ICR_R			(*((volatile unsigned long*)0x4002541C))
#define 	NVIC_EN0_INT30				0x40000000	//Interrupt30enable
#define 	PORTF_CLK_EN				0x20		//Clock enable for PortF
#define 	SW1							0x10		//Enable user switchSW1
#define 	INT_PF4						0x10		//Interrupt at PF4
//-------------lcd----------------------------------------------------------------
// register definitions for Port B, which is used as LCD data bus
# define GPIO_PORTB_DATA_R 				*(( volatile unsigned long *)0x400053FC )
# define GPIO_PORTB_DIR_R 				*(( volatile unsigned long *)0x40005400 )
# define GPIO_PORTB_DEN_R 				*(( volatile unsigned long *)0x4000551C )
# define GPIO_PORTB_AFSEL_R 			*(( volatile unsigned long *)0x40005420 )
/* register definitions for Port E (pins 3, 4, 5) used as LCD control bus */																												
# define GPIO_PORTE_PUR_R				*(( volatile unsigned long *)0x400240E0 )
# define GPIO_PORTE_DATA_R 				*(( volatile unsigned long *)0x400240E0 )
# define GPIO_PORTE_DIR_R 				*(( volatile unsigned long *)0x40024400 )
# define GPIO_PORTE_DEN_R 				*(( volatile unsigned long *)0x4002451C )
# define GPIO_PORTE_AFSEL_R 			*(( volatile unsigned long *)0x40024420 )
// LCD module control pins and data bus definitions
# define LCD_E_PIN 0x08
# define LCR_RW_PIN 0x10
# define LCD_RS_PIN 0x20
# define LCD_DATA_BUS GPIO_PORTB_DATA_R
# define LCD_CONTROL_BUS GPIO_PORTE_DATA_R
//-------------timer----------------------------------------------------------------
// Timer 1CCPO - PC0 base address
# define TM_BASE 0x40031000

// Peripheral clock enabling for timer and GPIO
# define RCGC_TIMER_R * (volatile unsigned long * ) 0x400FE604
# define CLOCK_GPIOF 0x00000020 // Port F clock control

// General purpose timer register definitions
# define GPTM_CONFIG_R 		*((volatile long * )(TM_BASE + 0x000))
# define GPTM_TA_MODE_R 	*((volatile long * )(TM_BASE + 0x004))
# define GPTM_CONTROL_R 	*((volatile long * )(TM_BASE + 0x00C))
# define GPTM_TA_IL_R 		*((volatile long * )(TM_BASE + 0x028))
# define GPTM_TA_PRESCALE_R *((volatile long * )(TM_BASE + 0x038))
# define GPTM_TAR		    *((volatile long * )(TM_BASE + 0x048))
# define GPTM_TAV	        *((volatile long * )(TM_BASE + 0x050))

// GPIO alternate function configuration for port C
# define GPIO_PORTF_AFSEL_R 	*((volatile unsigned long*) 0x40025420)
# define GPIO_PORTF_PCTL_R 		*((volatile unsigned long*) 0x4002552C)
# define GPIO_PORTF_DEN_R 		*((volatile unsigned long*) 0x4002551C)
# define GPIO_PORTF_DIR_R		*((volatile unsigned long*) 0x40025400)

// Timer2 A bit field definitions for mode configuration
# define TIM_16_BIT_CONFIG 0x00000004 // 16 - bit timer
# define TIM_EDGE_TIME_MODE 0x00000004 // Time capture on edge
# define TIM_CAPTURE_MODE 0x00000003 // Timer capture mode

// Timer event type bit filed definitions
# define TIM_A_ENABLE 0x00000001 // Enable timer A
# define TIM_A_EVENT_BOTH_EDGES 0 x0000000C // Event on both edges

// Reload values for Timer A with prescale
# define TIM_A_INTERVAL 0x0000FFFF
# define TIM_A_PRESCALE 0x000000FF

void EnableInterrupts(void);						//Disableinterrupts
void DisableInterrupts(void);						//Enable interrupts
void Init_INT_GPIO(void);							//Initialize GPIO and Interrupts
void WaitForInterrupt(void);
void GPIO_Port_Init ( void );
void Delay (volatile unsigned int delay );
void LCD_Init ( void );
void LCD_Clear ( void );
void LCD_Send_Command ( unsigned char command );
void LCD_Send_Integer(int number);
void LCD_Send_Data ( unsigned char data );
void LCD_Send_String ( char * ptr );
void Welcome_Screen(void);
void Timer2A_Init(void);
void delay_ms(int n);
unsigned int rpm;
//-------------button----------------------------------------------------------------
void Init_INT_GPIO(void){
	volatile unsigned delay_clk;
	SYSCTL_RCGCGPIO_R |= PORTF_CLK_EN;	//EnableclockforPORTF
	delay_clk = SYSCTL_RCGCGPIO_R;		//dummyreadtostabletheclock
	
	//GPIO
	GPIO_PORTF_DEN_R |= SW1;		//EnabledigitalI/OonPF4,PF3-PF1
	GPIO_PORTF_DIR_R &= ~(SW1);		//MakePF4inputandPF3-PF1output
	GPIO_PORTF_PUR_R |= SW1;		//EnableweakpulluponPF4
	
	//INTERRUPT
	DisableInterrupts();
	GPIO_PORTF_IS_R &= ~INT_PF4;	//PF4 is edge sensitive
	GPIO_PORTF_IBE_R &= ~INT_PF4;	//PF4 is not both edges
	GPIO_PORTF_IEV_R &= ~INT_PF4;	//PF4 is falling edge
	GPIO_PORTF_ICR_R |= INT_PF4;	//Clear interrupt flag for PF4
	
	GPIO_PORTF_IM_R |= INT_PF4;		//EnableinterruptonPF4
	NVIC_PRI7_R = 0X00A00000;		//SetPF4priority5
	NVIC_EN0_R = NVIC_EN0_INT30;	//Enableinterrupt30inNVIC
	EnableInterrupts();
}

void delay_ms(int n) {
	int i,j;
	for(i=0;i<n;i++)
	for(j=0;j<3180;j++)
	 {}
}

void GPIOPortF_Handler(void){
		GPIO_PORTF_ICR_R=INT_PF4;
		
		LCD_Clear();
		Delay (1) ;
		LCD_Send_String("Calculating...");
		
		GPTM_CONTROL_R |= TIM_A_ENABLE; // Enable the timer
		delay_ms(1000);  //delay for 1second to count rpm
		rpm = GPTM_TAR;
		GPTM_CONTROL_R &= ~1; /* disable TIMER1A during setup */
		GPTM_TAV = 0;   //reset values
		GPTM_TAR = 0;
		rpm = (rpm*60)/4;   //multiply by 60 to get in minutes and divide for number of blades.
		LCD_Send_String("RPM = ");
		Delay (1) ;
		LCD_Send_Integer(rpm);
}
//-------------lcd----------------------------------------------------------------
void GPIO_Port_Init (void) {
	SYSCTL_RCGCGPIO_R |= 0x12 ; // GPIO configuration for Port B/E
	Delay(100);
	GPIO_PORTB_DIR_R |= 0xFF ; // Port B as output
	GPIO_PORTB_DEN_R |= 0xFF ; // digital enable
	GPIO_PORTB_AFSEL_R &= ~0xFF; // regular port function

	// GPIO Configuration for Port E pins 3, 4 and 5
	GPIO_PORTE_DIR_R |= 0x38 ;       // Port E pins as output
	GPIO_PORTE_DEN_R |= 0x38 ;       // digital enable
	GPIO_PORTE_AFSEL_R &= ~0x38;     // regular port function
		
	// GPIO_Init();    
	LCD_Init();
	LCD_Clear();
}

void LCD_Send_Command (unsigned char command ) {
	LCD_DATA_BUS = command ;
	LCD_CONTROL_BUS = 0;     //put into command mode

	// Generate a pulse on the LCD Enable pin
	Delay (1) ;
	LCD_CONTROL_BUS |= LCD_E_PIN ;
	Delay (1) ;
	LCD_CONTROL_BUS &= ~( LCD_E_PIN );
	Delay (4) ;
}

/* This function writes user data to the LCD module . */
void LCD_Send_Data (unsigned char data )
{
	LCD_DATA_BUS = data ;
	LCD_CONTROL_BUS = LCD_RS_PIN ;

		// Generate a pulse on the LCD Enable pin
	Delay (1) ;
	LCD_CONTROL_BUS |= LCD_E_PIN ;
	Delay (1) ;
	LCD_CONTROL_BUS &= ~( LCD_E_PIN );
	Delay (4) ;
}


/* This function initializes the LCD module for 8- bit data
interface , turns ON the display and cursor . */
void LCD_Init ( void ) {
	LCD_CONTROL_BUS = 0;
	Delay (1500) ; // Wait for approx . 15 ms

	LCD_Send_Command (0x38); // Function Set : 8 bit , 2 line
	LCD_Send_Command (0x10); // Set cursor
	LCD_Send_Command (0x0E); // Display ON , cursor ON
	LCD_Send_Command (0x06); // Entry mode
}

void LCD_Clear ( void ) {
	LCD_Send_Command (0x01); // Clear display
	Delay (170) ; // Delay for approx . 1.6 ms
	LCD_Send_Command (0x02); // Move cursor to home
	Delay (170) ; // Delay for approx . 1.6 ms
}

/* This function writes character string to LCD display . */
void LCD_Send_String ( char * ptr ) {
	while (* ptr ) {
		LCD_Send_Data (* ptr );
		ptr ++;
	}
}

void LCD_Send_Integer(int number) {// This function prints integer on LCD
	char buffer[10];
	Delay(10);
	sprintf(buffer,"%d",number); // function sprintf converts integer to string
	LCD_Send_String(buffer);
}

void Delay ( volatile unsigned int delay ) {
	volatile unsigned int i, j;
	for (i = 0; i < delay ; i++) { // introduces a delay of about 10 us at 16 MHz
		for (j = 0; j < 12; j ++) ;
	}
}
//-------------timer----------------------------------------------------------------
void Timer2A_Init(void) {
  SYSCTL_RCGCGPIO_R |= CLOCK_GPIOF;
  RCGC_TIMER_R |= 0x10; // Enable clock for timer 1A
  
  // GPIO port F pin 2 configuration  -EP0
  GPIO_PORTF_DIR_R |= 	0x00000004;  //input
  GPIO_PORTF_DEN_R |=   0x00000004;  //digital enable
  GPIO_PORTF_AFSEL_R |= 0x00000004;  // Alternate function selected
  GPIO_PORTF_PCTL_R |=  0x00000700;  // PC0 as timer capture
  
  // timer
  GPTM_CONTROL_R &= ~TIM_A_ENABLE;
  GPTM_CONFIG_R = TIM_16_BIT_CONFIG; // Configure 16 - bit timer
  GPTM_TA_MODE_R |= 0x13; /* up-count, edge-count, capture mode */
  GPTM_CONTROL_R &= ~TIM_A_EVENT_BOTH_EDGES;  //positive edge
  GPTM_TA_IL_R = TIM_A_INTERVAL;
  GPTM_TA_PRESCALE_R = TIM_A_PRESCALE;
}
//-------------main----------------------------------------------------------------
int main(){
	Init_INT_GPIO();
	GPIO_Port_Init();
	Timer2A_Init();
	while(1) {
		WaitForInterrupt();
	}
}