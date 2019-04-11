/*
 * FinalFinal.c
 *
 * Created: 12/4/2018 4:10:49 PM
 * Author : Sean and Sal
 * 
 */ 
 //add AVRXSerial and AVRXClocks to lib
#include <avr/io.h>
#include <AVRXlib/AVRXSerial.h>
#include <AVRXlib/AVRXClocks.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#define SPIPort PORTC
#define DELAYTIME  10	//delay time in ms was 10
#define CLKPER  2000/8*DELAYTIME
#define BAUD 9600
#define BAUDCONT 12
#define TARGETV 150// target velocity in encoder steps per 100ms
#define DIFFERL .975//Percent of the target that the left wheel goes .975 final 

char bufTxArray[32];	//buffer transmit array
char bufRxArray[32];	//buffer receive array
volatile int state=0;//the state of the machine 
volatile int count=0,cycle=0,countUpdate=0;//used for control 
volatile int countR=0,countL=0,PulseCount=0;//encoders on right and left wheels
volatile char bdirchange=0,SignalIn=0;
volatile int gearcount=0,countpuck=0;
float PWRight = .5;//port E pin 2 LEFT IS RIGHT AND RIGHT IS LEFT
float PWLeft = .5;//port E pin 0
float PWGear=.80;//on port A pin 0
float PWPuck =.8;//on port A pin 2

void SetInt();
void SetUSART();
void SetPins();
void ChangeState(int newState);
void PwmUpdate();

XUSARTst stU;
volatile char bSend = 0, bReadFlag=0;

ISR(USARTC0_TXC_vect){
	Tx_Handler(&stU);
}
ISR(USARTC0_RXC_vect){
	Rx_Handler(&stU);
}
ISR(TCC0_OVF_vect){
	PORTE_OUT&=0b11111010;
	PORTA_OUT&=0b11111010;
	count++;
	countpuck++;
	gearcount++;
	countUpdate++;
}
ISR(TCC0_CCA_vect){//left motor
	PORTE_OUT|=0b00000001;
}
ISR(TCC0_CCB_vect){//right motor
	PORTE_OUT|=0b00000100;
}
ISR(TCC0_CCC_vect){//gear
	PORTA_OUT|=0b00000001;
}
ISR(TCC0_CCD_vect){//puck
	PORTA_OUT|=0b00000100;
}
ISR(PORTE_INT0_vect){
	countR++;
}
ISR(PORTE_INT1_vect){
	countL++;
}
ISR(PORTA_INT0_vect){//limit switch
	if(gearcount>10)
	{
		bdirchange=1;
		gearcount=0;
	}
}
ISR(PORTA_INT1_vect){
	//bfound=1;
	if(SignalIn==0)
	{
		countpuck=0;
		SignalIn=1;
	}
	PulseCount++;
}


int main(void)
{
	cli();		//clear
	SetUSART();
	SetPins();
	SetInt();
	sei();		//set

    /* Replace with your application code */
    while (1) 
    {
	if(PWLeft<.05||PWRight<.05)
		{//failsafe if its overdriving the motors 
		PORTB_OUT=~0x50;
		PWLeft=.04;
		PWRight=.04;
		PORTE_OUT&=0b11111010;
		return 0;//===================================================
		}
		switch (state)
		{
		case 0://going straight 
			if(SignalIn==1&&countpuck>25)//handle puck output
			{
				if(PulseCount>15&&PulseCount<45)
				{	
					TCC0_INTCTRLB|=PMIC_LOLVLEX_bm<<6;//puck's PWM on
					PORTB_OUT=0x00;
				}
				PulseCount=0;
			}
			if(SignalIn==1&&countpuck>50)
			{
				TCC0_INTCTRLB&=0x3f;//puck's PWM off
				PORTB_OUT=0xf0;
				SignalIn=0;
			}//done handle puck output 
			if(bdirchange==1)//handle direction change
			{
				gearcount=0;
				TCC0_INTCTRLB&=0xcf;//turn off gear's interrupts
				//PORTB_OUT^=0x30;
				bdirchange=2;
			}
			if(bdirchange==2&&gearcount>5)
			{
				PORTA_OUT^=0x02;//change direction pin
				TCC0_INTCTRLB|=PMIC_LOLVLEX_bm<<4;//gear's interrupts back on
				gearcount=0;
				//PORTB_OUT=0xf0;
				bdirchange=0;
			}//done with handling direction change 
			if(countUpdate>=10)
			{
				countUpdate=0;
				PwmUpdate();
			}
			if(count>=1000 && (cycle%2)==0)
				ChangeState(1);
			if(count>=1000 && (cycle%2)!=0)	
				ChangeState(2);
			break;
		case 1://turning right 
			if(SignalIn==1&&countpuck>50)
			{
				TCC0_INTCTRLB&=0x3f;//puck's PWM off
				SignalIn=0;
			}//done handle puck output
			if(countL>=9350)//5150
				ChangeState(0);
			break;
		case 2://turning left
			if(SignalIn==1&&countpuck>50)
			{
				TCC0_INTCTRLB&=0x3f;//puck's PWM off
				SignalIn=0;
			}//done handle puck output
			if(countR>=9000)//49000
				ChangeState(0);
		}
    }
}

void SetInt()
{
	//counter stuf===============>
	TCC0_PER = CLKPER;
	TCC0_CTRLA = 0x04;		//set to 8
	TCC0_CCA = (uint16_t)(((double)CLKPER))*PWLeft;//forward left 
	TCC0_CCB =(uint16_t)(((double)CLKPER))*PWRight;//right
	TCC0_CCC = (uint16_t)(((double)CLKPER))*PWGear;//gear
	TCC0_CCD =(uint16_t)(((double)CLKPER))*PWPuck;//puck
	TCC0_INTCTRLB = PMIC_LOLVLEX_bm|PMIC_LOLVLEX_bm<<2|PMIC_LOLVLEX_bm<<4;
	TCC0_INTCTRLA = PMIC_MEDLVLEX_bm;// set the timer interrupt to medium level
	//external stuff ============================>
	PORTA_INT0MASK=0x01<<4|0x01<<6;//set pin 4 and 6 to int 0 for limit switches 
	PORTA_INT1MASK=0x01<<5;//set pin 5 to int 1 for metal detector signal
	PORTA_PIN4CTRL=0x01;//set to rising edge0x01
	PORTA_PIN5CTRL=0x01;
	PORTA_PIN6CTRL=0x01;
	PORTA_INTCTRL=PMIC_MEDLVLEX_bm|PMIC_MEDLVLEX_bm<<2;//set
	PORTE_INT0MASK=0x01<<4;//set pin 4 to int 0 for right drive motor
	PORTE_INT1MASK=0x01<<5;//set pin 5 to int 1 for left drive motor
	PORTE_PIN4CTRL=0x01;//set to rising edge
	PORTE_PIN5CTRL=0x01;
	PORTE_INTCTRL=PMIC_MEDLVLEX_bm|PMIC_MEDLVLEX_bm<<2;//set encoder to med
	PMIC_CTRL=PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm; //turn on medium level interrupts
}

void SetUSART()
{
	unsigned long pClk;
	GetSystemClocks(NULL, &pClk);
	stU.TxCB=bufTxArray;
	stU.RxCB=bufRxArray;
	USART_init(&stU, 0xC0, pClk, _USART_RXCIL_LO|_USART_TXCIL_LO, 96, 0,_USART_CHSZ_8BIT, _USART_PM_DISABLED, _USART_SM_1BIT);
	USART_buffer_init(&stU, 32,32);
	stU.fOutMode = 0;//_OUTPUT_CRLF;
	stU.fInMode = _INPUT_LF;
	USART_enable(&stU, (USART_TXEN_bm | USART_RXEN_bm));
	PMIC_CTRL |= PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm; //turn on low/med level interrupt
}

void SetPins()
{
	PORTE_DIR=0x0f;
	PORTE_OUT=0x01<<1|0x01<<3;//A for back 
	PORTB_DIR=0xf0;//led port to out
	PORTB_OUT=0xf0;//LEDs off
	PORTA_DIR=0x0f;//for the limit switches, gear motor, and puck motor  
}

void ChangeState(int newState)
{
	switch (newState)
	{
	case 0:
		TCC0_INTCTRLB |= PMIC_LOLVLEX_bm|PMIC_LOLVLEX_bm<<2;
		TCC0_CCA = (uint16_t)(((double)CLKPER))*PWLeft;//for going forward
		TCC0_CCB = (uint16_t)(((double)CLKPER))*PWRight;
		TCC0_INTCTRLB|=PMIC_LOLVLEX_bm<<4;//gear's interrupts back on
		cycle++;
		break;
	case 1:
		TCC0_INTCTRLB &= ~(uint8_t)((PMIC_LOLVLEX_bm<<2)|(PMIC_LOLVLEX_bm<<4));//check this line if error=================
		TCC0_CCA = (uint16_t)(((double)CLKPER))*PWLeft;//turning right 
		TCC0_INTCTRLB&=0xcf;//turn off gear's interrupts
		break;
	case 2:
		TCC0_INTCTRLB&= ~(uint8_t)((PMIC_LOLVLEX_bm)|(PMIC_LOLVLEX_bm<<4));//check this line if error=================
		TCC0_CCB=(uint16_t)(((double)CLKPER))*PWRight;
		TCC0_INTCTRLB&=0xcf;//turn off gear's interrupts
		break;
	}
	count=0;
	state=newState;
}

void PwmUpdate()
{
	PWLeft-=((2.0/(1.0+exp(-0.05*(float)((TARGETV*DIFFERL)-countL))))-1.0)*(.03);
	PWRight-=((2.0/(1.0+exp(-0.05*(float)(TARGETV-countR))))-1.0)*(.03);
	TCC0_CCA=(uint16_t)(((double)CLKPER))*PWLeft;
	TCC0_CCB=(uint16_t)(((double)CLKPER))*PWRight;
	countR=0;
	countL=0;
}