//Dead-time generation lab for bootstrap ic
/*
	*Firmware Description*
	Build        : 13-01-2023
	Design for   : ATMEGA88 + IR2101 base BLDC Motor Driver board EDR
	Firmware By  : U.Sorrawit 
	Board By     : P.Kittphop
	
	-Connection
	Phase-A: timer0 : OC0A : AH <non-inverting mode>			Arduino pin: 6
					: OC0B : AL <inverting Mode + deadtime>		Arduino pin: 5
					
	Phase-B: timer1 : OC1A : BH <non-inverting mode>			Arduino pin: 9
					: OC1B : BL <inverting mode + deadtime>		Arduino pin: 10
	
	Phase-C: timer2 : OC2A : CH <non-inverting mode>			Arduino pin: 11
					: OC2B : CL <inverting mode + deadtime>		Arduino pin: 3
	Generating the PWM in phase correct mode at 31.25 KHz
	
	Hall Effect Sensor
	Hall-A			: PCINT8	: PC0
	Hall-B			: PCINT9	: PC1
	Hall-C			: PCINT10	: PC2
	
	Accelerator		: ADC		: PC3
	
	Version Note:
	 - Change hall effect sensor and step verification technique
*/

#define F_CPU 16000000UL

#define MAX_PWM 200       //MAX_Value is 204
#define DEADTIME_GEN 21
#define AH 6
#define AL 5
#define BH 1
#define BL 2
#define CH 3
#define CL 3
//PORTC 
#define HALL_A 0
#define HALL_B 1
#define HALL_C 2

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void phaseAOut(int _PWM);
void phaseBOut(int _PWM);
void phaseCOut(int _PWM);
void ALow();
void BLow();
void CLow();
void debug();
int	A_IN();
void RUN_SEQ(uint8_t stp, uint8_t pwm_val);
void safety(int pwmValue);
int readHallEffectSensor();
	
uint8_t PWM_STEP = 0 ;
int ADCOUTPUT = 0;

ISR(ADC_vect)
{
	cli();
	ADCOUTPUT = ADC;
	sei();
}

int main(void)
{
	int pwmValue = 0, step = 0,stepPrevious = 0,stepOut = 0;
	DDRD |= (1 << 6)|(1 << 5)|(1 << 3);
	DDRB |= (1 << 1)|(1 << 2)|(1 << 3)|(1 << 5);
	DDRC  = 0x00;
	//PORTC |= (1 << HALL_A)|(1 << HALL_B)|(1 << HALL_C);
	PORTB |= (1 << 5);
	//Phase-A Register Setup:
	TCCR0A = 0x03;
	TCCR0B = 0x01; 
	//Phase-B Register Setup:
	TCCR1A = 0x01;
	TCCR1B = 0x01;
	//Phase-C Register Setup:
	TCCR2A = 0x03;            
	TCCR2B = 0x01;
	
	//Enable Global interrupt bit
	sei();
	
	//ADC module setup
	ADMUX |= (1 << REFS0) | (1 << MUX0) | (1 << MUX1);
	ADCSRA |= (1 << ADEN) | (1 << ADIE);

	//safety 
	pwmValue = A_IN();
	safety(pwmValue);
	
	//Set All phase low to charge the bootstrap capacitors
	ALow();
	BLow();
	CLow();
	
	while(1){
		step = readHallEffectSensor();
		if(step != stepPrevious && step != 0){
			
			stepOut = step;
			stepPrevious = step;		
		}
		pwmValue = A_IN();
		RUN_SEQ(stepOut,pwmValue);
	}
	return 0;
}

void RUN_SEQ(uint8_t stp, uint8_t pwm_val)
{
	if (pwm_val >= 1)
	{
		switch (stp)
		{
			case 1:
				ALow();
				phaseBOut(pwm_val);
				phaseCOut(0);
			break;
			case 2:
				ALow();
				phaseBOut(0);
				phaseCOut(pwm_val);
			break;
			case 3:
				phaseAOut(0);
				BLow();
				phaseCOut(pwm_val);
			break;
			case 4:
				phaseAOut(pwm_val);
				BLow();
				phaseCOut(0);
			break;
			case 5:
				phaseAOut(pwm_val);
				phaseBOut(0);
				CLow();
			break;
			case 6:
				phaseAOut(0);
				phaseBOut(pwm_val);
				CLow();
			break;
			default:
				phaseAOut(0);
				phaseBOut(0);
				phaseCOut(0);
			
		}
	}
	else{
		phaseAOut(0);
		phaseBOut(0);
		phaseCOut(0);		
	}
}

void phaseAOut(int _PWM){
	if(_PWM > 0 && _PWM <= MAX_PWM){
		TCCR0A = 0xB1;
		OCR0B = _PWM+DEADTIME_GEN;
		OCR0A = _PWM;
	}
	else{
		TCCR0A = 0;
		PORTD &= ~(1 << AL);
		PORTD &= ~(1 << AH);
	}
}

void phaseBOut(int _PWM){
	if(_PWM > 0 && _PWM <= MAX_PWM){
		TCCR1A = 0xB1;
		OCR1B = _PWM+DEADTIME_GEN;
		OCR1A = _PWM;
	}
	else{
		TCCR1A = 0;
		PORTB &= ~(1 << BH);
		PORTB &= ~(1 << BL);
	}	
}

void phaseCOut(int _PWM){
	if(_PWM > 0 && _PWM <= MAX_PWM){
		TCCR2A = 0xB1;
		OCR2B = _PWM+DEADTIME_GEN;
		OCR2A = _PWM;
	}
	else
	{
		TCCR2A = 0;
		PORTD &= ~(1 << CL);
		PORTB &= ~(1 << CH);
	}	
}

void ALow(){
	TCCR0A = 0;
	PORTD |= (1 << AL);
	PORTD &= ~(1 << AH);
}

void BLow(){
	TCCR1A = 0;
	PORTB &= ~(1 << BH);
	PORTB |= (1 << BL);	
}

void CLow(){
	TCCR2A = 0;
	PORTD |= (1 << CL);
	PORTB &= ~(1 << CH);
}


int A_IN()
{
	ADCSRA |= (1 << ADSC);
	int output = 0;
	while (ADCSRA & (1 << ADSC));
	output = ADCOUTPUT / 1023.000 * MAX_PWM;
	//output = (ADCOUTPUT - 184) * (MAX_PWM) / (634) ;
	//output = (ADCOUTPUT - 162) * (MAX_PWM) / (720) ;
	if (output > MAX_PWM)
		output = MAX_PWM;
	else if (output < 0)
		output = 0;
	return output;
}	



void debug(){
	for(int i = 0; i <= MAX_PWM;i++){
		phaseAOut(i);
		phaseBOut(i);
		phaseCOut(i);
		_delay_ms(25);
	}
	_delay_ms(500);
	for(int i =  MAX_PWM; i >= 0;i--){
		phaseAOut(i);
		phaseBOut(i);
		phaseCOut(i);
		_delay_ms(25);
	}
	_delay_ms(2000);
	ALow();
	BLow();
	CLow();
	_delay_ms(8000);
}

void safety(int pwmValue){
	if(pwmValue >= 40)
	{
		phaseAOut(0);
		phaseBOut(0);
		phaseCOut(0);
		while(pwmValue >= 40)
		pwmValue = A_IN();
	}
}

int readHallEffectSensor(){
	uint8_t read = PINC & 7;
	int step = 0;
	switch (read)
	{
		case 5:
		step = 1;
		break;
		case 1:
		step = 2;
		break;
		case 3:
		step = 3;
		break;
		case 2:
		step = 4;
		break;
		case 6:
		step = 5;
		break;
		case 4:
		step = 6;
		break;
		default:
		step = 0;
	}
	return step;	
}
