#define F_CPU 16000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define max_out 127
// PORTD
#define HALL_A 2  //Hall-effect Sensor A
#define HALL_B 3  //Hall-effect Sensor B
#define HALL_C 4  //Hall-effect Sensor C
#define PWM_B  5    //PWM output 2 for Half-bridge Driver B
// PORTC
#define SD_A 0    //Half-Bridge Driver A Shutdown pin
#define SD_B 1    //Half-Bridge Driver B Shutdown pin
#define SD_C 2    //Half-Bridge Driver C Shutdown pin

//PORTB
#define PWM_A 1   //PWM output 1 for Half-bridge Driver A
#define PWM_C 3   //PWM output 3 for Half-bridge Driver C

void RUN_SEQ(uint8_t stp, uint8_t pwm_val);
int A_IN();
int A_IN_DC();


uint8_t hall_A = 0, hall_B = 0, hall_C = 0, PWM_STEP = 0, step = 0, pv_step = 0;
uint8_t ADC_HIGH_BIT = 0, ADC_LOW_BIT = 0;

ISR(ADC_vect)
{
	ADC_LOW_BIT = ADCL;
	ADC_HIGH_BIT = ADCH;
}

ISR(PCINT2_vect)
{
	uint8_t chk = (PIND >> HALL_A) & 7;
	switch (chk)
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
}


/*ISR(PCINT2_vect)
{
	uint8_t chk = (PIND >> HALL_A) & 7;
	switch (chk)
	{
		case 5:
		step = 6;
		break;
		case 1:
		step = 5;
		break;
		case 3:
		step = 4;
		break;
		case 2:
		step = 3;
		break;
		case 6:
		step = 2;
		break;
		case 4:
		step = 1;
		break;
		default:
		step = 0;
	}
}*/


int main(){
	DDRC = 0x07;
	DDRB = 0x0e;
	DDRB |= (1 << 5);
	PORTB |= (1 << 5);
	DDRD |= (1 << PWM_B) | (1 << 6);
	PORTD |= (1 << HALL_A) | (1 << HALL_B) | (1 << HALL_C);
	TCCR0A = 0x03;             //TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B = 0x01;
	TCCR1A = 0x01;             //TCCR1A |= (1 << COM1A1) | (1 << WGM10);
	TCCR1B = 0x09;
	TCCR2A = 0x03;             //TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = 0x01;
	sei();
	ADMUX |= (1 << REFS0) | (1 << MUX0) | (1 << MUX1);
	ADCSRA |= (1 << ADEN) | (1 << ADIE);
	PCICR = (1 << PCIE2);
	PCMSK2 = 0x1c;
	uint8_t chk = (PIND >> HALL_A) & 7;
	switch (chk)
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
	int PWM_STEP = A_IN();
	if(PWM_STEP >= 40)
	{
		PORTC = 0;
		while(PWM_STEP >= 40)
		PWM_STEP = A_IN();
	}
	pv_step = step;
	while(1){
		PWM_STEP = A_IN();
		RUN_SEQ(step, PWM_STEP);
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
        PORTB = 0x00;               //PORTB &= ~(1 << PWM_A); PORTB &= ~(1 << PWM_C);
        PORTC = 0x01;
        PORTD = 0x3c;
        TCCR0A = 0x23;              //TCCR0A |= (1 << COM0B1);
        TCCR1A = 0x01;              //TCCR1A &= ~(1 << COM1A1);
        TCCR2A = 0x03;              //TCCR2A &= ~(1 << COM2A1);
        break;
      case 2:
        PORTC = 0x01;
        PORTD = 0x1c;
        PORTB = 0x08;               //PORTB &= ~(1 << PWM_A); PORTB |= (1 << PWM_C);
        TCCR0A = 0x03;              //TCCR0A &= ~(1 << COM0B1);
        TCCR1A = 0x01;              //TCCR1A &= ~(1 << COM1A1);
        TCCR2A = 0x83;              //TCCR2A |= (1 << COM2A1);
        break;
      case 3:
        PORTC = 0x02;
        PORTD = 0x1c;
        PORTB = 0x08;               //PORTB &= ~(1 << PWM_A); PORTB |= (1 << PWM_C);
        TCCR0A = 0x03;              //TCCR0A &= ~(1 << COM0B1);
        TCCR1A = 0x01;              //TCCR1A &= ~(1 << COM1A1);
        TCCR2A = 0x83;              //TCCR2A |= (1 << COM2A1);
        break;
      case 4:
        PORTC = 0x02;
        PORTD = 0x1c;
        PORTB = 0x02;               //PORTB |= (1 << PWM_A); PORTB &= ~(1 << PWM_C);
        TCCR0A = 0x03;              //TCCR0A &= ~(1 << COM0B1);
        TCCR1A = 0x81;              //TCCR1A |= (1 << COM1A1);
        TCCR2A = 0x03;              //TCCR2A &= ~(1 << COM2A1);
        break;
      case 5:
        PORTC = 0x04;
        PORTD = 0x1c;
        PORTB = 0x02;               //PORTB |= (1 << PWM_A); PORTB &= ~(1 << PWM_C);
        TCCR0A = 0x03;              //TCCR0A &= ~(1 << COM0B1);
        TCCR1A = 0x81;              //TCCR1A |= (1 << COM1A1);
        TCCR2A = 0x03;              //TCCR2A &= ~(1 << COM2A1);
        break;
      case 6:
        PORTB = 0x00;               //PORTB &= ~(1 << PWM_A); PORTB &= ~(1 << PWM_C);
        PORTC = 0x04;
        TCCR0A = 0x23;              //TCCR0A |= (1 << COM0B1);
        TCCR1A = 0x01;              //TCCR1A &= ~(1 << COM1A1);
        TCCR2A = 0x03;              //TCCR2A &= ~(1 << COM2A1);
        break;
      default:
        PORTC  &= 0x00;
		}
	}
	else
	PORTC  &= 0x00;
}


int A_IN()
{
	ADCSRA |= (1 << ADSC);
	int output = 0;
	while (ADCSRA & (1 << ADSC));
	output = ADC_LOW_BIT + ((ADC_HIGH_BIT & 1) * 256) + (((ADC_HIGH_BIT & 2) >> 1) * 512);
	output = output / 1023.000 * max_out;
	//output = (output - 184) * (max_out) / (634) ;
	//output = (output - 162) * (max_out) / (720) ;
	if (output > max_out)
	output = max_out;
	else if (output < 0)
	output = 0;
	OCR0B = output;
	OCR1A = output;
	OCR2A = output;
	return output;
}