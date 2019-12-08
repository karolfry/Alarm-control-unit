/*
 * main.c
 *
 *  Created on: 15-11-2013
 *      Author: K@ROL
 */


#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN (1<<PD5)
#define LED_ON PORTD &=~LED_PIN
#define LED_OFF PORTD |= LED_PIN
#define LED_TOG PORTD ^=LED_PIN

#define OUT_PIN (1<<PD6)
#define OUT_H	PORTD |= OUT_PIN
#define OUT_L 	PORTD &= ~OUT_PIN

#define HALF_BIT 500
#define DEFAULT_SYNC_LENGHT 80

#define SW_PIN (1<<PD2)
#define SW_DOWN (!(PIND & SW_PIN))

uint8_t trig=0;
uint8_t flaga =0;
void send_man_jeden(void)
{
	OUT_H;
	_delay_us(HALF_BIT);
	OUT_L;
	_delay_us(HALF_BIT);

}

void send_man_zero(void)
{
	OUT_L;
	_delay_us(HALF_BIT);
	OUT_H;
	_delay_us(HALF_BIT);
}

void man_sync(uint8_t bit_cnt)
{
	uint8_t i;
	if( !bit_cnt) bit_cnt = DEFAULT_SYNC_LENGHT;
	for(i=0 ; i<bit_cnt; i++) send_man_zero();
	OUT_H;
	_delay_us(HALF_BIT*3);

}

void man_send_byte(uint8_t byte, uint8_t toggle) {

	uint8_t i;
	uint8_t data=0b11100100;	// pierwszy bajt ramki (istotne 6 starszych bitów)
	uint8_t neg_bajt = ~byte;	// zanegowana wartoœæ przesy³anego bajtu

	uint8_t sreg;
	// blokujemy przerwania na czas emisji danych
	// aby zachowaæ odpowiednie czasy trwania impulsów


	// emisja pierwszych 6-ciu bitów startowych
	for( i=0; i<6; i++ ) {
		if ( !(data & 0x80) ) send_man_zero();
		else send_man_jeden();
		data<<=1;
	}

	// zanegowana wartoœæ bitu toggle
	if ( !toggle ) send_man_jeden();
	else send_man_zero();

	// w³aœciwy bit toggle
	if ( toggle ) send_man_jeden();
	else send_man_zero();

	// w³aœciwy bajt danych
	for( i=0; i<8; i++ ) {
		if ( !(byte & 0x80) ) send_man_zero();
		else send_man_jeden();
		byte<<=1;
	}

	// zanegowana postaæ bajtu danych
	for( i=0; i<8; i++ ) {
		if ( !(neg_bajt & 0x80) ) send_man_zero();
		else send_man_jeden();
		neg_bajt<<=1;
	}

	// zablokowanie nadajnika aby nie generowa³ szumu
	OUT_H;

	// odblokowujemy przerwania, jeœli by³y uprzednio w³¹czone
	//SREG = sreg;

	_delay_us(HALF_BIT*3);
}



int main(void)
{
	DDRD |= LED_PIN;
	DDRD &= ~SW_PIN;
	PORTD |= SW_PIN;

	DDRD |= OUT_PIN;
	man_sync(0);
	OUT_H;
	while(1)
	{
		if(SW_DOWN)
		{
		LED_TOG;
		man_send_byte(1,1);
		OUT_L;
		}
		else
		{
			LED_ON;
			man_send_byte(2,1);
			OUT_L;
		}
	}
}
