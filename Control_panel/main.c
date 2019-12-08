/*
 * main.c
 *
 *  Created on: 04-10-2013
 *      Author: Karol
 */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define REL1_Pin (1<<PD1)
#define REL1_ON PORTD &= ~REL1_Pin
#define REL1_OFF PORTD |= REL1_Pin

#define REL2_Pin (1<<PD0)
#define REL2_ON PORTD &= ~REL2_Pin
#define REL2_OFF PORTD |= REL2_Pin

#define IN1_PIN (1<<PB1)
#define Sw !(PINB & IN1_PIN)

#define IN1_ON PORTB |= IN1_PIN
#define IN1_OFF PORTB &= ~IN1_PIN

volatile uint8_t bajt;
volatile uint8_t toggle_bit;
volatile uint8_t Manchester_flag;
volatile uint8_t key_time;			// ilo�� powt�rze� ramki przy wci�� wci�ni�tym klawiszu - autorepeat
uint8_t active =0, temp=0,alarm=0;

#define _micro_s(num) (((num)*((F_CPU/1000UL)/8))/1000)

#define TOLERANCE 40
#define HALF_BIT 	500
#define MIN_HALF_BIT 	_micro_s(HALF_BIT 	- 		TOLERANCE)
#define MAX_HALF_BIT 	_micro_s(HALF_BIT 	+ 		TOLERANCE)
#define MAX_BIT 	  _micro_s( (HALF_BIT*2) 	+ 		(TOLERANCE*2))

void man_rx_init(void) {
	/* inicjalizacja IR */
//	IR_DIR &= ~IR_PIN;		// pin IR jako wej�cie, poniewa� jest domy�lnie to pomijamy

	// KONFIGURACJA PRACY PRZERWANIA ICP I TIMERA1
	TCCR1B = (1<<CS11);	// ustawienie preskalera dla Timer1 = 8
	TCCR1B &= ~(1<<ICES1);	// reakcja na zbocze opadaj�ce
	TIMSK |= (1<<TICIE1);	// odblokowanie przerwania ICP
}

int main(void)
{
	DDRD |= REL1_Pin | REL2_Pin;
	REL1_OFF;
	REL2_OFF;

	DDRB |= IN1_PIN;
	//PORTB |= IN1_PIN;

	IN1_ON;
	_delay_ms(1000);
	IN1_OFF;
	man_rx_init();
	sei();


    //ADC
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1); // preskaler
    ADMUX |= (1<<REFS0); // nap odniesienia
    ADMUX |= (1<<ADLAR); // pom 8-bit




	while(1)
	{
		if(Manchester_flag==1)
		{
			if((bajt == 1) & (active==0))
					{
					IN1_ON;
					REL1_ON;
					_delay_ms(500);
					REL1_OFF;
					active=1;
					alarm=0;
					ADCSRA |=(1<<ADEN); // w� przetwornika
					_delay_ms(1000);

					}
			if((bajt == 2) & (active==1))
					{
					IN1_OFF;
					REL1_ON;
					_delay_ms(300);
					REL1_OFF;
					_delay_ms(500);
					REL1_ON;
					_delay_ms(300);
					REL1_OFF;
					active=0;
					ADCSRA &=~(1<<ADEN); // w� przetwornika
					}
			Manchester_flag=0;
		}



if(active==1)
{
		if(!(ADCSRA & (1<<ADSC)))
				{
					temp=ADCH;
					if(temp<100 || temp>190) alarm++;
					else alarm=0;
					_delay_ms(60);
					ADCSRA |= (1<<ADSC);
				}

		if(alarm>8)REL1_ON;

}


	}





}

// typ wyliczeniowy wykorzystywany w przerwaniu
// do okre�lania bie��cego statusu ramki
enum STAT { FRAME_RESTART, FRAME_OK, FRAME_END, FRAME_PROGRESS };

//***************** procedura obs�ugi przerwania ICP1
ISR(TIMER1_CAPT_vect) {
	// zmienne na potrzeby obs�ugi przerwania
	static uint16_t LastCapture;
	uint16_t PulseWidth;
	static uint8_t IrPulseCount;
	static uint32_t IrData;
	static enum STAT frame_status;
	uint8_t tbit=0;
	uint8_t nbajt=0;
	static uint8_t mancnt;
	static uint8_t last_toggle;

	PulseWidth = ICR1 - LastCapture;	// pomiar impulsu
	LastCapture = ICR1;

	TCCR1B ^= (1<<ICES1);			// zmiana zbocza wyzwalaj�cego na przeciwne

	// gdy czas by� d�u�szy ni� MAX_BIT - pocz�tek ramki danych
	if (PulseWidth > MAX_BIT ) mancnt = 0;

	// zainicjowanie odbioru ramki danych
	if (mancnt == 0) {
		IrData = 0;
		IrPulseCount = 0;
		TCCR1B |= (1<<ICES1);
		mancnt++;
		frame_status = FRAME_OK;
	}
	else
	if (frame_status == FRAME_OK) {
		// gdy zak��cenia (szpilki) - RESTART
		if ( PulseWidth < MIN_HALF_BIT ) frame_status = FRAME_RESTART;

		// gdy b��d ramki danych (mo�e inny standard ni� RC5) RESTART
		if ( PulseWidth > MAX_BIT ) frame_status = FRAME_RESTART;

		if (frame_status == FRAME_OK) {

			// mo�na w��czy� aby zobaczy� miganie diody LED
			// gdy pracuje przerwanie w trakcie pojawiaj�cych
			// si� szum�w na wej�ciu odbiornika
			//PORTB ^= (1<<PB6);

			// zwi�kszamy licznik gdy czas d�u�szy ni� HALF_BIT
			// aby zawsze reagowa� na �rodkowe zbocze
			if (PulseWidth > MAX_HALF_BIT) mancnt++;

			if ( (mancnt % 2) == 0 ) {

				IrData = IrData << 1;	// przesuwamy bity w lewo
				// je�li zbocze narastaj�ce to bit = 1
				if ( (TCCR1B & (1<<ICES1)) ) IrData |= 0x0001;

				IrPulseCount++;				// zwi�kszamy licznik odebranych bit�w
				if (IrPulseCount == 23)	{	// je�li odebrano pe�n� ramk�
					// je�li u�ytkownik obs�u�y� ju� poprzedni� ramk�
					if (Manchester_flag == 0)	{

						// wy�uskanie 3 fragment�w z ca�ej ramki
						nbajt = ~IrData;		// negujemy ponownie zanegowany bajt w nadajniku
						bajt 	= IrData >> 8;	// w�a�ciwy bajt danych
						tbit 	= (IrData >> 16);	// pierwszy bajt ramki z bitami startowymi
													// oraz bitem toggle

						// obs�uga licznika ramek na podstawie toggle bitu
						if (tbit == last_toggle) key_time++;
						else key_time = 0;

						// zapami�tujmey ostatni� warto�� bitu toggle
						last_toggle = tbit;

						// sprawdzamy czy bajt danych jest = drugiemu po zanegowaniu
						if( nbajt==bajt ) {
							// sprawdzamy czy ramce by�y prawid�owa sekwencja bit�w starowych
							if( (tbit & 0b01111100) == 0b01100100 ) {
								// sprawdzamy czy wyst�pi� prawid�owo zanegowany bit toggle
								if( (tbit & 0b00000011) < 3 && (tbit & 0b00000011) > 0 ) {
									// odzyskujemy warto�� bitu toggle
									toggle_bit = tbit & 0b00000001;
									// ustawiamy flag� odbioru ramki
									Manchester_flag = 1;
								}
							}
						}
					}
					// restart odbioru po zako�czeniu dekodowania ramki
					frame_status = FRAME_RESTART;
				}
			}
			mancnt++;
		}
	}

	if (frame_status == FRAME_RESTART) {
		mancnt = 0;
		TCCR1B &= ~(1<<ICES1);
	}
}
