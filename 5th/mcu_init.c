#include "mcu_init.h"

//////////////////////////////////////////////////////////////////
//InitIO()
//Initialize Input & Output of Port
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitIO(){
	//TO DO
	DDRA = 0xFF;	//0b 1111 1111	//LED				//PA0=LED1, PA1=LED2
	DDRC = 0xFF;	//0b 1111 1111	//?
	DDRD = 0x08;	//0b 0000 1000	//SWITCH			//PD0=SW1, PD1=SW2
	DDRB = 0xE7;	//0b 1110 0111  //PWM1&SPI			//PB0=SS, PB1=SCK, PB2=MOSI, PB3=MISO, PB5=PWM_A, PB6=PWM_B, PB7=PWM_C
	DDRE = 0x1A;	//0b 0001 1010	//UART0&Encoder		//PE0=RX, PE1=TX, PE4=H_A, PE5=H_B, PE6=H_C
	DDRF = 0x00;	//0b 0000 0000	//ADC				//PF0=MOTOR_CUR
	
	//PORTA = 0x00;	//0b 0000 0000	//LED1, LED2 ON
	PORTB = 0x07;	//0b 0000 0111
}

//////////////////////////////////////////////////////////////////
//InitExtInt()
//Initialize External Interrupt
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitExtInt(){
	//TO DO
	EICRA = INT1_FALLING | INT0_FALLING;
	EIMSK = INT1_ENABLE | INT0_ENABLE;
}

//////////////////////////////////////////////////////////////////
//InitTimer0()
//Initialize Timer0
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer0(){
	//TO DO
}

//////////////////////////////////////////////////////////////////
//InitTimer1()
//Initialize Timer1 20kHz
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer1(){
	
	//TO DO
	TCCR1A = 0b11100010;	//OC1A SET, OC1B Clear, OC1C Normal
	TCCR1B = 0b00011001;	//fast PWM, no prescaler
	ICR1 = ICR_TOP;			//799
	OCR1C = 0;
	OCR1A = 0;		//1 L
	OCR1B = 0;		//2 L
	TCNT1 = 0;
}

//////////////////////////////////////////////////////////////////
//InitTimer2()
//Initialize Timer2
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer2(){
	//TO DO
}

//////////////////////////////////////////////////////////////////
//InitTimer3()
//Initialize Timer3 
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer3(){
	//TO DO
	TCCR3A = 0b00000000;	//Normal Normal
	TCCR3B = 0b00000011;	//64 prescler
	ETIMSK = 0x04;			//Overflow Interrupt
}

//////////////////////////////////////////////////////////////////
//InitADC()
//InitADC
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitADC(){
	//TO DO
	ADMUX = 0x40;
	ADCSRA = 0x86;	//64 prescaler
}

//////////////////////////////////////////////////////////////////
//GetADC()
//GetADC
// Input : adc chanel
// Output : ADC Result
//////////////////////////////////////////////////////////////////
int GetADC(char ch){
	//TO DO
	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}

//////////////////////////////////////////////////////////////////
//InitUart0()
//InitUart0
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitUart0(){
	//TO DO
	//Uart
	UCSR0A = 0x00;
	UCSR0B = 0x98;
	UCSR0C = 0x06;
	UBRR0L = 103;		//9600
}

//////////////////////////////////////////////////////////////////
//InitUart1()
//InitUart1
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitUart1(){
	//TO DO
	DDRD = (DDRD & 0xF3) | 0x08;
	UCSR1A = 0x00;
	UCSR1B = USART_RECV_ENABLE | USART_TRANS_ENABLE;
	UCSR1C = USART_CHAR_SIZE_8BIT;
	UBRR1L = USART_115200BPS;
}
//////////////////////////////////////////////////////////////////
//InitSPI()
//InitSPI
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitSPI(){
	SPCR = 0x50;
	SPSR = 0x01;
}

//////////////////////////////////////////////////////////////////
//TransUart0()
//TransUart0
// Input : Transmit data
// Output : None
//////////////////////////////////////////////////////////////////
void TransUart0(unsigned char data){
	//TO DO
	while(!(UCSR0A & 0x20));
	UDR0 = data;
}

//////////////////////////////////////////////////////////////////
//TransUart1()
//TransUart1
// Input : Transmit data
// Output : None
//////////////////////////////////////////////////////////////////
void TransUart1(unsigned char data){
	//TO DO
	while(!(UCSR1A & 0x20));
	UDR1 = data;
}

//////////////////////////////////////////////////////////////////
//RecvUart0()
//RecvUart0
// Input : None
// Output : Recved Data
//////////////////////////////////////////////////////////////////
unsigned char RecvUart0(){
	//TO DO
	return UDR0;
}

//////////////////////////////////////////////////////////////////
//RecvUart1()
//RecvUart1
// Input : None
// Output : Recved Data
//////////////////////////////////////////////////////////////////
unsigned char RecvUart1(){
	//TO DO
	return UDR1;
}

//////////////////////////////////////////////////////////////////
//TransNumUart0()
//TransNumUart0
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void TransNumUart0(int num){
	//TO DO
	if(num < 0){
		TransUart0('-');
		num *= -1;
	}
	TransUart0( ((num%10000000) / 1000000) + 48);
	TransUart0( ((num%1000000) / 100000) + 48);
	TransUart0( ((num%100000) / 10000) + 48);
	TransUart0( ((num%10000) / 1000) + 48);
	TransUart0( ((num%1000) / 100) + 48);
	TransUart0( ((num%100) / 10) + 48);
	TransUart0( num%10 + 48 );
}

//////////////////////////////////////////////////////////////////
//SendShortUART0()
//SendShortUART0
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void SendShortUART0(int16_t num){
	if(num < 0){
		TransUart0('-');
		num *= -1;
	}
	TransUart0( ((num%100000) / 10000) + 48);
	TransUart0( ((num%10000) / 1000) + 48);
	TransUart0( ((num%1000) / 100) + 48);
	TransUart0( ((num%100) / 10) + 48);
	TransUart0( num%10 + 48 );
}

//////////////////////////////////////////////////////////////////
//TransNumUart1()
//TransNumUart1
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void TransNumUart1(int num){
	//TO DO
	if(num < 0){
		TransUart1('-');
		num *= -1;
	}
	TransUart1( ((num%10000000) / 1000000) + 48);
	TransUart1( ((num%1000000) / 100000) + 48);
	TransUart1( ((num%100000) / 10000) + 48);
	TransUart1( ((num%10000) / 1000) + 48);
	TransUart1( ((num%1000) / 100) + 48);
	TransUart1( ((num%100) / 10) + 48);
	TransUart1( num%10 + 48 );
}

//////////////////////////////////////////////////////////////////
//SendShortUART1()
//SendShortUART1
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void SendShortUART1(int16_t num){
	if(num < 0){
		TransUart1('-');
		num *= -1;
	}
	TransUart1( ((num%100000) / 10000) + 48);
	TransUart1( ((num%10000) / 1000) + 48);
	TransUart1( ((num%1000) / 100) + 48);
	TransUart1( ((num%100) / 10) + 48);
	TransUart1( num%10 + 48 );
}

//////////////////////////////////////////////////////////////////
//SPI_MasterSend()
//SPI_MasterSend
// Input : data
// Output : None
//////////////////////////////////////////////////////////////////
void SPI_MasterSend(unsigned char data){
	SPDR = data;
	while (!(SPSR & 0x80));
	data = SPDR;
}

//////////////////////////////////////////////////////////////////
//SPI_MasterSend()
//SPI_MasterSend
// Input : None
// Output : data
//////////////////////////////////////////////////////////////////
unsigned char SPI_MasterRecv(void)
{
	SPDR = 0x00;
	while (!(SPSR & 0x80));
	return SPDR;
}