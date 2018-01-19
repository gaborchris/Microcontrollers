// C. Gabor
// 11.22.17
//remote temp sensor

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define TRUE 1
#define FALSE 0
#define ONE 0xF0
#define SECOND 0
#define MINUTE 1
#define RADIO 2
#define ALARM 1
#define NORMAL 0
#define HOUR 2
#define MAX_CHAN 10790
#define MIN_CHAN 8810
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

uint16_t adc_result;     //holds adc result 
uint16_t lm73_temp;  //a place to assemble the temperature from the lm73

uint8_t alarm_armed; //status that the alarm is set to go off
uint8_t mode_display;	//determines which display to show for the LEDs
uint8_t alarm_on;	//status that says that alarm is on, helpful for snooze

uint8_t normal_time_display[5];	//the LED display for thr normal time encoded to 7 segment
uint8_t normal_time[3];	//the actual values of the time in seconds, minutes, hours

//holds data to be sent to the segments. logic zero turns segment on
uint8_t alarm_time_display[5];	//the display of the alarm
uint8_t alarm_time[3];	//the actual time of the alarm
uint8_t snooze_time[3];	//the actual time of the snooze value, ie: when the next alarm goes off after snoozing

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]; 

//variable that button that was pushed
uint8_t debounced_state[8];

uint8_t timer0_counter;	// a counter for the TCNT0 vector

uint16_t alarm_count;	//a variable to set the tones of the alarm

char temp_str_array[16];  //holds string to send to lcd
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
char uart_recieve[2];
volatile uint8_t  rcv_rdy;
char rx_char; 
char lcd_str_array[16];  //holds string to send to lcd

volatile uint16_t  current_fm_freq =  9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
volatile uint16_t  eeprom_fm_freq =  9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
volatile uint16_t  current_volume; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
volatile uint16_t  eeprom_volume; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
extern uint8_t  si4734_wr_buf[9];
extern uint8_t  si4734_rd_buf[9];
extern uint8_t  si4734_tune_status_buf[8];
extern volatile uint8_t STC_interrupt;     //indicates tune or seek is done

uint8_t radio_time_display[5];	//the display of the alarm
uint8_t Radio_On;
void Time_Display();
void Init_Radio();  

uint8_t prev_encoder_state;

void Change_Radio(){
    fm_pwr_up();
    //    _delay_ms(1);
    fm_tune_freq();
    fm_tune_freq();
    //   _delay_ms(1);
}
//******************************************************************************
//                          External Interrupt 7 ISR                     
// Handles the interrupts from the radio that tells us when a command is done.
// The interrupt can come from either a "clear to send" (CTS) following most
// commands or a "seek tune complete" interrupt (STC) when a scan or tune command
// like fm_tune_freq is issued. The GPIO2/INT pin on the Si4734 emits a low
// pulse to indicate the interrupt. I have measured but the datasheet does not
// confirm a width of 3uS for CTS and 1.5uS for STC interrupts.
//
// I am presently using the Si4734 so that its only interrupting when the 
// scan_tune_complete is pulsing. Seems to work fine. (12.2014)
//
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){STC_interrupt = TRUE;}
//******************************************************************************

ISR(USART0_RX_vect){
    set_cursor(2,8);
    static uint8_t i;
    rx_char = UDR0;              //get character
    lcd_str_array[i++]=rx_char;  //store in array 
    //if entire string has arrived, set flag, reset index
    if(rx_char == '\0'){
	rcv_rdy=1; 
	lcd_str_array[--i]  = (' ');     //clear the count field
	lcd_str_array[i+1]  = (' ');
	lcd_str_array[i+2]  = (' ');
	i=0;  
    }
}
void Local_Temp(uint16_t lm73_temp){
    set_cursor(2,2);
    twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
    Time_Display();
    lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
    lm73_temp = (lm73_temp<<8); //shift it into upper byte 
    lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
    itoa(lm73_temp>>7,temp_str_array, 10); //convert to string in array with itoa() from avr-libc                           
    string2lcd(temp_str_array); //send the string to LCD (lcd_functions)
}    
//*************************************
//ADC_input
//finds the voltage on the photo resistor
//returns of the value of the voltage on this pin in a 16 bit integer value
//values range from about 0-1000, indicating 0V through 5V
//*************************************
void ADC_Input(){

    ADCSRA |=(1<<ADSC); //poke ADSC and start conversion

    while(bit_is_clear(ADCSRA, ADIF)){} //spin while interrupt flag not set

    ADCSRA |= (1<<ADIF);  //its done, clear flag by writing a one 

    adc_result = ADC;                      //read the ADC output as 16 bits

}

//*************************************
//Set Dim
//used to set the dimness on the 7 seg display
//change the PWM output compare register to change how often PWM toggles
//the higher the voltage onthe ADC, the higher the brightness
//each value set for evensmoothing
//*************************************
void Set_Dim(){
    switch(adc_result/20){
	case 47: OCR2 = 0xff; break;	//about 4.7 volts on the ADC
	case 46: OCR2 = 0xff; break;
	case 45: OCR2 = 0xff; break;
	case 44: OCR2 = 0xff; break;
	case 43: OCR2 = 0xff; break;
	case 42: OCR2 = 0xff; break;
	case 41: OCR2 = 0xef; break;	//start dimming the display at this value
	case 40: OCR2 = 0xdf; break;
	case 39: OCR2 = 0xcf; break;
	case 38: OCR2 = 0xbf; break;
	case 37: OCR2 = 0xaf; break;
	case 36: OCR2 = 0x9f; break;
	case 35: OCR2 = 0x8f; break;
	case 34: OCR2 = 0x7f; break;
	case 33: OCR2 = 0x6f; break;
	case 32: OCR2 = 0x5f; break;
	case 31: OCR2 = 0x4f; break;
	case 30: OCR2 = 0x3f; break;
	case 29: OCR2 = 0x2f; break;
	case 28: OCR2 = 0x1f; break;
	case 27: OCR2 = 0x15; break;
	case 26: OCR2 = 0x12; break;
	case 25: OCR2 = 0x10; break;
	case 24: OCR2 = 0x0f; break;
	case 23: OCR2 = 0x08; break;
	case 22: OCR2 = 0x05; break;
	case 21: OCR2 = 0x02; break;
	case 20: OCR2 = 0x01; break;
	case 19: OCR2 = 0x00; break;	//display dim as possible at this value
	case 18: OCR2 = 0x00; break;
	case 17: OCR2 = 0x00; break;
	case 16: OCR2 = 0x00; break;
	case 15: OCR2 = 0x00; break;
	case 14: OCR2 = 0x00; break;
	case 13: OCR2 = 0x00; break;
	case 12: OCR2 = 0x00; break;
	case 11: OCR2 = 0x00; break;
	case 10: OCR2 = 0x00; break; 
	case 9: OCR2 = 0x00; break;
	case 8: OCR2 = 0x00; break;
	case 7: OCR2 = 0x00; break;
	case 6: OCR2 = 0x00; break;
	case 5: OCR2 = 0x00; break;
	case 4: OCR2 = 0x00; break;
	case 3: OCR2 = 0x00; break;
	case 2: OCR2 = 0x00; break;
	case 1: OCR2 = 0x00; break;
	case 0: OCR2 = 0x00; break;
	default: OCR2 = 0xff; break;
    }
}
//*****************************************
//Sets the values of the dec_to_7seg array to their corresponding values that will show up on the LED display
//*****************************************
void dec_to_7seg_init(){
    dec_to_7seg[0]=0b11000000;
    dec_to_7seg[1]=0b11111001;
    dec_to_7seg[2]=0b10100100;
    dec_to_7seg[3]=0b10110000;
    dec_to_7seg[4]=0b10011001;
    dec_to_7seg[5]=0b10010010;
    dec_to_7seg[6]=0b10000010;
    dec_to_7seg[7]=0b11111000;
    dec_to_7seg[8]=0b10000000;
    dec_to_7seg[9]=0b10010000;
    dec_to_7seg[10]=0b11111111;
}

//*************************************
//tnct0 init
//initializes the tcnt0 mode
//set to interrupt on compare match, uses external clock
//used to find the time each second for real world time
//*************************************
void tcnt0_init(void){
    TIMSK  |=  (1<<TOIE0); //enable TCNT0 overflow interrupt
    TCCR0  |=  (1<<CS00)|(0<<CS01)|(0<<CS02); //normal mode
    ASSR   |=  (1<<AS0);	//use external clock
}                     

//*************************************
//tnct2 init
//initializes the tcnt2 mode
//PWM mode, inverted mode
//used for the dimming of the 7 seg as PWM source
//*************************************
void tcnt2_init(void){
    TCCR2  |=  (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<COM20)|(1<<CS20); //PWM mode, inverted mode
    OCR2 = 0xff; //OCR2 intially high so that 7 seg is at max

}                     
//*************************************
//tnct3 init
//initializes the tcnt3 mode
//set the fast PWM with ICR3 at top 
//used to set the volume on the audio amplifer
//*************************************
void TCNT3_Init(void){
    DDRE |= (1<<PE3);
    //Fast PWM
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);
    //ICR3 as TOP
    TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS30);
    OCR3A=0xF000; //set at this value	so it is initially off
    ICR3 = 0xF000;	//cleared at this value

}


//*************************************
//change_normal_display
//set the noraml dispay time to match the value stored in the array using enodings for 7 seg
//*************************************
void change_normal_display(){
    uint8_t digit_value;	//dummy variable

    digit_value = normal_time[MINUTE] % 10; //minutes
    normal_time_display[0] = dec_to_7seg[digit_value]; 	//set the new time for the display array

    digit_value = normal_time[MINUTE] - (normal_time[MINUTE]%10);	//logic for second digit
    digit_value = digit_value/10;
    normal_time_display[1] = dec_to_7seg[digit_value]; 	//set the new time for the display array

    digit_value = normal_time[HOUR] % 10; //minutes
    normal_time_display[3] = dec_to_7seg[digit_value]; 

    digit_value = normal_time[HOUR] - (normal_time[HOUR]%10);	//logic for second digit
    digit_value = digit_value/10;
    normal_time_display[4] = dec_to_7seg[digit_value]; 	//set the new time for the display array

}
//*************************************
//change alarm display
//set the alarm display to match the value stored in the array for minute and hour
//*************************************
void change_alarm_display(){
    uint8_t digit_value;	//dummy variable

    digit_value = alarm_time[MINUTE] % 10; //minutes
    alarm_time_display[0] = dec_to_7seg[digit_value]; 	//set this new time in the display array

    digit_value = alarm_time[MINUTE] - (alarm_time[MINUTE]%10);	//logic to see second digit
    digit_value = digit_value/10;
    alarm_time_display[1] = dec_to_7seg[digit_value]; 	//set this new time in the display array

    digit_value = alarm_time[HOUR] % 10; //minutes
    alarm_time_display[3] = dec_to_7seg[digit_value]; 

    digit_value = alarm_time[HOUR] - (alarm_time[HOUR]%10);	//logic to see second digit
    digit_value = digit_value/10;
    alarm_time_display[4] = dec_to_7seg[digit_value]; 	//set this new time in the display array

}
void change_radio_display(){
    uint16_t digit_value;	//dummy variable
    uint16_t station;

    station = current_fm_freq/10;

    //0
    digit_value  = station%10; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
    digit_value &= 0b01111111;
    radio_time_display[0] = dec_to_7seg[digit_value]; 	//set this new time in the display array

    station = station - station%10;

    //1
    digit_value = station%100;
    digit_value = digit_value/10;
    radio_time_display[1] = dec_to_7seg[digit_value]; 	//set this new time in the display array
    radio_time_display[1] &= 0b01111111;

    station = station - station%100;

    //3
    digit_value = station%1000;
    digit_value = digit_value/100;
    radio_time_display[3] = dec_to_7seg[digit_value]; 	//set this new time in the display array

    station = station - station%1000;

    //4
    digit_value = station%10000;
    digit_value = digit_value/1000;
    if(digit_value == 0){
	digit_value = 10;

    }
    radio_time_display[4] = dec_to_7seg[digit_value]; 	//set this new time in the display array


    radio_time_display[2] = 0b11111111; 	//set this new time in the display array

}

//*****************************************
//spiRecieve
//Recieves the input from the SPI interface
//*****************************************
uint8_t spiRecieve(void)
{
    uint8_t x;          //variable to store recieved value
    PORTE |= 0x80;  //Set PORTE7 high for loading parallel data from encoder
    PORTE &= ~0x40; //Set PORTE6 low to load in parallel data from econders to buffer
    PORTE |= 0x40;  //Set PORTE6 back high
    PORTE &= ~0x80; //Set PORTE7 low to serially load out the contents from the encoder buffer
    SPDR = 0x00; //send a dummy value to the SPI interface
    while(!(SPSR &(1<<SPIF))){}  //wait for SPI to finish sending
    x = SPDR; //read the value loaded into the SPI buffer from the encoder buffer
    return x;
}
void encoder_logic(void){
    uint8_t i;
    i = ~spiRecieve();	//Recieves the input from the encoders and negates this
    if(mode_display == RADIO){
	switch(i & 0x03){   //look at bottom 2 bits of the recieved encoder byte
	    case 0x00: 					//The value of the bottom 2 bits from the encoder
		switch(prev_encoder_state & 0x03){		//compare to last encoder state
		    case 0x02: 
			if(current_fm_freq != MAX_CHAN) 
			    current_fm_freq += 20; 
			else
			    current_fm_freq = MIN_CHAN;
			change_radio_display();  
			if(Radio_On == TRUE)
			    Change_Radio();
			break;    	//turned clockwise, set increment to 1
		    case 0x01: 
			if(current_fm_freq != MIN_CHAN) 
			    current_fm_freq -= 20; 
			else
			    current_fm_freq = MAX_CHAN;
			change_radio_display();  
			if(Radio_On == TRUE)
			    Change_Radio();
			break;	//turned counterclockwise, set decrement to 1
		    default: break;	//any other case, (including value hasn't changed)
		}
		break;
	    default: 
		break;
	}                                                   
    }
    if(Radio_On == TRUE){
	switch((i & 0x0C)>>2){       //look at next 2 bits of the recieved encoder byte shifted over by 2
	    case 0x00: 		 //same logic as first encoder state comparison
		switch((prev_encoder_state & 0x0C)>>2){
		    case 0x02: 
			    if(OCR3A >=0x1000)
				OCR3A -= 0x1000;
			break;
		    case 0x01: 
			    if(OCR3A <=0xE000)
				OCR3A += 0x1000;
			break;
		    default: break;
		}
	    default:
		break;                                
	}
    }
    prev_encoder_state = i;	//now set the current encoder state to the stored encoder state for next interrupt
}
//*************************************
//Compare_Alarm_To_Normal
//used to see if the current time matches the alarm set time
//check the hour, minute and second for both the alarm time and the snooze time
//*************************************
uint8_t Compare_Alarm_To_Normal(){
    if(alarm_time[HOUR] == normal_time[HOUR]){
	if(alarm_time[MINUTE]==normal_time[MINUTE])
	    if(alarm_time[SECOND]==normal_time[SECOND])
		return TRUE;	//return true if all the times match between normal time and alarm time
    }
    if(snooze_time[HOUR] == normal_time[HOUR]){
	if(snooze_time[MINUTE]==normal_time[MINUTE])
	    if(snooze_time[SECOND]==normal_time[SECOND])
		return TRUE;	//return true if all the times match between normal time and snooze time
    }
    return FALSE;	//otherwise, return false
}

//*************************************
//Enable Alarm
//Turn on the volume of PWM to max
//Turn on the interrupt for compare match
//*************************************
void Enable_Alarm(){
    OCR3A = 0x0000;	//turn on the volume to max by making PWM always on
    TIMSK |= (1<<OCIE1A);	//enable the interrupts
    alarm_on = TRUE;	// set the state for alarm going off
}

//*************************************
//Disable Alarm
//turn off the alarm by ignoring the output compare interrupt
//Also turn on or turn off the volume
//*************************************
void Disable_Alarm(){
    OCR3A = 0xFFFF;	//set the volume off by making the PWM always off, prevents static on the speakers
    TIMSK &= ~(1<<OCIE1A);	//turn of the interrupt
    alarm_on = FALSE;	//set the state of the alarm
}

//*************************************
//ISR for the TCNT1 compare on CTC mode
//this sets the tone of the sound pin going to the audio amplifier
//the OCR1A is changed so that the tone changes from beeps to tones
//*************************************
ISR(TIMER1_COMPA_vect){

    TCCR1A = 0x00;	//disable the pin connected to the OCR1A output so that it does not mess up the 7 seg display

    alarm_count++;	//increment the alarm count for beeping and music control

    if(alarm_count % 2 == 0){	//toggle the PORTD pin 3 pin which connects to audio amplifier on the even counts
	PORTD |= (1<<PD3);
    }
    else{	//toggle the PORTD pin 3 on the odd counts
	PORTD &= ~(1<<PD3);
    }
    switch(alarm_count){	//creates weird tones by changing the OCR1A register value
	case 10: OCR1A = 0x0150; break;
	case 20: OCR1A = 0x0140; break;
	case 30: OCR1A = 0x0130; break;
	case 40: OCR1A = 0x0120; break;
	case 50: OCR1A = 0x0150; break;
	case 60: OCR1A = 0x0020; break;
		 //random sounds
	case 180: OCR1A = 0xffff; break;	//random sounds
	case 181: OCR1A = 0x0040; break;

	case 310: OCR1A = 0x0150; break;	//random sounds
	case 320: OCR1A = 0x0140; break;
	case 330: OCR1A = 0x0130; break;
	case 340: OCR1A = 0x0120; break;
	case 350: OCR1A = 0x0150; break;
	case 360: OCR1A = 0x0140; break;
	case 410: OCR1A = 0x0150; break;
	case 420: OCR1A = 0x0140; break;
	case 430: OCR1A = 0x0130; break;
	case 440: OCR1A = 0x0120; break;	//random sounds
	case 450: OCR1A = 0x0020; break;

	case 510: OCR1A = 0x0250; break;
	case 520: OCR1A = 0x0100; break;
	case 530: OCR1A = 0x0160; break;	//random sounds
	case 540: OCR1A = 0x0020; break;
	case 550: OCR1A = 0x0150; break;
	case 560: OCR1A = 0x0060; break;
	case 710: OCR1A = 0x0050; break;
		  //random sounds
	case 1200: OCR1A = 0xffff; break;
	case 1201: OCR1A = 0x0020; break;
	case 1401: alarm_count = 0; break;	//set the alarm count back to 0 to repeat this pattern

    }
}

//*************************************
//ISR for timer 0 overflow
//Each time timer 0 reaches top, update the real world time
//Also check the brightness and update the dimness for this
//brightness checked here because brightness does not need to be updated very often (20-30 fps is fine to human eye)
//*************************************
ISR(TIMER0_OVF_vect){
    timer0_counter++;	//update the global timer
    if(timer0_counter % 10 ==0){   // at about 13x per second, check the brightness in the outside world and update the PWM
	ADC_Input();	//get the brightness from the ADC light sensor
	Set_Dim();	//set the PWM dimness on the 7 seg display PWM pin
    }
    if(timer0_counter == 64){	//half second interval
	uart_putc('c');
	normal_time_display[2] &= 0b11111100;	//turn on the colon so that it blinks on every half second
    }
    else if(timer0_counter == 128){	//one second interval
	normal_time_display[2] |= 0b00000011;	//turn colon back off so that it blinks off every half second

	normal_time[SECOND]++;	//add a second to the real world time since a second has elapsed
	if(normal_time[SECOND] == 60){	// do some logic to see if the minute needs to be incremented
	    normal_time[MINUTE]++; 
	    normal_time[SECOND] = 0;
	    change_normal_display();	//update the 7 seg display
	}
	if(normal_time[MINUTE] == 60){	//check if an hour needs to be added 
	    normal_time[HOUR]++; 
	    normal_time[MINUTE] = 0;
	    change_normal_display();	//update the 7 seg display
	}
	if(normal_time[HOUR] == 24){	//check if the hour needs to roll over to 0
	    normal_time[HOUR] = 0;
	    change_normal_display();	//update the 7 seg display
	}
	timer0_counter = 0;	//each second, set the time back to 0
	set_cursor(2,0);
	string2lcd(" L:"); //send the string to LCD (lcd_functions)
	set_cursor(2,6);
	string2lcd(" R:"); //send the string to LCD (lcd_functions)
	Local_Temp(lm73_temp);
    }

    if(alarm_armed){	//check if alarm is set to go off, only do this is alarm is armed
	if(Compare_Alarm_To_Normal()){	//returns true if the compare match is found between the real time and the time set for the alarm to go off
	    Enable_Alarm();	//turn the alarm on if these checks pass
	}
    }
}

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(){
    uint8_t i=0;  //make a counter variable
    static uint16_t state[8] = {0,0,0,0,0,0,0,0};  //set the state for each button to 0 initially
    while(i<8){  //function adapted from debouce in lab1 code, checks each pin in in porta for if it was pressed
	state[i]= (state[i]<< 1) | (! bit_is_clear(PINA, i)) | 0xE000;  //check if the pin was pressed
	if (state[i] == 0xF000){  //if the pin was pressed, store this information in debounced_state variable
	    debounced_state[i] = 1;
	    return 1;
	}
	else{
	    debounced_state[i] = 0;
	    i+=1;
	}
    }
    return 0;
}

//*********************************
//LED_display function  
//Displays the contents in the segment_data array to the LED display on PORTA
//Changes PORTB to cycle through the digital display LEDs
//*********************************
void LED_display(uint8_t* array){

    PORTA = array[0]; //send 7 segment code to LED segments
    PORTB = 0x00; //send PORTB the digit to display
    _delay_ms(0.2);  //delay the display so that it has time to light up
    PORTA = array[1]; //update digit to display
    PORTB = 0x10;
    _delay_ms(0.2);
    PORTA = array[2]; //update digit to display
    PORTB = 0x20;
    _delay_ms(0.2);
    PORTA = array[3];
    PORTB = 0x30;
    _delay_ms(0.2);
    PORTA = array[4];
    PORTB = 0x40;
    _delay_ms(0.2);
    PORTB = 0x60;
}
//*************************************
//Time_Display
//Determines which display should be put on the 7 seg and calls that function
//*************************************
void Time_Display(){
    switch(mode_display){	//check mode_display to see which mode the display should be in
	case ALARM: 		//based on defines at the top
	    LED_display(alarm_time_display);	 //set the display on the seven segment display
	    break;
	case NORMAL:
	    LED_display(normal_time_display);	 //set the display on the seven segment display
	    break;
	case RADIO:
	    LED_display(radio_time_display);
	    break;
    }
}
//*************************************
//Snyc_Snooze function
//Sets the snooze time to be equal to the alarm time
//makes sure that the snooze time matches up with the alarm
//*************************************
void Sync_Snooze(){
    snooze_time[SECOND]=alarm_time[SECOND];    
    snooze_time[MINUTE]=alarm_time[MINUTE];    
    snooze_time[HOUR]=alarm_time[HOUR];    
}
//*************************************
//Snooze function
//Sets the snooze time to be 10 seconds more than the current time
//if this rolls over, make sure makes logical sense time wise, for example, no 25oclock
//*************************************
void Snooze(){
    snooze_time[HOUR] = normal_time[HOUR];	//set the current normal time
    snooze_time[MINUTE] = normal_time[MINUTE];
    snooze_time[SECOND] = normal_time[SECOND] + 10;	//add 10 seconds 
    if(snooze_time[SECOND] >= 60){		//logical to handle rolling over real time
	snooze_time[SECOND] = snooze_time[SECOND] % 60;
	snooze_time[MINUTE]++;
	if(snooze_time[MINUTE] == 60){	//correct minute
	    snooze_time[MINUTE] = 0;
	    snooze_time[HOUR]++;
	    if(snooze_time[HOUR] == 24){	//correct hour
		snooze_time[HOUR] = 0;
	    }
	}
    }
}
//*************************************
//Get_Input
//The main code that does the logic on the push buttons for user interface
//Two modes exist, normal mode and alarm mode
//normal mode allows the user to change time, set the alarm, snooze the alarm, etc
//alarm mode only allows the user to set the alarm time
//*************************************
void Get_Input(){
    if(chk_buttons()){             //check that button was pressed
	switch(mode_display){		//two modes, alarm and normal mode
	    case NORMAL:
		if(debounced_state[1] == 1){		//this toggles mode of the display
		    mode_display ^= 1; 	//switches between alarm mode and normal mode display
		}
		else if(debounced_state[2] == 1){	//this is the snooze button
		    if(alarm_on){		//only snoozes if the alarm is currently going off
			Disable_Alarm();	//turn off alarm first 
			Snooze();	//snooze, which sets the snooze time to the right time ahead
			alarm_count=0;	//set the tone back to the initial value
		    }
		}
		else if(debounced_state[0] == 1){	//alarm set button, ie, enables alarm to go off when hits the alarm time
		    switch(alarm_armed){	//for toggling alarm
			case TRUE:		//if alarm was alread armed
			    set_cursor(1,0);
			    string2lcd("          ");	//set the LCD to read armed alarm
			    Disable_Alarm();	//turn off the alarm if its going off
			    normal_time_display[2] |= 0b00000100;	//change the alarm indication on the 7 seg
			    alarm_time_display[2] |= 0b00000100;	//do for both alarm and normal mode dislay
			    alarm_armed = FALSE;	//set the state
			    break;
			case FALSE:	//if alarm isn't armed yet
			    set_cursor(1,0);
			    string2lcd(" Alarm set");	//set the LCD to read armed alarm
			    normal_time_display[2] &= ~0b00000100;	//toggle the alarm to be indicated on the 7 seg
			    alarm_time_display[2] &= ~0b00000100;
			    alarm_armed = TRUE;	//alarm now on
			    break;
		    }
		}
		else if(debounced_state[3] == 1){
		    change_radio_display();
		    mode_display = RADIO;
		}
		else if(debounced_state[4] == 1){
		    if(Radio_On == FALSE){
			Init_Radio();
			_delay_ms(1);
			OCR3A=0x5000; 
			Change_Radio();
			Radio_On = TRUE;
		    }
		    else{
			OCR3A = 0xF000;
			radio_pwr_dwn();
			Radio_On = FALSE;


		    }
		}
		else if(debounced_state[6] == 1){	//add an hour button
		    if(normal_time[HOUR]<23)
			normal_time[HOUR]++;	//add an hour
		    else
			normal_time[HOUR] = 0;
		    change_normal_display();	//update 7 seg array 
		}

		else if(debounced_state[7] == 1){	//add a minute button
		    if(normal_time[MINUTE]<59)
			normal_time[MINUTE]++;
		    else
			normal_time[MINUTE] = 0;
		    change_normal_display();	//update 7 seg array 
		}
		break;

	    case ALARM:		//for alarm mode
		if(debounced_state[1] == 1){	//toggle which mode we are in to display on the 7 seg
		    mode_display ^= 1; 
		}
		else if(debounced_state[6] == 1){	//add an hour button
		    if(alarm_time[HOUR]<23)
			alarm_time[HOUR]++;
		    else
			alarm_time[HOUR] = 0;
		    Sync_Snooze();			//this sets the snooze time to be matched with the new alarm time
		    change_alarm_display();	//update 7 seg display
		}

		else if(debounced_state[7] == 1){	//add a minute button
		    if(alarm_time[MINUTE]<59)
			alarm_time[MINUTE]++;
		    else
			alarm_time[MINUTE] = 0;
		    Sync_Snooze();	//sets the snooze time to be matched with the new alarm time
		    change_alarm_display();	//update 7 seg display
		}
		break;
	    case RADIO:
		if(debounced_state[3] == 1){
		    change_radio_display();
		    mode_display = NORMAL;
		}
		else if(debounced_state[4] == 1){
		    if(Radio_On == FALSE){
			OCR3A=0x5000; 
			Change_Radio();
			Radio_On = TRUE;
		    }
		    else{
			radio_pwr_dwn();
			Radio_On = FALSE;
		    }
		}
		break;
	}
    }
}

//*************************************
//init_variables
//initializes the variables that are global so that they are set to zero initially
//prevents any odd values or states from loading when booted
//*************************************
void init_variables(){
    alarm_armed = FALSE;
    mode_display = NORMAL;
    timer0_counter=0;

    normal_time[SECOND] = 0;
    normal_time[MINUTE] = 0;
    normal_time[HOUR] = 0;
    normal_time_display[2] = 0b11111111;
    change_normal_display();

    alarm_time[SECOND] = 0;
    alarm_time[MINUTE] = 0;
    alarm_time[HOUR] = 0;
    alarm_time_display[2] = 0b11111100;
    change_alarm_display();

    snooze_time[SECOND] = alarm_time[SECOND];
    snooze_time[MINUTE] = alarm_time[MINUTE];
    snooze_time[HOUR] = alarm_time[HOUR];

    alarm_count=0;
    TCCR1A = 0;
    OCR1A = 0x0160;
    TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11);
    TCCR1C = 0x00;
    memset(lcd_str_array, '\0', sizeof(lcd_str_array));

    Radio_On = FALSE;
}
//*************************************
//spi_init
//initializes the SPI for use with the LCD display on the Atmega board
//*************************************
void spi_init(void){
    /* Run this code before attempting to write to the LCD.*/
    DDRF  |= 0x08;  //port F bit 3 is enable for LCD
    PORTF &= 0xF7;  //port F bit 3 is initially low

    DDRB  |= 0x07;  //Turn on SS, MOSI, SCLK
    PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
    //see: /$install_path/avr/include/avr/iom128.h for bit definitions   

    DDRE |= 0b11000000; //sets PORTE to ouput on PIN7 and PIN6 for the decoder on the encoder board
    //Master mode, Clock=clk/4, Cycle half phase, Low polarity, MSB first
    SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
    SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)  
}
void Init_Radio(){
    //hardware reset of Si4734
    PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
    DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
    PORTE |=  (1<<PE2); //hardware reset Si4734 
    _delay_us(200);     //hold for 200us, 100us by spec         
    PORTE &= ~(1<<PE2); //release reset 
    _delay_us(30);      //5us required because of my slow I2C translators I suspect
    //Si code in "low" has 30us delay...no explaination
    DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

    EIMSK |= (1<<INT7);
    EICRB |=(1<<ISC70);
}



//***********************************************************************************
//*************************************
//Main
//initialized values, then runs a loop that just takes inputs from the pushbuttons and displays the 7 segment display
//*************************************
int main()
{

    lm73_temp = 0;  //a place to assemble the temperature from the lm73
    dec_to_7seg_init();
    init_variables();

    tcnt0_init();	//intialize timer/counter 0
    tcnt2_init();	//intialize timer/counter 0
    TCNT3_Init();	//intialize timer/counter 0

    DDRD |= 0b00001000;
    DDRB |= 0xF0;  
    uart_init();
    spi_init();
    lcd_init();
    init_twi(); //initalize TWI (twi_master.h)  
    clear_display();
    DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
    PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off

    ADMUX = 0b01000111; //single-ended, input PORTF bit 7, right adjusted, 10 bits

    ADCSRA = 0b10000111; //ADC enabled, don't start yet, single shot mode
    sei();		//set global interrupts
    set_cursor(2,0);
    string2lcd(" L:"); //send the string to LCD (lcd_functions)
    set_cursor(2,6);
    string2lcd(" R:"); //send the string to LCD (lcd_functions)

    DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
    DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume
    PORTE |= 0x04; //radio reset is on at powerup (active high)
    //  PORTE |= 0x40; //pulse low to load switch values, else its in shift mode
    PORTE |= 0x08;

    while(1){

	DDRA=0x00; //make PORTA an input port
	PORTA=0xFF; //make PORTA have pullups

	PORTB=0b01110000; //enable tristate buffer for pushbutton switches

	_delay_ms(0.1); //insert delay for buffer
	Get_Input();

	PORTB = 0x00; //disable tristate buffer for pushbutton switches

	DDRA=0xFF; //make PORTA an output

	Time_Display();

	if(rcv_rdy==1){
	    //set_cursor(2, 8);
	    string2lcd(lcd_str_array);
	    rcv_rdy=0;
	}
	encoder_logic();  //call encoder logic to check the encoders and set the value

    }//while

}//main
