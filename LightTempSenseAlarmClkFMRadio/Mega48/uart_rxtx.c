//C Gabor
//Uart functions for sending temp reading
//Connect two mega128 boards via rs232 and they should end to each
//other a message and a sequence number.
//
//Change the message you send to your partner for checkoff.
//
//You can test this code by a "loopback" if you connect rx to tx
//on the DB9 connector.
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "uart_functions_m48.h"
#include <util/delay.h>
#include "twi_master.h"
#include "lm73_functions.h"

extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
char temp_string[5];

void Local_Temp(){
    twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
    _delay_ms(2);
    lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
    lm73_temp = (lm73_temp<<8); //shift it into upper byte 
    lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
    itoa(lm73_temp>>7,temp_string, 10); //convert to string in array with itoa() from avr-libc                           
}    
/*
   ISR(USART_RX_vect){
   uart_puts(temp_string);
   uart_putc('\0');
   }
 */

int main(){
    init_twi(); //initalize TWI (twi_master.h)  
    uart_init();  
    memset(temp_string, '\0', sizeof(char) * 5);
    sei();
    DDRD |= 0b00000010;
    PORTB = 0x00;

    while(1){
	if(uart_getc() != '\0'){ 	//wait for a signal from the uart
	    Local_Temp();
	    uart_puts(temp_string);
	    uart_putc('\0');
	}
	//Local_Temp();
    }
}//main

