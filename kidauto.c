/*******************************************************
Project : KidAuto motor controller
Chip type               : ATmega8A
AVR Core Clock frequency: 1,000000 MHz
*******************************************************/

#define F_CPU 1000000
#include <mega8.h>
#include <delay.h>


#define TM1637_setData1(void) PORTD |= (1<<7) // PORTD.7=1
#define TM1637_setData0(void) PORTD &= ~(1<<7) // PORTD.7=0
#define TM1637_setClk1(void) PORTB |= (1<<0) // PORTB.0=1
#define TM1637_setClk0(void) PORTB &= ~(1<<0) // PORTB.0=0

#define TM1637_ADDR_AUTO 0b01000000 //0x40
#define TM1637_ADDR_FIXED 0b01000100 //0x44
#define TM1637_STARTADDR 0b11000000 //0xC0
const unsigned char digitHEX[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00, 0x40 }; // 0,1,2,3,4,5,6,7,8,9, ,-


#define setAUXenable() PORTC |= (1<<1) //PORTC.1=1
#define setAUXdisable() PORTC &= ~(1<<1) //PORTC.1=0

#define setRELAYenable() PORTC |= (1<<5) //PORTC.5=1
#define setRELAYdisable() PORTC &= ~(1<<5) //PORTC.5=0

#define MIN_PWM_START 20 // 0..255 min of OCR1BL value allowed

const unsigned char thPwm[10] = { 30,   50,   70,   90,   110,   130,   150,   170,  190,  210 };
                          //  25    50    75    100   125    150    175    200   225   250  // should be OCR1BL value
                          //  9.8   19,6  29,4  39,2  49.0   58,8   68,6   78,4  88,2  98,0 // shold be PWM in %

unsigned char adc3_curr = 0; // Batt ADC
unsigned char adc3_last = 0; // Batt ADC
unsigned int adc3_tmp = 0; // adc3 tmp     
unsigned char adc3_cnt = 0; // adc3 tmp
    
unsigned char adc6_curr = 0; // Throttle ADC new value for Hystresis func
unsigned char adc6_last = 0; // Throttle ADC last value for Hystresis func

unsigned char thrott = 0; // Throttle %
//unsigned char rev_sw_on = 0; // Reverse switch ON/OFF
//unsigned char rev_lock_flag = 0; // Reverse mode LockVar
unsigned char revOn_probes = 0; // Reverse switch good ON brobes count
unsigned char last_Dwd_cnt = 0; // last forward usage counter
unsigned char last_Rwd_cnt = 0; // last backward usage counter

//unsigned char charge_ena_flag = 0; // Batt charger power connected
//unsigned char low_batt_flag = 0; // Batt low power flag


#define BATTCHAENA 0 // bit0 - Batt charger power connected flag (1 - connected, 0 - disconnected)
#define BATTLOW 1 // bit1 - Batt low power flag (1 - Batt LOW, 0 - Batt OK) 
#define THROTTLOCK 2 // bit2 - Motor PWM immediately shutdown flag (1 - Disable PWM, 0 - Normal)
#define REVON 3 // bit3 - Reverse switch turned on (1 - Reverse, 0 - Normal)
#define REVDENY 4 // bit4 - Reverse movement prohibited. Reasons - forward movement is not yet finish, etc. (1 - Prohibit, 0 - can already be used)
unsigned char flags1 = 0b00000000;  


// Rewrite this code using a PWM controller
void beep1() {
   int i; 

    for(i=0; i <= 200; i++) { 
      PORTD.6 = 1; delay_us(25); 
      PORTD.6 = 0; delay_us(375); 
    } 
}


// Rewrite this code using a PWM controller
void beep2() {
   int i; 

    for(i=0; i <= 200; i++) { 
      PORTD.6 = 1; delay_us(25); 
      PORTD.6 = 0; delay_us(375); 
    } 
    delay_ms(100);
    for(i=0; i <= 200; i++) { 
      PORTD.6 = 1; delay_us(25); 
      PORTD.6 = 0; delay_us(375); 
    } 
}


void tm1637_start(void)  { // data transfer start
    //TM1637_setData1();
    //TM1637_setClk1();
    //delay_us(2);
    
	TM1637_setData0(); //PORTD.7 = 0;    // data to ground - start of i2c transmission
	delay_us(2);    // delay for the slave device to have time to react (optional)
	TM1637_setClk0(); //PORTB.0 = 0;    // clock to ground. Technically the data transfer function already starts with this, but this saves memory.
}


void tm1637_stop(void) {  // data transfer end
	TM1637_setData0(); //PORTD.7 = 0;    // data to ground (clock is on ground from previous function)
	delay_us(2);    // delay, which determines the speed of data transmission here and hereafter (by DS - up to 600 kHz, here is 500 kHz)
    
	TM1637_setClk1(); //PORTB.0 = 1;    // clock is pulled to ground
	delay_us(2);
	TM1637_setData1(); //PORTD.7 = 1;    // crossing from low to high indicates the end of transfer (by i2c standart)
}


void tm1637_write(unsigned char databyte) { // writing a byte to a chip
	unsigned char i;
	for (i=0; i<8; i++) {  // 8 times (write to the port bit by bit. from least to most significant bit of a byte.)
		if ( databyte & 0b00000001 ) {
			TM1637_setData1(); //PORTD.7 = 1;   // if the low bit is a one, the data leg must be pulled up ...
		} else {
			TM1637_setData0(); //PORTD.7 = 0;   // else - pulled down.
		}
		databyte = (databyte>>1); // shift bits to the right by one
		TM1637_setClk1(); //PORTB.0 = 1;    // clock is pulled up
		delay_us(2);    // increase clock pause
		TM1637_setClk0(); //PORTB.0 = 0;    // setting the "clock" to the ground where it was earlier
	}
	TM1637_setData0(); //PORTD.7 = 0;  // the data leg to the ground to avoid burning the port during ACK
	TM1637_setClk1(); //PORTB.0 = 1;  // clock to high
	delay_us(2);  // set ACK to low at the end of byte transfer (spec from DS)
	TM1637_setClk0(); //PORTB.0 = 0;  // clock to low
}



// Function for selecting and polling ADC pin 0,1,2.... one-time, by channel number
unsigned char AdcPortRead(unsigned char muxuse) {
    //unsigned char adcval;

    ADMUX = muxuse;         // use ADC MUX port num

    ADMUX |= (1 << ADLAR); // The 8 most significant bits will be stored in ADCH and thge 8 least significant bits will be stored in ADCL. It's useful to get the result from ADCL as for an 8-bit DAC.
	
    ADMUX |= (1 << REFS0); // use AVcc as the reference (P.S. the AREF pin is on Vcc bus +5v)
    ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0); // 128 freq prescale 
    ADCSRA |= (1 << ADEN); // Enable the ADC
	delay_us(10); // The dalay that is required for stabilizing the ADC input voltage
    
    ADCSRA |= (1 << ADSC);    // Start the ADC conversion   
    //ADCSRA &= ~(1 << ADLAR); // Stops ADC

    while( ADCSRA & (1<<ADSC) );      // waits for the ADC to finish 

    //adcval = ADCL;
    //adcval = (ADCH << 8) + adcval; // ADCH is read so ADC can be updated again

    return ADCH; // 8-bit
}

void StartPwm(void) {
    TCCR1B |= (1 << WGM12); // Fast PWM 8bit, TOP 0x00FF (Waveform Generator Mode)
    TCCR1A |= (1 << WGM10); // Fast PWM 8bit, TOP 0x00FF (Waveform Generator Mode)
    TCCR1A |= (1 << COM1B1); // Enable Fast PWM (non-inverting mode) on port OC1B only (OC1A stay disconnected COM1A1=0 COM1A0=0 by default init)
    TCCR1B |= (1 << CS11); // set prescaler to 8 and starts PWM (485 Hz)
    OCR1B = 0x0000; // 16bit; set PWM level on port OC1B to 0%
}

void StopPwm(void) {
    TCCR1A = 0x00; // reset Timer/Counter Control Register 1 A
    TCCR1B = 0x00; // reset Timer/Counter Control Register 1 B
    TCNT1 = 0x0000; // reset Timer/Counter Register (stores the counter value, 16 bit)
    OCR1A = 0x0000; // reset Output Compare Register A (stores the compare value, 16 bit)
    OCR1B = 0x0000; // reset Output Compare Register B (stores the compare value, 16 bit)                                                   
    PORTB.2 = 0; // port out signal OFF - 0
}


void DispPict(unsigned char cmd) { // 8..1, 0 decimal
	unsigned char dbuf[4] = {0}; // all 4 digits are 0
	//unsigned char i = 0;
    if (cmd > 9) { // 10 and over
		dbuf[0] = 0b0; dbuf[1] = 0b0; dbuf[2] = 0b0; dbuf[3] = 0b0; // all 4 dig a NULL
    } else if (cmd == 9) {
		dbuf[0] = 0b00111001; dbuf[1] = 0b00001001; dbuf[2] = 0b00001001; dbuf[3] = 0b00111111; // charge pic
	} else if (cmd == 8) {
		dbuf[0] = 0b00001110; dbuf[1] = 0b00001110; dbuf[2] = 0b00001110; dbuf[3] = 0b00001110; // _|_|_|_|
	} else if (cmd == 7) {
        dbuf[0] = 0b00001110; dbuf[1] = 0b00001110; dbuf[2] = 0b00001110; dbuf[3] = 0b00001100; // _|_|_|_i
	} else if (cmd == 6) {
		dbuf[0] = 0b00001110; dbuf[1] = 0b00001110; dbuf[2] = 0b00001110; dbuf[3] = 0b00001000; // _|_|_|_
	} else if (cmd == 5) {
		dbuf[0] = 0b00001110; dbuf[1] = 0b00001110; dbuf[2] = 0b00001100; dbuf[3] = 0b00001000; // _|_|_i_
	} else if (cmd == 4) {
		dbuf[0] = 0b00001110; dbuf[1] = 0b00001110; dbuf[2] = 0b00001000; dbuf[3] = 0b00001000; // _|_|_ _
	} else if (cmd == 3) {
		dbuf[0] = 0b00001110; dbuf[1] = 0b00001100; dbuf[2] = 0b00001000; dbuf[3] = 0b00001000; // _|_i_ _
	} else if (cmd == 2) {
		dbuf[0] = 0b00001110; dbuf[1] = 0b00001000; dbuf[2] = 0b00001000; dbuf[3] = 0b00001000; // _|_ _ _
	} else if (cmd == 1) {
		dbuf[0] = 0b00001100; dbuf[1] = 0b00001000; dbuf[2] = 0b00001000; dbuf[3] = 0b00001000; // _i_ _ _
	} else { // 0
		dbuf[0] = 0x7D; dbuf[1] = 0x77; dbuf[2] = 0x78; dbuf[3] = 0x38; // ÁAtL
	}
	
	tm1637_start();
	 tm1637_write(0b01000000); // The ADDR_AUTO (0x40) command writes data to the display register. If so, ADDR_FIXED (0x44)
	tm1637_stop();
	
	tm1637_start(); 
     tm1637_write(0b11000000); // STARTADDR (0xC0) command to write data to the display register
     tm1637_write(dbuf[0]);
     tm1637_write(dbuf[1]);
     tm1637_write(dbuf[2]);
     tm1637_write(dbuf[3]);
	tm1637_stop();
    
    //tm1637_start();
	// tm1637_write(0b10001000); // display brightness. 0b10001000 to 0b10001111
    //tm1637_stop();
}

void DispUNum(unsigned char num) { // 8..1, 0 decimal
    unsigned char dig1=0, dig2=0, dig3=0;
    
    while (num > 99) {
        num -= 100;
        dig1++;
    }
    
    while (num > 9) {
        num -= 10;
        dig2++;
    }
	       
    dig3 = num;
    
	tm1637_start();
	 tm1637_write(0b01000000); // The ADDR_AUTO (0x40) command writes data to the display register. If so, ADDR_FIXED (0x44)
	tm1637_stop();
	
	tm1637_start(); 
     tm1637_write(0b11000000); // STARTADDR (0xC0) command to write data to the display register
     tm1637_write(digitHEX[10]);
     tm1637_write(digitHEX[dig1]);
     tm1637_write(digitHEX[dig2]);
     tm1637_write(digitHEX[dig3]);
	tm1637_stop();
    
    //tm1637_start();
	// tm1637_write(0b10001000); //display brightness. 0b10001000 to 0b10001111
    //tm1637_stop();
}


void CalcShowBatt(unsigned char adc_lev) {

    if (adc_lev > 209) DispPict(8); 
    else if (adc_lev > 204) DispPict(7);
    else if (adc_lev > 199) DispPict(6);
    else if (adc_lev > 194) DispPict(5);
    else if (adc_lev > 189) DispPict(4);
    else if (adc_lev > 184) DispPict(3);
    else if (adc_lev > 179) DispPict(2);
    else if (adc_lev > 174) DispPict(1);
    else {
        flags1 |= (1<<BATTLOW); // Battery charge is low! Put one in the bit. The flag is reset only when the MC is rebooted (ignition off/on) or by the event "Charging cord disconnected".
        // low batt power
        DispPict(0);
    } 
}



// Every second timer
interrupt [TIM0_OVF] void timer0(void) {
	unsigned char idx = 0;
 
	TCNT0 += 230; // boost up counter (experimental)
	//PORTB.1 = !PORTB.1; // Sys led blink      


	// Global counters for the last actual use of the Forward and Backward motion
	if ( last_Dwd_cnt > 0 ) last_Dwd_cnt--; 
	if ( last_Rwd_cnt > 0 ) last_Rwd_cnt--;



    //-- Checking the charger cord connection/disconnection event
	if ( flags1 & (1<<BATTCHAENA) ) { // The current state of the flag "Charge is connected". Set 1 to the bit
		if (PINC.4 == 0) { 
        	// Event -> Charge power disconnected
			flags1 &= ~(1<<BATTCHAENA); // The charger cord is disconnected (previously connected). Set 0 to the bit
            flags1 &= ~(1<<BATTLOW); // We consider that the battery has become charged again (the low battery flag may have been set earlier). Set 0 to the bit
            //flags1 &= ~(1<<SHUTMOT); // Allow driving (forced motor shutdown flag). Set 0 to the bit
            setAUXenable(); // Enable AUX power
            beep2();
		}
    } else { // Current flag status - Charging is disabled
		if (PINC.4 == 1) { 
        	// Event -> Charge power connected
			flags1 |= (1<<BATTCHAENA); // The charger cord was plugged in (previously unplugged). Set 1 to the bit
            //flags1 |= (1<<SHUTMOT); // Disallow driving (forced motor shutdown flag). Set 1 to the bit 
            setAUXdisable(); // Disable AUX power
            DispPict(9); // show charge picture
            beep1();
		}
    }  


      
    // Battery charge calculation
    // Averaging ADC of the battery voltage (8 passes). delay_ms is not used, accumulate sum in accumulator adc3_tmp
    if ( adc3_cnt < 8 ) {
        // ADC averaging counter has not yet overflowed
        adc3_curr = AdcPortRead(3); // Batt ADC
        adc3_tmp += adc3_curr; 
        adc3_cnt ++;
        //delay_ms(1);
    } else {    
        // the ADC averaging counter has just overflowed. Calculate the new value
        adc3_curr = (adc3_tmp>>3); // shiftR 3 equal divide x/8
        adc3_cnt = 0;
        adc3_tmp = 0;
        
        // Battery charge ADC hysteresis
        if ( adc3_curr > (adc3_last+1) ) { // up value event
            adc3_last = adc3_curr;
            //DispUNum(adc3_last);    
            if ( ~flags1 & (1<<BATTCHAENA) ) CalcShowBatt(adc3_curr); // if BATTCHAENA==0
        } else if ( adc3_curr < (adc3_last-1) ) { // down value event
            adc3_last = adc3_curr;
            //DispUNum(adc3_last);
            if ( ~flags1 & (1<<BATTCHAENA) ) CalcShowBatt(adc3_curr); // if BATTCHAENA==0
        }        
    }
    
     
	// Accelerator pedal signal voltage processing
    // (5v/255) * 3 = 0,0588235 volt/1adc real
    // SS16 diode real forward voltage = -0.3V                      
    // full batt: (12.6 - 0.3) =12.3;  12.3/ 0,0588235 =  210 ADC val
    // low batt: (10.5 - 0.3) =10.2;  10.2/ 0,0588235 = 173 ADC val
    
    adc6_curr = AdcPortRead(6); // Accelerator pedal ADC
    // Accelerator pedal hysteresis
    if ( adc6_curr > (adc6_last+19) ) { 
    	// event -> value UP
        adc6_last = adc6_curr;
        //DispUNum(adc6_last);
    } else if ( adc6_curr < adc6_last ) {
    	// event -> value DOWN
        adc6_last = adc6_curr;
        //DispUNum(adc6_last);
    }       


     
    // Search the table of PWM preset values (0..255) by the current pedal ADC value (0..255)
    // Characterization table thPwm[]
    idx = 0;
    while (idx < 10) {
        if (adc6_last < thPwm[idx]) {
        	break;
        }
        idx++; // 1..10 - the trick is that increment is triggered only after entering the array and becomes greater than zero. Therefore idx is really from 0 to 10
    }                                                                                          
    //thrott = idx*25; went down so no one would read it before the cycle.
    
    if ( flags1 & (1<<THROTTLOCK) ) { // if the drive lock flag has been set previously
    	if (idx == 0) { // and the pedal is now released
        	// evnt -> Pedal released
			flags1 &= ~(1<<THROTTLOCK); // to release the drive lock. Bit to zero
        }
    }
    //DispUNum(idx);        	

      
    
	// Processing of the reverse control shifter position (on/off)
    if ( PIND.5 == 0 ) { // Now the reverse control is off
    	if ( flags1 & (1<<REVON) ) { // Previously the reverse shifter was on, so now the event is now off
        	// event -> OFF
        	revOn_probes = 0;
        	flags1 &= ~(1<<REVON); // Reverse is off - reverse control rocker in normal position. Set 0 in bit. rev_sw_on = 0;
            if ( last_Dwd_cnt > 0 ) { flags1 |= (1<<THROTTLOCK); }// if drove - temporarily prohibit driving. set the bit to 1
			if ( last_Rwd_cnt > 0 ) { flags1 |= (1<<THROTTLOCK); }// if drove - temporarily prohibit driving. set the bit to 1
			beep2();
        } 
    } else { // Now the reverse control is on
    	if ( ~flags1 & (1<<REVON) ) { // Previously the reverse shifter was off, so now the event is now on
        	// pre event -> on
        	if (revOn_probes < 20) { // contact bounce compensation
				revOn_probes++;
			} else { // over 20 good probes - 100% is on
            	// event -> ON
                flags1 |= (1<<REVON); // Reverse is on - the reverse control rocker is engaged. Set 1 to bit. rev_sw_on = 1;
            	if ( last_Dwd_cnt > 0 ) { flags1 |= (1<<THROTTLOCK); }// if drove - temporarily prohibit driving. set the bit to 1
				if ( last_Rwd_cnt > 0 ) { flags1 |= (1<<THROTTLOCK); }// if drove - temporarily prohibit driving. set the bit to 1
                beep1();
			}
        }
    }
                        
    
    // Blocking the ride at certain flags    
    if ( flags1 & (1<<BATTCHAENA) ) { // The current state of the flag is "Charge is connected". The bit must contain one
		thrott = 0;
        return;
    }
    if ( flags1 & (1<<BATTLOW) ) { // The current state of the flag is "Low Battery". The bit must contain one
		thrott = 0;
        return;
    }
    
    if ( flags1 & (1<<THROTTLOCK) ) { // Current flag status - Reverse control shifter was switched on the move - prohibit driving. The bit must contain one
		thrott = 0; 
        return;
    }
    
    thrott = idx*25;
}




void main(void) {
    // Declare your local variables here
    //unsigned char pwm_gen_state = 0; // generator current state ON/OFF
    unsigned char ovf_cnt1 = 0;  
    

    DDRB = 0b11111111; // all 8 pins of port B mode OUT
    PORTB = 0b00000000; // all 8 pins of port B to GND

    DDRC = 0b1111111; // all 7 pins of port C mode OUT
    PORTC = 0b0000000; // all 7 pins of port C to GND

    DDRD = 0b11111111; // all 8 pins of port D mode OUT
    PORTD = 0b00000000; // all 8 pins of port D to GND

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    TCCR0=(0<<CS02) | (0<<CS01) | (0<<CS00);
    TCNT0=0x00;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: Timer1 Stopped
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer2 Stopped
    // Mode: Normal top=0xFF
    // OC2 output: Disconnected
    ASSR=0<<AS2;
    TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
    TCNT2=0x00;
    OCR2=0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<TOIE0);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);

    // USART initialization
    // USART disabled
    UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

    // Analog Comparator initialization
    // Analog Comparator: Off
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
    SFIOR=(0<<ACME);

    // ADC initialization
    // ADC disabled
    ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

    // SPI initialization
    // SPI disabled
    SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

    // TWI initialization
    // TWI disabled
    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);


	// other    
    PORTC.5 = 0; // power off Fwd Relay (PC5 pin 28 mode OUT) 
    PORTC.1 = 1; // enable AUX power
    PORTD.6 = 0; // buzzer off 
    DDRD &= ~(1<<DDD5); // set PD5 pin 9 mode IN (non-inverting sig) - Fwd OFF/ON switch           
    DDRC &= ~(1<<DDC3); // set PC3 pin 26 mode IN - ADC3 for VBatt 1:3
    DDRC &= ~(1<<DDC4); // set PC4 pin 27 mode IN - Charge init ON status 

    
	// configure ADC
    //ADMUX |= (1 << ADLAR); // store the result in ADCH+ADCL
    //ADMUX |= (1 << REFS0); // use AVcc as the reference (pin AREF íà Vcc +5v)
    //ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0); // 128 freq prescale 
    //ADCSRA |= (1 << ADEN); // Enable the ADC

    
	// configure timer for "interrupt [TIM0_OVF] void timer0(void) { }" 
    TIMSK |= (1 << TOIE0); // Enable TIMER0
    //TCCR0 |= (1 << CS02); // set prescaler to 256 and start the timer
    TCCR0 |= (1 << CS02)|(1 << CS00); // set prescaler to 1024 and start the timer
    
	
	// Initialize LED display on TM1637
	tm1637_stop();
	tm1637_start();
	  tm1637_write(0b10001011); // display brightness. From 0b10001000 to 0b10001111
	tm1637_stop(); 
	DispUNum(000);
	delay_ms(500);

    beep1();

    #asm("sei"); // enable interrupts
    //#asm("cli"); // disable interrupts
                                         
    
    while (1) { 

        if ( ~flags1&(1<<REVON) && last_Rwd_cnt == 0 ) { // if (REVON==0 && REVDENY==0 && last_Rwd_cnt==0) - can drive FORWARD immediately
                  
                if ( OCR1BL < thrott && thrott > MIN_PWM_START ) { // accelerate
                    
                    if ( OCR1BL < MIN_PWM_START ) {
                        OCR1BL = MIN_PWM_START;
                    } else {
                        OCR1BL++;                      
                        delay_ms(5);
                    }
                } 
                    
                if ( OCR1BL > thrott ) { // decelerate
                    if ( OCR1BL < MIN_PWM_START ) {
                        OCR1BL = 0;
                    } else {
                        OCR1BL--;
                        delay_ms(5);
                    }
                }                  
                
                if ( OCR1BL > 0 ) {
                	last_Dwd_cnt = 40;
                }
        }
         
        
        if ( flags1&(1<<REVON) && last_Dwd_cnt == 0 ) { // if (REVON==1 && REVDENY==1 && last_Dwd_cnt==0) - can drive FORWARD immediately
                if ( OCR1BL < thrott && thrott > MIN_PWM_START ) { // acceleration
                    if ( OCR1BL < MIN_PWM_START ) {
                        OCR1BL = MIN_PWM_START; // rough start from MIN_PWM
                    } else {
                        OCR1BL++;
                        delay_ms(10); // smooth acceleration
                    }                 
                } 
                    
                if ( OCR1BL > thrott ) { // deceleration
                    if ( OCR1BL < MIN_PWM_START ) {
                        OCR1BL = 0;
                    } else {
                        OCR1BL--;
                        delay_ms(2); // smooth deceleration
                    }              
                }     
                
                if ( OCR1BL > 0 ) {
                	last_Rwd_cnt = 40;
                }
                          
        }
        
		if ( OCR1BL > 0 ) { // check the register of current PWM level
			if ( !(TCCR1A & (1<<COM1B1)) ) { // if PWM has already stoped
            	// event -> PWM on
                if ( flags1&(1<<REVON) ) {	// also the backward switch is ON - switching on the BACKWARD drive relay
					delay_ms(50);
					setRELAYenable(); //PORTC.5 = 1; // enable the BWD drive relay
					//PORTD.7 = 1; // enable RED Led
					delay_ms(50);
                }
				StartPwm();   
			}
		} else { // PWM has already been absent
			if ( TCCR1A & (1<<COM1B1) ) { // if PWM is already activated
            	// event -> PWM off
				StopPwm();    
				delay_ms(50);
				setRELAYdisable(); // force the BWD relay to be disabled
				flags1 &= ~(1<<REVDENY); //rev_lock_flag = 0; // unlock BWD mode
				delay_ms(50);
			}   
		}

    }

}

