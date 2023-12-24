#ifndef F_CPU
#define F_CPU 16000000UL  // Set 16 MHz clock speed
#endif

#include <avr/io.h> // This header defines all the Input/Output for all AVR microcontrollers
#include <util/delay.h> // This header defines two fuctions, _delay_ms (miliseconds) and _delay_us (microseconds)
#include<avr/interrupt.h>


/* RTC pin assignment and functionality */
// Manipulate the logic of SCLK and IO signal
#define SCLK 3
#define IO 4
#define CE 5

/* Define States */
enum STATE {idle, set_time, run_mode};
enum STATE state = idle; // idle: display 00:00
enum set_time_state {setHour, setMin};
enum set_time_state changeTime = setHour;


volatile int digit[4],                // 4 digits of 7-segment LED
             hour = 0,                
             minute = 0,              
             sec = 0;                 

volatile bool timer_flag = false;     // We use this flag to generate clock signal for DS1302
volatile bool idle_mode = false;   // Check if program is in Idle mode (got reset or boot up)

// BUTTON STATES
volatile int Digit_state = 0;
volatile int Increase_state = 0;
volatile int Run_state = 0;

volatile bool ModePressed = false;

volatile int number[] = {
  0x80, // 0
  0xF1, // 1
  0x48, // 2
  0x60, // 3
  0x31, // 4
  0x22, // 5
  0x02, // 6
  0xF0, // 7
  0x00, // 8
  0x20  // 9
};

/* INSTRUCTIONS
*     SEGMENTS
*       A-G: D0 D1 D3 D4 D5 D6 D7
*
*     DIGITS
*       D1-D4: B0 B1 B2 B3
*
*     BUTTONS
*       STATE TRANSITION (ISR) D2
*       INCREASE HOUR: C0
*       INCREASE MINUTE: C1
*       RUN CLOCK: C2
*
*     DS1302
*       SCLK (CLK): C3
*       I/O (DATA): C4
*       RESET (RST): C5
*
*     LED SIGNS
*       LED SET MODE: B4 (GREEN LED)
*       LED RUN MODE: B5 (RED LED) 
*/


void setup(){
  // Segments output LED
  DDRD |= (1<<DDD0)|(1<<DDD1)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7);
  
  // Output for 4 digits (B0 -> B3)
  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3);

  /* IO assignment for PORTC*/
  DDRD &= ~(1 << DDD2); // Button to set time
  DDRC &= ~(1 << DDC0); // Button to change digit
  DDRC &= ~(1 << DDC1); // Button to increase time 
  DDRC &= ~(1 << DDC2); // Button to run clock

  /* DS1302 PIN ASSIGNMENTS*/
  DDRC |= (1 << DDC3) | (1 << DDC4) | (1 << DDC5); // C3:SCLK, C4: I/O, C5:CE
  
  /* LED SIGNALS*/
  // PORTB4 (Green Light): run mode 
  // PORTB5 (Red Light): set time
  DDRB |= (1 << DDB4) | (1 << DDB5);  
  
}
int main(void)
{
  setup();

  // Clear external interrupt flag
  EIFR |= (1 << INTF0);
  // Enable external interrupt INT0 (PORTD2) and trigger sensing on falling edge
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC01);


  // Enable CTC Timer Interrupt
  TCCR1B |= (1 << WGM12) | (1 << CS10); // Set timer 1 to CTC mode with prescaler of 1
  TIMSK1 |= (1 << OCIE1A); // Turn on Output Compare A Match Interrupt Enable (Mask Register)
  OCR1A = 39; // Set top value so that f = 200kHZ => T/2 = 2.5 us

  sei(); // Turn on the Global Interrupt Enable Bit
  while (1)
  {
    // Determine state instruction
    switch (state)
    {
      case idle:
        {
          write_time();               // Write intial time (00:00:00) to the timer module
          idle_mode = true;        // Enable display letter mode
          PORTC |= (1 << PORTC2);
          PORTB &= ~(1 << PORTB4);    //  Turn off red line - set time
          PORTB &= ~(1 << PORTB5);    // Turn off blue light - run time

          // STATE TRANSITION
          if (ModePressed){
            ModePressed = false;
            state = set_time;
            _delay_ms(5);
          }
          break;
        }
      case set_time:
        {
          read_time();                // Read time from the module
          sec = 0;                    // Reset second to 0 since we do not have any option to set second
          idle_mode = false;       // Enable display digit mode
          
          switch (changeTime)
          {
            case (setHour):
            {
              if (!(PINC & (1 << PINC0)) && Digit_state == 0) // Check if the PORTC0 is low (button pressed)
              {
                Digit_state = 1;
                changeTime = setMin;
                _delay_ms(20);
              }
              if ((PINC & (1 << PINC0)) && Digit_state == 1) {Digit_state = 0;}

              // INCREASE HOUR
              if (!(PINC & (1 << PINC1))){
                if (hour == 24){
                  hour = 0;
                } else {
                  hour++;
                  _delay_ms(20);
                }
              }
              break;
            }

            case (setMin):
            {
              if (!(PINC & (1 << PINC0)) && Digit_state == 0) // Check if the PORTC0 is low (button pressed)
              {
                Digit_state = 1;
                changeTime = setHour;
                _delay_ms(20);
              }
              if ((PINC & (1 << PINC0)) && Digit_state == 1) {Digit_state = 0;}

              // INCREASE HOUR
              if (!(PINC & (1 << PINC1))){
                if (minute == 60){
                  minute = 0;
                  
                } else {
                  minute++;
                  _delay_ms(25);
                }
              }
              break;
            }

            default:
              changeTime = setHour;
              break;
          }
          
          write_time();               // Update the set time to the timer module

          PORTB &= ~(1 << PORTB4);     // Turn off green light - run time
          PORTB |= (1 << PORTB5);    // Turn on green light - set time

          // STATE TRANSITION to RUN MODE
          if (!(PINC & (1 << PINC2)) && Run_state == 0){  // BUTTON C2 PRESSED
            Run_state = 1;
            state = run_mode;
            _delay_ms(20);
          }
          if ((PINC & (1 << PINC2)) && Run_state == 1) {Run_state = 0;}
          break;
        }
        
      case run_mode:
        {
          idle_mode = false;
          PORTB |= (1 << PORTB4); // Turn off red line - run time
          PORTB &= ~(1 << PORTB5);  // Turn on blue light - set time
          read_time();             // Read time stored in the module

          // STATE TRANSITION
          if (ModePressed){
            ModePressed = false;
            state = set_time;
            _delay_ms(20);
          }
          break;
        }

      default:
        {
          idle_mode = true;     // Enable display letter mode
          PORTC |= (1 << PORTC2);
          PORTB &= ~(1 << PORTB4); // Turn off red line - set time
          PORTB &= ~(1 << PORTB5); // Turn off blue light - run time
          break;
        }
    }
    
    Digit(); // Set each digit value
    display_LED();   // Display LEDs

  }
}

ISR(TIMER1_COMPA_vect)  // ISR for Compare Match A interrupt of Timer 1
{
  timer_flag = true; // Raise every 2.5us
}

// ISR for external interrupt of INT0 (PORTD2)
ISR(INT0_vect) 
{
  ModePressed = true;
}


// Function sweeps each digit of the 4 digits LED at 250 Hz
void display_LED()
{
  for (int i = 0; i < 4; i++)
{
    PORTB &= ~((1<<0) | (1<<1) | (1<<2) | (1<<3));

    if (idle_mode == true) { // WRITE 00:00 IN IDLE MODE
      PORTB |= (1<<0) | (1<<1) | (1<<2) | (1<<3);
      PORTD = number[0];
    }
    else {
      PORTD = number[digit[i]]; // Set value for the LED
    }
    PORTB |= (1 << i); // Turn on each digit turn by turn
    _delay_ms(1); 
  }
}


// Function determine the value of each digit by calculating its weight
void Digit(){
  digit[0] = hour / 10;
  digit[1] = hour % 10;
  digit[2] = minute / 10;
  digit[3] = minute % 10;
}


// Function start transfering data
void beginTransfer()
{
  PORTC &= ~(1<<CE); // CE ON
  PORTC &= ~(1<<SCLK); // SCLK LOW when CE ON
  PORTC |= (1<<CE); // CE ON
}


// Function endr transfering data
void endTransfer()
{
  PORTC &= ~(1<<SCLK); // Turn off SCLK
  PORTC &= ~(1<<CE); // CE OFF
}


// Function generates the SCLK communicating with ds1302
void GenSclk()
{
  PORTC |= (1<<SCLK); // Rising edge
  TCNT1 = 0; 
  if (timer_flag)
  {
    timer_flag == false;  
  }
  PORTC &= ~(1<<SCLK); // Falling edge
  TCNT1 = 0; 
  if (timer_flag)
  {
    timer_flag == false;
  }
} 

// Send the given command (address) to the RTC
void setAddress(unsigned char address)
{
  // set I/O to output mode
  DDRC |=(1<<IO);

  // loop through each bit of the address -  begin with A0
  for (int i = 0 ; i < 8; i++)
  {
    // check if each bit of address is 1 - 0x80 = 0b1000_0000 & 0b0000 0001 = 0b0000 0000 -> send 0 else send 1
    if (address & 0x01)
    {
      PORTC |= (1<<IO);
    } else {
      PORTC &= ~(1<<IO);
    }
    address >>= 1; // move to the next bit - 0b1000_0000 -> 0b0100_0000

    // Generate the serial clock for communication
    GenSclk();
    if (i < 7) // the first data bit to be transmitted occurs on the first falling edge after the last bit of the cmd byte is written - now we wait until the MCU is ready to receive data
    { // Apply for read data - read_rtc()
      PORTC &= ~(1<<3); // Falling edge
      TCNT1 = 0;
      while (timer_flag == false) {}
      timer_flag = 0;
    }
  }
}


// Write data to the given address
void write_rtc(unsigned char address, unsigned char data)
{
  beginTransfer(); // Begin the read/write process
  setAddress(address); // send the address of the desired register
  DDRC |= (1<<IO); // set I/O to output mode
  
  // falling edge of the last bit of the cmd byte (Please refer to comments in send_addr())
  PORTC &= ~(1<<SCLK); 
  
  // send data to the rtc at the given address
  // loop through each bit of the address -  begin with D0
  for (int i = 0; i < 8; i++)
  {
    if (data & 0x01) // check if each bit of address is 1 - 0x80 = 0b1000_0000 & 0b0000 0001 = 0b0000 0000 -> send 0 else send 1
    {
      PORTC |= (1<<IO);
    } else {
      PORTC &= ~(1<<IO);
    }

    data >>= 1; // move to the next bit - 0b1000_0000 -> 0b0100_0000
    // Generate the serial clock for communication
    GenSclk();
  }

  endTransfer(); // Stop read/write process
}


// Read data from the given address
unsigned char read_rtc(unsigned char address)
{
  // uint8_t data = 0;
  unsigned char data = 0; // char = 1 byte = 8bits = 0b0000_0000

  beginTransfer(); // Begin the read/write process
  setAddress(address); // send the address of the desired register

  DDRC &= ~(1<<IO); // set I/O to input mode
  PORTC &= ~(1<<SCLK); // falling edge of the last bit of the cmd byte - now ready to receive data (Please refer to comments in send_addr())

  // Receive the data sequence (begin with D0) from rtc 
  for (int i = 0; i < 8; i++)
  {
    data >>= 1; // move to the next bit - 0b0000_0000 -> 0b0000_0000 (first loop) -> 0b1000_0000 -> 0b0100_0000 (second loop)
    if ( (PINC & (1<<PINC4)) != 0) {
      data |= 0x80; // check if each bit of address is not 0 - data = data | 0x80 -> data = 0b0000_0000 | 0b1000_0000 = 0b1000 0000
    }
    // Generate the serial clock for communication
    GenSclk();
  }

  // set I/O to output mode
  DDRC |= (1<<IO);

  // Disable active mode
  endTransfer(); // Stop read/write process
  return data;
}


// Function to convert the decimal to BCD - if dec = 21 the BCD = 0b0010_0001 (0010 = 2 and 0001 = 1)
unsigned char dec2bcd(unsigned char value)
{
  unsigned char Data10 = 0; // Tenth Unit
  unsigned char Data1 = 0;
  unsigned char result = 0;

  Data10 = value / 10; // 21/10 = 2 = 0b0000_0010
  Data1 = value % 10; // 21%10 = 1 = 0b0000_0001
   // Data10 << 4 = 0b0000_0010 << 4 = 0b0010_0000 
   // -> (Data10 << 4) | Data1 = 0b0010_0000 | 0b0000_0001 = 0b0010_0001 (BCD format)
  result = (Data10 << 4) | Data1;
  return result;
}


// Function to convert the data from BCD to decimal - Ex: data = 0b0010_0001 = 0x21 = 0d33
unsigned char bcd2dec(unsigned char value)
{
  unsigned char temp = 0; 
  temp = value; // store the original value of data - Ex: temp = data = 0b0010_0001
  value = (temp >> 4) * 10; // temp >> 4 = 0b0000_0010 = 2 ->  (temp >> 4) * 10 = (0b0000_0010) * 10 = 2 * 10 = 0d20
  temp &= 0x0F; // 0b0010_0001 &= 0x0F -> 0b0010_0001 &= 0b0000_1111 -> temp = 0b0000_0001 = 0d1
  value = value + temp; // data = 20 + 1 = 21

  return value;
}


// Read time from the ds1302
void read_time()
{
  unsigned char r_sec = 0, r_minute = 0, r_hour = 0;
  r_sec = read_rtc(0x81);
  r_minute = read_rtc(0x83);
  r_hour = read_rtc(0x85);
  sec = bcd2dec(r_sec); // read sec
  minute = bcd2dec(r_minute); // read minute
  hour = bcd2dec(r_hour); // read hour
}

// Write time to the ds1302
void write_time()
{
  unsigned char w_sec = 0, w_minute = 0, w_hour = 0;
  w_sec = dec2bcd(sec);
  w_minute = dec2bcd(minute);
  w_hour = dec2bcd(hour);
  write_rtc(0x80, w_sec); // write second in BCD fortmat to the second register of DS1302
  write_rtc(0x82, w_minute); // write minute in BCD format to the minute register of DS1302
  write_rtc(0x84, w_hour);// write hour in BCD fomrat to the hour register of DS1302
}
