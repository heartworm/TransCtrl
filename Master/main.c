#define F_CPU 8000000UL

#define SOLENOID_MASK 0b00101101
#define SHIFT_DELAY 2000 //milliseconds
#define CLUTCH_ENGAGE_DELAY 2000 //milliseconds
#define CLUTCH_RELEASE_DELAY 2000 //milliseconds
#define CLUTCH_DUTY 0.5f //float
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR
  
/*
	Gear    SolenoidA   SolenoidB
	1       ON          OFF
	2       ON          ON
	3       OFF         ON
	4       OFF         OFF
*/


#define PIN_A 3
#define PIN_B 4
#define PIN_SLU 3


#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile bool blockingShift;
const float msPerTick = 256.0/8000.0;

enum Gear {
    GEAR_FIRST, GEAR_SECOND, GEAR_THIRD, GEAR_FOURTH
};

struct TransState {
    bool clutch;
    enum Gear gear;
};

struct TransState currentState = {false, GEAR_SECOND};
struct TransState desiredState = {false, GEAR_SECOND};

void initTimeout();
void initEnabledPin();
bool isEnabled();
void initUart();
float getTimeoutProgress();
void setTimeout(float ms, bool blockShift);
void onTimeout();
void initClutchPwm();
void initSolenoids();
void setClutchDuty(float duty);
void setSolenoids(enum Gear gear);
int main();
void trySendStatus();
void shift();

void initClutchPwm() {
    // TODO: set timer2 to waveform gen
    DDRB |= _BV(PIN_SLU);
    TCCR2A |= _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
    OCR2A = 0;
    TCCR2B |= _BV(CS22);
}

void initSolenoids() {
    PORTD |= _BV(PIN_A) | _BV(PIN_B);
}

void initTimeout() {
    TIMSK1 |= _BV(1) | _BV(0); //enable overflow and compare register 1 interrupt
}

void setTimeout(float ms, bool blockShift) {
    blockingShift = blockShift;
    OCR1A = ms / msPerTick;
    // OCR1A = 0xFFFF;
    TCNT1 = 0;
    TCCR1B |= _BV(CS12); //set prescale to clk/256, turn on timer
}

ISR(TIMER1_OVF_vect) {
    onTimeout();
}

ISR(TIMER1_COMPA_vect) {
    onTimeout();
}

void initUart() {
    UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
    UBRR0L = BAUDRATE;                           // set baud rate
    UCSR0B|= (1<<TXEN0)|(1<<RXEN0);                // enable receiver and transmitter
    UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}

void onTimeout() {
    TCCR1B = 0; //turn off timer1
    OCR1A = 0;
    blockingShift = false; //turn off blockingShift
}

void initEnabledPin() {
    DDRC &= ~(_BV(5));
}

bool isEnabled() {
    return 0x01 & (PINC >> 5);
}

void shift() {
    if (!isEnabled()) {
        desiredState.clutch = false;
    }

    if (!blockingShift) {
        if (currentState.gear != desiredState.gear) { // If we are changing gear
            if (currentState.clutch) { //and the clutch is engaged
              currentState.clutch = false; //first release the clutch 
              setTimeout(CLUTCH_RELEASE_DELAY, true); //and wait a while
            } else { //otherwise 
              currentState.gear = desiredState.gear; //change gear
              setTimeout(SHIFT_DELAY, true); //and block other operations
            }
        } else { //otherwise we're in the desired gear
          if (currentState.clutch != desiredState.clutch) { // if we're changing the clutch position
            currentState.clutch = desiredState.clutch; //then set it to the desired position
            if (desiredState.clutch) { 
              setTimeout(CLUTCH_ENGAGE_DELAY, false);
            } else {
              setTimeout(CLUTCH_RELEASE_DELAY, true);
            }
            // desiredState.clutch = !desiredState.clutch;
          }
        }
    }
    // currentState.clutch = false;
    if (currentState.clutch) {
      setClutchDuty(getTimeoutProgress() * CLUTCH_DUTY);
      // setClutchDuty(1.0f);
    } else {
      setClutchDuty(0.0f);
    }

    setSolenoids(currentState.gear);
}

void setSolenoids(enum Gear gear) {
    PORTD = (PORTD & ~(_BV(PIN_A) | _BV(PIN_B))) | ((0x03 & (SOLENOID_MASK >> (gear*2))) << PIN_A);
    DDRD = (DDRD & ~(_BV(PIN_A) | _BV(PIN_B))) | ((0x03 & (SOLENOID_MASK >> (gear*2))) << PIN_A);
}

void setClutchDuty(float duty) {
    //there's enough leakage current through the pin that if it's not put into high impedance mode, 
    //the mosfet turns partially on at 0 duty!
    if (duty == 0) {
      DDRB &= ~(_BV(PIN_SLU)); 
    } else {
      DDRB |= _BV(PIN_SLU);
    }
    OCR2A = (uint8_t) (duty * (float)0xFF);
}

float getTimeoutProgress() {
    if (TCCR1B == 0) { //if the timer has finished (isn't running)
        return 1.0;
    } else { // otherwise return a fractional value
        return (float)TCNT1 / (float)OCR1A;
    }
}

void trySendStatus() {
    if (UCSR0A & _BV(UDRE0)) {
      uint8_t packet = 0x00;
      packet |= 0x03 & (uint8_t) currentState.gear;
      packet |= 0x04 & (currentState.clutch << 2);
      packet |= 0x08 & (isEnabled() << 3);

      UDR0 = packet;
    }
}

void recvCommand() {
    if (UCSR0A & _BV(RXC0)) {
      uint8_t packet = UDR0;
      desiredState.gear = packet & 0x03;
      desiredState.clutch = 0x01 & (packet >> 2);
    }
}

int main() {
    sei();
    initTimeout();
    initSolenoids();
    initClutchPwm();
    initUart();
    while(1) {
      recvCommand();
      shift();
      trySendStatus();
    }
}