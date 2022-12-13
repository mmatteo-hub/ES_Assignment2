
// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "parser.h"
#include "my_timer_lib.h"
#include "my_print_lib.h"
#include "my_circular_buffer_lib.h"
#include "my_btn_lib.h"
#include <stdio.h>
#include <string.h>

#define TASK_COUNT 3

volatile circular_buffer in_buffer;
volatile circular_buffer out_buffer;
parser_state pstate;

typedef struct
{
    int n;  // number of periods elapsed from last task call
    int N;  // after how many periods should the task be called
    void (*task)(void);  // the task
} Heartbeat;

typedef struct{
    int rpm;
} mcref_data;

typedef struct{
    float amp;
    float temp;
    int n;
} mcfbk_data;

Heartbeat schedInfo[TASK_COUNT];
mcfbk_data out_sdata = {0};

// Handles the reading of the UART periferal and related errors
void handleUARTReading()
{
    // Check if there is something to read from UART
    while(U2STAbits.URXDA == 1)
        // Put the data in the circular buffer
        if(cb_push_back(&in_buffer, U2RXREG) == -1){
            // Wait for a possible ongoing transmission
            while (SPI1STATbits.SPITBF == 1);
            // Write on the LCD
            SPI1BUF = '1';
        }
}

void handleUARTWriting()
{
    char word;
    // until the UART buffer is not full, and there is something in the output buffer
    while (U2STAbits.UTXBF == 0 && out_buffer.count != 0){
        cb_pop_front(&out_buffer, &word);
        U2TXREG = word;
    }
}

// This is triggered when the receiver UART buffer is 3/4 full
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // Reset the interrupt flag
    IFS1bits.U2RXIF = 0;
    // Handle the reading of the buffer
    handleUARTReading();
}

// This is triggered when the transmitter UART buffer becomes empty
void __attribute__((__interrupt__, __auto_psv__)) _U2TXInterrupt()
{
    // Reset the interrupt flag
    IFS1bits.U2TXIF = 0;
    // Handle the reading of the buffer
    handleUARTWriting();
}

// Handle the overflow of the UART
void handleUARTOverflow()
{
    // Overflow did not occur, do nothing
    if(U2STAbits.OERR == 0)
        return;
    
    // Wait for a possible ongoing transmission
    while (SPI1STATbits.SPITBF == 1);
    // Write on the LCD
    SPI1BUF = '2';
    // Clear the UART buffer by storing all the available data
    handleUARTReading();
    // Clear the UART overflow flag
    U2STAbits.OERR = 0;
}

void parse_input(const char* msg, mcref_data* sdata){
    sdata->rpm = extract_integer(msg);
}

void parse_output(char* msg, mcfbk_data* sdata){
    float amp = sdata->amp / sdata->n;
    float temp = sdata->temp / sdata->n;
    sprintf(msg,"$MCFBK,%f,%f*", amp,temp);
}

void control_task(){
    // Main control step, 200Hz
    mcref_data in_sdata;
    char word;
    int bits;
    // Start sampling
    ADCON1bits.SAMP = 1;
    // Temporarely disable the UART interrupt to read data
    // This does not cause problems if data arrives now since we are empting the buffer
    IEC1bits.U2RXIE = 0;
    // Handle the reading of the buffer
    handleUARTReading();
    // Enable UART interrupt again
    IEC1bits.U2RXIE = 1;
    // Check if there was an overflow in the UART buffer
    handleUARTOverflow();
    while (in_buffer.count != 0){                           // until the input buffer is not full
        cb_pop_front(&in_buffer, &word);                    // read the first character of the buffer
        int ret = parse_byte(&pstate, word);                // parse the character
        if (ret == NEW_MESSAGE){                            // if the parsing returns that the message is complete
            if (strcmp(pstate.msg_type, "MCREF") == 0){     // compare the type of the message with the desired one
                parse_input(pstate.msg_payload, &in_sdata); // extract the data from the payload
                float multiplier = 0.002 * in_sdata.rpm;    // compute the multiplier for the duty cycle (1000rpm => 5V => 2)
                PDC2 = multiplier*PTPER;
            }
        }
    }
    while (!ADCON1bits.DONE);        // wait for the end of the ADC conversion
    bits = ADCBUF0;                  // read the converted value of the potentiometer
    float amp = 0.0488759*bits - 30; // compute the simulated analogue current
    if (amp > 15)                    // if the current exceeds 15A
        LATBbits.LATB1 = 1;          // turn on the led D4
    else
        LATBbits.LATB1 = 0;          // turn off the led D4
    out_sdata.amp += amp;
    bits = ADCBUF1;
    out_sdata.temp += bits * 0.48828125 - 50;
    out_sdata.n++;
}

void feedback_task(){
    // send temp and current feedback to UART receiver, 1Hz
    char out[30];
    int i = 0;
    parse_output(out, &out_sdata);
    out_sdata = (mcfbk_data){0};
    while (out[i] != '\0'){
        cb_push_back(&out_buffer, out[i]);
        i++;
    }
    IEC1bits.U2TXIE = 0;
    handleUARTWriting();
    IEC1bits.U2TXIE = 1;
}

void blink_task(){
    // toggle led D3, 1Hz
     LATBbits.LATB0 = !LATBbits.LATB0; // toggle led D3
}

void scheduler()
{
    for(int i=0; i<TASK_COUNT; ++i)
    {
        if(++schedInfo[i].n < schedInfo[i].N)
            continue;
        schedInfo[i].task();
        schedInfo[i].n = 0;
    }
}

int main(void) {
    // Parser initialization
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    // Set pins of leds D3 and D4 as output
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    // Init schedInfo struct, the heartbeat is set to 5ms
    schedInfo[0] = (Heartbeat){0, 1, control_task};    // task 1 runs every heartbeat
    schedInfo[1] = (Heartbeat){0, 200, feedback_task}; // task 2 runs every 200 heartbeat
    schedInfo[2] = (Heartbeat){0, 200, blink_task};    // task 3 runs every 200 heartbeat
    // Init circular buffers to read and write on the UART
    cb_init(&in_buffer);         // init the circular buffer to read from UART
    cb_init(&out_buffer);        // init the circular buffer to write on UART
    init_uart();                 // init the UART
    init_spi();                  // init the SPI (for debug pourposes, it prints error messages)
    tmr_wait_ms(TIMER1, 1500);   // wait 1.5s to start the SPI correctly
    tmr_setup_period(TIMER1, 5); // initialize heatbeat timer
    // Init PWM to set the voltage to the armature of the DC motor
    PTCONbits.PTMOD = 0;    // free running mode
    PTCONbits.PTCKPS = 0b0; // prescaler
    PWMCON1bits.PEN2H = 1;  // output high bit
    PWMCON1bits.PEN2L = 1;  // output low bit
    PTPER = 1842;           // time period
    PDC2 = 0;               // duty cycle, at the beginning the motor is still
    PTCONbits.PTEN = 1;     // enable the PWM
    // Init the ADC converter in manual sampling and automatic conversion
    ADCON3bits.ADCS = 8;      // Tad = 4.5 Tcy
    ADCON1bits.ASAM = 0;      // start
    ADCON1bits.SSRC = 7;      // end
    ADCON3bits.SAMC = 16;     // auto sampling time
    ADCON2bits.CHPS = 0b01;   // selecting the channel to convert
    ADCHSbits.CH0SA = 0b0010; // chose the positive input of the channels
    ADCHSbits.CH123SA = 1;    
    ADPCFG = 0xfff3;          // select the AN2 and AN3 pins as analogue
    ADCON1bits.SIMSAM = 1;    // simultaneous sampling
    ADCON1bits.ADON = 1;      // turn the ADC on
    
    // main loop
    while (1) {
        scheduler();
        tmr_wait_period(TIMER1);
    }
    return 0;
}