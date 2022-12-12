
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
    float current;
    float temp;
} mcfbk_data;

Heartbeat schedInfo[TASK_COUNT];

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

// This is triggered when the UART buffer is 3/4 full
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // Reset the interrupt flag
    IFS1bits.U2RXIF = 0;
    // Handle the reading of the buffer
    handleUARTReading();
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

void parse_output(){
    
}

void control_task(){
    // main control step, 200Hz
    
    ADCON1bits.SAMP = 1; // start sampling current
    
    // Temporarely disable the UART interrupt to read data
    // This does not cause problems if data arrives now since we are empting the buffer
    IEC1bits.U2RXIE = 0;
    // Handle the reading of the buffer
    handleUARTReading();
    // Enable UART interrupt again
    IEC1bits.U2RXIE = 1;

    // Check if there was an overflow in the UART buffer
    handleUARTOverflow();
    
    mcref_data in_sdata;
    
    char word;
    while (in_buffer.count != 0){
        cb_pop_front(&in_buffer, &word);
        int ret = parse_byte(&pstate, word);
        if (ret == NEW_MESSAGE){
            if (strcmp(pstate.msg_type, "MCREF") == 0){
                parse_input(pstate.msg_payload, &in_sdata);
                float multiplier = 0.002 * in_sdata.rpm;
                PDC2 = multiplier*PTPER;
            }
        }
    }
    while (!ADCON1bits.DONE); // wait for the end of the conversion
    int bits = ADCBUF0;
    float amp = 0.0488759*bits - 30;
    if (amp > 15)
        LATBbits.LATB1 = 1;
    else
        LATBbits.LATB1 = 0;
}

void feedback_task(){
    // send temp and current feedback to UART receiver, 1Hz
}

void blink_task(){
    // toggle led D3, 1Hz
     LATBbits.LATB0 = !LATBbits.LATB0; // toggle led
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
    // parser initialization
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    
    TRISBbits.TRISB0 = 0; // set the pin as output, led D3
    TRISBbits.TRISB1 = 0; // set the pin as output, led D4

    schedInfo[0] = (Heartbeat){0, 1, control_task}; // task 1 runs every heartbeat
    schedInfo[1] = (Heartbeat){0, 200, feedback_task}; // task 2 runs every 50 heartbeat
    schedInfo[2] = (Heartbeat){0, 200, blink_task}; // task 3 runs every 100 heartbeat
    
    cb_init(&in_buffer);              // init the circular buffer structure
    cb_init(&out_buffer);              // init the circular buffer structure
    init_uart();                   // init the UART
    init_spi();                    // init the SPI
    tmr_wait_ms(TIMER1, 1500);     // wait 1.5s to start the SPI correctly
    tmr_setup_period(TIMER1, 5); // initialize heatbeat timer
    
    PTCONbits.PTMOD = 0;     // free running mode
    PTCONbits.PTCKPS = 0b0;  // prescaler
    PWMCON1bits.PEN2H = 1;   // output high bit
    PWMCON1bits.PEN2L = 1;   // output low bit
    PTPER = 1842;            // time period
    PDC2 = 0*PTPER;            // duty cycle
    PTCONbits.PTEN = 1;      // enable the PWM

    ADCON3bits.ADCS = 8; // Tad = 4.5 Tcy
    // sampling mode: manual sampling, automatic conversion
    ADCON1bits.ASAM = 0; // start
    ADCON1bits.SSRC = 7; // end
    ADCON3bits.SAMC = 16; // auto sampling time
    ADCON2bits.CHPS = 0; // selecting the channel to convert
    ADCHSbits.CH0SA = 0b0010; // chose the positive input of the channels
    ADPCFG = 0xfffb;     // select the AN2 pin as analogue
    ADCON1bits.ADON = 1; // turn the ADC on
    
    // main loop
    while (1) {
        scheduler();
        tmr_wait_period(TIMER1);
    }
    return 0;
}