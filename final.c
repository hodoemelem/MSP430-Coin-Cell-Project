/******************************************************************************
 *                                                                            *
 *             Final project: Coin cell                                       *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * name:                    Henry Ugochukwu Odoemelem                         *
 * matriculation number:    1463255                                           *
 * e-mail:                  henry.odoemelem@student.uni-siegen.de             *
 * Date:                    20-07-2020                                        *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * Hardware Setup                                                             *
 *                                                                            *
 *             VCC               MSP430FR5969                                 *
 *              |             -----------------                               *
 *           (SENSOR)        |                 |                              *
 *              +-------- -->|P4.2/A10     P4.6|--> (LED1)                    *
 *              |            |             P1.0|--> (LED2)                    *
 *             |R|           | 1 MHz           |                              *
 *              |             -----------------                               *
 *             GND                                                            *
 *                                                                            *
 ******************************************************************************/

#include <msp430fr5969.h>
#include <stdint.h>

#define LIGHT_THRS 154                // 154 is 15lux for 8 bits
//#define LIGHT_THRS 50

#define TXBUF 24                        // Size of circular transmitter buffer.
#define RXBUF 8                         // Size of circular receiver buffer.

volatile uint8_t rxBuf[RXBUF];          // The receiver circular buffer (rx).
volatile uint8_t rxBufS = 0;            // rx Start position.
volatile uint8_t rxBufE = 0;            // rx End position.
static uint8_t rxChar = 0;

volatile uint8_t txBuf[TXBUF];          // The transmitter circular buffer (tx).
volatile uint8_t txBufE = 0;            // tx start position.
volatile uint8_t txBufS = 0;            // tx end position.

volatile uint8_t highCount = 0;         // varaible to count number of high samples before falling edge
volatile uint8_t lowCount = 0;          // variable to count number of low samples before raising edge
volatile uint8_t start_preamble = 0;    // variable to detect first preamble bit has been received

volatile struct                         // State variables of the FSMs:
{
    uint8_t Main : 4;                   // FSM of main routine:
                                        // 0:   BIT0
                                        // 1:   BIT1
                                        // 2:   BIT2
                                        // 3:   BIT3
                                        // 4:   BIT4
                                        // 5:   BIT5
                                        // 6:   BIT6
                                        // 7:   BIT7
                                        // 8:   ERROR
                                        // 9:   ACK
                                        // 10 : MEASURE
                                        // 11:  TRANSMIT

} States;

typedef enum                            // State values of main FSM:
{
    MAIN_BIT0,                          // 0:   BIT0
    MAIN_BIT1,                          // 1:   BIT1
    MAIN_BIT2,                          // 2:   BIT2
    MAIN_BIT3,                          // 3:   BIT3
    MAIN_BIT4,                          // 4:   BIT4
    MAIN_BIT5,                          // 5:   BIT5
    MAIN_BIT6,                          // 6:   BIT6
    MAIN_BIT7,                          // 7:   BIT7
    MAIN_ERROR,                         // 8:   ACK
    MAIN_ACK,                           // 9:   ERROR
    MAIN_MEASURE,                       // 10:  MEASURE
    MAIN_TRANSMIT                       // 11:  TRANSMIT
} STATES_MAIN;

volatile uint16_t adc_val = 0;          // variable to hold sensor value
volatile uint8_t light_flag = 0;        // flag to indicate a light source is present.
volatile uint8_t preamble_flag = 0;     // flag to rx preamble
volatile uint8_t process_flag = 0;      // flag for ack., measurement and transmission processes in progress
volatile uint8_t exit_flag = 0;         // flag to reset the program if the light source is connected

/* TIMEOUT TIMER */
void start_timeout(void);               // Initialize timeout watchdog timer.
void stop_timeout(void);                // Stop timeout watchdog timer.

void uartTx(uint16_t data);             // UART transmit, to visible transmitted characters if necessary

/* MAIN PROGRAM */
void main(void)
{
    // Stop watchdog timer.
    WDTCTL = WDTPW | WDTHOLD;

    // Initialize the clock system to generate 1 MHz DCO clock.
    CSCTL0_H    = CSKEY_H;              // Unlock CS registers.
    CSCTL1      = DCOFSEL_0;            // Set DCO to 1 MHz, DCORSEL for
                                        // high speed mode not enabled.
    CSCTL2      = SELA__VLOCLK |        // Set ACLK = VLOCLK = 10 kHz.
                  SELS__DCOCLK |        // Set SMCLK = DCOCLK.
                  SELM__DCOCLK;         // Set MCLK = DCOCLK.
                                        // SMCLK = MCLK = DCOCLK = 1 MHz.
    CSCTL3      = DIVA__1 |             // Set ACLK divider to 1.
                  DIVS__1 |             // Set SMCLK divider to 1.
                  DIVM__1;              // Set MCLK divider to 1.
                                        // Set all dividers to 1.
    CSCTL0_H    = 0;                    // Lock CS registers.


    // Initialize unused GPIOs to minimize energy-consumption.
    // Port 1:
    P1DIR = 0xFF;
    P1OUT = 0x00;
    // Port 2:
    P2DIR = 0xFF;
    P2OUT = 0x00;
    // Port 3:
    P3DIR = 0xFF;
    P3OUT = 0x00;
    // Port 4:
    P4DIR = 0xFF;
    P4OUT = 0x00;
    // Port J:
    PJDIR = 0xFFFF;
    PJOUT = 0x0000;

    // Initialize port 1:
    P1DIR |= BIT0;                      // P1.0 - output for LED2, off.
    P1OUT &= ~BIT0;

    // Initialize port 4:
    P4DIR |= BIT6;                      // P4.6 - output for LED1, off.
    P4OUT &= ~BIT6;


    /* INIT ADC PIN */

    // Initialize ADC input pins:
    P4SEL0   |= BIT2;                   // P4.2 ADC option select
    P4SEL1   |= BIT2;

    /*Enable Pull up resistors*/
    P1REN |= BIT1;                      // P1.1 pull up resistor enabled
    P1OUT |= BIT1;

    P4REN |= BIT5;                      // P4.5 pull up resistor enabled
    P4OUT |= BIT5;


    // Initialize port 2:
    // Select Tx and Rx functionality of eUSCI0 for hardware UART.
    // P2.0 - UART Tx (UCA0TXD).
    // P2.1 - UART Rx (UCA0RXD).
    P2SEL0 &= ~(BIT1 | BIT0);
    P2SEL1 |= BIT1 | BIT0;

    // Disable the GPIO power-on default high-impedance mode to activate the
    // previously configured port settings.
    PM5CTL0 &= ~LOCKLPM5;


    /* Initialization of the serial UART interface */

    UCA0CTLW0 |= UCSSEL__SMCLK |        // Select clock source SMCLK = 1 MHz.
                 UCSWRST;               // Enable software reset.
    // Set Baud rate of 9600 Bd.
    // Recommended settings available in table 30-5, p. 779 of the User's Guide.
    UCA0BRW = 6;                        // Clock prescaler of the
                                        // Baud rate generator.
    UCA0MCTLW = UCBRF_8 |               // First modulations stage.
                UCBRS5 |                // Second modulation stage.
                UCOS16;                 // Enable oversampling mode.
    UCA0CTLW0 &= ~UCSWRST;              // Disable software reset and start
                                        // eUSCI state machine.

    States.Main = MAIN_BIT0;            // start main FSM at BIT0

    /*INIT ADC REF */
    /* Initialization of the reference voltage generator */
    // Configure the internal voltage reference for 2.5 V:
    while(REFCTL0 & REFGENBUSY);
    REFCTL0 |= REFVSEL_2|REFON;
    while(!(REFCTL0 & REFGENRDY));

    /* INIT ADC */
   // Configure the analog-to-digital converter such that it samples
   ADC12CTL0 = 0;                      // Disable ADC trigger.
   ADC12CTL0 = ADC12ON;                // Turn on ADC.
   ADC12CTL1 = ADC12SHP   |            // Use sampling timer.
               ADC12CONSEQ_2|          // Repeat-single-channel mode.
               ADC12SHS_1;             // Use timer TA0.1 as trigger.
   ADC12CTL2 = ADC12RES_0;             // 8 bit conversion resolution.
   ADC12IER0 = ADC12IE0;               // Enable ADC conversion interrupt.
   ADC12MCTL0 = ADC12INCH_10|          // Select ADC input channel A10.
                ADC12VRSEL_1;          // Reference voltages V+ = VREF buffered
                                       // = 2.5V and V- = AVSS = GND.
   ADC12CTL0 |= ADC12ENC;              // Enable ADC trigger.
   ADC12CTL0 |= ADC12SC;               // Trigger sampling.

   // Initialize timer A0 to trigger ADC12 at 50 Hz:
   TA0CTL = TACLR;                     // clear and stop the timer
   TA0CCR0 = 50 - 1;                   // 1 digit - 0.4 ms; Period of 20ms is 50 counts
   TA0CCTL1 = OUTMOD_7;                // reset/set
   TA0CCR1 = 4;                        // 8% duty cycle
   TA0CTL = TASSEL__ACLK |             // 10k Hz ACLK
             ID__4|                    // first input divider stage of 4 -> 2500Hz
             MC__UP;                   // Enable up mode

   while(ADC12IFGR0 & ADC12IFG0);      // Wait for first 8 bit sample.

   // Set transmission preamble
   txBuf[0] = 1;
   txBuf[1] = 0;
   txBuf[2] = 0;
   txBuf[3] = 1;
   txBuf[4] = 0;
   txBuf[5] = 1;
   txBuf[6] = 0;
   txBuf[7] = 1;


   // Enable interrupts globally.
   __bis_SR_register(GIE);

    /* MAIN LOOP */
    while(1)
    {
         if(preamble_flag == 1 || process_flag == 1)           // preamble, ack, measure or transmit ?
         {
             if(rxBufS != rxBufE )                             // Received characters available?
             {
                rxChar = rxBuf[rxBufS++];                      // Get received character from circular buffer.
                rxBufS = rxBufS & (RXBUF-1);                   // Roll over the index if the buffer size is reached.
             }
             else if(preamble_flag == 1 && process_flag == 0 ) // if here, then no new preamble bit has been received
             {
                 __bis_SR_register(LPM3_bits);                 // Enter LPM3
                 rxChar = rxBuf[rxBufS++];                     // Get received character from circular buffer.
                 rxBufS = rxBufS & (RXBUF-1);                  // Roll over the index if the buffer size is reached.
             }

            /*FSM MAIN */
            switch(States.Main)
            {
               case MAIN_BIT0 :
                       if(rxChar == 0)
                       {
                          States.Main = MAIN_BIT1;             // Go to state of BIT1.
                       }
                       else
                       {
                           States.Main = MAIN_ERROR;           // Go to ERROR state.
                       }
                     break;
                case MAIN_BIT1:
                        if(rxChar == 1)
                        {
                           States.Main = MAIN_BIT2;            // Go to state of BIT2.
                        }
                        else
                        {
                           States.Main = MAIN_ERROR;           // Go to ERROR state.
                        }
                     break;
                case MAIN_BIT2:

                        if(rxChar == 1)
                        {
                           States.Main = MAIN_BIT3;            // Go to state of BIT3.
                        }
                        else
                        {
                           States.Main = MAIN_ERROR;           // Go to ERROR state.
                        }

                     break;
                case MAIN_BIT3:
                        if(rxChar == 0)
                        {
                           States.Main = MAIN_BIT4;            // Go to state of BIT4.
                        }
                        else
                        {
                           States.Main = MAIN_ERROR;           // Go to ERROR state.
                        }
                     break;
                case MAIN_BIT4:
                        if(rxChar == 1)
                        {
                           States.Main = MAIN_BIT5;            // Go to state of BIT5.
                        }
                        else
                        {
                           States.Main = MAIN_ERROR;          // Go to ERROR state.
                        }
                      break;
                case MAIN_BIT5:
                        if(rxChar == 0)
                        {
                           States.Main = MAIN_BIT6;           // Go to state of BIT6.
                        }
                        else
                        {
                            States.Main = MAIN_ERROR;         // Go to ERROR state.
                        }
                     break;
                case MAIN_BIT6:
                        if(rxChar == 1)
                        {
                           States.Main = MAIN_BIT7;           // Go to state of BIT7.
                        }
                        else
                        {
                           States.Main = MAIN_ERROR;          // Go to ERROR state.
                        }
                     break;
                case MAIN_BIT7:
                        if(rxChar == 0)
                        {
                           rxBufE = 0;
                           rxBufS = 0;
                           process_flag = 1;
                           preamble_flag = 0;
                           //Return back to 50 Hz sampling of ADC after preamble
                           // Initialize timer A0 to trigger ADC12 at 50 Hz:
                           TA0CTL = TACLR;               // clear and stop the timer
                           TA0CCR0 = 50 - 1;             // 1 digit - 0.4 ms; Period - 20ms , 50 counts
                           TA0CCTL1 = OUTMOD_7;          // reset/set
                           TA0CCR1 = 4;                  // 8% duty cycle
                           TA0CTL = TASSEL__ACLK |       // 10k Hz ACLK
                                         ID__4|          // first input divider stage of 4 -> 2500Hz
                                         MC__UP;         // enable up mode
                           while(ADC12IFGR0 & ADC12IFG0);// Wait for first 8 bit sample.
                           States.Main = MAIN_ACK;       // Go to state of ACKNOWLEDGEMENT.
                       }
                       else
                       {
                           States.Main = MAIN_ERROR;     // Go to ERROR state.
                       }

                     break;
                case MAIN_ERROR:
                     preamble_flag = 0;                  // Reset preamble flag causing program to quit
                     break;
                case MAIN_ACK:
                     /* Delay 250ms with TA1CCR1*/
                     TA1CTL = TACLR;                     // clear and stop the timer.
                     P1OUT |= BIT0;                      // Turn LED2 on
                     TA1CCR1 = 625;                      // 625 counts for 250 ms
                     TA1CCTL1 |= CCIE;                   // Enable TA1CCTL1 CC interrupt
                     TA1CTL = TASSEL__ACLK  |            // Select clock source ACLK, 10 KHz.
                              ID__4        |             // First input divider stage of 4 -> 2500Hz
                              MC__CONTINUOUS;            // CONTINUOUS mode.
                     __bis_SR_register(LPM3_bits);       // Enter LPM3
                     _no_operation();
                     P1OUT &=~ BIT0;                     // Turn LED2 off
                     TA1CCTL1 &=~ CCIE;                  // Disable TA1CCTL1 CC interrupt
                     TA1CTL = TACLR;                     // clear and stop the timer.

                     // setup timer B0 for measurement task
                     TB0CTL = TASSEL__ACLK  |             // Select clock source ACLK, 10 KHz.
                                ID__4        |            // first input divider stage of 4 -> 2500Hz
                                MC__CONTINUOUS;           // CONTINUOUS mode.

                     States.Main = MAIN_MEASURE;
                     txBufE = 8;
                     break;
                case MAIN_MEASURE:
                     /* delay 10 ms with TB0CCR1 */
                     TB0R = 0;                            // clear counter
                     TB0CCR1 = 25;                        // 25 counts is 10 ms
                     TB0CCTL1 |= CCIE;                    // Enable TA1CCTL1 CC interrupt
                     __bis_SR_register(LPM3_bits);        // Enter LPM3
                     _no_operation();
                     P4OUT &=~ BIT6;                      // Turn LED1 off.

                     /* delay 980ms with TB0CCR1  */
                     TB0R = 0;                            // clear counter
                     TB0CCR1 = 2450;                      // 2450 counts is 980 ms
                     TB0CCTL1 |= CCIE;                    // Enable TA1CCTL1 CC interrupt
                     __bis_SR_register(LPM3_bits);        // Enter LPM3
                     _no_operation();


                     /* delay 10 ms with TB0CCR2*/
                     TB0R = 0;
                     TB0CCR2 = 25;                        // 25 counts is 10 ms
                     P4OUT |= BIT6;                       // Turn LED1 on.
                     TB0CCTL2 |= CCIE;                    // Enable TB0CCTL2 CC interrupt
                     __bis_SR_register(LPM3_bits);        // Enter LPM3
                     _no_operation();

                     break;
                case MAIN_TRANSMIT:
                     /* delay 10 ms with TB0CCR1 */
                     TB0R = 0;
                     TB0CCR1 = 25;                        // 25 counts is 10 ms
                     TB0CCTL1 |= CCIE;                    // Enable TB0CCTL1 CC interrupt
                     __bis_SR_register(LPM3_bits);        // Enter LPM3
                     _no_operation();
                     P4OUT &=~ BIT6;                      // Turn LED1 off.
                     TB0CTL = TBCLR;                      // Clear and Stop TB0

                     //TA1CTL SW PWM for transmitting
                     //uartTx(txBuf[txBufS]);             // Tx measured data
                     TA1CTL = TACLR;                      // clear and stop the timer
                     TA1CCR0 = 5000;
                     TA1CCR2 = 3750;                      // 1:  3750 counts is 75% duty cycle of 0.5 sec, first tx preamble is 1
                     TA1CCTL0 |= CCIE;                    // Enable TA1CCTL0 CC interrupt
                     TA1CCTL2 |= CCIE;                    // Enable TA1CCTL2 CC interrupt
                     P1OUT |= BIT0;                       // Turn LED1 on.
                     TA1CTL = TASSEL__ACLK |              // 10kHz
                              ID__1|                      // first input divider stage 1 -> 10KHz
                              MC__UP;                     // enable up mode
                     __bis_SR_register(LPM3_bits);        // Enter LPM3
                     _no_operation();
                     break;
             }
         }
        else if(process_flag == 0)                        // if processing is done, reset all variables.
        {
            // Reset all variables
            States.Main = MAIN_BIT0;                      // Go to state of BIT0.
            preamble_flag = 0;
            process_flag = 0;
            lowCount = 0;
            highCount = 0;
            light_flag = 0;
            start_preamble = 0;
            rxBufE = 0;
            rxBufS =0;
            txBufE = 0;
            txBufS = 0;

            //Return back to 50 Hz sampling of ADC if Error occurs in preamble
            // Initialize timer A0 to trigger ADC12 at 50 Hz:
            TA0CTL = TACLR;                               // clear and stop the timer
            TA0CCR0 = 50 - 1;                             // 1 digit - 0.4 ms; Period - 20ms , 50 counts
            TA0CCTL1 = OUTMOD_7;                          // reset/set
            TA0CCR1 = 4;                                  // 8% duty cycle
            TA0CTL = TASSEL__ACLK |                       // 10k Hz ACLK
                     ID__4|                               // first input divider stage of 4 -> 2500Hz
                     MC__UP;                              // enable up mode
            while(ADC12IFGR0 & ADC12IFG0);                // Wait for first 8 bit sample.


             if(exit_flag == 1)  // turn on LED2 for 1 sec is process was interrupted
             {
                 P1OUT &=~ BIT0;                          // Turn LED1 off.
                 P4OUT &=~ BIT6;                          // Turn LED2 off.

                /* delay 1 sec with TA1CCR1*/
                TA1CTL = TACLR;                           // clear and stop the timer
                TA1CCR1 = 2500;                           // 2500 counts for 1 sec
                P1OUT |= BIT0;                            // Turn LED2 on
                TA1CCTL1 |= CCIE;                         // Enable TA1CCTL1 CC interrupt
                TA1CTL = TASSEL__ACLK  |                  // Select clock source ACLK, 10 KHz.
                       ID__4        |                     // first input divider stage of 4 -> 2500Hz
                       MC__CONTINUOUS;                    // CONTINUOUS mode.
                __bis_SR_register(LPM3_bits);             // Enter LPM3
                P1OUT &=~ BIT0;                           // Turn LED2 off
                TA1CTL = TACLR;                           // clear and stop the timer
                TA1CCTL0 &=~ CCIE;                        // Enable TA1CCTL0 CC interrupt
                TA1CCTL1 &=~ CCIE;
                TA1CCTL2 &=~ CCIE;
                TB0CTL = TBCLR;
                TB0CCTL1 &=~ CCIE;
                TB0CCTL2 &=~ CCIE;
              }
             exit_flag = 0;                               // Reset exit flag
             __bis_SR_register(LPM3_bits);                //Enter LPM3
         }
    }
}


/* ISR TIMER B0 - CCR0 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR (void)
{   // TB0 CCR0

}

/* ISR TIMER B0 - CCR1, CCR2 AND TBIFG */
#pragma vector = TIMER0_B1_VECTOR
__interrupt void Timer0_B1_ISR (void)
{
    switch(__even_in_range(TB0IV, TB0IV_TB0IFG))
    {
    case TB0IV_TB0CCR1:
         TB0CCTL1 &=~ CCIE;
         __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3
         break;
    case TB0IV_TB0CCR2:
       /* Measurement*/
       TB0CCTL2 &=~ CCIE;
       txBuf[txBufE] = 1;                                // Assign 1 to tx buffer, humididty
       txBuf[txBufE+8] = 1;                              // Assign 1 to tx buffer, stress
       if(!(P1IN & BIT1))                                // S2 released ? , humidity
       {
          txBuf[txBufE] = 0;                             // overwrite the 1 written with a 0, if S2 is pushed and released
       }
       if(!(P4IN & BIT5))                                // S1 released ? , stress
       {
          txBuf[txBufE + 8] = 0;                         // overwrite the 1 written with a 0, if S1 is pushed and released
       }
       txBufE++;                                         // increment buffer index
       __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3
       if(txBufE > 15 || process_flag == 0 )             // quit measurement if buffer size is reached or processing is abruptly ended
       {
           States.Main = MAIN_TRANSMIT;                  // Go to state of TRANSMIT data.
       }
       break;
    case TB0IV_TB0IFG:

        break;
    }
}

/* ISR TIMER TA1 - CCR0 */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR (void)
{   // TA1 CCR0
    /* Transmission */
    if(txBufS <= 23 && process_flag == 1)                // index less than 24?
    {
      P1OUT |= BIT0;                                     // Turn LED2 on.
      if (txBuf[txBufS] == 1)
      {
         TA1CCR2 = 3750;                                 // 1:  3750 counts is 75% duty cycle of 0.5 sec
         //uartTx(txBuf[txBufS]);                        // Tx measured data
      }
      else if(txBuf[txBufS] == 0)
      {
         TA1CCR2 = 1250;                                 // 0: 1250 counts is  25% duty cycle of 0.5 sec
         //uartTx(txBuf[txBufS]);                        // Tx measured data
      }
    }
    else if(txBufS > 23 || process_flag == 0)            // quit measurement if buffer size is reached or processing is abruptly ended
    {
       process_flag = 0;                                 // reset process flag
       TA1CCTL0 &=~ CCIE;                                // Enable TA1CCTL0 CC interrupt
       TA1CCTL2 &=~ CCIE;                                // Enable TA1CCTL2 CC interrupt
       TA1CTL = TACLR;                                   // clear and stop the timer
       __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3

    }

}

/* ISR TIMER TA1 - CCR1, CCR2 AND TAIFG */
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
   switch(__even_in_range(TA1IV, TA1IV_TA1IFG))
   {
      case TA1IV_TA1CCR1:                                // TA1 CCR1
           __bic_SR_register_on_exit(LPM3_bits);         // Exit LPM3
           break;
      case TA1IV_TA1CCR2:                                // TA1 CCR2
           if (txBufS <=23 && process_flag == 1)         // index less than 24 ?
           {
             P1OUT &=~ BIT0;                             // Turn LED2 off.
             txBufS++;                                   // increment buffer index
           }
           break;
      case TA1IV_TA1IFG:                                 // TA1 TAIFG
           break;
    }
}

/* ### TIMEOUT TIMER ### */

void start_timeout(void)                                 // Start timeout counter.
{
    // Initialize watchdog timer for timeout interrupt:
    // Timeout interrupt after interval 3.27s.
    WDTCTL = WDTPW|                                     // watchdog timer password
            WDTCNTCL|                                   // clear watchdog counter value
            WDTTMSEL|                                   // Select Interval timer mode
            WDTSSEL__ACLK|                              // select ACLK clock source, 10kHz
            WDTIS__32K;                                 // watchdog timer interval
                                                        // (1/10kHz) multiplied by 32768 to give 3.27s timeout.
    SFRIE1 |= WDTIE;                                    // Enable Watchdog timer interrupt
}

void stop_timeout(void)                                 // Stop the timeout counter.
{
    WDTCTL = WDTPW | WDTHOLD;                           // Stop watchdog timer.
    SFRIE1 &=~ WDTIE;                                   // Disable Watchdog timer interrupt
}

/* ISR WATCHDOG */
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
      stop_timeout();                                   // Stop watch dog timer
      preamble_flag = 1;                                // set preamble flag

      // Initialize timer A0 to trigger ADC12 at 10 Hz for receiving preamble:
      TA0CTL = TACLR;                                   // clear and stop the timer
      TA0CCR0 = 250 - 1;                                // 0.1s , 10 Hz sampling rate
      TA0CCTL1 = OUTMOD_7;                              // reset/set
      TA0CCR1 = 5;                                      // 2% duty cycle
      TA0CTL = TASSEL__ACLK |                           // 10K ACLK
                   ID__4|                               // first input divider stage of 4 -> 2500 Hz
                   MC__UP;                              // Enable up mode
      while(ADC12IFGR0 & ADC12IFG0);                    // Wait for first 8 bit sample.
      __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3
}


/* ### ADC ### */

/* ISR ADC */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
    {
    case ADC12IV_NONE:        break;        // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG:  break;        // Vector  2:  ADC12MEMx Overflow
    case ADC12IV_ADC12TOVIFG: break;        // Vector  4:  Conversion time overflow
    case ADC12IV_ADC12HIIFG:  break;        // Vector  6:  ADC12BHI
    case ADC12IV_ADC12LOIFG:  break;        // Vector  8:  ADC12BLO
    case ADC12IV_ADC12INIFG:  break;        // Vector 10:  ADC12BIN
    case ADC12IV_ADC12IFG0:                 // Vector 12:  ADC12MEM0 Interrupt

         adc_val = ADC12MEM0;                                        // Get sensor adc value
         if (preamble_flag == 0 && process_flag == 0)                // idle?
         {  /* code block to check if light source is high for more than 3sec continonuously */
            //uartTx(adc_val);                                       // tx current adc val
            if(adc_val > LIGHT_THRS && light_flag == 0)              // Light source high and light flag previously reset?
            {
               start_timeout();                                      // Start timeout timer
               light_flag = 1;                                       // Set light flag
            }
            else if (adc_val <= LIGHT_THRS && light_flag == 1)       // Light source low and light flag previously set?
            {
               stop_timeout();                                       // Stop timeout watchdog timer.
               light_flag = 0;                                       // Reset light flag
            }
         }
         else if(preamble_flag == 1 )                                // preamble flag set?
         {  /* code block to count samples and detect raising or falling edges of preamble bits */
            /* preamble is (0b01101010, 1 bit/0.5sec , +-20%) */
            /* sampling rate is 10Hz, i.e. 1sample/0.1seconds  */

            if (adc_val > LIGHT_THRS && start_preamble == 1 )        // light source high and first preamble bit received already? then raising edge detected
            {
               if(lowCount >=4 && lowCount <= 6 )                    // 4 <= number of low(0) samples before raising edge <= 6 ?(a low bit?)
               {
                   rxBuf[rxBufE++] = 0;                              // Store 0 in circular  receive buffer.
                   rxBufE = rxBufE & (RXBUF-1);                      // Rollover index when the the buffer size is reached
                   lowCount = 0;                                     // reset lowCount
                   __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3
               }
               else if (lowCount != 0)                               // Error bit(s),  as less than 4 or more than 6 low(0) samples
               {
                     preamble_flag = 0;                              // reset preamble flag
                     __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3
               }
               highCount++;                                          // count number of high(1) samples before falling edge
            }
            else if (adc_val <= LIGHT_THRS)                          // light source low ? then falling edge detected
            {  start_preamble = 1;                                   // first preamble bit (0) detected
               if(highCount >= 4 && highCount <= 6)                  // 4 <= number of high(1) samples before falling edge <= 6 ?( a high(1) bit?)
               {
                   rxBuf[rxBufE++] = 1;                              // Store 1 in circular  receive buffer.
                   rxBufE = rxBufE & (RXBUF-1);                      // Rollover index when the the buffer size is reached
                   highCount = 0;                                    // reset highCount
                   __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3
               }
               else if(highCount >= 8 && highCount <= 12)            // 8 <= number of high(1) samples before falling edge <= 12 ?(two high(1) bits?)
               {
                   rxBuf[rxBufE++] = 1;                              // Store 1 in circular  receive buffer.
                   rxBuf[rxBufE++] = 1;                              // Store 1 in circular  receive buffer.
                   rxBufE = rxBufE & (RXBUF-1);                      // Rollover index when the the buffer size is reached
                   highCount = 0;                                    // reset highCount
                   __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3
               }
               else if(highCount !=0)                                // Error bits(s)
               {
                     preamble_flag = 0;                              // reset preamble flag
                     __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3
               }
               lowCount++;                                           // count number of low(0) samples before raising edge

               if(lowCount > 6 )                                     // light source removed during preamble ?
               {                                                     // Error as more than 6 samples
                   preamble_flag = 0;                                // reset preamble flag
                   __bic_SR_register_on_exit(LPM3_bits);             // Exit LPM3
               }
            }
         }
         else if(States.Main == MAIN_MEASURE || States.Main == MAIN_TRANSMIT)  // processing stage ?
         {
             /* code block to set exit flag if light source is low for more than 20 ms during processing */
             if (adc_val > LIGHT_THRS && exit_flag == 1  )            // light source high and exit flag previously set ?
             {
                 exit_flag = 0;                                       // Reset exit flag
             }
             else if (adc_val <= LIGHT_THRS  )                        // light source low?
             {
                 if(exit_flag == 0 )                                  // is exit flag reset?
                 {
                   exit_flag = 1;                                     // set exit flag
                 }
                 else if(exit_flag == 1){                             // is exit flag set?
                     process_flag = 0;                                // reset process flag
                     __bic_SR_register_on_exit(LPM3_bits);            // Exit LPM3
                 }
             }
         }

         break;
    case ADC12IV_ADC12IFG1:   break;        // Vector 14:  ADC12MEM1
    case ADC12IV_ADC12IFG2:   break;        // Vector 16:  ADC12MEM2
    case ADC12IV_ADC12IFG3:   break;        // Vector 18:  ADC12MEM3
    case ADC12IV_ADC12IFG4:   break;        // Vector 20:  ADC12MEM4
    case ADC12IV_ADC12IFG5:   break;        // Vector 22:  ADC12MEM5
    case ADC12IV_ADC12IFG6:   break;        // Vector 24:  ADC12MEM6
    case ADC12IV_ADC12IFG7:   break;        // Vector 26:  ADC12MEM7
    case ADC12IV_ADC12IFG8:   break;        // Vector 28:  ADC12MEM8
    case ADC12IV_ADC12IFG9:   break;        // Vector 30:  ADC12MEM9
    case ADC12IV_ADC12IFG10:  break;        // Vector 32:  ADC12MEM10
    case ADC12IV_ADC12IFG11:  break;        // Vector 34:  ADC12MEM11
    case ADC12IV_ADC12IFG12:  break;        // Vector 36:  ADC12MEM12
    case ADC12IV_ADC12IFG13:  break;        // Vector 38:  ADC12MEM13
    case ADC12IV_ADC12IFG14:  break;        // Vector 40:  ADC12MEM14
    case ADC12IV_ADC12IFG15:  break;        // Vector 42:  ADC12MEM15
    case ADC12IV_ADC12IFG16:  break;        // Vector 44:  ADC12MEM16
    case ADC12IV_ADC12IFG17:  break;        // Vector 46:  ADC12MEM17
    case ADC12IV_ADC12IFG18:  break;        // Vector 48:  ADC12MEM18
    case ADC12IV_ADC12IFG19:  break;        // Vector 50:  ADC12MEM19
    case ADC12IV_ADC12IFG20:  break;        // Vector 52:  ADC12MEM20
    case ADC12IV_ADC12IFG21:  break;        // Vector 54:  ADC12MEM21
    case ADC12IV_ADC12IFG22:  break;        // Vector 56:  ADC12MEM22
    case ADC12IV_ADC12IFG23:  break;        // Vector 58:  ADC12MEM23
    case ADC12IV_ADC12IFG24:  break;        // Vector 60:  ADC12MEM24
    case ADC12IV_ADC12IFG25:  break;        // Vector 62:  ADC12MEM25
    case ADC12IV_ADC12IFG26:  break;        // Vector 64:  ADC12MEM26
    case ADC12IV_ADC12IFG27:  break;        // Vector 66:  ADC12MEM27
    case ADC12IV_ADC12IFG28:  break;        // Vector 68:  ADC12MEM28
    case ADC12IV_ADC12IFG29:  break;        // Vector 70:  ADC12MEM29
    case ADC12IV_ADC12IFG30:  break;        // Vector 72:  ADC12MEM30
    case ADC12IV_ADC12IFG31:  break;        // Vector 74:  ADC12MEM31
    case ADC12IV_ADC12RDYIFG: break;        // Vector 76:  ADC12RDY
    default: break;
    }
}


/* ### SERIAL INTERFACE ### */

void uartTx(uint16_t data)
{
    while((UCA0STATW & UCBUSY));                                 // Wait while module is busy.
    UCA0TXBUF = data;                                            // Transmit data byte.
}
