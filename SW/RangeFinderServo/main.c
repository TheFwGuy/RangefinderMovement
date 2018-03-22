/**
 *  @file pwmled.c 
 *  @brief Experimental code to handle PWM using an MSP430F2012 Texas Instrument 
 *  microcontroller
 *  @author Stefano B.
 *  @version 01 beta
 *  @date May 2008
 *  @details This code is designed to experiment a PWM signal generation
 *
 *  Version features
 *  Two pushbutton allow to increase and decrase or reset the Duty cicle
 *
 *  Built with IAR Embedded Workbench Version: 3.40A
 */

#include <string.h>
#include "msp430x20x2.h"

/*
 *  Functions prototype
 */
void Init(void);                    /* Init LED */
unsigned char testButton(unsigned char);

/*
 *  Global defines
 */
#define PWMMAXSTEP  256           /* Maximum ste for the PWM */
#define PWMINITIALVALUE 0         /* Initial value duty cicle */

#define FALSE         0
#define TRUE          1

/* I/O defines */

#define S1_BUTTON 0
#define S2_BUTTON 1

#define LED_ON  P1OUT |= BIT0
#define LED_OFF P1OUT &= ~BIT0
#define TEST_LED P1OUT & BIT0

/*
 *  Global variables
 */
unsigned short DutyCycle;

/*
 *  Main entry file
 */
void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
//  P1DIR |= 0x01;                        // Set P1.0 to output direction

  Init(); 

  for(;;)
  {
     if(testButton(S1_BUTTON))    /* Check S1 pushbutton */
     {
        /*
         *  Button pressed
         */
        if(DutyCycle < PWMMAXSTEP)
          DutyCycle++;
        
        CCR1 = DutyCycle;
        LED_ON;
     }   
        
     if(testButton(S2_BUTTON))    /* Check S2 pushbutton */
     {
        /*
         *  Button pressed
         */
        if(DutyCycle > 0)
          DutyCycle--;

        CCR1 = DutyCycle;
        LED_OFF;
     }   
  }
}

/*
 *  Init PWM Management - set timer
 */
void Init(void)
{
  WDTCTL = WDTPW + WDTHOLD;     /* Stop watchdog timer */

  /*
   *  Set DCO
   */
  BCSCTL1 = CALBC1_1MHZ;      /* Set up 1 Mhz using internal calibration value */
  BCSCTL2 = DIVS_3;           /* Divide SMCLK by 8 */
  DCOCTL  = CALDCO_1MHZ;

  /*
   *  Set I/O
   *  P1.0 -> I/O output - original LED on the board
   *  P1.2 -> Timer_A output - PWM out
   *  P2.6 <- input  - button 1 - Increase
   *  P2.7 <- input  - button 2 - Decrease
   */
  P1OUT = 0;                  /* Force out low */

  P1SEL |= OUT;               /* Set P1.2 on OUT1 */
  P1SEL |= BIT4;
  
  P2SEL &= ~BIT6;
  P2SEL &= ~BIT7;

  P1DIR |= 0x1F;              /* Set P1.0, P1.1 output direction */
  P2DIR  = 0x00;              /* Leave P2.6 and P2.7 in input direction */
 
  /*
   *  Set Timer
   */
  CCR0 = PWMMAXSTEP;          /* Set max step */
  CCR1 = PWMINITIALVALUE+128; /* Set initial PWM value */

  CCTL1 = OUTMOD_7;
  TACTL = TASSEL_2 + ID_3 + MC_1; /* SMCLK, up mode */

  DutyCycle = 128;            /* Reset variable */
}

/**
 * testButton
 * @brief Checking the pushbutton status
 *
 * The function is reading a pushbutton and return it's status.
 * Just to avoid multiple readings, the function is returning TRUE
 * (i.e. pushbutton pressed) only if detect a transition from true to
 *  false, with at least some tick delay.
 *  In other words is reading the pushbutton and IF the reading is
 *  high (pushbutton pressed), is waiting for the reading to return false
 *  and then report TRUE.
 *  Primitive but for test purpose is enough.
 *
 * @param button button to be checked. 0 -> S1  1 -> S2
 * @return True if button is pressed, False if not pressed
 */
unsigned char testButton(unsigned char button)
{
   unsigned char retvalue = 0;
   unsigned char smallDelay = 0;
   
   retvalue = P2IN & (0x80 >> button);
   
   if(retvalue)
   {
      for (smallDelay=200; smallDelay>0; smallDelay--);
           
      retvalue = P2IN & (0x80 >> button);
      if(retvalue)
      {
         while(retvalue)
            retvalue = P2IN & (0x80 >> button);
         return(TRUE);    
      }
      else
        return(FALSE);
   }
   else
      return (FALSE);
}

/*
 *  This code is documented using DoxyGen 
 *  (http://www.stack.nl/~dimitri/doxygen/index.html)
 */



