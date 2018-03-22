/**
 *  @file pwmtest2.c 
 *  @brief Experimental code to handle PWM using an MSP430F2012 Texas Instrument 
 *  microcontroller
 *  @author Stefano B.
 *  @version 01 beta
 *  @date June 2008
 *  @details This code is designed to experiment a PWM signal generation
 *
 *  Version features
 *  Two pushbutton allow to increase and decrase or reset the Duty cicle
 *  The PWM is generated via an interrupt function triggered by the timer.
 *  Is NOT used the PWM capability of the timer.
 *
 *  The timer will be set in UP mode (i.e. counting up to the value in CCR0).
 *  The timer will generate an interrupt every .1 ms
 *  Internal management (SW counters) will generate different PWM outputs for different I/O.
 *  Initially the one used will be P1.2 (the same used in pwmtest1).
 *  The goal is to have a PWM with 20 ms period and duty cycle with 256 steps
 *
 *  Built with IAR Embedded Workbench Version: 3.40A
 */

#include <string.h>
#include "msp430x20x2.h"

/*
 *  Functions prototype
 */
__interrupt void Timer_A (void);    /* Timer A0 interrupt service routine */
void Init(void);                    /* Init LED */
unsigned char testButton(unsigned char);

/*
 *  Global defines
 */

/* 
 *  The idea is to generate a PWM at 50 Hz with small steps to adjust the
 *  duty cycle.
 *  The Servomotor accept pulses between 380 uSec and 2.320 mSec.
 *  The frequency of the PWM (usually 50 Hz) set the actuation speed and the feedback
 *  response, so is better to have it set to 50 Hz.
 */
#define TMRVALUE 20              /* Timer will generate a interrupt every .01 ms */
#define PWM1_MAXSTEP  2000       /* Maximum step for the PWM 1 */
#define PWMINITIALVALUE 36       /* Initial value duty cicle (generate 380 uSec) */
#define POSIT1          78       /* Initial value for positioning the arm */
#define POSIT2          161      /* Ending value for positioning the arm */
#define SPEED          3000         /* Delay (in Seconds) for the activation */

#define FALSE         0
#define TRUE          1
/*
 *  State machine states
 */
#define RESET         0
#define POSIT         1
#define POSITOLD      2
#define MOVINGUP      3
#define MOVINGDOWN    4
#define WAITINGUP     5
#define WAITINGDOWN   6
#define INCREASE      7
#define DECREASE      8

/* I/O defines */

#define S1_BUTTON 0
#define S2_BUTTON 1

#define LED_ON  P1OUT |= BIT0
#define LED_OFF P1OUT &= ~BIT0
#define TEST_LED P1OUT & BIT0

#define PWM1_ON  P1OUT |= BIT2
#define PWM1_OFF P1OUT &= ~BIT2


/*
 *  Global variables
 */
unsigned short Pwm1_reach;      /* PWM 1 position to reach */
unsigned short Pwm1_dc;         /* PWM 1 output ducty cycle */
unsigned short Pwm1_cn;         /* PWM 1 counter */
unsigned char  Pwm1_State;      /* PWM 1 state machine */
unsigned short Pwm1_delay;      /* Used for PWM 1 rekated delay */
/*
 *  Main entry file
 */
void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer

  Init(); 

  for(;;)
  {
     /*
      *  The two pushbutton have different purposes.
      *  S1 change state
      *  S2 change value
      *
      *  States (at the start the state is the RESET)
      *    RESET  - pressing S2 the duty cycle is set at initial value (for the motor)
      *    POSIT  - pressing S2 the servo is positioned on one of the two work positions - with delay
      *    POSITOLD  - pressing S2 the servo is positioned on one of the two work positions - no delay
      *    INCREASE - pressing S2 the duty cycle is increased by one
      *    DECREASE - pressing S2 the duty cycle is decreased by one
      */
     if(testButton(S1_BUTTON))    /* Check S1 pushbutton */
     {
        /*
         *  Button pressed
         */

        switch(Pwm1_State)
        {
           case RESET:
           default:
              Pwm1_State = POSIT;
              break;
              
           case POSIT:
              Pwm1_State = POSITOLD;
              break;

           case POSITOLD:
              Pwm1_State = INCREASE;
              break;
              
           case INCREASE:
              Pwm1_State = DECREASE;
              break;

           case DECREASE:
              Pwm1_State = RESET;
              break;

           case MOVINGUP:
           case MOVINGDOWN:
           case WAITINGUP:
           case WAITINGDOWN:
              /*
               *  Ignore it - do nothing
               */
              break;
        }
     }   
        
     if(testButton(S2_BUTTON) || \
        Pwm1_State == MOVINGUP || \
        Pwm1_State == MOVINGDOWN || \
        Pwm1_State == WAITINGUP || \
        Pwm1_State == WAITINGDOWN )    /* Check S2 pushbutton */
     {
        /*
         *  Button pressed
         */
        switch(Pwm1_State)
        {
           case RESET:
           default:
              Pwm1_dc = PWMINITIALVALUE;  /* Set default PWM 1 value */
              break;
              
           case POSIT:
              /*
               *  Assign the reaching goal and the direction (increment
               *  or decrement)
               */
              if(Pwm1_dc == POSIT1)
              {
                Pwm1_reach = POSIT2;
                Pwm1_State = MOVINGUP;
              }
              else if(Pwm1_dc < POSIT1)
              {
                Pwm1_reach = POSIT1;
                Pwm1_State = MOVINGUP;
              }
              else if(Pwm1_dc > POSIT1)
              {
                 if(Pwm1_dc == POSIT2)
                 {
                   Pwm1_reach = POSIT1;
                   Pwm1_State = MOVINGDOWN;
                 }   
                 else if(Pwm1_dc > POSIT2)
                 {
                   Pwm1_reach = POSIT2;
                   Pwm1_State = MOVINGDOWN;
                 }   
                 else if(Pwm1_dc < POSIT2)
                 {
                   Pwm1_reach = POSIT2;
                   Pwm1_State = MOVINGUP;
                 }   
              }   
              break;

           case POSITOLD:
              if(Pwm1_dc != POSIT1)
                Pwm1_dc = POSIT1;
              else
                Pwm1_dc = POSIT2;
              break;
              
           case INCREASE:
              if(Pwm1_dc < PWM1_MAXSTEP)
                 Pwm1_dc +=1;
              break;

           case DECREASE:
              if(Pwm1_dc > 0)
                Pwm1_dc -= 1;
              break;

           case MOVINGUP:      
              if(Pwm1_dc == Pwm1_reach)
                Pwm1_State = POSIT;
              else
              {  
                Pwm1_State = WAITINGUP;
                Pwm1_delay = SPEED;
                Pwm1_dc++;
              }  
              break;

           case MOVINGDOWN:              
              if(Pwm1_dc == Pwm1_reach)
                Pwm1_State = POSIT;
              else
              {  
                Pwm1_State = WAITINGDOWN;
                Pwm1_delay = SPEED;
                Pwm1_dc--;
              }  
              break;             

           case WAITINGUP:              
              if(Pwm1_delay == 0)
                Pwm1_State = MOVINGUP;
              break;             

           case WAITINGDOWN:              
              if(Pwm1_delay == 0)
                Pwm1_State = MOVINGDOWN;
              break;             
        }
     }   
     
     /* 
      *  Debug !
      *  The servomotor has a very precise range :
      *  Between 380 uSec and 2.320 ms, i.e. 36 steps to 220
     */
     if(Pwm1_dc >= 36 && Pwm1_dc <= 220 )
       LED_ON;
     else
       LED_OFF;
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
  DCOCTL  = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;       /* Set up 16 Mhz using internal calibration value */
  BCSCTL2 = DIVS_3;             /* SMCLK divided by 8 */

  /*
   *  Set I/O
   *  P1.0 -> I/O output - original LED on the board
   *  P1.2 -> Timer_A output - PWM out
   *  P2.6 <- input  - button 1 - Increase
   *  P2.7 <- input  - button 2 - Decrease
   */
  P1OUT = 0;                  /* Force out low */

  P1SEL &= ~OUT;              /* Set P1.2 on I/O port */
  P1SEL |= BIT4;              /* Set P1.4 on SMCLK for test */
  
  P2SEL &= ~BIT6;
  P2SEL &= ~BIT7;

  P1DIR |= 0x1F;              /* Set P1.0, P1.1 output direction */
  P2DIR  = 0x00;              /* Leave P2.6 and P2.7 in input direction */

  /*
   *  Set variables
   */

  Pwm1_cn = 0;                /* Reset virtual PWM 1 counter */
  Pwm1_dc = PWMINITIALVALUE;  /* Set default PWM 1 value */
  Pwm1_reach = PWMINITIALVALUE;
  Pwm1_State = RESET;
  
  /*
   *  Set Timer
   */
  CCTL0 = CCIE;               /* CCR0 interrupt enabled */
  TACTL = TASSEL_2 + MC_1;    /* SMCLK, up mode */
  TACCR0 = TMRVALUE;          /* Approx 13uS (26uS / 2) */

  /*  NORMAL MODE */
  TACCTL0 &= ~0x0080;         /* Disable Out0 */

  _BIS_SR(GIE);               /* Enable interrupt */
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

/**
 * Timer_A
 * @brief Timer A0 interrupt service routine
 *
 * This function handle the Timer A interrupt, in order to perform the
 * physical transmission of the message.
 *
 * The function is based on a state machine in order to optimize the 
 * operations, so to don't have too long operations under interrupt.<br>
 * The timer is set to generate an interrupt every 13uSec, so to be able
 * to generate on the output pin (if enabled) the 38kHz frequency.<br>
 * Every MSTICK interrupts we have 1 ms timeframe, used to drive the
 * message sending.
 *
 * @param none 
 * @return None
 */
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A( void )
{
  if(Pwm1_delay)
    Pwm1_delay--;
  
  if(Pwm1_cn < PWM1_MAXSTEP)
  {
    Pwm1_cn++;
    if(Pwm1_cn <= Pwm1_dc)
      PWM1_ON;
    else
      PWM1_OFF;
  }  
  else
  {
    /*
     *  Expired counter
     *  Reload counters - force starting output
     */
    Pwm1_cn = 0;
    PWM1_OFF;   
  }  

}

/*
 *  This code is documented using DoxyGen 
 *  (http://www.stack.nl/~dimitri/doxygen/index.html)
 */



