/**
 *  @file rf_motor.c
 *  @brief Handlimg the servomotor for the RangeFinder (rf) using PWM with a 
 *  MSP430F2012 Texas Instrument microcontroller
 *  @author Stefano B.
 *  @version 01 beta
 *  @date June 2008
 *  @details This code is designed to handle a servocontroller using a PWM 
 *  signal
 *
 *  Version features
 *  A pushbutton and a input (in parallel) allow to select one of the two
 *  position in order tomove the RangeFinder up and down.
 *  The PWM is generated via an interrupt function triggered by the timer.
 *  Is NOT used the PWM capability of the timer.
 *
 *  The timer will be set in UP mode (i.e. counting up to the value in CCR0).
 *  The timer will generate an interrupt every .01 ms
 *  Internal management (SW counter) will generate the PWM outputs.
 *
 *  Pinout  PCB Pin   Mode  Description
 *  P1.0      P2      Out   Debug LED - general purpose
 *  P1.1      P3      Out   Unused                                   
 *  P1.2      P4      Out   PWM1 output - servomotr control
 *  P1.3      P5      Out   Unused
 *  P1.4      P6      SMCLK Debug - report the SMCLK frequency
 *  P1.5      P7      Out   Test - indicate a RF detection
 *  P1.6      P8      In    Remote Input - signal from RF receiver
 *  P1.7      P9      Out   Unused
 *  P2.6      P13     In    Pushbutton S1
 *  P2.7      P12     In    Unused
 *
 *  Built with IAR Embedded Workbench Version: 3.40A
 */

#include <string.h>
#include "msp430x20x2.h"

/*
 *  Functions prototype
 */
__interrupt void Timer_A (void);    /* Timer A0 interrupt service routine */
__interrupt void Port1_isr(void);       /* Port 1 I/O interrupt */
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
#define TMRVALUE        160      /* Timer will generate a interrupt every .01 ms */
#define PWM1_MAXSTEP    2000     /* Maximum step for the PWM 1 */
#define PWMINITIALVALUE 161      /* Initial value duty cicle (generate 380 uSec) */
#define POSIT_START     161       /* Initial value for positioning the arm */
#define POSIT_END       78      /* Ending value for positioning the arm */
#define SPEED           3000     /* Delay for the activation (see note) */

#define COUNTHIGH       625     /* Values to recognize a frequency */
#define COUNTLOW        624
#define COUNTOLER       10        /* Tolerance on the signal (+/-) */

#define VALIDATE_RF     10       /* Validate delay - long delay - .01 sec */
#define WAITEND_RF      10       /* Validate delay - long delay - .01 sec */
#define IGNORE_RF       1000     /* ms when the signal must be ignore = 1 s */

#define PRESCALER       100      /* Value to obtain a 1 ms timing */
/*
 *  Note about the SPEED define.
 *  This define is used to load a counter, decremented in the timer interrupt.
 *  So every .01 mS the counter is decremented.
 *  With the value of 3000, we will have a .03 Secs of delay between every increment or
 *  decrement of the duty cycle.
 *  The SPEED value is multiplied fo .01 mS, so : 3000 * 0.01 ms = 0.03 Sec
 */
#define FALSE         0
#define TRUE          1
/*
 *  State machine states
 */
#define POSIT         0
#define MOVINGUP      1
#define MOVINGDOWN    2
#define WAITINGUP     3
#define WAITINGDOWN   4

#define IDLE          0
#define DETHIGH       1
#define DETLOW        2
#define DETEND        3

#define VALIDATE      1
#define WAITDETEND    2
#define IGNORE        3

/* I/O defines */

#define S1_BUTTON 0
#define S2_BUTTON 1

#define LED_ON  P1OUT |= BIT0
#define LED_OFF P1OUT &= ~BIT0
#define LED_TOGGLE P1OUT ^= BIT0

#define PWM1_ON  P1OUT |= BIT2
#define PWM1_OFF P1OUT &= ~BIT2

/*
 *  Debug pins
 */
#define TEST0_ON  P1OUT |= BIT3
#define TEST0_OFF P1OUT &= ~BIT3
#define TEST0_TOGGLE P1OUT ^= BIT3

#define TEST_ON  P1OUT |= BIT5
#define TEST_OFF P1OUT &= ~BIT5
#define TEST_TOGGLE P1OUT ^= BIT5


/*
 *  Global variables
 */
unsigned short Pwm1_reach;      /* PWM 1 position to reach */
unsigned short Pwm1_dc;         /* PWM 1 output ducty cycle */
unsigned short Pwm1_cn;         /* PWM 1 counter */
unsigned char  Pwm1_State;      /* PWM 1 state machine */
unsigned short Pwm1_delay;      /* Used for PWM 1 rekated delay */

unsigned char RfDetState;       /* State machine for detection */
unsigned short RfDetCounter;     /* Counter for RF detection */
unsigned char RfDetected;       /* Flag to report the RF status 1= RF present */
unsigned char RfDetConfirmSt;   /* Confirmation state */
unsigned short RfPrescaler;     /* Prescaler for long delays */
unsigned short RfLongDelay;     /* Counter for long delay - 1000 = 1 Sec. (1 ms - 65 sec) */
unsigned short RfShortDelay;    /* Counter for short delay - 1000 = 0.01 Sec */


unsigned char Command;          /* Equivalent to the pushbutton */

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
      *  Validation for RF command.
      *
      *  States (at the start the state is the POSIT)
      *    IDLE - detecting a valid receiving command 
      *    VALIDATE - verify a valid signal if persist for 5 ms
      *    WAITDETEND - waiting for the detect command to expire
      *    IGNORE - ignore any activity for a specified time
      *
      *  The RF detection assume a ON condition if the signal is correctly received
      *  for at least 5 ms. Then waits that the signal cease before to assume is ended.
      *  At the end of the cycle detection the system ignore any activity for at least 100 ms
      */
     switch(RfDetConfirmSt)
     {
        case IDLE:
           if(RfDetected && Pwm1_State == POSIT)
           {
              RfLongDelay = VALIDATE_RF;
              RfDetConfirmSt = VALIDATE;
           }   
           break;

        case VALIDATE:
           if(RfDetected)
           {
              if(!RfLongDelay)
              {
                 /*
                  *  RF command detected !
                  *  Notify that, then load the counter for the wait end
                  */
                 LED_ON;
                 RfLongDelay = WAITEND_RF;
                 RfDetConfirmSt = WAITDETEND;
              }
           }
           else
              RfDetConfirmSt = IDLE;
           break;
           
        case WAITDETEND:   
           if(!RfDetected)
           {
              if(!RfLongDelay)
              {
                 /*
                  *  RF command ceased !
                  *  Notify that, then load the counter for the ignore state
                  */
                 LED_OFF;
                 Command = TRUE;
                 RfDetConfirmSt = IGNORE;
                 RfLongDelay = IGNORE_RF;  /* Reload timer */
              }
           }
           else
              RfLongDelay = WAITEND_RF;  /* Reload timer */
           
           break;

        case IGNORE:   
           if(!RfLongDelay)
           {
              RfDetConfirmSt = IDLE;
              Command = FALSE;
           }
           break;
           
        default:
          RfShortDelay   = 0;
          RfLongDelay    = 0;
          RfDetConfirmSt = IDLE;
          break;
     }   

     /*
      *  The pushbutton or RF trigger the arm movement - other states keep the engine running
      *
      *  States (at the start the state is the POSIT)
      *    POSIT  - pressing the pushbutton the servo is positioned on one of the two work positions - with delay
      *    MOVINGUP - the duty cycle is increasing
      *    MOVINGDOWN - the dutycycle is decreasing
      *    WAITINGUP - wait for moving up
      *    WAITINDOWN - wait for moving down
      */
     if(testButton(S2_BUTTON) || \
        Command == TRUE || \
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
           default:
           case POSIT:
              Command = FALSE;    /* Reset the RF command */
              /*
               *  Assign the reaching goal and the direction (increment
               *  or decrement)
               */
              if(Pwm1_dc == POSIT_START)
              {
                Pwm1_reach = POSIT_END;
              }
              else if(Pwm1_dc < POSIT_START)
              {
                Pwm1_reach = POSIT_START;
              }
              else if(Pwm1_dc > POSIT_START)
              {
                 if(Pwm1_dc == POSIT_END)
                 {
                   Pwm1_reach = POSIT_START;
                 }   
                 else if(Pwm1_dc > POSIT_END)
                 {
                   Pwm1_reach = POSIT_END;
                 }   
                 else if(Pwm1_dc < POSIT_END)
                 {
                   Pwm1_reach = POSIT_END;
                 }
              }

              if(Pwm1_reach > Pwm1_dc)
                 Pwm1_State = MOVINGUP;
              else
                 Pwm1_State = MOVINGDOWN;
              break;

           case MOVINGUP:      
              if(Pwm1_dc == Pwm1_reach)
              {
                Command = FALSE;    /* Reset the RF command */
                Pwm1_State = POSIT;
              }  
              else
              {  
                Pwm1_State = WAITINGUP;
                Pwm1_delay = SPEED;
                Pwm1_dc++;
              }  
              break;

           case MOVINGDOWN:              
              if(Pwm1_dc == Pwm1_reach)
              {
                Command = FALSE;    /* Reset the RF command */
                Pwm1_State = POSIT;
              }  
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
  }
}

/*
 *  Init
 * @brief Initclock, timer and initialize variables
 *
 * @param none
 * @return None
 */
void Init(void)
{
  WDTCTL = WDTPW + WDTHOLD;     /* Stop watchdog timer */

  /*
   *  Set DCO
   */
  DCOCTL  = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;       /* Set up 16 Mhz using internal calibration value */

  /*
   *  Set I/O
   *  P1.0 -> I/O output - original LED on the board
   *  P1.2 -> Timer_A output - PWM out
   *  P2.6 <- input  - button 1 - Increase
   *  P2.7 <- input  - button 2 - Decrease
   */
  P1OUT = 0;                  /* Force out low */

  P1SEL &= ~BIT6;             /* Set P1.6 for normal I/O */
//  P1SEL |= BIT6;              /* Set P1.6 for CCR1 timer_a capture */
  P1SEL |= BIT4;              /* Set P1.4 on SMCLK for test */
  
  P2SEL &= ~BIT6;
  P2SEL &= ~BIT7;

  P1DIR |= 0xBF;              /* Set P1.6 input, the rest in OUTPUT */
  P2DIR  = 0x00;              /* Leave P2.6 and P2.7 in input direction */

  P1REN |= BIT6;              /* Enable Pull-up/Down resistor on P1.6 - P1OUT is 0 so is a pull-down */
  P1IES &= ~BIT6;             /* Set P1.6 interrupt generation on the low-to-high transition */  
  P1IE |= BIT6;               /* Enable interrupt on P1.6 */
  /*
   *  Set variables
   */
  Pwm1_cn        = 0;                /* Reset virtual PWM 1 counter */
  Pwm1_dc        = PWMINITIALVALUE;  /* Set default PWM 1 value */
  Pwm1_reach     = PWMINITIALVALUE;
  Pwm1_State     = POSIT;

  RfDetState     = IDLE;
  RfDetCounter   = 0;
  RfDetConfirmSt = IDLE;
  RfDetected     = FALSE;
  RfPrescaler    = PRESCALER;
  RfShortDelay   = 0;
  RfLongDelay    = 0;
  
  Command = FALSE;
  
  /*
   *  Set Timer
   */
  TACTL = TASSEL_2 + MC_1;    /* Uses SMCLK, count in up mode */
  TACCTL0 = CCIE;             /* Use TACCR0 to generate interrupt */
  TACCR0 = TMRVALUE;          /* Approx .01 ms */

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
 * This function handle the Timer A interrupt, in order to detect the RF
 * signal and drive the PWM output
 *
 * The functions are based on a state machine in order to optimize the 
 * operations, so to don't have too long operations under interrupt.<br>
 * The timer is set to generate an interrupt every 0.01 ms.
 *
 * @param none 
 * @return None
 */
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A( void )
{
   if(Pwm1_State == POSIT)
   {  
      /*
       *  RF detection only if the servomotor is not moving
       *  This parts works together with the I/O interrupt in order to
       *  recognize a specific incoming frequency
       */
      switch(RfDetState)
      {
         case IDLE:
         default:
           /*
            *  Do nothing - waiting from the I/O interrupt to start the
            *  detection sequence
            */
           break;

         case DETHIGH:
            /* 
             *  Verify that the signal stays high at least COUNTHIGH reads
             *  (minus COUNTOLER reads)
             */
            TEST_ON;
            if(P1IN & BIT6)
            {
               if(RfDetCounter < COUNTHIGH)
                  RfDetCounter++;
               else
               {
                  RfDetState = DETLOW;
                  RfDetCounter = 0;
               }   
            }  
            else
            {
               if(RfDetCounter >= (COUNTHIGH - COUNTOLER))
               {
                 RfDetState = DETLOW;
                 RfDetCounter = 0;
               }
               else
               {
                 RfDetState = DETEND;
                 RfDetected = FALSE;
               }  
            }
            TEST_OFF;
            break;

         case DETLOW:
            /* 
             *  Verify that the signal stays low at least COUNTLOW reads
             *  (minus COUNTOLER reads)
             */
           TEST_ON;
           if(!(P1IN & BIT6))
           {
              if(RfDetCounter < COUNTLOW)
                 RfDetCounter++;
              else
              {
                 RfDetState = DETEND;
                 RfDetCounter = 0;
                 RfDetected = TRUE;
              }   
            }  
            else
            {
               if(RfDetCounter >= (COUNTLOW - COUNTOLER))
               {
                 RfDetState = DETEND;
                 RfDetCounter = 0;
                 RfDetected = TRUE;
               }
               else
               {
                  RfDetState = DETEND;
                  RfDetected = FALSE;
               }   
            }
           TEST_OFF;
           break;
        case DETEND:
           /*
            *  Wait for the next raise before to re-enable the interrupt
            */
           if(P1IN & BIT6)
           {        
             RfDetState = IDLE;
             P1IE |= BIT6;         /* Reenable the P1.6 interrupt */
           }  
           break;
     }      
   }


  /*
   *  Delay management
   *  This counter is used in the states WAITUP and WAITDOWN in order to
   *  create a delay in the PWM generation
   */
  if(Pwm1_delay)
    Pwm1_delay--;

  /*
   *  Delay management for detection confirmation
   */
  if(RfShortDelay)
    RfShortDelay--;

  /*
   *  Prescaler management for long delays
   */
  if(RfPrescaler)
    RfPrescaler--;
  else
  {
    RfPrescaler = PRESCALER;
    if(RfLongDelay)
      RfLongDelay--;
  }  
  
  /*
   *  PWM 1 management
   *  This part of the code is handling the PWM 1 generation
   */
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

/**
 * I/O Port 1 
 * @brief I/O port 1 interrupt service routine
 *
 * This function handle the I/O Port 1 interrupt.
 *
 * @param none 
 * @return None
 */
#pragma vector=PORT1_VECTOR
__interrupt void Port1_isr(void)
{
  if(P1IFG & BIT6)
  {  
    /*
     *  Interrupt on Pin 1.6 ! 
     *  Be sure is not a spike !
     */
    if(P1IN & BIT6)
    {  
       RfDetState = DETHIGH;     /* Change detection state machine */
       RfDetCounter = 0;         /* Reset counter */
       P1IE &= ~BIT6;            /* Disable interrupt */
    }   

    P1IFG &= ~BIT6;  /* Reset I/O interrupt on P1.6 */
  }  
} 
/*
 *  This code is documented using DoxyGen 
 *  (http://www.stack.nl/~dimitri/doxygen/index.html)
 */



