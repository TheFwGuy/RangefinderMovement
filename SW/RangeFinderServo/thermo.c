/**
 *  @file thermo.c
 *  @brief Internal thermometer management via ADC
 *  @author Stefano B.
 *  @version 01. beta
 *  @date August 2007
 *  @details This module will initialize the ADC and read the internal
 *  temperature senseor.<br>
 *  The code is directly derived from a Texas Instrument example
 *
 *  I/O Pin used on the MSP430F2012 for the temperature 
 *
 *  none
 *
 */

#include "msp430x20x2.h"
#define CELSIUS
#define AVTEMP        5             /* Number of reading for the temperature */


/**
 * ADC_Init
 * @brief Initialize the ADC
 *
 * This function initialize the ADC in order to read the internal temperature
 * sensor.
 *
 * @param None 
 * @return None
 */
void 
ADC_Init(void)
{
  ADC10CTL1 = INCH_10 + ADC10DIV_1;         /* Temp Sensor ADC10CLK/2 */
  ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
  __enable_interrupt();                     /* Enable interrupts. */
}

/**
 * Read_Temp
 * @brief Read the temperature
 *
 * This function read the internal temperature sensor.
 *
 * @param None 
 * @return The reading of the temperature
 */
long 
Read_Temp(void)
{
   char index;
   long temp = 0;
      
   for(index=0; index<AVTEMP; index++)
   {  
      ADC10CTL0 |= ENC + ADC10SC;         /* Sampling and conversion start */
      __bis_SR_register(CPUOFF + GIE);    /* LPM0 with interrupts enabled */
      
      temp += ADC10MEM;
      __no_operation();                    
   }   

   temp /= AVTEMP;
   
#if defined(CELSIUS)    
   /* oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278 */
   temp = ((temp - 673) * 423) / 1024;
#else
   /* oF = ((A10/1024)*1500mV)-923mV)*1/1.97mV = A10*761/1024 - 468 */
   temp = ((ADC10MEM - 630) * 761) / 1024;
#endif
   __no_operation();                       
   
   return(temp);
}

/* ADC10 interrupt service routine */
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
  __bic_SR_register_on_exit(CPUOFF);    /* Clear CPUOFF bit from 0(SR) */
}

/**
 * xtoa
 * @brief String conversion function
 *
 * This function convert a number into a string.
 * Used for ltoa
 *
 * @param val  Value to convert
 * @param *buf Pointer to the buffer where to write the result
 * @param radix  Conversion radix (es. 10 for decimal)
 * @param negative set to 1 if the number is negative
 * @return None
 */
static void 
xtoa(unsigned long val, char *buf, unsigned radix, int negative)
{
  char *p;
  char *firstdig;
  char temp;
  unsigned digval;

  p = buf;

  if (negative) 
  {
    // Negative, so output '-' and negate
    *p++ = '-';
    val = (unsigned long)(-(long) val);
  }

  // Save pointer to first digit
  firstdig = p;

  do 
  {
    digval = (unsigned) (val % radix);
    val /= radix;

    // Convert to ascii and store
    if (digval > 9)
      *p++ = (char) (digval - 10 + 'a');
    else
      *p++ = (char) (digval + '0');
  } while (val > 0);

  // We now have the digit of the number in the buffer, but in reverse
  // order.  Thus we reverse them now.

  *p-- = '\0';
  do 
  {
    temp = *p;
    *p = *firstdig;
    *firstdig = temp;
    p--;
    firstdig++;
  } while (firstdig < p);
}

/**
 * itoa
 * @brief Integer to String conversion function
 *
 * This function convert an integer number into a string.
 *
 * @param val  Value to convert
 * @param *buf Pointer to the buffer where to write the result
 * @param radix  Conversion radix (es. 10 for decimal)
 *
 * @return pointer to the converted buffer
 */
char 
*itoa(int val, char *buf, int radix)
{
  if (radix == 10 && val < 0)
    xtoa((unsigned long) val, buf, radix, 1);
  else
    xtoa((unsigned long)(unsigned int) val, buf, radix, 0);

  return buf;
}

/**
 * ltoa
 * @brief Long to String conversion function
 *
 * This function convert a long number into a string.
 *
 * @param val  Value to convert
 * @param *buf Pointer to the buffer where to write the result
 * @param radix  Conversion radix (es. 10 for decimal)
 *
 * @return pointer to the converted buffer
 */
char 
*ltoa(long val, char *buf, int radix)
{
  xtoa((unsigned long) val, buf, radix, (radix == 10 && val < 0));
  return buf;
}

