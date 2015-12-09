#include <18F45K20.h>
#device ICD=TRUE
#device adc=10

#FUSES WDT                      //Watch Dog Timer
#FUSES WDT8                     //Watch Dog Timer uses 1:8 Postscale (32ms)
#FUSES INTRC_IO                 //Internal RC Osc, no CLKOUT
#FUSES PUT                      //Power Up Timer
#FUSES BROWNOUT_NOSL            //Brownout enabled during operation, disabled during SLEEP
#FUSES BORV30                   //Brownout reset at 3.0V
#FUSES NOPBADEN                 //PORTB pins are configured as digital I/O on RESET
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES DEBUG                    //Debug mode for use with ICD

#use delay(int=64000000,RESTART_WDT)

#use FIXED_IO( D_outputs=PIN_D1,PIN_D0 )
#use STANDARD_IO(C)
#use rs232(baud=115200,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,restart_wdt)

