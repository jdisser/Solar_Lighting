//A light for dark places, when all other lights go out.

/*
First run with data logging
   clouds are causing batdata.cmins to reset losing acc charge
   algorithm not able to follow low light
      possible change in mpp? follow mpp not fixed voltage



*/
#include <SSL1000.h>

  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>
  




//ADC channel assignments for 18F45K20
#define VCELL  4        //solar cell voltage/4 on pin A5 
#define VFBAT  0        //filtered battery voltage on pin A0 (was 3)
#define IBAT   1        //~100mv/ma voltage on pin A1
#define VREF12  15       //channel 15 is 1.2V internal reference
#define TBATC  2        //sensor connected to A2 
#define TAMBC  3        //sensor connected to A3
#define IPNL   7        //voltage gain10 on pin E2 AN7









//PIN definition

#define LED       PIN_C1   //Lamp drive pin
#define BATDRV    PIN_C2   //battery switcher drive pin
#define BUTTON    PIN_B0   //button connected to portb.0
#define DISP0     PIN_D0   //display LED
#define DISP1     PIN_D1   //display LED

//constants
//voltage units are in mv
//current units are in ma/10
//timer loops are 33ms

#define nCELL        4                 //number of nimh cells in pack

#define BATOVRV      nCELL*1800           //charge shutdown battery voltage
#define BATFULV      nCELL*1200           //safe to charge if < 1.2/cell
#define BATLOV       nCELL*1000           //stop discharge here
//there is a voltage level here of n*.200 for BATDEAD state
#define BATMISSING   nCELL*50             //no battery present

#define DIODEV       200               //diode voltage
#define PWMDIODEV    450               //diode drop higher with more current

                                       
#define MCHGPV       17000             //voltage for maximum charge rate                                       

#define MPDI         1500              //maximum ma reduction in charge current setpoint
                                       //if too hi may disrupt EOC detection

#define ARRAYSIZE    4                 //this is the declared size = # members


#define DUTYCYL100   200*4             //100% duty cycle (200 ~= 80KHZ) *4 for 10 bit PWM value
#define DUTYMAX      (6*DUTYCYL100)/10 //maximum duty cycle for circuit
#define DUTYMIN      (1*DUTYCYL100)/10 //Vo < Vbat with worst case voltages in simulation
#define DEADBAND     75                //no change if delt <= deadband
#define PROPBAND     300               //integral changes if absdelt < PROPBAND
#define TESTVOLT     2250              //setpoint for test code
#define VOMAX        BATOVRV           //maximum output voltage
#define VOMIN        200*nCELL         //minimum output voltage(also dead bat V)
#define VINMIN       LOPV              //minimum input (from cell)
#define VINMAX       26000             //maximum input (from cell)
#define CMINLIM      50000             //MAmin charge limit (700mah*60*1.2)
#define CMINSTART    15*FASTCHG/10     //ignore initial voltage peak 10min 
#define DELTAQ       2*FASTCHG/10-100  //minutes between dv readings, < to insure trigger
#define TRICKLE      700               //70ma trickle rate
#define FASTCHG      5000              //500ma fast charge rate
#define DELTAVLIM    30*nCELL          //dV to trigger end of charge 
#define DELTAILIM    100               //maximum current deviation
#define DELTAPVLIM   50                //maximum panel voltage differential
#define PVSTEP       2*DELTAPVLIM      //step>lim
#define ISTART       200               //start current 
#define TSLOPE       10                //slope in mv/Tcentigrade
#define MAXDELTAT    15*TSLOPE         //maximum temperature differential
#define BATDIVFAC    2                 //correction factor for divider
#define REGTIMER     10                //tick timer for chginreg state
#define MPSTEP       10                //current step in max pwr procedure
#define ISTEP        5                 //step change in current ramp
#define PVSCANSTEP   250               //step size in panel voltage for scan
#define SCANSIZE     16                //scan volt rng is SCANSIZE*PVSCANSTEP


#define MPPCYLTIME   4                 //cyls between mppd adjustments
#define INREGLIM     MPPCYLTIME/2      //counter limit for in regulation during mpp cyl
#define DPLIM        500               //change in panel power to trigger a new scan




//#define LOBATVREF    (600/BATLOV*1025)    //BATV>2.3 -> VDD=BATV-200 ->AD < LOBATREF
                                          //vpanel<DARKPV
                                          
#define VDDCONST     ((1024*600)/2)       //constant for Vdd reading 

#define SETTLETIME    10                 //#of loops for PS to settle must be > ARRAYSIZE
#define CHGHOLDTIME   16                  //cyls before voltage check in hold
#define TICKSPERSEC   30                  //ticks per second
#define BUTDEBOUNCE   3                   //timer loops to debounce button
#define BUTHOLD       15                  //loops to sense button held
#define TUNULL        0                   //enumeration of timer events
#define TUSEC         1
#define TUMIN         2
#define TUHOUR        3
#define LOADTIME      50                  //minimum number of load minutes to next recharge

#define PARTIAL       1                   //used in Init_Batdata to select initialized variables

//global variable declarations


//state variables
enum states{INIT,RESET,IDLE,CHARGE,LIGHT,FLASHING,SLEEP}state;
enum pnlstates{PVDRK,PVLO,PVNCHG,PVCHG}pnlstate;
enum batstates{BATGONE,BATDEAD,BATLO,BATNORM,BATFUL,BATOVR}batstate;
enum chgstates{CHGINIT,CHGINREG,CHGSTRT,SCANNING,CHARGING,CHGOFF,CHGDONE,CHGHOLD}chgstate;
enum scanstates{MPSETPOINT,MPSLEW,MPRECORD,MPTEST,MPMAX,MPFAULT,MPSTAB}scanstate;
enum scanerrs{PNLCRASH,DCLOW,DCHIGH,BILOW,BIHIGH,NOMAX,NOSTAB}scanerr;
enum slopes{POSITIVE,ZERO,NEGATIVE,PEAK,UNKNOWN}slope; //charge voltage slope was in batdata struct 02/01/12
enum ltstates{LTINIT,LTON,LTTIME,LTOFF}ltstate;
//enum tunits{TUNULL,TUSEC,TUMIN,TUHOUR};                            //global type
enum btnstates{BTNUP,BTNPRESS,BTNHOLD,BTN2PRESS}btnstate;

//analog variables
unsigned long dutycycle;
unsigned long vpanel,ipanel;
unsigned long mppd;
unsigned long LEDduty;
//unsigned int32 pnlpwr;

//timer variables
int debounce;
int ticks;



struct tstruct{                              //timer
   char tticks;
   char tsec;
   char tmin;
   char thour;
   char tevent;
}tdata;
char btntimer;

//flags
int1  pressed;          //flag indicates button pressed if TRUE
int1  inreg;            //FALSE if abs dI > DELTAILIM
int1  peakdetected;     //TRUE if charge state detected peak while inreg
                        //if true charge prevented until battery discharges
                        //for a minimum of LOADTIME
int1 flashlight;        //TRUE for flashing light
int1 overtemp;          //dT > MAXDELTAT
int1 maxmppd;           //the max pwr pt factor has been reached
int1 pnlvlow;           //Vp < Vnchg
int1 pnlinreg;          //Vp +/- DELTAPVLIM from pnldata.vpnlsp
int1 mpppvinreg;        //panel voltage stat counter monitor
int1 mppbiinreg;        //battery current stat counter monitor
int1 batiovr;           //TRUE if bati > batilim
int1 mppdpllim;         //dp low limit TRUE if dp<DPLIM
int1 targetinreg;       //the ramp function has reached target

//data structures and structure functions

struct Sdataobj{                             //Structured Data Object
   signed long    a[ARRAYSIZE];              //circular readings buffer
   signed long    da[ARRAYSIZE];             //circular delta buffer
   signed int32   sum;                       //integral of array
   signed long    deltasum;                  //sum of differentials
   signed long    slope;                     //R(n)-R(0)
   signed long    peak;                      //peak value since initialize
   signed long    min;                       //minimum value
   signed long    avg;                       //average of array
   signed long    last;                      //last array element added
   signed long    lastdelta;                 //last delta
   signed long    *indap;                    //buffer pointers
   signed long    *outdap;                   //
   signed long    *inap;                     //
   signed long    *outap;                    //
   char           i;                         //current element index
}batvsdo,pnlvsdo,btempsdo,batisdo,pnlisdo,pnlpsdo;   //batvoltage,pnlvoltage,battemp,batcurrent,panelcurrent,panelpwr

//SDO pointers



void SDO_Init(struct Sdataobj *sdop)         //initialize an SDO
{
   for(sdop->i = 0;sdop->i<ARRAYSIZE;(sdop->i)++)
      {
      sdop->a[sdop->i] = 0;
      sdop->da[sdop->i] =0;
      }
      
      sdop->sum = 0;
      sdop->deltasum =0;
      sdop->slope = 0;
      sdop->peak = 0;
      sdop->min = 0;
      sdop->avg = 0;
      sdop->last = 0;
      sdop->lastdelta=0;
      sdop->inap = sdop->a;
      sdop->outap = sdop->a;
      sdop->indap = sdop->da;
      sdop->outdap = sdop->da;
      sdop->i = 0;
}

char SDO_prvi(char i)   //returns the previous index in the array
{
   if(i>0)
      return (--i);
   else
      return ARRAYSIZE-1;
}

char SDO_nxti(char i)   //returns the next index (last in series)
{
   if(i<(ARRAYSIZE-1))
      return ++i;
   else
      return 0;
}


//function is passed an SDO object, a current reading, and optionally a setpoint
//the reading is stored and statistics are computed on a circular array of readings
//if a setpoint is passed the delta array will contain a series of setpoint differences
//if no setpoint or setpoint=0 is passed the delta array will contain a series of differentials

void SDO_AddReading(struct Sdataobj *sdop, signed long ae,signed long set=0)
{
   int j,k;                            //loop counter and indexer
   signed long temp,dtemp;
   
   sdop->i = SDO_nxti(sdop->i);              //point the index to next new element
                                       //note index normally points to current element

   sdop->inap = sdop->a + sdop->i;     //initialize the array in pointer
   *(sdop->inap)=ae;                   //store the new element
   
   sdop->last = ae;                    //store the latest element
   
   if(ae>sdop->peak)
      sdop->peak = ae;                 //store the peak

   sdop->indap = sdop->da + sdop->i;
   
   if(set != 0)
      *(sdop->indap)=ae-set;           //if setpoint store r(n)-set
   else
      *(sdop->indap)=ae-sdop->a[SDO_prvi(sdop->i)]; //else store r(n)-r(n-1)
      
   sdop->lastdelta = *(sdop->indap);
   
   sdop->sum = 0;
   sdop->deltasum=0;
   //sdop->max = ae;                  //assume latest reading is the extrema
   sdop->min = ae;
   sdop->outap = sdop->a;           //start the loop at the first elements
   sdop->outdap = sdop->da;
  
   
   for(j=0;j<=ARRAYSIZE-1;j++)
   {
     temp = *((sdop->outap)+j);
     dtemp = *((sdop->outdap)+j);
       sdop->sum += temp;               //add the array elements in sum
       sdop->deltasum += dtemp;
       
      
      if(temp < sdop->min)
         sdop->min = temp;
   }

   sdop->avg = sdop->sum/ARRAYSIZE;    //calc avg note:signed 32/const assigned to signed 16????
   
   k=sdop->i;                          //set the indexer to the new element
   sdop->slope = sdop->a[k]-sdop->a[SDO_nxti(k)]; //slope = new - last(next after new)

   
}

struct batstruct {
   unsigned long setpointi;                  //charge current setpoint in 1/10th ma
   unsigned long chargei;                    //charge current reading
   unsigned long batilim;                    //current limit
   //unsigned long testi;
   //signed long deltai;                       //reading - setpoint
   //unsigned long absdeltai;                  //absolute value of present delta
   //signed long deltaiarray[4];               //array of last n readings
   //char diai;                                //array indexs
   //signed long deltaisum;                    //sum of last 4 deltas
   unsigned long vbat;                       //battery voltage mv
   unsigned long vsetbat;                    //target output voltage mv
   unsigned long vbatpeak;                   //peak charge voltage
   unsigned long cmins;                      //total charge in ma-minutes
   unsigned long deltacmin;                  //cmins since last dv
   unsigned long csecsum;                    //holds tick based sum
//   char ticks,secs;                          //timers
//   enum slopes{POSITIVE,ZERO,NEGATIVE,PEAK,UNKNOWN}slope; //charge voltage slope note:IDE might not like this in a struct try moving 
   unsigned long lastvbat;
   signed long deltav;                       //dV per unit charge
   unsigned long dischgmin;                  //total minutes of load discharge
   unsigned long battemp;                    //mv diode sensor reading
   unsigned long battemppeak;                //max voltage (min temp)
   unsigned long ambtemp;                    //ambiant temperature
   signed long dambt;                        //differential temperature
}batdata;

struct pnlstruct {

   unsigned int32 netpower;                  //Vp*Ip pwr delivered to charge circuit note:calc here then demote for SDO
   unsigned int16 vpnlsp;                    //panel setpoint varies to drive power
   signed long    voc;                       //open circuit voltage
   signed long    vstart;                    //panel voltage at start of scan
   signed long    pscan[SCANSIZE];           //power values from scan
   int8           mppcyl;                    //cyl counter for pwr measurements
   int8           mpppvctr;                  //inc if pv !inreg
   int8           mppbictr;                  //inc if bi !inreg
   int8           pmaxindex;                 //index of maximum power
   int8           scanindex;                 //index for scanning

}pnldata;

void Clear_Pscan(void)
{
   int i;
   
   for(i=0;i<SCANSIZE-1;i++)
      pnldata.pscan[i]=0;
}

void Init_Pnldata (void)                     //prepare the structure for starting
{
   pnldata.netpower = 0;
   pnldata.vpnlsp = pnlvsdo.avg;
   pnldata.mppcyl = 0;
   pnldata.mpppvctr = 0;
   pnldata.mppbictr = 0;
   pnldata.vstart = 0;
   pnldata.voc = 0;
   Clear_Pscan();
   mpppvinreg=TRUE;
   mppbiinreg=TRUE;
   mppdpllim=FALSE;

}

/*
Sets the charge current variable mppd to control the panel voltage
returns 0 if no errors
returns 1 if mppd=maxcurrent
returns 2 if mppd=min current
*/
int SetMppd (unsigned long maxcurrent)
{
   int err;
   
   err=0;
   
   if(pnlvsdo.avg < pnldata.vpnlsp)
      {
      if(mppd > MPSTEP)
         mppd-=MPSTEP;
      else
         err=2;
      }
   else if (mppd<maxcurrent)
         mppd+=MPSTEP;
        else
         err=1;
         
   return err;

}


   



//function definitions
#int_RB
void  RB_isr(void) 
{

}

void SendData(void)
{

   unsigned long temp;
   int8 temp2;

   temp = tdata.thour *60;
   temp += tdata.tmin;
               
   temp2 = inreg? 1:0;
   
   printf("%5Ld,%4Ld,%4Ld,%4lu,%5Ld,%1u,%3Ld,%4Ld,%4Ld,%4Lu,%1u,%5Lu,%3Lu\n\r",
                           pnlvsdo.avg,
                           pnlisdo.avg,
                           batisdo.avg,
                           batdata.setpointi,
                           batisdo.lastdelta,
                           temp2,
                           btempsdo.avg,
                           batvsdo.avg,
                           batdata.deltav,
                           batdata.vbatpeak,
                           chgstate,
                           batdata.cmins,
                           temp);      //56characters ~4.5ms
}

char Debounce_Button(int1 dbinput, int dbtime)
{
   if(dbinput)
      {
      if(btntimer<254)
         ++btntimer;

      if(btntimer >= dbtime)
         {
//        btntimer = 0;
          return TRUE;
         }
      else
         return FALSE;
      }
   else
      {
      btntimer=0;
      return FALSE;
      }
}








void Clear_Timer(void)    //clears tdata structure and returns null
{
   tdata.tticks=0;
   tdata.tsec = 0;
   tdata.tmin = 0;
   tdata.thour = 0;
   
   tdata.tevent=TUNULL;
}

unsigned long Get_Timer(void)
{
   char temp;
   unsigned long templong;
   
   temp = TUNULL;
   templong = 0;
   
   if(tdata.tticks == TICKSPERSEC)
      {
      tdata.tticks = 0;
      if(tdata.tsec <= 59)
         {
         ++tdata.tsec;
         temp = TUSEC;
         }
         else
            {
            tdata.tsec = 0;
            if(tdata.tmin <= 59)
               {
               ++tdata.tmin;
               temp = TUMIN;
               }
               else
                  {
                  tdata.tmin = 0;
                  tdata.tsec = 0;
                  ++tdata.thour;
                  temp = TUHOUR;
                  }
            
            }

      }
      else
         ++tdata.tticks;
         
    tdata.tevent=temp;
    
    templong += tdata.tmin;
    
    templong += 60*tdata.thour;

    return templong;             //return total elapsed minutes
}




//configures and starts the PWM module with a variable duty cycle
void Start_PWM (unsigned long pwmduty)
{
  
   setup_timer_2(T2_DIV_BY_1,DUTYCYL100/4,1);    // use /4 for 8 bit timer value       
   set_pwm1_duty(pwmduty);                     //
   setup_ccp1(CCP_PWM);
   output_drive(BATDRV);                        //CCP1 pin C2 is battery drive           

}

void Start_PWM_LED (unsigned long pwmduty)
{
 
   setup_timer_2(T2_DIV_BY_1,DUTYCYL100/4,1);   // use /4 for 8 bit timer value     
   set_pwm2_duty(pwmduty);                     //
   setup_ccp2(CCP_PWM);
   output_drive(LED);                          //CCP1 pin C2 is battery drive           

}

//shuts off the battery pwm module and disables the output pin
void Stop_PWM (void)
{
  output_float(BATDRV);                   //disable the output pin
  setup_ccp1(CCP_OFF);                    //shutoff the PWM
}

//shuts off the LED pwm module and disables the output pin
void Stop_PWM_LED (void)
{
  output_float(BATDRV);                   //disable the output pin
  setup_ccp2(CCP_OFF);                    //shutoff the PWM
}

//returns a calibrated reaading in mv of the selected channel

unsigned long get_ADMV(char channel)     
{
    unsigned long cal1200mv,adresult,calresult;

   
   set_adc_channel(VREF12);
   delay_us(100);
   cal1200mv = read_adc(ADC_START_AND_READ);     //read the reference voltage channel
   set_adc_channel(channel);
   delay_us(100);
   adresult = read_adc(ADC_START_AND_READ);     //read the selected channel
   
   
   //calresult = 1200/cal1200mv = mv/step*16 
   calresult = 19200/cal1200mv;
   //calresult = channel steps*mv/step*16
   calresult = adresult*calresult;
   //calresult =  channel mv*16/16
   calresult = calresult/16;
   
   return(calresult);
   
}



//returns the battery charge current in ma/10
//i(ma/10)= mv/50
//shunt resistor changed to .5 and gain resistors changed to 5k+5K 013012
long get_BATI(void)                 
{
   unsigned long bati;
   
   bati = get_ADMV(IBAT);
   //bati /= 50;                    //10ohm shunt X5 gain
                       
   

   //bati = Ulongdivision(bati,5);   //return 1/10ths 
   bati *= 2;                       //013012
                                    //.5ohm shunt x10 gain returns 10ths
   return(bati);

}


//returns the duty cycle to produce a specified output from a specified input
//imposes limits on the input and output voltage settings
//imposes limits on the duty cycle period produced
unsigned long Calc_Duty(unsigned long voltagein, unsigned long voltageout)
{

   unsigned long tempduty;
/*  this logic is questionable since measurements are assumed to be valid 
   if(voltagein < VINMIN)
      voltagein = VINMIN;
   else if (voltagein > VINMAX)
      voltagein = VINMAX;
*/      
   if(voltageout < VOMIN)
      voltageout = VOMIN;
   else if (voltageout > VOMAX)
      voltageout = VOMAX;                            //limit the voltage ranges
   
   
   voltageout += PWMDIODEV;                           //numerator (vout + vd)
   voltagein += voltageout;                           //denominator (vout + vd + vin)
   voltageout /= 64;
   voltagein /= 64;                                   //scale to prevent overflow
   voltageout = voltageout*DUTYCYL100;                //include the max duty factor
   
   tempduty=voltageout/voltagein;
   
   if(tempduty < DUTYMIN)                          //limit the dutycycle
      tempduty=DUTYMIN;
   else if (tempduty > DUTYMAX)
      tempduty = DUTYMAX;
      
   return(tempduty);   

}

//read panel voltages and set panel state

#define DARKPV       1000              //<PVDARK   It's dark or no panel
#define LOPV         4500              //<PVLO     Circuit is on bat pwr
#define NOCPV        12000             //<PVNCHG   shut charging down below this

                                      

void Get_Panel_Status(void)
{
   vpanel = get_ADMV(VCELL);
   vpanel *= 11;                  //for -100K-10K- divider
   ipanel = get_ADMV(IPNL);
   ipanel /= 5;                     //.47Ohm shunt Gain 10 => 1ma/5mv
   
   SDO_AddReading(&pnlvsdo,vpanel,pnldata.vpnlsp);
   SDO_AddReading(&pnlisdo,ipanel);
   
   if(labs(pnlvsdo.lastdelta)>DELTAPVLIM)
      pnlinreg=FALSE;
   else
      pnlinreg=TRUE;
      
 
   if(state != CHARGE)
      pnldata.voc=pnlvsdo.avg;         //update open circuit voltage if panel not loaded
   else                                //calculate panel power when charging
      {
      pnldata.netpower = (int32)pnlvsdo.avg*(int32)pnlisdo.avg;
      pnldata.netpower /= 1000;                                 //calc avg pwr in mw
      SDO_AddReading(&pnlpsdo,pnldata.netpower);                //assigns int32 to sint16 !!!?
         
      }
   
//set the panel state
   if (pnlvsdo.avg<=DARKPV)
         pnlstate = PVDRK;        //circuit on battery power
   else if
      (pnlvsdo.avg<= LOPV)               
         pnlstate = PVLO;          //pnl is < Vreg
   else if
      (pnlvsdo.avg<= NOCPV)               
         pnlstate = PVNCHG;        //stop charging if panel voltage falls to here
   else
      pnlstate = PVCHG;            //panel in charge range

//set the low voltage flag

   if(pnlstate <= PVNCHG)
      pnlvlow = TRUE;
   else
      pnlvlow = FALSE;
}


void Getdeltav()    //compares batv to peak to detect -dv condition stored in slope
                    //this function runs after DELTAQ charge has been stored in the battery
                    //the result is dV/dQ (patentable????)
                    //note the SDO variables are a short term avg used to smooth the
                    //data used in the batdata variables that represent longer term variations
{

 
   
    batdata.deltav = batdata.vbat-batdata.lastvbat;
   

    batdata.lastvbat = batdata.vbat;                  //store the present reading for the next delta


//note: large dV/dQ might indicate end of charge possibly combined with a positive dT/dt


//slope based peak detection with temperature sensing

   if((batdata.cmins <= CMINSTART)||(!inreg))        //disable peak detection if not in regulation
      {
      batdata.vbatpeak = batdata.vbat;               //disable peak detection during start timer
      slope = ZERO;                                  //slope is neutral during the initial timer
      }
   else
      {
        if(batdata.vbat < batdata.vbatpeak)            //PEAK slope indicates end of charge cycle
        {
          if(labs(batdata.vbatpeak - batdata.vbat)>DELTAVLIM)
            {
               slope = PEAK;                               //needs testing at various currents and T's & dT's
               peakdetected = TRUE;                        //this flag gets cleared after load runs for min. time
            }                                              //to prevent rentry and overcharging
       
         }                                           
      else
         {
         if (batdata.vbat == batdata.vbatpeak)
            slope = ZERO;
           else
             {
                 slope = POSITIVE;
                 batdata.vbatpeak = batdata.vbat;        //store the peak on each chg interval to avoid transient peaks
             } 
         }
      }
   if(overtemp==TRUE)
      peakdetected = TRUE;                             //in all cases stop charging when the battery gets hot!!!
}

void Get_BatTemp(void)
{

   unsigned long temp,temp2;

   set_adc_channel(TBATC);
   delay_us(100);
   temp = read_adc(ADC_START_AND_READ);     //look at raw AD result
   
   set_adc_channel(TAMBC);
   delay_us(100);
   temp2 = read_adc(ADC_START_AND_READ);     //look at raw AD result
   
   //batdata.dambt = batdata.ambtemp - batdata.battemp;     //battery T above ambient
   
   SDO_AddReading(&btempsdo,temp,temp2);        //delta = Tbat-Tamb
   
   if(state == INIT)
     btempsdo.peak = btempsdo.last;
      
   if(btempsdo.lastdelta>MAXDELTAT)
      overtemp = TRUE;
   else
      overtemp = FALSE;


}

//reads battery current and voltage sets battery state
//reads the setpoint delta
//takes present current reading every second and totals avg ampminutes
//clears ampmin total if battery low
void Get_Battery_Status(void)
{

   unsigned long temp;
   
   
   temp = Get_ADMV(VFBAT);              //get the latest battery voltage
   temp *= BATDIVFAC;                   //for modified circuit
                                        //improved resolution was 4 013012
   
   SDO_AddReading(&batvsdo,temp);
   
   batdata.vbat = batvsdo.avg;          //use the average to reduce noise


   if(state == CHARGE)
      {
      
      SDO_AddReading(&batisdo,get_BATI(),batdata.setpointi);
      
      

   if(labs(batisdo.lastdelta) > DELTAILIM)
      inreg=FALSE;                                       //set in regulation flag
   else
      inreg=TRUE;
      
   if(batisdo.avg>batdata.batilim)
      batiovr=TRUE;
   else
      batiovr=FALSE;

      temp=Get_Timer();                                  //elapsed minutes
      
      switch(tdata.tevent)
         {
            case TUNULL:
            break;
            case TUSEC:
               temp = batisdo.avg/10;   //convert chargei to ma
               batdata.csecsum += temp;
               
               

               SendData();

                        
            break;
            case TUMIN:
            case TUHOUR:
                  temp = batdata.csecsum/60;      //average over 60 readings
                  batdata.cmins += temp; 
                  if((DELTAQ-batdata.deltacmin) > temp)
                     batdata.deltacmin += temp;
                  else  //once per unit charge here
                     {
                     batdata.deltacmin = 0;
                     Getdeltav();                              //dV/dQ 
                     }
                  batdata.csecsum = 0;
            break;
            default:
            break;
         }

      }
   else
      {
      //batdata.chargei = 0;
      //batdata.deltai = 0;
      //batdata.absdeltai = 0;
      batdata.vbatpeak = batdata.vbat;    //track battery voltage if not charging
      batvsdo.peak = batvsdo.last;
      }
   




   if(batvsdo.avg < BATMISSING)
      batstate = BATGONE;                 //battery not present
   else 
      
      if(batvsdo.avg <= VOMIN)
      {
      batstate = BATDEAD;
      if(state != CHARGE)
         batdata.cmins = 0;               //clear the charge reg when bat is dead
      }
      
      else if(batvsdo.avg <= BATLOV)
         {
         batstate = BATLO;
         if(state != CHARGE)
            batdata.cmins = 0;               //clear the charge reg when bat is lo
         }
         else if (batvsdo.avg <= BATFULV)
            batstate = BATNORM;
            else if (batvsdo.avg <= BATOVRV)
               batstate = BATFUL;
               else
                  batstate = BATOVR;
   
}

//calculates battery charger voltage setpoint from
//Vset=Vbat + Iset*Rballast
//where Rballast = 2ohm (was 10ohm)
//Iset in 1/10th ma units
//Voltages all in mv
//Vset(mv)=Vbat(mv) + Iset*10 * 2ohm/10
void Get_vsetbat()
{
   unsigned long temp;
   temp = batdata.setpointi/5; 
   batdata.vsetbat = batvsdo.avg + temp;       //was vbat + iset 013012

}

int Adjust_DC(signed long delta)   //regulation function
{

int err;    //err=1 dc at min err=2 dc at max
   
   err=0;

   if(delta > 0)
         {
         if(dutycycle>DUTYMIN)
            --dutycycle;
         else
            err=1;
         }
      else if(delta !=0)
         {
         if(dutycycle<DUTYMAX)
            ++dutycycle;
         else
            err=2;
         
         }
   set_pwm1_duty(dutycycle);
  
   return err;
}

void Init_Batdata(char varopt=0)                //if a nonzero value is passed
                                                //only selected variables will be init
{

               if(varopt==0)                    //these are NOT initialized with partial
                  {
                  SDO_Init(&batvsdo);              //initialize the data objects
                  SDO_Init(&pnlvsdo);
                  SDO_Init(&btempsdo);
                  SDO_Init(&batisdo);
                  SDO_Init(&pnlisdo);
                  SDO_Init(&pnlpsdo);
                  batdata.dischgmin = 0;           //clear the load timer on powerup
                  }
               
               mppd=0;
               
               batdata.csecsum = 0;
               //batdata.deltaisum = 0;
               batdata.setpointi = 0;
               batdata.batilim=0;
               batiovr=FALSE;
               batdata.vsetbat = 0;
               batdata.vbatpeak = batdata.vbat;
               batvsdo.peak = batvsdo.last;

               batdata.deltacmin =0;
    
               
               slope = UNKNOWN;
               dutycycle = 0;
               LEDduty=0;
               batdata.lastvbat=batdata.vbat;         //initial condition
               batdata.deltav=0;                      //initial condition
               btntimer = 0;
               Clear_Timer();
               
               inreg = TRUE;
               targetinreg=FALSE;
//               peakdetected = FALSE;               //will prevent charging if true
               maxmppd=FALSE;
               
               output_drive(PIN_C6);                  //refresh the tx tris

}

int Ramp(unsigned long target)         //setpoint ramping function will move one step
                                       //if current in regulation or error is in dir
                                       //of ramp. returns TRUE when setpoint reaches target

{

signed long temp;
int1 sign;

   if(batisdo.lastdelta>=0)
      sign=TRUE;              //reading>=setpoint
   else
      sign=FALSE;


   if (target == 0)                     //prevent overflow
      target = ISTEP;
      
   if (target > batdata.setpointi)
   {
      if((inreg==TRUE)||sign)
         {
         batdata.setpointi += ISTEP;   //ramp current up one step
         
         }
   }
   else if (target != batdata.setpointi) //setpoint less than target
   {
      if((inreg==TRUE)||(!sign))
         {
         batdata.setpointi -= ISTEP;   //ramp current down one step
         
         }
   }
  
  temp = labs(batdata.setpointi - target);
  if(temp<=ISTEP)
   return TRUE;
  else
   return FALSE;
   
}

void Init_Scan(void)
{
   pnldata.scanindex = 0;
   pnldata.vstart=pnlvsdo.avg;
   pnldata.vpnlsp = pnldata.vstart;
   Clear_Pscan();
   scanstate = MPSETPOINT;
   pnldata.mppcyl = 0;
}

void main()
{
   //setup_oscillator(OSC_16MHZ|OSC_INTRC|OSC_31250|OSC_PLL_ON);  //as generated by the wiz
   //setup_oscillator(OSC_16MHZ|OSC_INTRC|OSC_PLL_ON);
   
   
   delay_us(1000);
   
   state=INIT;             //initialize on POR use restart_cause to direct

   while (TRUE)
   {

      
      switch(state)
      {
         case INIT:

            port_B_pullups(0x01);
            
            output_drive(PIN_C6);            //set tx to ouput

            //setup button int
            //ext_int_edge (0, H_TO_L);
            //enable_interrupts(INT_RB);
            //enable_interrupts(GLOBAL);            
            
            //setup_wdt(WDT_ON);             //timebase set in fuses see .h
            setup_wdt (WDT_OFF);             //for debug

            


            //setup_uart(115200);            //not needed with USE


            

            //setup comparator (not used)
            setup_comparator(NC_NC_NC_NC);// This device COMP currently not supported by the PICWizard
            
            //setup AD converter
            setup_adc(ADC_CLOCK_DIV_64);
            setup_adc_ports (sAN0|sAN1|sAN2|sAN3|sAN5|sAN7);     //ports E0,E1 planned for LED control
            
            //setup voltage reference
            //setup_vref(VREF_6th);      //use .6V internal reference
            setup_vref(VREF_FVR);         //1.2V internal reference
            
            Stop_PWM();                //shut off the BAT PWM module 
            Stop_PWM_LED();            //shut off the LED PWM module
            
            //initialize variables
               chgstate = CHGOFF;
               pnlstate = PVDRK;
               batstate = BATNORM;
               output_low(LED);
               flashlight=FALSE;
               batdata.cmins = 0;
               
               Init_Batdata();
               
               if(input_state(BUTTON))               //read the button on reset
                  btnstate = BTNUP;
               else
                  btnstate = BTNPRESS;
                  
               peakdetected = FALSE;
             
               debounce = 0;

            ticks = 0;
            
            state = RESET;
         break;
         
         case RESET:

         
 
            if(ticks < SETTLETIME)                  //wait here for the circuit to settle
               ++ticks;
            else
            {
               
                  
               state = IDLE;                     //for ported code test
               //state = FLASHING;
               //Start_PWM_LED(20);
               ticks=0;
            }
         
         break;
        
         case IDLE:
         
            //SendData();
            //break;
         
            
               
         //check bit flag pressed for button and goto light state if battery is charged
            if((btnstate==BTNPRESS) && !(batstate <= BATLO))
               {
               state = LIGHT;
               ltstate=LTINIT;
               ticks = 0;
               break;
               }
            else 
            {
               if((pnlstate==PVCHG)&&(peakdetected==FALSE))             //dont charge again if peaked
               {
                  if((batstate < BATOVR)&&(batstate != BATGONE)) //will not charge a completely dead battery     
                  {
                     state = CHARGE;
                     chgstate = CHGINIT;
                     ticks = 0;
                  }
               }
            }
            
            
            
            //build hardware and check that system comes here if no panel and good battery
         
         break;
         
         case CHARGE:
         
           if(batstate==BATOVR)          //ramped shutdown if over voltage
               chgstate= CHGOFF;
                        
           if(batstate==BATGONE)          //fast shutdown if battery removed
               chgstate = CHGOFF;
               
           if((btnstate==BTNPRESS)&&(chgstate != CHGDONE))  //press button to stop charge note cycle back into routine!!!
               chgstate = CHGOFF;

/*
           if(pnlvlow)
               chgstate = CHGOFF;         //shut charger if panel voltage drops too low ???             
                  
*/

         
            switch(chgstate)
            {
               
               
               case CHGINIT:        //implement temperature conditions
                  Init_Pnldata();
                  batdata.setpointi = ISTART;      //start with min. current setpoint
                  dutycycle = DUTYMIN;
                  Start_PWM(dutycycle);            //start with DC -> Vo < Vbat
                  batdata.dischgmin = 0;           //clr the discharge timer
                  chgstate=CHGINREG;
                  ticks=0;
                  
               break;
               
               case CHGINREG:
                                                   //adjust DC to setpoint 
                  if(!inreg)                       
                     Adjust_DC(batisdo.lastdelta);
                  else
                  { 
                    if (ticks < REGTIMER)         //delay to stabilize the system
                       ++ticks;
                    else 
                       chgstate=CHGSTRT;            //no other way out except PV???
                  }

                  if(pnlstate!=PVCHG)              //if panel collapses here retry later
                     {
                     ticks=0;
                     chgstate=CHGHOLD;
                     
                     }

               break;
               
               case CHGSTRT:
               //start the charger with trickle charge until batstate>BATLO
                  

                     if(Ramp(TRICKLE))
                        targetinreg=TRUE;
                     else
                        targetinreg=FALSE;
                        
                     Adjust_DC(batisdo.lastdelta);
                       
                     if(targetinreg==TRUE)
                        if(inreg)
                           if(batstate>BATLO)
                              {
                              ticks = 0;
                              chgstate = SCANNING;
                              Init_Scan();
                              mppd=TRICKLE;
                              }
                        
                        //timeout needed here see nimh handbook

                     if(pnlstate!=PVCHG)     //if panel collapses here retry later
                        {
                        ticks=0;
                        chgstate=CHGHOLD;
                        
                        }
                  
               break;
               case SCANNING:
               
                  if(pnldata.mppcyl<MPPCYLTIME) //delay several cyls to allow response
                     ++pnldata.mppcyl;
                  else
                     {
                     pnldata.mppcyl = 0;
                     
                     switch(scanstate)
                        {
                        case MPSETPOINT:
                           pnldata.vpnlsp=pnldata.vstart-pnldata.scanindex*PVSCANSTEP;
                           scanstate = MPSLEW;
                        break;
                        case MPSLEW:
                           switch(SetMppd(FASTCHG)){
                              case 0://no error
                                 Ramp(mppd);
                                 switch(Adjust_DC(batisdo.lastdelta)){
                                    case 0://no error
                                    break;
                                    case 1://minimum dc reached
                                       scanerr=DCLOW;
                                       scanstate=MPFAULT;
                                    break;
                                    case 2://maximum dc reached
                                       scanerr=DCHIGH;
                                       scanstate=MPFAULT;
                                    break;
                                 }
                              break;
                              case 1://min current
                                 scanerr=BILOW;
                                 scanstate=MPFAULT;
                              break;
                              case 2://max current
                                 scanerr=BIHIGH;
                                 scanstate=MPFAULT;
                              break;
                           }
                           if(pnlinreg)
                              scanstate=MPRECORD;
                           if(pnlstate!=PVCHG)
                              {
                              scanerr=PNLCRASH;
                              scanstate=MPFAULT;
                              }
                           
                        break;
                        case MPRECORD:
                           pnldata.pscan[pnldata.scanindex]=pnlpsdo.avg;
                           scanstate=MPTEST;
                           
                        break;
                        case MPTEST:
                        
                           if(pnldata.scanindex==0)
                              {
                              ++pnldata.scanindex;    //need 2 readings for a dP
                              scanstate=MPSETPOINT;
                              break;
                              }
                           if((pnldata.pscan[pnldata.scanindex]-pnldata.pscan[pnldata.scanindex-1])<0)
                              {
                              scanstate=MPMAX;        //-dP indicates max power
                              break;
                              }
                              
                           if(pnldata.scanindex<(SCANSIZE-1))
                              {
                              ++pnldata.scanindex;
                              scanstate=MPSETPOINT;   //keep scanning for a -dP
                              }
                           else
                              {
                              scanstate=MPMAX;        //use last sp for max
                              }
                           
                        break;
                        case MPMAX:
                           pnldata.pmaxindex=pnldata.scanindex-1; //previous reading is max
                           pnldata.vpnlsp=pnldata.vstart-pnldata.pmaxindex*PVSCANSTEP;  //store the max power vp setpoint
                           scanstate=MPSTAB;
                        break;
                        case MPFAULT:
                           switch(scanerr){
                           
                              case PNLCRASH://pnl voltage has dropped before setpoint
                                 if(pnldata.scanindex>0)
                                    scanstate=MPMAX;     //use the previous index
                                 else
                                    chgstate=CHGHOLD;    //unless failed on first

                              break;
                              case DCHIGH://dutycycle is at maximum b4 charge i setpoint
                                 if(pnldata.scanindex>0)
                                    scanstate=MPMAX;     //use the previous index
                                 else
                                    chgstate=CHGHOLD;    //unless failed on first
                              break;
                              case DCLOW://dutycycle is at min
                                 if(pnldata.scanindex==0)
                                    ++pnldata.scanindex;
                                 scanstate=MPMAX;     //use previous reading for max 
                              break;
                              case BIHIGH://charge current at max limit
                                 if(pnldata.scanindex==0)
                                    ++pnldata.scanindex;
                                 scanstate=MPMAX;     //use previous reading for max 
                              break;
                              case BILOW://no charge current
                                 chgstate=CHGHOLD;//stop charging wait for pnl to rise
                              break;
                              case NOMAX://no power peak detected
                                 ++pnldata.scanindex;
                                 scanstate=MPMAX;     //use last reading for max
                              break;
                              case NOSTAB:
                                 chgstate=CHGSTRT;    //any error at this point must be rescanned
                              
                              break;
                           
                           
                           
                           
                           
                           }
                        break;
                        case MPSTAB:
                           //this state is entered from MPMAX with the panel setpoint
                           //at the max power point. It slews the panel to this new setpoint
                           //and exits when the panel is stable (inreg)
                           switch(SetMppd(FASTCHG)){
                              case 0://no error
                                 Ramp(mppd);
                                 switch(Adjust_DC(batisdo.lastdelta)){
                                    case 0://no error
                                    break;
                                    case 1://minimum dc reached
                                       scanerr=NOSTAB;
                                       scanstate=MPFAULT;
                                    break;
                                    case 2://maximum dc reached
                                       scanerr=NOSTAB;
                                       scanstate=MPFAULT;
                                    break;
                                 }
                              break;
                              case 1://min current
                                 scanerr=NOSTAB;
                                 scanstate=MPFAULT;
                              break;
                              case 2://max current
                                 scanerr=NOSTAB;
                                 scanstate=MPFAULT;
                              break;
                           }
                           if(pnlinreg)
                              {
                              chgstate=CHARGING; //exit when stable
                              }
                           if(pnlstate!=PVCHG)
                              {
                              scanerr=NOSTAB;
                              scanstate=MPFAULT;
                              }
                        
                        
                        break;
                        default:
                        break;
                     
                        }
                     
                     }
               
               
               break;
               case CHARGING:          //implement temperature conditions
               
                  if(pnldata.mppcyl<MPPCYLTIME) //delay several cyls to allow response
                     ++pnldata.mppcyl;
                  else
                  {
                     pnldata.mppcyl=0;
                  
                     if(pnlstate!=PVCHG)              //if panel collapses here retry later
                        {
                        chgstate=CHGHOLD;
                        break;
                        } 
   
                     //test for abs dPmax > LIM
                     If(labs((pnldata.pscan[pnldata.pmaxindex]-pnlpsdo.avg)>DPLIM))
                        {
                        chgstate=CHGSTRT;    //rescan if power has changed since last max point scan
                        break;
                        }
   
   
   
                     switch(SetMppd(FASTCHG)){
                        case 0:
                        break;
                        case 1://min current
                        break;
                        case 2://max current
                        break;
   
                     }
                     
                     
                     
                     Ramp((peakdetected==TRUE)? TRICKLE:mppd);     //implement mppt here
   
                     if(batdata.cmins > CMINLIM) //total charge exceeded
                        {
                        chgstate=CHGOFF;
                        ticks = 0;
                        break;
                        
                        }
   
   
                     if(batstate < BATOVR) //stop charging if overvoltage
                        {
                     
                        if(labs(batisdo.lastdelta) > PROPBAND)//use proportional here
                           {
                           //adjust DCY according to deltai
                              Adjust_DC(batisdo.lastdelta);
   
                           } 
                        else //use itegral here
                           {
                              if(labs(batisdo.lastdelta) > DEADBAND)//adjust if out of band
                              {
                              //adjust DCY according to deltasum
                                 Adjust_DC(batisdo.deltasum);
                                 
                              }
   
                           }
                     
                        }
                     else  //battery voltage over limit
                        {
                        chgstate=CHGOFF;
                        ticks = 0;
                        break;
                        
                        }
                     ++ticks;

                  }
               break;

               case CHGOFF:
                  Stop_PWM();          //shut down charge circuit possible ramp here???
                  chgstate=CHGDONE;
                  
               break;
               case CHGDONE:
               //reset variables and change state

                  Init_Batdata(PARTIAL);
                  
                  state = IDLE;  //main state machine
                  
                  
               break;
               case CHGHOLD:
                  Stop_PWM();          //shut down charge circuit possible ramp here???
                  
                  if(ticks<CHGHOLDTIME)
                     ++ticks;
                  else
                     {
                     ticks=0;
                     if(pnlstate!=PVCHG)
                        chgstate=CHGDONE;
                     else
                        chgstate=CHGINIT;
                     
                     }
                  
                  
                  
               break;
               default:
               break;
            
            
            }
         
         break;
         
         case LIGHT:
         
            switch(ltstate)
            {
               case LTINIT:
                  Clear_Timer();
                  ltstate=LTON;
                     
               break;
               case LTON:
                  Output_High(LED);
                  If(btnstate==BTNUP)
                    ltstate=LTTIME;
                                    
               break;
               case LTTIME:
                  if (Get_Timer()>LOADTIME)        //this runs the timer and
                     peakdetected=FALSE;           //clears the peak detect charge lockout
                                                   //to prevent rentry into charge algorithm
                     
                  if((btnstate==BTNPRESS)||(batstate==BATLO))
                     {
                     ltstate=LTOFF;
                     batdata.dischgmin += Get_Timer();   //add total discharge time
                     }

               break;
               case LTOFF:
                  
                  if(btnstate==BTNUP)
                     state=IDLE;                   //wait till button released
                  Output_Low(LED);   
               
               
               break;
               default:
               break;

            
            }
         
         break;
         
         case FLASHING:

            if(ticks == 2)
               {
                  if(flashlight){         //flash display LED's for ported code test
                     output_low(DISP0);
                     output_high(DISP1);
                     
                     if(LEDduty<(DUTYCYL100-5))
                  {
                        LEDduty += 5;
                        set_pwm2_duty(LEDduty);
                  }
                else
                  flashlight=FALSE;
                     
                  }else{
                     output_high(DISP0);
                     output_low(DISP1);
                     
                     if(LEDduty>5)
                  {
                        LEDduty -= 5;
                        set_pwm2_duty(LEDduty);
                  }
                else
                  flashlight=TRUE;
                  }
                  ticks=0;   
               }
            ++ticks;
            
            
               
         
         break;
         
         case SLEEP:
         
         break;
         
         default:
         
         break;
   
   
   
      
      }
      
      //get the status of the battery and panel
      Get_Panel_Status();
      Get_Battery_Status();
      Get_BatTemp();

      pressed = !(input_state(BUTTON));      //input low when pressed   
   
      switch(btnstate)
         {
           case BTNUP:
               if(Debounce_Button(pressed,BUTDEBOUNCE))
                  btnstate = BTNPRESS;
           break;
           case BTNPRESS:
               if(Debounce_Button(pressed,BUTHOLD))
                  btnstate = BTNHOLD;
               else
                  If(!pressed)
                     {
                     btnstate = BTNUP;
                     Debounce_Button(0,1);
                     }
           break;
           case BTNHOLD:
               if(!pressed)
                  {
                  Debounce_Button(0,1);   //clear debounce timer
                  btnstate = BTNUP;
                  }
       
           break;
           case BTN2PRESS:
           break;
           default:
           break;
         }



      
      
      //put the output control code here
      
      
      
      
      //put timebase code here
      delay_ms(37);        //to simulate WDT sleep
      restart_wdt();
      /*if(ticks)
      --ticks;*/
      
   }

}
