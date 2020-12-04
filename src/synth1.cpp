#include <FreqMeasure.h> 
#include <DigiPotX9Cxxx.h>
#include <Wire.h>
//#include <MemoryFree.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <FreqCount.h>
#include <eRCaGuy_Timer2_Counter.h>
#include "wiring_private.h"
#include "pins_arduino.h"

//Adafruit_MCP23017 mcp0;
//Adafruit_MCP23017 mcp1;
//Adafruit_MCP23017 mcp2;

byte DebugLevel = 1;

DigiPot pot0(2,3,40);
DigiPot pot1(2,3,42);
DigiPot pot2(2,3,44);

DigiPot pot3(2,3,46);
DigiPot pot4(2,3,48);
DigiPot pot5(2,3,50);

// VOLUME POT
DigiPot pot12(2,3,52);

// All osc pots : 0 to 5 : OSC1, 6 to 11: OSC2
DigiPot *pots[6] = { &pot0 , &pot1, &pot2, &pot3, &pot4, &pot5 };
// All pots :
DigiPot *allpots[7] = { &pot0 , &pot1, &pot2, &pot3, &pot4, &pot5, &pot12};


volatile unsigned long RiseCount;
volatile unsigned long FallCount;


const double notes_freq[49] PROGMEM = {

  130.8,
  138.6,
  146.8,
  155.6,
  164.8,
  174.6,
  185.0,
  196.0,
  207.7,
  220.0,
  233.1,
  246.9,
  261.6,
  277.2,
  293.7,
  311.1,
  329.6,
  349.2,
  370.0,
  392.0,
  415.3,
  440.0,
  466.2,
  493.9,
  523.3,
  554.4,
  587.3,
  622.3,
  659.3,
  698.5,
  740,
  784,
  831,
  880,
  932,
  988,
  1047,
  1109,
  1175,
  1245,
  1319,
  1397,
  1480,
  1568,
  1661,
  1760,
  1865,
  1976,
  2093
};


byte k;
int vcosel;
char inp;


byte midi_to_pot[6];
int freqrefpin;
double notefreq;
double minfreq = 130.8;
double maxfreq = 2093.0;

const byte numChars = 40;
char receivedChars[numChars]; // an array to store the received data
bool newData = false;

// Init all pots.
  void StartAllPots(DigiPot *ptr[7]) {

    byte m = 0;

    for(m = 0; m < 7; m++) {
  
        ptr[m]->begin();
        delay(5);
    
      }
    
  }

  
void MaxVcoPots(DigiPot *ptr[6], byte (&curr_pot_vals)[6], byte vcosel) {

    byte m = 0;
    byte k = 0;
    byte max_pot = 3;

    for(m = max_pot*vcosel; m < max_pot*vcosel + max_pot; m++) 
    {
  
      for(k=0;k<100;k++)
      {
        ptr[m]->increase(1);
        delay(5);
      }
      curr_pot_vals[m] = 99;
     
    }
    
  }

void PrintDigiPot(byte (&curr_pot_vals)[6], byte vcosel)
{
  if (vcosel == 0)
  {
    Serial.print("<P0= ");
    Serial.print(curr_pot_vals[0]);
    Serial.print(" P1= ");
    Serial.print(curr_pot_vals[1]);
    Serial.print(" P2= ");
    Serial.print(curr_pot_vals[2]);
    Serial.print(">");
  }
  else if (vcosel == 1)
  {
    Serial.print("<P3= ");
    Serial.print(curr_pot_vals[3]);
    Serial.print(" P4= ");
    Serial.print(curr_pot_vals[4]);
    Serial.print(" P5= ");
    Serial.print(curr_pot_vals[5]);
    Serial.print(">");
  }

  Serial.flush();
}

void DebugPrint(String token, double value, byte msgdebuglevel)
{
 
 if (msgdebuglevel <= DebugLevel)
  {
  Serial.print("<");
  Serial.print(token);
  Serial.print("=");
  Serial.print(String(value,5));
  Serial.print(">");
  Serial.flush();
  }
}

//Convert frequency to R value using curve fitting
double hertz_to_R(double freq, int subvco) {

double R;
if (subvco == 0) 
{

    R = -5.59912866*pow(log(freq),3) + 121.32633193*pow(log(freq),2) -887.09997093*log(freq) + 2198.77893925;
  //R = -6.2753*pow(log(freq),3) + 134.2073*pow(log(freq),2) - 968.1969*log(freq) + 2367.8146;
  //R = -6.3355*pow(log(freq),3) + 135.5047*pow(log(freq),2) - -977.6253*log(freq) + 2390.9722;


}
else if (subvco == 1) 
{
 
    R = -5.80833298*pow(log(freq),3) + 125.24643318*pow(log(freq),2) -911.17411721*log(freq) + 2247.03355536;
  //R = -5.9465*pow(log(freq),3) + 127.6*pow(log(freq),2) - 922.9374*log(freq) + 2260.6137;
  //R = -6.0024*pow(log(freq),3) + 128.804*pow(log(freq),2) - 931.6833*log(freq) + 2282.123;

}

return R;

}

// not used. does not lead to adequate results
double hertz_to_R_v2(double freq, int subvco) {

double R;
R = 1/(freq*0.44e-6);

return R;

}

double d_hertz_to_R(double freq, int subvco) {
// dertivative of hertz to_R
double dR;
if (subvco == 0) 
{

    dR = -16797*(pow(log(freq),2))/(1000*freq) +60663*log(freq)/(250*freq) - 8871/(10*freq);
}
else if (subvco == 1) 
{

    dR = -2178*(pow(log(freq),2))/(125*freq) +62623*log(freq)/(250*freq) - 455587/(500*freq);

}

return dR;

}



// convert saw freq components (subosc1 and 2 of OSC to resistance values of each timer R
void saw_freq_duty_to_R(double freq, double duty, double &R1 , double &R2, double &f1, double &f2, int vco) {

// if duty cycle is extreme, subosc may have to generate frequencies above or under the possible range.
// In this case, the duty cycle is set to the max. allowable setting.
if (freq < minfreq) { freq = minfreq; }
else if (freq > maxfreq) { freq = maxfreq; }

double period = 1.0/freq;
f1 = 1.0/(2*(duty*period));
f2 = 1.0/(2*((1.0 - duty)*period));

if (f1 < minfreq) { 
  f1 = minfreq; 
  duty = 1.0/(2*(f1*period));
  f2 = 1.0/(2*((1.0 - duty)*period));
  }
else if (f2 < minfreq) {
  f2 = minfreq;
  duty = 1 - 1.0/(2*(f2*period));
  f1 = 1.0/(2*(duty*period));
  }
if (f1 > maxfreq) { 
  f1 = maxfreq;
  duty = 1.0/(2*(f1*period));
  f2 = 1.0/(2*((1.0 - duty)*period));
  }
else if (f2 > maxfreq) {
  f2 = maxfreq;
  duty = 1 - 1.0/(2*(f2*period));
  f1 = 1.0/(2*(duty*period));
  }
/*
Serial.print("<f1,f2,duty,r1,r2:");
Serial.print(String(f1,3));
Serial.print(",");
Serial.print(String(f2,3));
Serial.print(",");
Serial.print(String(duty,3));
Serial.print(",");
*/


R1 = hertz_to_R(f1,0);
R2 = hertz_to_R(f2,1);

Serial.print(String(R1,3));
Serial.print(",");
Serial.print(String(R2,3));
Serial.print(">");
Serial.flush();

}

//Splits R to the settings of the three serial pots (100K, 10K, 1K)
void R_to_pot(double R, byte &val_pot100K, byte &val_pot10K, byte &val_pot1K, byte subvco) {

  double Rtmp;
  const double R1OOK_total_R_vco0 = 96.77;
  const double R1OK_total_R_vco0 = 10.08;
  const double R1K_total_R_vco0 = 1.055;
  const double trim_pot_R_vco0 = 2.73;

  const double R1OOK_total_R_vco1 = 97.26;
  const double R1OK_total_R_vco1 = 10.42;
  const double R1K_total_R_vco1 = 1.033;
  const double trim_pot_R_vco1 = 2.54;


  //const double wiper_R = 0.12;
  

  if (subvco == 0) {

  // R is in kOhms
  // we substract wiper resistance and trim pot resistance.
  Rtmp = R - trim_pot_R_vco0;

    if(Rtmp > R1OOK_total_R_vco0) 
    {

      val_pot100K = 99;
      Rtmp = Rtmp - R1OOK_total_R_vco0;
      val_pot10K = (byte) constrain(int(Rtmp/(R1OK_total_R_vco0/99 + 0.5)),0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_vco0/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_vco0/99) + 0.5);
    }
    else {
      val_pot100K = (byte) constrain(int(int(Rtmp/(R1OOK_total_R_vco0/99))/10)*10,0,99);
      Rtmp = Rtmp - val_pot100K*(R1OOK_total_R_vco0/99);
      val_pot10K = (byte) constrain(int(Rtmp/(R1OK_total_R_vco0/99) + 0.5),0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_vco0/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_vco0/99) + 0.5);
      
    }

  }
  
  else if (subvco == 1) {
  
    Rtmp = R - trim_pot_R_vco1;

    if(Rtmp > R1OOK_total_R_vco1) 
    {

      val_pot100K = 99;
      Rtmp = Rtmp - R1OOK_total_R_vco1;
      val_pot10K = (byte) constrain(int(Rtmp/(R1OK_total_R_vco1/99)),0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_vco1/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_vco1/99) + 0.5);
    }
    else 
    {
      val_pot100K = (byte) int(int(Rtmp/(R1OOK_total_R_vco1/99))/10)*10;
      Rtmp = Rtmp - val_pot100K*(R1OOK_total_R_vco1/99);
      val_pot10K = (byte) constrain(int(Rtmp/(R1OK_total_R_vco1/99)),0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_vco1/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_vco1/99) + 0.5);
      
    }

  }

}

//Change OSC frequency for TRI/SAW or SINE. Supply pot vals in this order for osc1 and osc2 osc1:100K,10K,1K,osc2:100K,10K,1K.
//Digitpot *ptr[] : array of pointers of digipots objects in the same order as pot vals.
//boolean val_saw : 0: generate sine. 1: generate saw/tri
//int val_vco : for saw, use 0 for OSC1, 1 for OSC2; for sine, use O for subosc1 of OSC1, 1 for subosc2 of OSC1, 2 for subosc1 of OSC2, 3 for subosc2 of OSC2.
void ChangeNote(byte pot_vals[6], byte (&curr_pot_vals)[6], DigiPot *ptr[6], boolean val_saw, int val_vco) {

  int m = 0;
  byte max_pot;
  int pot_val_change;
  //static byte curr_pot_vals[6] = {99,99,99,99,99,99};
  
  if (!val_saw) {
  max_pot = 3;  
  }
  else {
  max_pot = 6;
  }

  for(m = val_vco*max_pot; m < max_pot + val_vco*max_pot; m++) {

    //if (pot_vals[m] < 0) { pot_vals[m] = 0; }
    if (pot_vals[m] > 99) { pot_vals[m] = 99; }
    
    pot_val_change = int(pot_vals[m]) - int(curr_pot_vals[m]);
   /*
    Serial.print("<m:");
    Serial.print(m);
    Serial.print(">");
    
    Serial.print("<pot_val:");
    Serial.print(pot_vals[m]);
    Serial.print(">");
   
    Serial.print("<pot_val_curr:");
    Serial.print(curr_pot_vals[m]);
    Serial.print(">");
   
  
    Serial.print("<pot_val_change:");
    Serial.print(pot_val_change);
    Serial.print(">");
    */
   if (pot_val_change > 0) {
      ptr[m]->increase(pot_val_change);
   }
   else if (pot_val_change < 0) {
      ptr[m]->decrease(abs(pot_val_change));
   }

    curr_pot_vals[m] = pot_vals[m];
  }
}

void GenerateArbitraryFreq(byte (&curr_pot_vals)[6], double freq, double duty, double &f1, double &f2)
{

  double Rvco1;
  double Rvco2;
  
  byte R100_1;
  byte R10_1;
  byte R1_1;
  
  byte R100_2;
  byte R10_2;
  byte R1_2;

  byte midi_to_pot_G[6];
  
 
  saw_freq_duty_to_R(freq, duty, Rvco1 , Rvco2, f1, f2 ,0);
  
  Serial.print("<duty=");
  Serial.print(duty);
  Serial.print(">");


  Serial.print("<R1=");
  Serial.print(Rvco1);
  Serial.print(">");
  
  Serial.print("<R2=");
  Serial.print(Rvco2);
  Serial.print(">");

  Serial.print("<Generate_f1=");
  Serial.print(String(f1,3));
  Serial.print(">");
  
  Serial.print("<Generate_f2=");
  Serial.print(String(f2,3));
  Serial.print(">");
  

   R_to_pot(Rvco1, R100_1, R10_1, R1_1, 0);
   R_to_pot(Rvco2, R100_2, R10_2, R1_2, 1);

/*
   Serial.print("<");
   Serial.print(R100_1);
   Serial.print(",");

   Serial.print(R10_1);
   Serial.print(",");

   Serial.print(R1_1);
   Serial.print(",");

   Serial.print(R100_2);
   Serial.print(",");

   Serial.print(R10_2);
   Serial.print(",");

   Serial.print(R1_2);
   Serial.print(">");
*/


   midi_to_pot_G[0] = R1_1; 
   midi_to_pot_G[1] = R10_1; 
   midi_to_pot_G[2] = R100_1; 
   midi_to_pot_G[3] = R1_2; 
   midi_to_pot_G[4] = R10_2; 
   midi_to_pot_G[5] = R100_2; 

  /*
   for(k=6;k<12;k++){
   midi_to_pot[k] = 99;
   }
  */  

   //Change OSC freq with saw
   ChangeNote(midi_to_pot_G, curr_pot_vals, pots, true, 0);

}
  
void Check_all_pots_R(DigiPot *ptr[6]) 
{

 byte m = 0;
 byte k = 0;
 byte max_pot = 3;
 byte vcosel;
 byte vco_pin = 4;
 pinMode(vco_pin, OUTPUT);
   
for(vcosel = 0; vcosel < 2; vcosel++) 
  {
   if (vcosel == 0) 
   {
    digitalWrite(vco_pin, HIGH);
   }
   else if (vcosel == 1)
   {
    digitalWrite(vco_pin, LOW); 
   }


    Serial.print("<Min all pots vcosel:");
    Serial.print(vcosel);
    Serial.print(">");


      for(m = max_pot*vcosel; m < max_pot + max_pot*vcosel; m++) {
        
        for(k=0;k<100;k++)
        {
      
          ptr[m]->decrease(1);
          delay(5);
      
        }
    
      
      }

    Serial.print("<Min all pots vcosel done:");
    Serial.print(vcosel);
    Serial.print(">");

      delay(20000);


      for(m = max_pot*vcosel; m < max_pot + max_pot*vcosel; m++) 
      {

        Serial.print("<for vcosel:");
        Serial.print(vcosel);
        Serial.print(">");

        Serial.print("<max pot:");
        Serial.print(m);
        Serial.print(">");
      

        for(k=0;k<100;k++)
        {
      
          ptr[m]->increase(1);
          delay(5);
      
        }
         
        delay(20000);
        Serial.print("<max pot end:");
        Serial.print(m);
        Serial.print(">");
      

        for(k=0;k<100;k++)
        {
      
          ptr[m]->decrease(1);
          delay(5);
      
        }

      }

      for(m = max_pot*vcosel; m < max_pot + max_pot*vcosel; m++) 
      {
        
        for(k=0;k<100;k++)
        {
      
          ptr[m]->increase(1);
          delay(5);
      
        }
    
      }

  }

} 


float MeasureFrequencyDelta(byte totaltime,byte numberofsamples,float tunefrequency) {


  FreqMeasure.begin();

    float integrator = 0.0;
    float integrator_temp = 0.0;
    float sum;   

    byte count2 = 0;
    byte count;
    byte cycles;
    byte samples;

   for (cycles = 0; cycles < totaltime; cycles++) {
   sum = 0.0;
   count = 0;
   
   for(samples = 0; samples < numberofsamples; samples++) {
   if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count++;
        
     }
     delay(1000/numberofsamples);
    }
    
    if ((cycles > 0) && (count >0)) {
    integrator_temp = FreqMeasure.countToFrequency(sum / count);
    integrator = integrator_temp + integrator;
    count2++;
    Serial.print("<integrator_temp=");
    Serial.print(String(integrator_temp,3));
    Serial.print(">");

    }
    //delay(1000);
  }
 
    Serial.print("<count2=");
    Serial.print(count2);
    Serial.print(">");
   
        if (count2 > 0) {
        integrator = integrator/count2;
        } 
        else {  
        integrator = 0;
        }
        integrator = integrator - tunefrequency;
        
        FreqMeasure.end();

        return integrator;
}


float CountFrequencyDelta(byte totaltime,int gatetime,float tunefrequency) {


  FreqCount.begin(gatetime);

    float integrator = 0.0;
    //float integrator_temp = 0.0;
    unsigned long sum = 0;   

    byte count2 = 0;
    byte count;
    byte cycles;
    byte samples;
    count = 0;
   

  for (cycles = 0; cycles < totaltime; cycles++) 
  {
   
    
    if (FreqCount.available()) 
    {
      // average several reading together
      sum = sum + FreqCount.read();
      count++;
        
    }
    
    delay(1000);
    
  }
    
  if (count >0) 
  {
    integrator = sum / count;
    Serial.print("<integrator=");
    Serial.print(String(integrator,3));
    Serial.print(">");

  }

    //delay(1000);
  
 
    
  integrator = integrator - tunefrequency;
        
  FreqCount.end();

  return integrator;
}

/* Measures the length (in microseconds) of a pulse on the pin; state is HIGH
 * or LOW, the type of pulse to measure.  Works on pulses from 2-3 microseconds
 * to 3 minutes in length, but must be called at least a few dozen microseconds
 * before the start of the pulse.
 *
 * ATTENTION:
 * this function relies on micros() so cannot be used in noInterrupt() context
 */
unsigned long pulseInLong2(uint8_t pin, uint8_t state, unsigned long timeout)
{
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);

	unsigned long startMicros = timer2.get_count();

	// wait for any previous pulse to end
	while ((*portInputRegister(port) & bit) == stateMask) {
		if (timer2.get_count() - startMicros > timeout)
			return 0;
	}

	// wait for the pulse to start
	while ((*portInputRegister(port) & bit) != stateMask) {
		if (timer2.get_count() - startMicros > timeout)
			return 0;
	}

	unsigned long start = timer2.get_count();
	// wait for the pulse to stop
	while ((*portInputRegister(port) & bit) == stateMask) {
		if (timer2.get_count() - startMicros > timeout)
			return 0;
	}
	return timer2.get_count() - start;
}



void CountFrequencyDelta2(byte samplesnumber,float tunefrequency, double f1, double f2, double &f_err, double &f1_err, double &f2_err) {


   pinMode(34, INPUT);
   digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

    double f_total_measured = 0.0;
    double f1_measured = 0.0;
    double f2_measured = 0.0;
    
    
    //float integrator_temp = 0.0;
    unsigned long sumlow = 0;   
    unsigned long sumhigh = 0;   
    

    byte counthigh = 0;
    byte countlow = 0;
    byte count = 0;
    
    unsigned long pulsehigh = 0;
    unsigned long pulselow = 0;
    //noInterrupts();
  timer2.setup();
  while (count < samplesnumber)
  {
   
    pulsehigh = pulseInLong2(34,HIGH,20000);
    pulselow = pulseInLong2(34,LOW,20000);

    if (pulsehigh != 0)    
    {
    // average several reading together
    sumhigh += pulsehigh;
    counthigh++;
    }  
    if (pulselow != 0)
    {
    sumlow += pulselow;
    countlow++;
    }
/*
    Serial.print("<pulsehigh=");
    Serial.print(pulsehigh);
    Serial.print(">");
    Serial.print("<pulselow=");
    Serial.print(pulselow);
    Serial.print(">");
 */   
    count++;
    //delay(1000);
    
  }
  //interrupts();
  timer2.unsetup();
  if ((countlow >0) && (counthigh>0))
  {
    f_total_measured = 2000000.0/((sumhigh/counthigh) + (sumlow/countlow));
    f1_measured = 1000000.0/(sumhigh/counthigh);
    f2_measured = 1000000.0/(sumlow/countlow);

    Serial.print("<ftm=");
    Serial.print(String(f_total_measured,5));
    Serial.print(">");
    Serial.print("<f1m=");
    Serial.print(String(f1_measured,5));
    Serial.print(">");
    Serial.print("<f2m=");
    Serial.print(String(f2_measured,5));
    Serial.print(">");
    Serial.flush();

  }

    //delay(1000);
  
 
    
  f_err = f_total_measured - tunefrequency;
  f1_err = f1_measured - f1;
  f2_err = f2_measured - f2;
        
  //FreqCount.end();

  //return integrator;
}
void Rise() 
{
RiseCount = timer2.get_count();
}
void Fall()
{
FallCount = timer2.get_count();
}


void CountFrequencyDeltaNew2(byte samplesnumber,float tunefrequency, double f1, double f2, double &f_err, double &f1_err, double &f2_err) {

  const byte interruptPinRise = 18;
  const byte interruptPinFall = 19;
  
  pinMode(interruptPinRise, INPUT);
  pinMode(interruptPinFall, INPUT);

  RiseCount = 0;
  FallCount = 0;
  
  timer2.setup();

  attachInterrupt(digitalPinToInterrupt(interruptPinRise), Rise, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptPinFall), Fall, LOW);
  
  //FreqCount.begin(gatetime);

    double f_total_measured = 0.0;
    double f1_measured = 0.0;
    double f2_measured = 0.0;
    
    
    //float integrator_temp = 0.0;
    unsigned long sumlow = 0;   
    unsigned long sumhigh = 0;   
    unsigned long prevRiseCount;
    unsigned long prevFallCount; 
    unsigned long currRiseCount;
    unsigned long currFallCount; 


    byte count = 0;
    
    unsigned long PulseHighSum = 0;
    unsigned long PulseLowSum = 0;
    unsigned long PulseHighCount = 0;
    unsigned long PulseLowCount = 0;
    
    //noInterrupts();

  while (count < samplesnumber)
  {
      
    while ((PulseHighCount == 0) && (PulseLowCount == 0))
    {
      //Serial.print("<PULSEWAIT>");  
      //delay(100);
      if ((RiseCount != 0) && (FallCount != 0))
      {
        /*       
        */
        //delay(1000);
      
        // average several reading together
        currFallCount = FallCount;
        currRiseCount = RiseCount;
        prevFallCount = currFallCount;
        prevRiseCount = currRiseCount;
        
        while (true)
        {
          currRiseCount = RiseCount;
          currFallCount = FallCount;
          prevFallCount = currFallCount;
        
          if (currRiseCount > prevRiseCount)
          {
            while (true)
            {
            currFallCount = FallCount;
            if ((currFallCount > prevFallCount) && (currFallCount > currRiseCount))
              {
                PulseHighCount = currFallCount - currRiseCount;
                break;
              }

            }
            break;
          }
        }
        
        
        prevRiseCount = RiseCount;
        prevFallCount = FallCount;
        
        while (true)
        {
          currFallCount = FallCount;
          currRiseCount = RiseCount;
          prevRiseCount = currRiseCount;

          if (currFallCount > prevFallCount)
          {
            while (true)
            {
            currRiseCount = RiseCount;
            if ((currRiseCount > prevRiseCount) && (currRiseCount > currFallCount))
              {
                PulseLowCount = currRiseCount - currFallCount;
                break;
              }

            }
            break;
          } // end if currFallCount != prevFallCount)
        } // end while true
      }  // end  if ((RiseCount != 0) && (FallCount != 0))
    } //end while ((PulseHighCount == 0) && (PulseLowCount == 0))
    PulseHighSum += PulseHighCount;
    PulseLowSum += PulseLowCount;
    
    Serial.print("<SAMPLE>");
    Serial.print("<pulsehighcount=");
    Serial.print(PulseHighCount);
    Serial.print(">");
    Serial.print("<Pulselowcount=");
    Serial.print(PulseLowCount);
    Serial.print(">");
        
    PulseHighCount = 0;
    PulseLowCount = 0;
    prevRiseCount = 0;
    prevFallCount = 0;
    count++;    
 
  } // end while (count < samplesnumber)
  detachInterrupt(interruptPinRise);
  detachInterrupt(interruptPinFall);
  timer2.unsetup();
  
  
  
  if (count >0) 
  {
    f_total_measured = 500000.0/((PulseHighSum + PulseLowSum )/ count);
    f1_measured = 250000.0/((PulseHighSum)/ count);
    f2_measured = 250000.0/((PulseLowSum)/ count);
  }

    Serial.print("<ftm=");
    Serial.print(String(f_total_measured,5));
    Serial.print(">");
    Serial.print("<f1m=");
    Serial.print(String(f1_measured,5));
    Serial.print(">");
    Serial.print("<f2m=");
    Serial.print(String(f2_measured,5));
    Serial.print(">");
    Serial.flush();

  

    //delay(1000);
  
  
  f_err = f_total_measured - tunefrequency;
  f1_err = f1_measured - f1;
  f2_err = f2_measured - f2;
        
  //FreqCount.end();

  //return integrator;
}


void SingleCountFrequencyDelta(byte samplesnumber,double fcomp, double &fcomp_err, bool low_or_high) {


   pinMode(34, INPUT);
   digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

    double f_measured = 0.0;
    
    
    
    //float integrator_temp = 0.0;
    unsigned long sum = 0;   
     
    

    byte count = 0;
    
    unsigned long pulse = 0;
    //noInterrupts();
    timer2.setup();

  while (count < samplesnumber)
  {
   
    pulse = pulseInLong2(34,low_or_high,20000);
    
    if (pulse != 0)
    {
    // average several reading together
    sum += pulse;
    count++;
    }  
    /*
    Serial.print("<pulsehigh=");
    Serial.print(pulsehigh);
    Serial.print(">");
    Serial.print("<pulselow=");
    Serial.print(pulselow);
    Serial.print(">");
    */
    //count++;
    //delay(1000);
    
  }
  //interrupts();
  timer2.unsetup();

  if (count >0) 
  {
   
    f_measured = 1000000.0/(sum/count);

    DebugPrint("f_measured",f_measured,0);
    DebugPrint("f_comp",fcomp,0);
    /*
    Serial.print("<f_measured=");
    Serial.print(String(f_measured,3));
    Serial.print(">");
    Serial.print("<fcomp=");
    Serial.print(String(fcomp,3));
    Serial.print(">");
    Serial.flush();
    */
  }

    //delay(1000);
  
 
    
  fcomp_err = f_measured - fcomp;

        
  //FreqCount.end();

  //return integrator;
}



void AutoTune(DigiPot *ptr[6], byte (&curr_pot_vals)[6], double freq, bool level, int val_vco, bool newmethod) {


  //Serial.println("AUTO TUNE START");
  //Serial.println(String(freq));
  if (freq == minfreq)
  {
    MaxVcoPots(ptr,curr_pot_vals,val_vco);
    Serial.print("<Autotune_end>");
    Serial.flush();
    return;
  }
  
  
  int tunetime = 4;
  


  int num_integrations = 0;
  int max_integrations = 90;
  int states[max_integrations][3];
  float states_integrator[max_integrations];
  
  
  int samples_number = 20;
 
  
  int max_pot;
  int pot_val_change = -1;
  double pot_val_temp = 0.0;
  double dev_cents = 0.0;
  const double tune_thresh = 3.0;
  max_pot = 3;
 
  int  m = val_vco*max_pot + max_pot -1;
  int rstep;
  int p;

  bool tuned = false;
  double dintegrator = 0.0;
  float integrator = 0.0;
  float integrator_1 = 0.0;
  float integrator_2 = 0.0;
  int integrator_count = 0;
  bool switch_pot = false;

  float min_integrator;
  int best_index = 0;

  
  char charintegratorfmt[18];
  char charintegrator[12];
  
  
  char charpotvals[11];
  char charnumintegrations[15];

  char closechar = '>';
  char openchar = '<';
  
  //test delay(2000);

   if (newmethod) 
        {
          SingleCountFrequencyDelta(50,freq,dintegrator,level); 
          integrator = (float) dintegrator;
          dev_cents = 1200 * log((integrator + freq)/freq)/log(2);

          pot_val_change = int(integrator/fabs(integrator));
          DebugPrint("INIT_POT_VAL_CHANGE",double(integrator),2);
          /*
          Serial.print("<INIT POT VAL CHANGE:");
          Serial.print(pot_val_change);
          Serial.print(">"); 
          Serial.flush();
          */                   
        }

while(!tuned) 
{
      if (num_integrations == max_integrations) 
      { 
        tuned = true;
        break;
      }
      if (newmethod)
      {
        if (fabs(dev_cents) <= tune_thresh) 
        { 
          tuned = true;
          DebugPrint("THRESH_ATT",double(m),2);
          
          states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
          states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
          states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
          states_integrator[num_integrations] = integrator;
          break;
        }
      }

    
    sprintf (charnumintegrations, "<NUM_INTEG:%d>", num_integrations);
    Serial.print(charnumintegrations);
    Serial.flush();

    // REMOVE : waiting for integrator to have at least 2 values before changing settings  
    if(integrator_count >= 0) 
    { 
       
       // We check pot_val_change equal or greater to 0, because value can be set to 0 in switchpot, 
       // in order to avoid touching the m+1 pot, but we still increase the m pot. see if(switch_pot)
        

        DebugPrint("M",double(m),2);
        DebugPrint("M_VAL",double(curr_pot_vals[m]),2);
        DebugPrint("M_VAL_CHANGE",double(pot_val_change),2);
        
      

      if(pot_val_change == 0)
      {
        DebugPrint("POT_NO_CHANGE",double(m),2);
        //We do nothing
        /*
        
        Serial.print("<POT NO CHG:m=");
        Serial.print(m);
        Serial.print(">");
        */
      }
      else
      {
        if (((pot_val_change/abs(pot_val_change)) == 1) && (curr_pot_vals[m] < 99)) 
        {


          if ((m <= (max_pot*val_vco +2)) && newmethod ) 
          {
            rstep = pow(10,m - max_pot*val_vco +1);
            pot_val_temp = ((1/0.28e-6)/(freq*freq*rstep))*integrator;
            //pot_val_temp = ((1/0.28e-6)/(freq*freq*10))*integrator;
            pot_val_change = max(int(pot_val_temp + 0.5),1);
      /*     
            Serial.print("<VALCALC:");
            Serial.print(String(pot_val_temp,5));
            Serial.print(">");
            Serial.flush();
      */     
            ptr[m]->increase(pot_val_change);        
            curr_pot_vals[m] = constrain( curr_pot_vals[m] + pot_val_change,0,99);
          }
          else
          {
            ptr[m]->increase(pot_val_change);        
            curr_pot_vals[m]++;
          }

          DebugPrint("POT_INC_M",double(m),2);
          DebugPrint("POT_INC_VAL",double(pot_val_change),2);
        /*
          Serial.print("<POT INC:m=");
          Serial.print(m);
          Serial.print(" VAL=");
          Serial.print(pot_val_change);
          Serial.print(">");
          Serial.flush();
          */
        }
        else if (((pot_val_change/abs(pot_val_change)) == -1) && (curr_pot_vals[m] > 0)) 
        {


          if ((m <= (max_pot*val_vco +2)) && newmethod ) 
          {
            rstep = pow(10,m - max_pot*val_vco +1);
            pot_val_temp = ((1/0.28e-6)/(freq*freq*rstep))*integrator;
            //pot_val_temp = ((1/0.28e-6)/(freq*freq*10))*integrator;
            pot_val_change = min(int(pot_val_temp - 0.5),-1);
    /*        
            Serial.print("<VALCALC:");
            Serial.print(String(pot_val_temp,5));
            Serial.print(">");
            Serial.flush();
    */        
            ptr[m]->decrease(abs(pot_val_change));
            curr_pot_vals[m] = constrain( curr_pot_vals[m] - abs(pot_val_change),0,99);        
          }
          else
          {
            ptr[m]->decrease(abs(pot_val_change));
            curr_pot_vals[m]--;
          }
      
          DebugPrint("POT_DEC_M",double(m),2);
          DebugPrint("POT_DEC_VAL",double(pot_val_change),2);
      
      /*    
          Serial.print("<POT DEC:m=");
          Serial.print(m);
          Serial.print(" VAL=");
          Serial.print(pot_val_change);
          Serial.print(">");
          Serial.flush();
      */
        }
        else if (((pot_val_change/abs(pot_val_change)) == -1) && (curr_pot_vals[m] == 0)) 
        {
      
          DebugPrint("POT_DEC_M_MINPOS",double(m),2);
          DebugPrint("POT_DEC_VAL",double(pot_val_change),2);
      
      /*
          Serial.print("<POT DEC but MIN POS:m=");
          Serial.print(m);
          Serial.print(">");
          Serial.flush();
      */
          if(fabs(integrator_1) > fabs(integrator_2) && (integrator_count > 1))
          //not going in the good direction, bumping
          {
            pot_val_change = 1;
            DebugPrint("POT_REV_MIN_M",double(m),2);
            DebugPrint("POT_REV_VAL",double(pot_val_change),2);
      
      
        /*
            Serial.print("<POT REVERSE AT MIN POS:m=");
            Serial.print(m);
            Serial.print(">");
            Serial.flush();
        */
          }
          else if (m>val_vco*max_pot)
          {
            // going good direction, but reached limit of pot.
            m--;
            switch_pot = true;
          }
          else {tuned = true;} // reached last pot.
          

        }
        else if (((pot_val_change/abs(pot_val_change)) == 1) && (curr_pot_vals[m] == 99)) 
        {
        
          DebugPrint("POT_INC_MAX_M",double(m),2);
          DebugPrint("POT_INC_VAL",double(pot_val_change),2);
      
          /*
          Serial.print("<POT INC but MAX POS:m=");
          Serial.print(m);
          Serial.print(">");
          Serial.flush();
          */

          if(fabs(integrator_1) > fabs(integrator_2) && (integrator_count > 1))
          {
            //not going in the good direction, bumping
    
            pot_val_change = -1;
            DebugPrint("POT_REV_MAX_M",double(m),2);
            DebugPrint("POT_INC_VAL",double(pot_val_change),2);
      
        /*
            Serial.print("<POT REVERSE AT MAX POS:m=");
            Serial.print(m);
            Serial.print(">");
            Serial.flush();
        */
          }
          else if (m>val_vco*max_pot)
          {
            // going good direction, but reached limit of pot.
            m--;
            switch_pot = true;
          }
          else {tuned = true;} //reached end pot
          
        }
      } // end pot_val_change != 0

    
      if (switch_pot) 
      {

        //switch_pot = false;
        if (pot_val_change != 0)
        {
          if (((pot_val_change/abs(pot_val_change)) == 1) && (curr_pot_vals[m+1] < 99)) 
          {

          /* delete that ??
            if ((curr_pot_vals[m+1] > 1) && (m == 1 + val_vco*max_pot))
            {
              Serial.println("<SW-M2-NORM>");
              ptr[m+1]->increase(pot_val_change);
              curr_pot_vals[m+1]++;
              pot_val_change = -1;
            }
          */ 
            if ((curr_pot_vals[m+1] <= 1) && (m == 1 + val_vco*max_pot))
            {
              // When reaching high frequencies, we increase pot for m==2 only when its value is greater than 1.    
          
              // We are above frequency, we have not increased pot2, so we can only increase pot 1 and pot 0.
              /*
              Serial.print("<SW-HIGHFREQ>");
              Serial.flush();
              */
              DebugPrint("SW_HIGHFREQ",double(m),2);
              pot_val_change = 1;
            }
            //else if ((curr_pot_vals[m] > 49) && m==val_vco*max_pot)
            else if ((curr_pot_vals[m] > 49) && m<=(val_vco*max_pot+1))
            {
              /*
              Serial.print("<SW-M1/2-NORM_FROM_BELOW>");
              Serial.flush();
              */
              DebugPrint("SW_M1M2_NORM_FROM_BELOW",double(m),2);
              pot_val_change = 1;
              ptr[m+1]->increase(pot_val_change);
              curr_pot_vals[m+1]++;
              pot_val_change = -1;
            
            }
            //else if ((curr_pot_vals[m] < 49) && m==val_vco*max_pot)
            else if ((curr_pot_vals[m] <= 49) && m<=(val_vco*max_pot +1))
            {
              /*
              Serial.println("<SW-M1/2-NORM_FROM_ABOVE>");
              Serial.flush();
              */
              DebugPrint("SW_M1M2_NORM_FROM_ABOVE",double(m),2);
              pot_val_change = -1;
              ptr[m+1]->decrease(abs(pot_val_change));
              curr_pot_vals[m+1]--;
              pot_val_change = 1;
            
            }
            
          }
          //should never happen...
          else if (((pot_val_change/abs(pot_val_change)) == -1) && (curr_pot_vals[m+1] > 0)) 
          {
            ptr[m+1]->decrease(abs(pot_val_change));
            curr_pot_vals[m+1]--;
            pot_val_change = 1;

            DebugPrint("NEVER_M",double(m),2);
            /*
            Serial.print("<SW-DEC POT:m=");
            Serial.print(m);
            Serial.print(">");
            Serial.flush();
            */
            
          }
          else if (((pot_val_change/abs(pot_val_change)) == -1) && (curr_pot_vals[m+1] = 0))
          {
          //do nothing. 
          DebugPrint("DO_NOTHING",double(m),2);
          }
        }
        else 
        {
          DebugPrint("SWITCH_POT_NO_CHG",double(m),2);
        }

        if ((pot_val_change == 0) && (curr_pot_vals[m] > 49))
        {
          pot_val_change = -1;
          DebugPrint("SW_POT_VAL_FROM_0_TO_-1,NO_DEC_M+1",double(m),2);
        }
        else if ((pot_val_change == 0) && (curr_pot_vals[m] <= 49) && (curr_pot_vals[m+1]>0))
        {
          ptr[m+1]->decrease(1);
          curr_pot_vals[m+1]--;
          DebugPrint("SW_POT_VAL_0_TO_1,DEC_M+1",double(m),2);
          pot_val_change = 1;
          /*
          Serial.print("<SW-M1/2-NORM_FROM_ABOVE:");
          Serial.print(m);
          Serial.print(">");
          Serial.flush();
         
          */

          
        }
         
      }
    // test delay(1000);
    } //end if(integrator_count > 0)

    integrator_1 = integrator;

    if (newmethod) 
    {
      SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      integrator = (float) dintegrator;
    }
    else 
    {
      integrator = MeasureFrequencyDelta(tunetime,samples_number,float(freq));
    }

    dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
    

    dtostrf(integrator, 10, 3, charintegrator);
    sprintf (charintegratorfmt, "<INTEG:%s>",charintegrator);
    Serial.print(charintegratorfmt);
    

    DebugPrint("DEV_CENTS",dev_cents,2);

    
    // We change tuning direction and m only when we have at least 2 integrator values
  if(integrator_count >= 0) 
  {

    float absintegrator = fabs(integrator);
    float absintegrator_1 = fabs(integrator_1);
    //float absintegrator_2 = fabs(integrator_2);
    

  //  if ((absintegrator > absintegrator_1) && (absintegrator_1 > absintegrator_2)
 //  && ( int(integrator/absintegrator) == int(integrator_1/absintegrator_1) ) 
//   && ( int(integrator_1/absintegrator_1) == int(integrator_2/absintegrator_2) ))

    if ((absintegrator > absintegrator_1) 
    && ( int(integrator/absintegrator) == int(integrator_1/absintegrator_1) )) 

      {
        switch_pot = false;
      pot_val_change = int(integrator/absintegrator);
      
    
      DebugPrint("SIG_CHG",double(m),2);
  
      }

    if ((absintegrator < absintegrator_1) 
    && ( int(integrator/absintegrator) == int(integrator_1/absintegrator_1) )) 
    //if ( (absintegrator < absintegrator_1) && (absintegrator_1 < absintegrator_2)
    // && ( int(integrator/absintegrator) == int(integrator_1/absintegrator_1) ) 
    // && ( int(integrator_1/absintegrator_1) == int(integrator_2/absintegrator_2) ))
      {
      DebugPrint("SIG_UNCHG",double(m),2);
      switch_pot = false;
      }
      /*
    if ( (absintegrator < absintegrator_1) && (absintegrator_1 > absintegrator_2)
    && ( int(integrator/absintegrator)  == int(integrator_1/absintegrator_1) )  
    && ( int(integrator_1/absintegrator_1) == int(integrator_2/absintegrator_2) ))
  
      {
      
      DebugPrint("SIG_CORRECTING",double(m));
      
  
      }
      */
    else if ( int(integrator/absintegrator) != int(integrator_1/absintegrator_1) )
    {
      
      //switch_pot = true;
      switch_pot = !switch_pot; // only switch_pot if it was false from the previous run.
      // Zero crossing detection.
      // pot_val_change reflects the direction of tuning for pots, we crossed 0, and we are tuning from "under"
      // Because initially pots are setup high resistance ~99 so we cannot finetune by decreasing frequency initially.
      // We have to check the signs of integrator_1 and integrator_2 to see if we cross from under or above. 
      if (integrator > 0) 
      {
        //We are above frequency, we have to change current pot setting to go under. (increase pot, decrease freq)
        pot_val_change = 1;
      }
      else if (integrator < 0) 
      {
        //We are under frequency. Change nothing
        pot_val_change = 0;
      }
      
      // Checking that m is in tuning bounds ( m > pot index for the most fine tune pot of the subosc)
      if (m > (val_vco*max_pot)) 
      {
      // Decreasing m for the tuning of the next pot.
        if (switch_pot)
        {
          m--;
          // Reseting integrator count
          integrator_count = 0;
        }
      }
      // We are at m=0 and check that we are not tuned yet, special case.
      // Since we crossed zero, we have to choose the pot setting
      // between current integrator and last integrator.
      else if (!tuned) 
      { 
        pot_val_change = int(integrator/absintegrator);

        if (absintegrator < absintegrator_1)
        {
          //current integrator is better, change nothing.
        }

        else if (absintegrator > absintegrator_1)
        {
          // previous integrator is better, correct pot.
          // but check that we are in bounds for pot setting

          if((pot_val_change == 1) && (curr_pot_vals[val_vco*max_pot] < 99)) 
          {
            ptr[val_vco*max_pot]->increase(pot_val_change);
            curr_pot_vals[val_vco*max_pot]++;
          }
          else if ((pot_val_change == -1) && (curr_pot_vals[val_vco*max_pot] > 0)) 
          {
            ptr[val_vco*max_pot]->decrease(abs(pot_val_change));
            curr_pot_vals[val_vco*max_pot]--;
          }

        }

        // perform a final check.
        if (newmethod) 
        {
          SingleCountFrequencyDelta(50,freq,dintegrator,level); 
          integrator = (float) dintegrator;
        }
        else
        {
          integrator = MeasureFrequencyDelta(tunetime,samples_number,float(freq));
        }
            
           
        DebugPrint("INTEG_FIN_TUNED",double(integrator),2);
        PrintDigiPot(midi_to_pot,val_vco);
        Serial.flush();
        tuned = true;
      }
    }
  }

 states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
 states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
 states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
 states_integrator[num_integrations] = integrator;
    
 integrator_2 = integrator_1;
 integrator_1 = integrator;
 
 integrator_count++;
 num_integrations++;
 
} // end tuned

           
                 
        min_integrator = fabs(states_integrator[0]);
        
        for (p=0;p<num_integrations;p++) {


          //DebugPrint("P",double(p));
          //DebugPrint("VAL",double(states_integrator[p]));
          
          Serial.print("<P=");
          Serial.print(p);
          Serial.print(" VAL=");
          Serial.print(String(states_integrator[p],5));
          Serial.print(" POT=");
          Serial.print(states[p][0]);
          Serial.print(",");
          Serial.print(states[p][1]);
          Serial.print(",");       
          Serial.print(states[p][2]); 
          Serial.print(">");
          Serial.flush();

          if (fabs(states_integrator[p]) < min_integrator) { 
            min_integrator = fabs(states_integrator[p]);
            best_index = p;
          }
        
        }
        Serial.print("<BEST_POT_VALS:");
        Serial.print(states[best_index][0]);
        Serial.print(",");
        Serial.print(states[best_index][1]);
        Serial.print(",");       
        Serial.print(states[best_index][2]);
        Serial.print(">");
        Serial.print("<BEST_P:");
        Serial.print(best_index);
        Serial.print(">");
        Serial.flush();
        

        int k = 0;
        for (m=val_vco*max_pot;m<val_vco*max_pot + max_pot;m++) 
        {
          pot_val_change = states[best_index][m - val_vco*max_pot] - curr_pot_vals[m];
          if (pot_val_change > 0) 
          { 
            curr_pot_vals[m] += pot_val_change;
            for(k=0;k<pot_val_change;k++) 
            {
              //delay(10);
              ptr[m]->increase(1);
            }
              
          }
          else 
          {
            curr_pot_vals[m] += pot_val_change;
            for(k=0;k<abs(pot_val_change);k++) 
            {
              //delay(10);
              ptr[m]->decrease(1);
            }
                        
          }
            
        }
        /*
        checkserial();
        sprintf (charpotvals, "<%02u,%02u,%02u>",curr_pot_vals[0],curr_pot_vals[1],curr_pot_vals[2]);
        Serial.print(charpotvals);            
        */
if (newmethod)
{
  SingleCountFrequencyDelta(50,freq,dintegrator,level); 
  integrator = (float) dintegrator;
  
}
else
{
  integrator = MeasureFrequencyDelta(tunetime*2,samples_number,float(freq));
}

}


void NewAutoTune(DigiPot *ptr[6], byte (&curr_pot_vals)[6], double freq, bool level, int val_vco) 
{

  static byte max_freq_pot[6] = {99,99,99,99,99,99};
  static bool max_freq_vco0_set = false;
  static bool max_freq_vco1_set = false;
  int m;
  int max_pot = 3;
  int pot_val_change;

  if (freq == minfreq)
  {
    MaxVcoPots(ptr,curr_pot_vals,val_vco);
    Serial.print("<Autotune_end>");
    Serial.flush();
    return;
  }
  
  DebugPrint("FREQ",double(freq),0);
  DebugPrint("MAXFREQ",double(maxfreq),0);

  if (freq == maxfreq) 
  {
    DebugPrint("MAXFREQ!",double(maxfreq),0);
    if ((max_freq_vco0_set && (val_vco == 0)) || (max_freq_vco1_set && (val_vco == 1)))
    {
      DebugPrint("MAXFREQ_RECALL",double(maxfreq),0);
      PrintDigiPot(max_freq_pot,val_vco);
      PrintDigiPot(curr_pot_vals,val_vco);
      for (m=val_vco*max_pot;m<val_vco*max_pot + max_pot;m++) 
      {
        pot_val_change = int(max_freq_pot[m]) - int(curr_pot_vals[m]);
        if (pot_val_change > 0) 
        { 
          curr_pot_vals[m] += pot_val_change;
          DebugPrint("MAX_FREQ_SET_VCO",double(val_vco),0);
          ptr[m]->increase(pot_val_change);
            
        }// end if max_freq
        else if (pot_val_change <0)
        {
          curr_pot_vals[m] += pot_val_change;
          DebugPrint("MAX_FREQ_SET_VCO",double(val_vco),0);
          ptr[m]->decrease(abs(pot_val_change));                  
        }
      } // end max_freq_pot change
      PrintDigiPot(curr_pot_vals,val_vco);
      return;
    } // end max_freq_recall
  } // end if max_freq
  
  int num_integrations = 0;
  int max_integrations = 90;
  int states[max_integrations][3];
  float states_integrator[max_integrations];
  int samples_number = 20;
 
  
  
  pot_val_change = -1;
  double pot_val_temp = 0.0;
  double pot_val_temp2 = 0.0;
  
  double dev_cents = 0.0;
  const double tune_thresh = 3.0;
  
 
  m = val_vco*max_pot + max_pot -1;
  int rstep;
  int p;

  int bias;

  bool tuned = false;
  double dintegrator = 0.0;
  double dintegrator2 = 0.0;
  
  float integrator = 0.0;
  float integrator_1 = 0.0;
  float min_integrator = 0.0;

  int integrator_count = 0;

  int best_index = 0;
  byte curr_pot_val_bck = 0;



  SingleCountFrequencyDelta(50,freq,dintegrator,level); 
  integrator = (float) dintegrator;
  dev_cents = 1200 * log((integrator + freq)/freq)/log(2);

  pot_val_change = int(integrator/fabs(integrator));
  DebugPrint("INIT_INTEGRATOR",double(integrator),2);
  
  while(!tuned) 
  {
    if (num_integrations == max_integrations) 
    { 
      tuned = true;
      break;
    }
    if (fabs(dev_cents) <= tune_thresh) 
    { 
      tuned = true;
      DebugPrint("THRESH_ATT",double(fabs(dev_cents)),0);
      
      states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
      states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
      states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[num_integrations] = integrator;
      break;
    } //end tune thresh att

    if (m == (max_pot*val_vco +2)) 
    {        
      // check bounds
      if((pot_val_change>0) && (curr_pot_vals[m] == 99))
      {
        m--;
        // we can low m in all cases because we are in high pot check.
        integrator_count = 0;
        continue;
        DebugPrint("BOUND_LIMIT_DEC_M",double(curr_pot_vals[m]),2);
      
      }
      else if((pot_val_change<0) && (curr_pot_vals[m] == 0))
      {
        m--;
        // we can low m in all cases because we are in high pot check.
        integrator_count = 0;
        continue;
        DebugPrint("BOUND_LIMIT_DEC_M",double(curr_pot_vals[m]),2);
      }
      
      DebugPrint("HIGH_POT_IDX",double(m),2);
      DebugPrint("HIGH_POT_BEFORE",curr_pot_vals[m],2);
      DebugPrint("HIGH_INT_BEFORE",double(integrator),2);
      
      curr_pot_vals[m] += pot_val_change;
    
      if (pot_val_change > 0) 
      { 
        ptr[m]->increase(pot_val_change); 
      }
      else if (pot_val_change < 0)
      { 
        ptr[m]->decrease(abs(pot_val_change)); 
      }
    
      integrator_1 = integrator;  
      SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      integrator = (float) dintegrator;
      pot_val_change = int(integrator/fabs(integrator));
      dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      
      DebugPrint("HIGH_POT_IDX",double(m),2); 
      DebugPrint("HIGH_POT_AFTER ",curr_pot_vals[m],2);
      DebugPrint("HIGH_INT_AFTER ",double(integrator),2);
      DebugPrint("POT_VAL_AFTER  ",pot_val_change,2);

      if (  int(integrator/fabs(integrator)) != int(integrator_1/fabs(integrator_1)) )
      {
        DebugPrint("ZERO_CROSS",double(curr_pot_vals[m]),2);

        // zero cross, check for change room in up or down direction for med/low pots.
        bias = 5000 - (100*(curr_pot_vals[m-1]) + curr_pot_vals[m-2]*10);  
        // approx. bias in ohms
        // if bias positive, there is room for going up (decrease freq)
        // if bias negative, there is room for going down (increase freq)
        pot_val_temp = d_hertz_to_R(freq,val_vco)*integrator*1000; // * 1000 to convert kOhms in Ohms
        pot_val_temp2 = d_hertz_to_R(freq,val_vco)*integrator_1*1000; // * 1000 to convert kOhms in Ohms

        DebugPrint("POT_VAL_TEMP", double(pot_val_temp),2);
        DebugPrint("POT_VAL_TEMP2", double(pot_val_temp2),2);
        DebugPrint("BIAS", double(bias),2);
                
        // pot_val_temp approx change in R (ohms) required.
        if (bias > 0)
        {
          if  (pot_val_temp > 0)
          {
            if (bias > pot_val_temp)
            {
              // bias is higher than pot_val temp, there is room for change using integrator
              // change nothing
              DebugPrint("CHG_NOTHING_VAL1", double(pot_val_temp),2);
              DebugPrint("CHG_NOTHING_BIAS1", double(bias),2);
                
            }
            else if (bias < pot_val_temp) 
            {

              DebugPrint("REVERT_VAL2", double(pot_val_temp),2);
              DebugPrint("REVERT_BIAS2", double(bias),2);
              integrator = integrator_1;
              curr_pot_vals[m] += pot_val_change;
              if (pot_val_change > 0) 
              { 
                ptr[m]->increase(pot_val_change);
              }
              else if (pot_val_change < 0)
              { 
                ptr[m]->decrease(abs(pot_val_change)); 
              }
            }           
          }
          else if (pot_val_temp < 0)// pot_val_temp < 0
          {
            if (bias > pot_val_temp2) // then pot_val_temp2 > 0, because of sign change
            {

              DebugPrint("REVERT_VAL3", double(pot_val_temp2),2);
              DebugPrint("REVERT_BIAS3", double(bias),2);
              integrator = integrator_1;
              curr_pot_vals[m] += pot_val_change;
              if (pot_val_change > 0) 
              { 
                ptr[m]->increase(pot_val_change);
              }
              else if (pot_val_change < 0)
              { 
                ptr[m]->decrease(abs(pot_val_change)); 
              }
            }
            else if (bias < pot_val_temp2)
            {
              // bias is lower than pot_val temp, there is no room for change using integrator_1
              // change nothing

              DebugPrint("CHG_NOTHING_VAL4", double(pot_val_temp2),2);
              DebugPrint("CHG_NOTHING_BIAS4", double(bias),2);

            }
          }
          
        }
        else if (bias < 0)
        {

          if  (pot_val_temp < 0)
          {
            if (bias < pot_val_temp)
            {
              // bias is lower than pot_val temp, there is room for change using integrator
              // change nothing
              DebugPrint("CHG_NOTHING_VAL5", double(pot_val_temp),2);
              DebugPrint("CHG_NOTHING_BIAS5", double(bias),2);

            }
            else if (bias > pot_val_temp)
            {

              DebugPrint("REVERT_VAL6", double(pot_val_temp),2);
              DebugPrint("REVERT_BIAS6", double(bias),2);
              integrator = integrator_1;
              curr_pot_vals[m] += pot_val_change;
              if (pot_val_change > 0) 
              { 
                ptr[m]->increase(pot_val_change);
              }
              else if (pot_val_change < 0)
              { 
                ptr[m]->decrease(abs(pot_val_change)); 
              }
            }            
          }
          else if (pot_val_temp > 0)
          {
            if (bias < pot_val_temp2) // then pot_val_temp2 < 0, because of sign change
            {
              // bias is lower than pot_val temp2, there is room for change using integrator_1
              // revert
              DebugPrint("REVERT_VAL7", double(pot_val_temp2),2);
              DebugPrint("REVERT_BIAS7", double(bias),2);

              integrator = integrator_1;
              curr_pot_vals[m] += pot_val_change;
              if (pot_val_change > 0) 
              { 
                ptr[m]->increase(pot_val_change);
              }
              else if (pot_val_change < 0)
              { 
                ptr[m]->decrease(abs(pot_val_change)); 
              }

            }
            else if (bias > pot_val_temp2)
            {
              // bias is higher thant pot_val_temp2, no room for change using integrator_1, use integrator
              // change nothing
              DebugPrint("CHG_NOTHING_VAL8", double(pot_val_temp),2);
              DebugPrint("CHG_NOTHING_BIAS8", double(bias),2);

            }
          }
          
        }
        
        // zero cross, is integrator or integrator_1 closest ?
        /*
        if (fabs(integrator) < fabs(integrator_1))
        {
          // Change nothing, current value closer.
          DebugPrint("CHANGE_NOTHING",double(curr_pot_vals[m]));
        }
        else
        {
          // revert
          curr_pot_vals[m] += pot_val_change;
          if (pot_val_change > 0) 
          { 
            ptr[m]->increase(pot_val_change);
          }
          else 
          { 
            ptr[m]->decrease(abs(pot_val_change)); 
          }
          //pot_val_change = -pot_val_change;
          
          delay(500);        
          SingleCountFrequencyDelta(50,freq,dintegrator,level);
          SingleCountFrequencyDelta(50,freq,dintegrator2,!level);
          
          if (fabs(float(dintegrator)) < fabs(float(dintegrator2)))
          { 
            integrator =  (float) dintegrator;
            DebugPrint("LEVEL CHECK, HIGH BEST",dintegrator);
            DebugPrint("LEVEL CHECK, LOW WORSE",dintegrator2);            
          }
          else
          {
            integrator = (float) dintegrator2;  
            DebugPrint("LEVEL CHECK, HIGH WORSE",dintegrator);
            DebugPrint("LEVEL CHECK, LOW BETTER",dintegrator2);            
         
          }

          pot_val_change = int(integrator/fabs(integrator));          
          dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
          DebugPrint("REVERSE VAL",double(curr_pot_vals[m]));
          DebugPrint("REVERSE INT",double(integrator));
        }
        */
        m--;
        integrator_count = 0;
      
      } // zero cross check end
      
      else  
      { 
        DebugPrint("CONTINUE_CHANGE",double(curr_pot_vals[m]),2);
        integrator_count++;
      } 
      
      states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
      states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
      states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[num_integrations] = integrator;
          
      
      num_integrations++;
    
    } // end high pot check.
    
    else if (m < (max_pot*val_vco +2)) 
    { //start low pot check      
      SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      integrator = (float) dintegrator;
      dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      rstep = pow(10,m - max_pot*val_vco +1);
      //pot_val_temp = ((1/0.28e-6)/(freq*freq*rstep))*integrator;
      pot_val_temp = -(d_hertz_to_R(freq,val_vco)*integrator*1000/rstep); // * 1000 to convert kOhms in Ohms
     
      //pot_val_temp = ((1/0.28e-6)/(freq*freq*10))*integrator;
      curr_pot_val_bck = curr_pot_vals[m];
      
      DebugPrint("LOW_POT_IDX",double(m),2);
      DebugPrint("LOW_POT_BEFORE",curr_pot_vals[m],2);
      DebugPrint("LOW_INT_BEFORE",double(integrator),2);
   
      //check bounds
      if((pot_val_temp>0) && (curr_pot_vals[m] == 99))
      {
        if (m == val_vco*max_pot) 
        { 
          DebugPrint("OUT_OF_BOUNDS_LOW_POT",double(curr_pot_vals[m]),2);
          tuned = true; 
          break;
        }
        m--;
        continue;
      }
      else if((pot_val_temp<0) && (curr_pot_vals[m] == 0))
      {
        if (m == val_vco*max_pot) 
        { 
          DebugPrint("OUT_OF_BOUNDS_LOW_POT",double(curr_pot_vals[m]),2);
          tuned = true; 
          break;
        }
        m--;
        continue;
      } // end check bounds

      if (pot_val_temp > 0)
      {
        pot_val_change = max(int(pot_val_temp - 0.5),1);
        ptr[m]->increase(pot_val_change);
        curr_pot_vals[m] = constrain( curr_pot_vals[m] + pot_val_change,0,99);        
        
      }
      else if (pot_val_temp < 0)
      {
        pot_val_change = min(int(pot_val_temp - 0.5),-1);
        ptr[m]->decrease(abs(pot_val_change));
        curr_pot_vals[m] = constrain( curr_pot_vals[m] + pot_val_change,0,99);       
      }
      
      integrator_1 = integrator;  
      SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      integrator = (float) dintegrator;
      
      //pot_val_change = int(integrator/fabs(integrator));
      dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      
      DebugPrint("LOW_POT_IDX",double(m),2);
      DebugPrint("LOW_POT_AFTER",curr_pot_vals[m],2);
      DebugPrint("LOW_INT_AFTER",double(integrator),2);
      
      if (int(integrator/fabs(integrator)) != int(integrator_1/fabs(integrator_1)))
      {
        // zero cross, is integrator or integrator_1 closest ?
        DebugPrint("ZERO_CROSS",double(curr_pot_vals[m]),0);

        if (fabs(integrator) <= fabs(integrator_1))
        {
          // Change nothing, current value closer.
          DebugPrint("CHANGE_NOTHING",double(curr_pot_vals[m]),2);

        }
        else
        {
            // revert
          pot_val_change = curr_pot_val_bck - int(curr_pot_vals[m]);
          integrator = integrator_1;
          
          if (pot_val_change > 0) 
          { 
            ptr[m]->increase(pot_val_change); 
          }
          else if (pot_val_change < 0)
          { 
            ptr[m]->decrease(abs(pot_val_change)); 
          }

          DebugPrint("REVERSE VAL",double(curr_pot_vals[m]),2);
        }
        if (m == (val_vco*max_pot)) 
        { 
          DebugPrint("LAST_POT_TUNED",double(curr_pot_vals[m]),0);
          tuned = true; 
          break;       
        }
        // we decrease m and reset integrator count for this m
        m--;
        integrator_count = 0;
      } // end zero cross check.
      else 
      { 
        DebugPrint("CONTINUE_CHANGE",double(curr_pot_vals[m]),0);
        integrator_count++; // zero not crossed, we keep on changing pot for this m
      } // end no zero cross check
      
      states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
      states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
      states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[num_integrations] = integrator;
      num_integrations++; //increasing num_integrations
      
      // check bounds.
    } // end low pot check             
  } // end while tuned

min_integrator = fabs(states_integrator[0]);
        
  for (p=0;p<num_integrations;p++) 
  {


    //DebugPrint("P",double(p));
    //DebugPrint("VAL",double(states_integrator[p]));
    
    Serial.print("<P=");
    Serial.print(p);
    Serial.print(" VAL=");
    Serial.print(states_integrator[p]);
    Serial.print(" POT=");
    Serial.print(states[p][0]);
    Serial.print(",");
    Serial.print(states[p][1]);
    Serial.print(",");       
    Serial.print(states[p][2]); 
    Serial.print(">");
    Serial.flush();

    if (fabs(states_integrator[p]) < min_integrator) { 
      min_integrator = fabs(states_integrator[p]);
      best_index = p;
    }

  }
  Serial.print("<BEST_POT_VALS:");
  Serial.print(states[best_index][0]);
  Serial.print(",");
  Serial.print(states[best_index][1]);
  Serial.print(",");       
  Serial.print(states[best_index][2]);
  Serial.print(">");
  Serial.print("<BEST_P:");
  Serial.print(best_index);
  Serial.print(">");
  Serial.flush();


  int k = 0;
  for (m=val_vco*max_pot;m<val_vco*max_pot + max_pot;m++) 
  {
    pot_val_change = int(states[best_index][m - val_vco*max_pot]) - int(curr_pot_vals[m]);
    if (pot_val_change > 0) 
    { 
      curr_pot_vals[m] += pot_val_change;
      for(k=0;k<pot_val_change;k++) 
      {
        //delay(10);
        DebugPrint("POT_LAST_CHANGE",double(1),2);
        ptr[m]->increase(1);
      }
        
    }
    else if (pot_val_change < 0) 
    {
      curr_pot_vals[m] += pot_val_change;
      for(k=0;k<abs(pot_val_change);k++) 
      {
        //delay(10);
        ptr[m]->decrease(1);
        DebugPrint("POT_LAST_CHANGE",double(-1),2);
      }
                  
    }
      
  } // end retune pots to best values

  if (freq == maxfreq)
  {

    DebugPrint("MAXFREQ_SET",double(maxfreq),2);

    if ((val_vco == 0) && !max_freq_vco0_set)
    {
      max_freq_pot[0] = curr_pot_vals[0];
      max_freq_pot[1] = curr_pot_vals[1];
      max_freq_pot[2] = curr_pot_vals[2];
      max_freq_vco0_set = true;
      DebugPrint("POT_MAXFREQ_SET",double(val_vco),0);
    
      
    }
    else if ((val_vco == 1) && !max_freq_vco1_set)
    {
      max_freq_pot[3] = curr_pot_vals[3];
      max_freq_pot[4] = curr_pot_vals[4];
      max_freq_pot[5] = curr_pot_vals[5];
      max_freq_vco1_set = true;
      DebugPrint("POT_MAXFREQ_SET",double(val_vco),0);
    
      
    }
  }

} // end NewAutoTune

void writeEEPROMpots (byte noteindex, byte pot2, byte pot1, byte pot0, float deviation, byte vcosel)
{

int tuneblock_size = 49*(sizeof(pot2)+sizeof(pot1)+sizeof(pot0)+sizeof(deviation));
int addr = vcosel*tuneblock_size + noteindex*(sizeof(pot2)+sizeof(pot1)+sizeof(pot0)+sizeof(deviation));

//Serial.print("<addr:");
//Serial.print(addr);
//Serial.print(">");


EEPROM.put(addr,pot2);
addr += sizeof(pot2);
EEPROM.put(addr,pot1);
addr += sizeof(pot1);
EEPROM.put(addr,pot0);
addr += sizeof(pot0);
EEPROM.put(addr,deviation);
}

void readEEPROMpots (byte noteindex, byte *pot2, byte *pot1, byte *pot0, float *deviation, byte vcosel)
{

int tuneblock_size = 49*(sizeof(*pot2)+sizeof(*pot1)+sizeof(*pot0)+sizeof(*deviation));
int addr = vcosel*tuneblock_size + noteindex*(sizeof(*pot2)+sizeof(*pot1)+sizeof(*pot0)+sizeof(*deviation));


//Serial.print("<addr:");
//Serial.print(addr);
//Serial.print(">");



EEPROM.get(addr,*pot2);
addr += sizeof(*pot2);
EEPROM.get(addr,*pot1);
addr += sizeof(*pot1);
EEPROM.get(addr,*pot0);
addr += sizeof(*pot0);
EEPROM.get(addr,*deviation);
}


void dumpEEPROMpots (byte *all_pot_vals, float *all_pot_devs, byte vcosel)
{
byte k;
all_pot_vals += 3;
all_pot_devs++;
int tuneblock_size = 49*(sizeof(byte)+sizeof(byte)+sizeof(byte)+sizeof(float));


//Serial.print("<tuneblock:");
//Serial.print(tuneblock_size);
//Serial.print(">");



for (k=1;k<49;k++) {

int addr = tuneblock_size*vcosel + k*(sizeof(byte)+sizeof(byte)+sizeof(byte)+sizeof(float));

//Serial.print("<addr:");
//Serial.print(addr);
//Serial.print(">");


EEPROM.get(addr,*all_pot_vals);
addr += sizeof(byte);
all_pot_vals++;
EEPROM.get(addr,*all_pot_vals);
addr += sizeof(byte);
all_pot_vals++;
EEPROM.get(addr,*all_pot_vals);
addr += sizeof(byte);
all_pot_vals++;
EEPROM.get(addr,*all_pot_devs);
all_pot_devs++;  
}
}




void checkserial()
{
if (!Serial) {
delay(10);
//Serial.begin(9600,SERIAL_8E2);
Serial.begin(9600);
Serial.println("<reset>");  
}

}


void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = 'Z';
  char rc;
 

  if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();
      delay(10);
      //Serial.println(rc);
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
        }
      }
      else {
        //Serial.println("endmarker");
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
      }
    }
  }
}

void SetNotePots(DigiPot *ptr[6], byte (&pot_vals)[6], bool val_saw, int val_vco) {

  int m = 0;
  int max_pot;
  int pot_val_change;
  static int curr_pot_vals[6] = {99,99,99,99,99,99};
  
  if (!val_saw) {
  max_pot = 3;  
  }
  else {
  max_pot = 6;
  }

  for(m = val_vco*max_pot; m < max_pot + val_vco*max_pot; m++) {

    if (pot_vals[m] < 0) { pot_vals[m] = 0; }
    if (pot_vals[m] > 99) { pot_vals[m] = 99; }
    
    pot_val_change = int(pot_vals[m]) - int(curr_pot_vals[m]);

   if (pot_val_change > 0) {
      ptr[m]->increase(pot_val_change);
   }
   else {
      ptr[m]->decrease(abs(pot_val_change));
   }

    curr_pot_vals[m] = pot_vals[m];
  }


}

float CheckTuning(DigiPot *ptr[6], byte (&curr_pot_vals)[6], byte noteindex, int val_vco) 
{

float notefreq;
float integrator;

SetNotePots(pots,curr_pot_vals,0,vcosel);

notefreq = pgm_read_float(&(notes_freq[noteindex]));
integrator = MeasureFrequencyDelta(2,10,notefreq);

return integrator;

}


void setup() 
{

  //char charstart[4] = "<F>";
  char charend[4] = "<E>";
  char charackdev[4] = "<D>";

  //char openchar = '<';
  //char closechar = '>';


  char charspeed[14] = "<SERIAL 9600>";
  char charfreq[8];
  char printcharfreq[10];
    
  bool dumpeeprom = false;
  bool checknotes = false;
  bool checknotes_formula = true;
  bool checkpots = false;
  bool generatefreq = false;
  bool donothing = false;

  byte vco_pin = 4;
  byte max_pot = 3;
  byte notestart;
  byte noteend;
   
  //Serial.begin(9600,SERIAL_8E2);
  Serial.begin(9600);
  Serial.print(charspeed);
  Serial.flush();   
  delay(3000);

  StartAllPots(allpots);

  freqrefpin = 9;
  vcosel = 0;
  pinMode(vco_pin, OUTPUT);
  digitalWrite(vco_pin, !vcosel);

  // use default address 0
  //mcp.pinMode(0, OUTPUT);

  MaxVcoPots(pots,midi_to_pot,0);
  MaxVcoPots(pots,midi_to_pot,1);
  
  pot12.increase(99);
  
  pot12.decrease(5);
  delay(5000);

  if (donothing) { return;}

  byte pot_vals[49][3];
  float pot_devs[49];

  byte rpot2;
  byte rpot1;
  byte rpot0;
  float olddeviation;
  char chardev[30];
  char chardevfmt[32];

  pot_vals[0][2] = 99;
  pot_vals[0][1] = 99;
  pot_vals[0][0] = 99;
  pot_devs[0] = 0.0;
    

  if (dumpeeprom) 
  {   
    dumpEEPROMpots((byte *)pot_vals,(float *)pot_devs,vcosel);

    for (k=0;k<49;k++) 
    {

      notefreq = pgm_read_float(&(notes_freq[k]));
      dtostrf(notefreq, 6, 2, charfreq);

      sprintf(printcharfreq,"<[%s,", charfreq);
      Serial.print(printcharfreq);    
      Serial.print(pot_vals[k][2]);
      Serial.print(",");
      Serial.print(pot_vals[k][1]);
      Serial.print(",");
      Serial.print(pot_vals[k][0]);
      Serial.print("]>");
    
      /*
      // not required for python curve fitting script 
      dtostrf(pot_devs[k], 6, 2, chardev);
      sprintf(chardevfmt,"%s", chardev);

      Serial.print(chardevfmt);
      Serial.print("]>");
      */
      Serial.flush();
      //  delay(100); 
    } // end for all notes dumpeeprom
  //return;
  } // end dumpeeprom 

  double f1;
  double f2;
  double f_err;
  double f1_err;
  double f2_err;
  bool level = true;
  float duty;
  bool arb_vcosel = false;


  if (checkpots) { Check_all_pots_R(pots); return;}

  if (generatefreq) 
  {
    Serial.print("<generating freq start>");
    GenerateArbitraryFreq(midi_to_pot,400.0, 0.5, f1, f2);
    delay(120000);
    Serial.print("<generating freq end>");
    return;
  }

  notestart = 1;
  noteend = 49;


  midi_to_pot[vcosel*max_pot] = pot_vals[notestart - 1][2];
  midi_to_pot[vcosel*max_pot +1] = pot_vals[notestart - 1][1];
  midi_to_pot[vcosel*max_pot +2] = pot_vals[notestart - 1][0];


  Serial.print("<");
  Serial.print(midi_to_pot[vcosel*max_pot +2]);
  Serial.print(",");
  Serial.print(midi_to_pot[vcosel*max_pot +1]);
  Serial.print(",");
  Serial.print(midi_to_pot[vcosel*max_pot]);
  Serial.print(">");
  Serial.flush();

  float integrator;
  char charintegratorfmt[13];
  char charintegrator[7];


  SetNotePots(pots,midi_to_pot,0,vcosel);


  Serial.print("<midi_to_pot:");
  Serial.print(midi_to_pot[vcosel*max_pot +2]);
  Serial.print(midi_to_pot[vcosel*max_pot +1]);
  Serial.print(midi_to_pot[vcosel*max_pot]);
  Serial.print(">");


  notefreq = pgm_read_float(&(notes_freq[notestart-1]));
  /*
  integrator = MeasureFrequencyDelta(2,10,notefreq);


  dtostrf(integrator, 6, 2, charintegrator);
  sprintf (charintegratorfmt, "<VALUE:%s>",charintegrator);
  Serial.print(charintegratorfmt);
  */

  //FreqCount.begin(1000);



  for (k=notestart;k<noteend;k++) 
  {
    notefreq = pgm_read_float(&(notes_freq[k]));
    dtostrf(notefreq, 6, 2, charfreq);
    //Serial.print(charstart);
    
    //Serial.print(openchar);
    //Serial.print(charfreq);
    //Serial.print(closechar);
    checkserial();
    sprintf(printcharfreq,"<F%s>", charfreq);
    Serial.print(printcharfreq);
    Serial.flush();

    if (!checknotes && !checknotes_formula) 
    {
      AutoTune(pots,midi_to_pot,notefreq,vcosel,false,false);
    }
    else if (checknotes)
    {
      midi_to_pot[vcosel*max_pot] = pot_vals[k][2];
      midi_to_pot[vcosel*max_pot +1] = pot_vals[k][1];
      midi_to_pot[vcosel*max_pot +2] = pot_vals[k][0];

      CheckTuning(pots,midi_to_pot,k,vcosel);
    }
    else if (checknotes_formula)
    {

      f1 = 0.0;
      f2 = 0.0;
      f_err = 0.0;
      f1_err = 0.0;
      f2_err = 0.0;
      duty = 0.75;

      GenerateArbitraryFreq(midi_to_pot,notefreq, 0.75, f1, f2);
      //test delay(2000);
      Serial.print("<Generated_f1=");
      Serial.print(String(f1,3));
      Serial.print(">");
      Serial.print("<Generated_f2=");
      Serial.print(String(f2,3));
      Serial.print(">");
      Serial.flush();
      
      CountFrequencyDelta2(50,notefreq,f1,f2,f_err,f1_err,f2_err);
      //if (duty > 0.5) { level = !level; arb_vcosel = !arb_vcosel;}
      
      // f1 = high , f2 = low
      NewAutoTune(pots,midi_to_pot,f1,1,0);
      // test delay(100);
      CountFrequencyDelta2(50,notefreq,f1,f2,f_err,f1_err,f2_err);
      
      NewAutoTune(pots,midi_to_pot,f2,0,1);
      // test delay(100);
      CountFrequencyDelta2(50,notefreq,f1,f2,f_err,f1_err,f2_err);
      
      //integrator = CountFrequencyDelta(6,1000,notefreq);
      /*
      Serial.print("<ferr=");
      Serial.print(String(f_err,3));
      Serial.print(">");
      Serial.print("<f1err=");
      Serial.print(String(f1_err,3));
      Serial.print(">");
      Serial.print("<f2err=");
      Serial.print(String(f2_err,3));
      Serial.print(">");
      */
      /*
      dtostrf(integrator, 10, 3, charintegrator);
      sprintf (charintegratorfmt, "<INTEG:%s>",charintegrator);
      Serial.print(charintegratorfmt);
      */  

    } // end check notes formula
    // test 
    checkserial();
    delay(10);
    Serial.print(charend);
    Serial.flush();
    delay(10); 
    // test       
    
    //Serial.print("T,");
    //Serial.print(String(notefreq));
    //Serial.print(",");
    //Serial.print(String(midi_to_pot[0]));
    //Serial.print(",");
    //Serial.print(String(midi_to_pot[1]));
    //Serial.print(",");
    //Serial.print(String(midi_to_pot[2]));
    //Serial.println("");
    bool getdeviation = false;
    float deviation;
    

    while(!getdeviation) 
    {
      recvWithEndMarker();
      //Serial.println("devloop");
      if (newData == true) 
      {
        //Serial.println("<yesdata>");
        //Serial.print("<");
        //Serial.print(receivedChars);
        //Serial.println(">");
        deviation = atof(receivedChars);
        //sscanf( receivedChars, "%f\n", &deviation);
        newData = false;
        getdeviation = true;             
      }
      delay(10);
      //Serial.println("<nodata>");
    }
    delay(1000);
    
    readEEPROMpots (k, &rpot2, &rpot1, &rpot0, &olddeviation, vcosel);

    if(fabs(olddeviation) > fabs(deviation)  && (!checknotes) && (!checknotes_formula)) 
    {

      writeEEPROMpots (k, midi_to_pot[vcosel*max_pot + 2], midi_to_pot[vcosel*max_pot +1], midi_to_pot[vcosel*max_pot], deviation, vcosel);
      //delay(1000);
      //Serial.print("<new dev better>");
      //Serial.flush();
    
    }
    else
    { 
        //Serial.print("<old dev better>"); 
    }


    Serial.print(charackdev);
    Serial.flush();
    /*
    Serial.print("<rpot2=");
    Serial.print(rpot2);
    Serial.print(">");

    Serial.print("<rpot1=");
    Serial.print(rpot1);
    Serial.print(">");

    Serial.print("<rpot0=");
    Serial.print(rpot0);
    Serial.print(">");
    */   
    /*
    dtostrf(olddeviation, 6, 2, chardev);
    sprintf(chardevfmt,"<%s>", chardev);
    Serial.print("<olddev=>");
    Serial.print(chardevfmt);

    dtostrf(deviation, 6, 2, chardev);
    sprintf(chardevfmt,"<%s>", chardev);
    Serial.print("<newdev=>");
    Serial.print(chardevfmt);
    
    */ 

    //AutoTune(pots,midi_to_pot,notefreq,0,true);

  } // end for notes


    //FreqCount.end();

  /*
    for (k=0;k<49;k++) 
    {
      notefreq = pgm_read_float(&(notes_freq[k]));
      AutoTune(pots,midi_to_pot,notefreq,1);
      

      Serial.println("{");
      Serial.println(String(midi_to_pot[3]));
      Serial.println(",");
      Serial.println(String(midi_to_pot[4]));
      Serial.println(",");
      Serial.println(String(midi_to_pot[5]));
      Serial.println("},\n");
    }
    
    */
    

} // end setup


// flip the pin #0 up and down

void loop() {

  delay(10);
  
  
  if (Serial.available() > 0) {

  char inp = Serial.read();
  //Serial.println(inp);

    if (inp == '0') {
      vcosel = 0;
      Serial.println("VCOSEL 0");
      digitalWrite(4,HIGH);
    }
    else if(inp == '1') {
      vcosel = 1;
      Serial.println("VCOSEL 1");
      digitalWrite(4,LOW);
    }

    if (!vcosel) {

    if (inp == 's') {
    pot0.increase(1);
    (midi_to_pot[0])++;
    //intp0++;
    PrintDigiPot(midi_to_pot,0); 
  }

    if (inp == 'd') {
    pot1.increase(1);
    (midi_to_pot[1])++;
    //intp1++;
    PrintDigiPot(midi_to_pot,0);
  }

    if (inp == 'f') {
    pot2.increase(1);
    (midi_to_pot[2])++;
    //intp2++;
    PrintDigiPot(midi_to_pot,0);
  }

    if (inp == 'x') {
    pot0.decrease(1);
    (midi_to_pot[0])--;
    //intp0--;
    PrintDigiPot(midi_to_pot,0);
  }

    if (inp == 'c') {
    pot1.decrease(1);
    (midi_to_pot[1])--;
    //intp1--;
    PrintDigiPot(midi_to_pot,0);
  }

    if (inp == 'v') {
    pot2.decrease(1);
    (midi_to_pot[2])--;
    //intp2--;
    PrintDigiPot(midi_to_pot,0);
  }

    }

    else {

    if (inp == 's') {
    pot3.increase(1);
    (midi_to_pot[3])++;
    //intp3++;
    PrintDigiPot(midi_to_pot,1);
  }

    if (inp == 'd') {
    pot4.increase(1);
    (midi_to_pot[4])++;
    //intp4++;
    PrintDigiPot(midi_to_pot,1);
  }

    if (inp == 'f') {
    pot5.increase(1);
    (midi_to_pot[5])++;
    //intp5++;
    PrintDigiPot(midi_to_pot,1);
  }

    if (inp == 'x') {
    pot3.decrease(1);
    (midi_to_pot[3])--;
    //intp3--;
    PrintDigiPot(midi_to_pot,1); 
  }

    if (inp == 'c') {
    pot4.decrease(1);
    (midi_to_pot[4])--;
    //intp4--;
    PrintDigiPot(midi_to_pot,1);
  }

    if (inp == 'v') {
    pot5.decrease(1);
    (midi_to_pot[5])--;
    //intp5--;
    PrintDigiPot(midi_to_pot,1); 
  }
      
    
    }
}

}

