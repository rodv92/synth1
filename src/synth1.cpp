#include <FreqMeasure.h> 
#include <DigiPotX9Cxxx.h>
#include <Wire.h>
#include <MCP4725.h>
//#include <MemoryFree.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <FreqCount.h>
#include <eRCaGuy_Timer2_Counter.h>
#include "wiring_private.h"
#include "pins_arduino.h"
#include <MIDI.h>


//Adafruit_MCP23017 mcp0;
//Adafruit_MCP23017 mcp1;
//Adafruit_MCP23017 mcp2;


MCP4725 DAC0(0x60);  
MCP4725 DAC1(0x61);  

MIDI_CREATE_DEFAULT_INSTANCE();

byte DebugLevel = 2;

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


//volatile unsigned long RiseCount;
//volatile unsigned long FallCount;
double coarse_freq[2][200];

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

byte PWM_Note_Settings[49][7];

byte k;
byte n;
int vcosel;
char inp;
bool InTuning = false;
bool midimode;

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

void PrintDigiPot(byte (&curr_pot_vals)[6], byte vcosel, byte msgdebuglevel)
{

  if (msgdebuglevel <= DebugLevel)
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
void DebugPrintToken(String token, byte msgdebuglevel)
{

 if (msgdebuglevel <= DebugLevel)
  { 
  Serial.print("<");
  Serial.print(token);
  Serial.print(">");
  Serial.flush();
  }
}

void DebugPrintStr(String token, byte msgdebuglevel)
{
  
  if (msgdebuglevel <= DebugLevel)
  {
  Serial.print(token);
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
R = 1/(freq*0.066e-6);

return R;

}

double hertz_to_R_DAC(double freq, double voltage) {

double R;
R = 1/(0.066E-6*freq - voltage/(3*32300)); 
return R;

}

double d_hertz_to_R_v2(double freq, int subvco) {

double dR;
dR = -1/(pow(freq,2)*0.066e-6);

return dR;

}

double d_freq_to_d_volt(double delta_freq, int subvco) {

double dV;
dV = - delta_freq * (32300*0.066e-6)/0.32;

return dV;

}

void Adjust_DAC(double delta_V, int subvco) {

  int dac_steps;
  dac_steps = int(delta_V/(3/4095) + 0.5*(delta_V)/fabs(delta_V));
  
  
  if (subvco == 0) 
  {
    DAC0.setValue(2047 + dac_steps);  
  }
  else if (subvco == 1)
  {
    DAC1.setValue(2047 + dac_steps); 
  }

   

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
void saw_freq_duty_to_R(double freq, double duty, double &R1 , double &R2, double &f1, double &f2) {

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
Serial1.print("<f1,f2,duty,r1,r2:");
Serial1.print(String(f1,3));
Serial1.print(",");
Serial1.print(String(f2,3));
Serial1.print(",");
Serial1.print(String(duty,3));
Serial1.print(",");
*/


R1 = hertz_to_R(f1,0);
R2 = hertz_to_R(f2,1);

}

// convert saw freq components (subosc1 and 2 of OSC to resistance values of each timer R
void saw_freq_duty_to_R_DAC(double freq, double voltage, double duty, double &R1 , double &R2, double &f1, double &f2) {

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
Serial1.print("<f1,f2,duty,r1,r2:");
Serial1.print(String(f1,3));
Serial1.print(",");
Serial1.print(String(f2,3));
Serial1.print(",");
Serial1.print(String(duty,3));
Serial1.print(",");
*/


R1 = hertz_to_R_DAC(f1,voltage);
R2 = hertz_to_R_DAC(f2,voltage);

}



void R_to_pot_DAC(double R, byte &val_pot100K, byte subvco)
{

//input : R is the resistance to distribute among the two 100K digipots.
//input : subvco is the subvco selector (0 or 1)
//output : &val_pot100K, reference to the serially aggregated steps of the two 100K digipots,
// 0 to 99 = pot1/4, 100 to 199 = pot2/5. with pot1/4 being maxed out.

  double Rtmp;
  const double R1OOK_total_R_vco0 = 102.1;
  const double R1O0Kb_total_R_vco0 = 98.9;
  const double R1K_total_R_vco0 = 1.055;
  const double trim_pot_R_vco0 = 5.19; // with added wiper_R*3

  const double R1OOK_total_R_vco1 = 105.3;
  const double R1O0Kb_total_R_vco1 = 98;
  const double R1K_total_R_vco1 = 1.033;
  const double trim_pot_R_vco1 = 5.35; // with added wiper_R*3


  //const double wiper_R = 0.12;
  

  if (subvco == 0) {

    // R is in kOhms
    // we substract wiper resistance and trim pot resistance.
    if (Rtmp > R1O0Kb_total_R_vco0) // in this case we use all the steps of pot1 (+99)
    {
      Rtmp = R - R1O0Kb_total_R_vco0 - R1K_total_R_vco0 - trim_pot_R_vco0;
      val_pot100K = (byte) int(Rtmp/(R1OOK_total_R_vco0/99) + 0.5) + 99;
      //(byte) int(Rtmp/(R1OOKb_total_R_vco0/99) + 0.5) + 99;
    
    }
    else // only pot2 is required, pot1 stays at step 0.
    { 
      Rtmp = R - R1K_total_R_vco0 - trim_pot_R_vco0;
      val_pot100K = (byte) int(Rtmp/(R1O0Kb_total_R_vco0/99) + 0.5);
    }
    
  }
  
  else if (subvco == 1) 
  {

    if(Rtmp > R1O0Kb_total_R_vco1) // in this case we use all the steps of pot4
    {
      Rtmp = R - R1O0Kb_total_R_vco1 - R1K_total_R_vco1 - trim_pot_R_vco1;
      val_pot100K = (byte) int(Rtmp/(R1OOK_total_R_vco1/99) + 0.5) + 99;
      //(byte) int(Rtmp/(R1OOKb_total_R_vco0/99) + 0.5) + 99;
  
    }
    else // only pot5 is required, pot4 stays at step 0.
    {
      Rtmp = R - R1K_total_R_vco1 - trim_pot_R_vco1;
      val_pot100K = (byte) int(Rtmp/(R1O0Kb_total_R_vco1/99) + 0.5);
    }
    

  }


}
//Splits R to the settings of the three serial pots (100K, 10K, 1K)
void R_to_pot(double R, byte &val_pot100K, byte &val_pot10K, byte &val_pot1K, byte subvco) {

  double Rtmp;
  const double R1OOK_total_R_vco0 = 96.77;
  const double R1OK_total_R_vco0 = 10.08;
  const double R1K_total_R_vco0 = 1.055;
  const double trim_pot_R_vco0 = 1.765;

  const double R1OOK_total_R_vco1 = 97.26;
  const double R1OK_total_R_vco1 = 10.42;
  const double R1K_total_R_vco1 = 1.033;
  const double trim_pot_R_vco1 = 1.349;


  //const double wiper_R = 0.12;
  

  if (subvco == 0) {

  // R is in kOhms
  // we substract wiper resistance and trim pot resistance.
  Rtmp = R - trim_pot_R_vco0;

    if(Rtmp > R1OOK_total_R_vco0) 
    {

      val_pot100K = 99;
      Rtmp = Rtmp - R1OOK_total_R_vco0;
      val_pot10K = (byte) constrain(int(int(Rtmp/(R1OK_total_R_vco0/99))/5)*5,0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_vco0/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_vco0/99) + 0.5);
    }
    else {
      val_pot100K = (byte) constrain(int(int(Rtmp/(R1OOK_total_R_vco0/99))/5)*5,0,99);
      Rtmp = Rtmp - val_pot100K*(R1OOK_total_R_vco0/99);
      val_pot10K = (byte) constrain(int(int(Rtmp/(R1OK_total_R_vco0/99))/5)*5,0,99);
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

    /*
    Serial1.print("<pot_val:");
    Serial1.print(pot_vals[m]);
    Serial1.print(">");
    */
    //if (pot_vals[m] < 0) { pot_vals[m] = 0; }
    if (pot_vals[m] > 99) { pot_vals[m] = 99; }
    
    pot_val_change = int(pot_vals[m]) - int(curr_pot_vals[m]);
   /*
    Serial1.print("<m:");
    Serial1.print(m);
    Serial1.print(">");
    
    Serial1.print("<pot_val:");
    Serial1.print(pot_vals[m]);
    Serial1.print(">");
   
    Serial1.print("<pot_val_curr:");
    Serial1.print(curr_pot_vals[m]);
    Serial1.print(">");
   
  
    Serial1.print("<pot_val_change:");
    Serial1.print(pot_val_change);
    Serial1.print(">");
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

void GenerateArbitraryFreqDAC(byte (&curr_pot_vals)[6], double freq, double duty, double &f1, double &f2)
{

  double Rvco1;
  double Rvco2;

  double tmp_f1a;
  double tmp_f2a;
  double tmp_f1b;
  double tmp_f2b;
  
  byte allR100steps_1;
  byte R100_1;
  byte R100b_1;
  byte R1_1 = 99;
  
  byte allR100steps_2;
  byte R100_2;
  byte R100b_2;
  byte R1_2 = 99;

  byte idx0;
  byte idx1;
 

  byte midi_to_pot_G[6];
  // Get Closest Coarse frequency, find k.
  //coarse_freq[0][k];
  saw_freq_duty_to_R_DAC(freq, 1.5, duty, Rvco1 , Rvco2, f1, f2);
  //f = 1/(R*C) * (1 + (R/53000)*1.5);
  //f = 1/(R*C) + (1.5/53000)/0.66E-6;
  //1/RC = f - (1.5/53000)/0.66E-6;
  //RC = 1/(f - (1.5/53000)/0.66E-6);
  //R= 1/(f- (1.5/53000)/0.66E-6);

  //Substract R10_1 and R1_1 and R_trim;


  DebugPrint("duty",duty,2);
  DebugPrint("R1",Rvco1,2);
  DebugPrint("R2",Rvco2,2);

  // estimating coarse pot R step
  R_to_pot_DAC(Rvco1, allR100steps_1, 0);
  R_to_pot_DAC(Rvco2, allR100steps_2, 1);
  
  idx0 = 199 - allR100steps_1;
  idx1 = 199 - allR100steps_2;

  //coarse_freq table lookup index guess...
  tmp_f1a = coarse_freq[0][idx0];
  tmp_f2a = coarse_freq[1][idx1];
  
  //try to bracket the frequency between two steps without reading the whole array
  // sliding window.
  
  // frequencies are sorted in increasing order in coarse_freq
  //VCO 0
  if (tmp_f1a < f1) 
  {
    if (idx0 < 199)
    {
      tmp_f1b = coarse_freq[0][++idx0];
      while ((tmp_f1b < f1) && (idx0 < 199))
      {
        tmp_f1b = coarse_freq[0][++idx0];
        tmp_f1a = coarse_freq[0][idx0-1];
      }
      if (((f1-tmp_f1a) <= (tmp_f1b -f1)) && (idx0 <= 199))
      {
        idx0--;
        // set index to closest frequency
      }
    }
  }
  else // (tmp_f1a > f1)
  {
    if (idx0 > 0)
    {
      tmp_f1b = coarse_freq[0][--idx0];
      while ((tmp_f1b > f1) && (idx0 > 0))
      {
        tmp_f1b = coarse_freq[0][--idx0];
        tmp_f1a = coarse_freq[0][idx0+1];
      }
      if (((tmp_f1a-f1) <= (f1-tmp_f1b)) && (idx0 >= 0))
      {
        idx0++;
        // set index to closest frequency
      }
    }
  }
  //VCO 1
  if (tmp_f2a < f2) 
  {
    if (idx1 < 199)
    {
      tmp_f2b = coarse_freq[1][++idx1];
      
      while ((tmp_f2b < f2) && (idx1 < 199))
      {
        tmp_f2b = coarse_freq[1][++idx1];
        tmp_f2a = coarse_freq[1][idx1-1];
      }
      if (((f2-tmp_f2a) <= (tmp_f2b -f2)) && (idx1 <= 199))
      {
        idx1--;
        // set index to closest frequency
      }
    }
  }
  else // (tmp_f2a > f2)
  {
    if (idx1 > 0)
    {
      tmp_f2b = coarse_freq[1][--idx1];

      while ((tmp_f2b > f2) && (idx1 > 0))
      {
        tmp_f2b = coarse_freq[1][--idx1];
        tmp_f2a = coarse_freq[1][idx1+1];
      }
      if (((tmp_f2a-f2) <= (f2-tmp_f2b)) && (idx1 >= 0))
      {
        idx1++;;
        // set index to closest frequency
      }
    }
  }
  
// Set the coarse R pot.

/*
   Serial1.print("<");
   Serial1.print(R100_1);
   Serial1.print(",");

   Serial1.print(R10_1);
   Serial1.print(",");

   Serial1.print(R1_1);
   Serial1.print(",");

   Serial1.print(R100_2);
   Serial1.print(",");

   Serial1.print(R10_2);
   Serial1.print(",");

   Serial1.print(R1_2);
   Serial1.print(">");
*/


   midi_to_pot_G[0] = R1_1; 
   midi_to_pot_G[1] = R100b_1; 
   midi_to_pot_G[2] = 99 - idx0; 
   midi_to_pot_G[3] = R1_2; 
   midi_to_pot_G[4] = R100b_2; 
   midi_to_pot_G[5] = 99 - idx1; 

  /*
   for(k=6;k<12;k++){
   midi_to_pot[k] = 99;
   }
  */  

   //Change OSC freq with saw
   ChangeNote(midi_to_pot_G, curr_pot_vals, pots, true, 0);


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
  
 
  saw_freq_duty_to_R(freq, duty, Rvco1 , Rvco2, f1, f2);
  

  DebugPrint("duty",duty,2);
  DebugPrint("R1",Rvco1,2);
  DebugPrint("R1",Rvco2,2);


  R_to_pot(Rvco1, R100_1, R10_1, R1_1, 0);
  R_to_pot(Rvco2, R100_2, R10_2, R1_2, 1);

/*
   Serial1.print("<");
   Serial1.print(R100_1);
   Serial1.print(",");

   Serial1.print(R10_1);
   Serial1.print(",");

   Serial1.print(R1_1);
   Serial1.print(",");

   Serial1.print(R100_2);
   Serial1.print(",");

   Serial1.print(R10_2);
   Serial1.print(",");

   Serial1.print(R1_2);
   Serial1.print(">");
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

    DebugPrint("MIN_ALL_POTS_VCO",double(vcosel),6);


      for(m = max_pot*vcosel; m < max_pot + max_pot*vcosel; m++) {
        
        for(k=0;k<100;k++)
        {
      
          ptr[m]->decrease(1);
          delay(5);
      
        }
    
      
      }

      DebugPrint("MIN_ALL_POTS_VCO_DONE",double(vcosel),6);

      delay(20000);


      for(m = max_pot*vcosel; m < max_pot + max_pot*vcosel; m++) 
      {
        
        DebugPrint("MAX_POT_VCO",double(vcosel),6);
        DebugPrint("MAX_POT_VCO_M",double(m),6);


        for(k=0;k<100;k++)
        {
      
          ptr[m]->increase(1);
          delay(5);
      
        }
         
        delay(20000);
        DebugPrint("MAX_ALL_POTS_VCO_DONE",double(vcosel),6);
      

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

/*
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
    Serial1.print("<integrator_temp=");
    Serial1.print(String(integrator_temp,3));
    Serial1.print(">");
    Serial1.flush();

    }
    //delay(1000);
  }
 
    Serial1.print("<count2=");
    Serial1.print(count2);
    Serial1.print(">");
    Serial1.flush();
   
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
*/
/*
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
    Serial1.print("<integrator=");
    Serial1.print(String(integrator,3));
    Serial1.print(">");
    Serial1.flush();

  }

    //delay(1000);
  
 
    
  integrator = integrator - tunefrequency;
        
  FreqCount.end();

  return integrator;
}
*/

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

	//unsigned long startMicros = timer2.get_count();
  unsigned long count = 0;

	// wait for any previous pulse to end
	while ((*portInputRegister(port) & bit) == stateMask) {
//		if (timer2.get_count() - startMicros > timeout)
//			return 0;
      if(count > timeout) {return 0;}
      count++;
	}
  count = 0;
	// wait for the pulse to start
	while ((*portInputRegister(port) & bit) != stateMask) {
		  if(count > timeout) {return 0;}
      count++;
    //if (timer2.get_count() - startMicros > timeout)
			//return 0;    
	}
	unsigned long start = timer2.get_count();
  count = 0;
	// wait for the pulse to stop
	while ((*portInputRegister(port) & bit) == stateMask) {
		//if (timer2.get_count() - startMicros > timeout)
			//return 0;
      if(count > timeout) {return 0;}
      count++;
	}
	return (timer2.get_count() - start);
}


void CountFrequency(byte samplesnumber, double &f_meas)
{
  pinMode(34, INPUT);
  //digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

  double f_total_measured = 0.0;
  double f_err_calc = 0.0;
  
  //float integrator_temp = 0.0;
  unsigned long sumlow = 0;   
  unsigned long sumhigh = 0;   
  

  byte counthigh = 0;
  byte countlow = 0;
  byte count = 0;
  
  unsigned long pulsehigh = 0;
  unsigned long pulselow = 0;

  timer2.setup();
  
  while (count < samplesnumber)
  {
   
    pulsehigh = pulseInLong2(34,HIGH,100000);
    pulselow = pulseInLong2(34,LOW,100000);

    if (pulsehigh != 0)    
    {
      // average several readings together
      sumhigh += pulsehigh;
      counthigh++;
    }  
    if (pulselow != 0)
    {
      sumlow += pulselow;
      countlow++;
    }
    count++;
  }
 
  timer2.unsetup();
  
  if ((countlow >0) && (counthigh>0))
  {
    f_total_measured = 2000000.0/((sumhigh/counthigh) + (sumlow/countlow));
    //f_err_calc = tunefrequency*1.55005013e-03 -1.45261187e-01;
    f_err_calc = f_total_measured*1.36503806e-03 -7.58951531e-02;
    f_meas = f_total_measured + f_err_calc;  
  }

    DebugPrint("f_total_meas",f_meas,4);
  
}

void CountFrequencyDeltaGlobal(byte samplesnumber,float tunefrequency, double &f_err) 
{


  pinMode(34, INPUT);
  //digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

  double f_total_measured = 0.0;
  double f_meas = 0.0;
  double f_err_calc = 0.0;
  
  //float integrator_temp = 0.0;
  unsigned long sumlow = 0;   
  unsigned long sumhigh = 0;   
  

  byte counthigh = 0;
  byte countlow = 0;
  byte count = 0;
  
  unsigned long pulsehigh = 0;
  unsigned long pulselow = 0;

  timer2.setup();
  
  while (count < samplesnumber)
  {
   
    pulsehigh = pulseInLong2(34,HIGH,100000);
    pulselow = pulseInLong2(34,LOW,100000);

    if (pulsehigh != 0)    
    {
      // average several readings together
      sumhigh += pulsehigh;
      counthigh++;
    }  
    if (pulselow != 0)
    {
      sumlow += pulselow;
      countlow++;
    }
    /*
    Serial1.print("<ph=");
    Serial1.print(pulsehigh);
    Serial1.print(">");
    Serial1.print("<pl=");
    Serial1.print(pulselow);
    Serial1.print(">");
    Serial1.flush();
    */    
    count++;
    //delay(1000);
    
  }
  //interrupts();
  timer2.unsetup();
  
  if ((countlow >0) && (counthigh>0))
  {
    f_total_measured = 2000000.0/((sumhigh/counthigh) + (sumlow/countlow));
    //f_err_calc = tunefrequency*1.55005013e-03 -1.45261187e-01;
    f_err_calc = tunefrequency*1.36503806e-03 -7.58951531e-02;
    //f_err_calc = 0.0;
    f_meas = f_total_measured + f_err_calc;
  
    f_err = f_meas - tunefrequency;
  }

    //delay(1000);
    /*
    Serial1.print("<countlow=");
    Serial1.print(countlow);
    Serial1.print(">");    
    Serial1.print("<counthigh=");
    Serial1.print(counthigh);
    Serial1.print(">");
    */
    DebugPrint("f_total_meas",f_meas,4);
    DebugPrint("f_total_gen",tunefrequency,4);

    /*
    Serial1.print("<f1m=");
    Serial1.print(String(f1_measured,5));
    Serial1.print(">");
    Serial1.print("<f2m=");
    Serial1.print(String(f2_measured,5));
    Serial1.print(">");
    Serial1.flush();  
    */
    
    //f_err = f_total_measured - tunefrequency;
    //f1_err = f1_measured - f1;
    //f2_err = f2_measured - f2;
    //f_meas = f_total_measured;
    //return integrator;

}


void CountFrequencyDelta2(byte samplesnumber,float tunefrequency, double f1, double f2, double &f_meas, double &f1_meas, double &f2_meas, double &f_err) {


   pinMode(34, INPUT);
   //digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

    double f_total_measured = 0.0;
    double f1_measured = 0.0;
    double f2_measured = 0.0;
    double f_err_calc = 0.0;
    
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
   
    pulsehigh = pulseInLong2(34,HIGH,100000);
    pulselow = pulseInLong2(34,LOW,100000);

    if (pulsehigh != 0)    
    {
    // average several readings together
    sumhigh += pulsehigh;
    counthigh++;
    }  
    if (pulselow != 0)
    {
    sumlow += pulselow;
    countlow++;
    }
    /*
    Serial1.print("<ph=");
    Serial1.print(pulsehigh);
    Serial1.print(">");
    Serial1.print("<pl=");
    Serial1.print(pulselow);
    Serial1.print(">");
    Serial1.flush();
    */    
    count++;
    //delay(1000);
    
  }
  //interrupts();
  timer2.unsetup();
  if ((countlow >0) && (counthigh>0))
  {
    f_total_measured = 2000000.0/((sumhigh/counthigh) + (sumlow/countlow));
    //f_err_calc = tunefrequency*1.55005013e-03 -1.45261187e-01;
    f_err_calc = tunefrequency*1.36503806e-03 -7.58951531e-02;
     
    
    //f_err_calc = 0.0;
    f_meas = f_total_measured + f_err_calc;
  
    f1_measured = 1000000.0/(sumhigh/counthigh);
    f2_measured = 1000000.0/(sumlow/countlow);
    f1_meas = f1_measured; 
    f2_meas = f2_measured;     
  //FreqCount.end();
    f_err = tunefrequency - f_meas;
  }

    //delay(1000);
    /*
    Serial1.print("<countlow=");
    Serial1.print(countlow);
    Serial1.print(">");    
    Serial1.print("<counthigh=");
    Serial1.print(counthigh);
    Serial1.print(">");
    */
    
    DebugPrint("f_glob_tot_meas",f_meas,5);

    
    /*
    Serial1.print("<f1m=");
    Serial1.print(String(f1_measured,5));
    Serial1.print(">");
    Serial1.print("<f2m=");
    Serial1.print(String(f2_measured,5));
    Serial1.print(">");
    Serial1.flush();  
    */
}

void SingleCountFrequencyDelta(byte samplesnumber,double f_global, double f_component, double &f_global_err, double &f_component_err, bool low_or_high, bool globaltune) 
{
  pinMode(34, INPUT);
  
  double f_total_measured = 0.0;
  double f_total_measured_compensated = 0.0;
  //double f1_measured = 0.0;
  //double f2_measured = 0.0;
  double f_err_calc = 0.0;
  
  //float integrator_temp = 0.0;
  unsigned long sumlow = 0;   
  unsigned long sumhigh = 0;   
  unsigned long sum = 0;   
  

  byte counthigh = 0;
  byte countlow = 0;
  byte count = 0;
  
  unsigned long pulsehigh = 0;
  unsigned long pulselow = 0;
  unsigned long pulse = 0;
  
  ///////
  double f_measured = 0.0;
   //MIDI.sendNoteOn(80, 127, 1);
     

    
  if (globaltune)
  {
     //MIDI.sendNoteOn(81, 127, 1);
    timer2.setup();
    //MIDI.sendNoteOn(82, 127, 1);
   
    while (count < samplesnumber)
    {
    
      pulsehigh = pulseInLong2(34,HIGH,100000);
      pulselow = pulseInLong2(34,LOW,100000);

      if (pulsehigh != 0)    
      {
      // average several readings together
      sumhigh += pulsehigh;
      counthigh++;
      }  
      if (pulselow != 0)
      {
      sumlow += pulselow;
      countlow++;
      }
      /*
      Serial1.print("<ph=");
      Serial1.print(pulsehigh);
      Serial1.print(">");
      Serial1.print("<pl=");
      Serial1.print(pulselow);
      Serial1.print(">");
      Serial1.flush();
      */    
      count++;
      //delay(1000);
      
    } // end while (count < samplesnumber)

    timer2.unsetup();
    if ((countlow >0) && (counthigh>0))
    {
      if (low_or_high)
      {
        f_measured = 1000000.0/(sumhigh/counthigh);        
      }
      else
      {
        f_measured = 1000000.0/(sumlow/countlow);
      }
      /*
      Serial1.print("<f_glb_cmp_meas=");
      Serial1.print(String(f_measured,3));
      Serial1.print(">");
      
      Serial1.print("<f_glb_cmp_trg=");
      Serial1.print(String(f_component,3));
      Serial1.print(">");
      Serial1.flush();
      */

      f_component_err = f_measured - f_component;

      f_total_measured = 2000000.0/((sumhigh/counthigh) + (sumlow/countlow));
      //f_err_calc = f_global*1.55005013e-03 -1.45261187e-01;
      f_err_calc = f_global*1.36503806e-03 -7.58951531e-02;
      //f_err_calc = 0.0;
      f_total_measured_compensated = f_total_measured + f_err_calc;
      f_global_err = f_total_measured_compensated - f_global;


      DebugPrint("f_total_meas",f_total_measured_compensated,4);
      DebugPrint("f_total_gen",f_global,4);

    } // end if ((countlow >0) && (counthigh>0))
    else
    {
      f_global_err = 0.0;
      f_component_err = 0.0;
    }
    

  } // end if (globaltune)
  else
  {
    //MIDI.sendNoteOn(83, 127, 1);
    timer2.setup();
    //MIDI.sendNoteOn(84, 127, 1);
   
    while (count < samplesnumber)
    {
   
      pulse = pulseInLong2(34,low_or_high,100000);
    
      if (pulse != 0)
      {
        // average several reading together
        sum += pulse;
        count++;
      }  
      /*
      Serial1.print("<pulsehigh=");
      Serial1.print(pulsehigh);
      Serial1.print(">");
      Serial1.print("<pulselow=");
      Serial1.print(pulselow);
      Serial1.print(">");
      Serial1.flush();
      */
      //count++;
      //delay(1000);
    
    } // end while (count < samplesnumber)
    //interrupts();
    timer2.unsetup();

    if (count >0) 
    {
   
      f_measured = 1000000.0/(sum/count);
      f_component_err = f_measured - f_component;
      f_global_err = 0.0;

      DebugPrint("f_sng_cmp_meas",f_measured,4);
      DebugPrint("f_sng_cmp_trg",f_component,4);
      
    } // end if (count>0)
    else
    {
      f_component_err = 0.0;
      f_global_err = 0.0;
    }
    
      //delay(1000);
    
  
      
  
  } // end else (!globaltune)

}


/*
void AutoTune(DigiPot *ptr[6], byte (&curr_pot_vals)[6], double freq, bool level, int val_vco, bool newmethod) {


  //Serial1.println("AUTO TUNE START");
  //Serial1.println(String(freq));
  if (freq == minfreq)
  {
    MaxVcoPots(ptr,curr_pot_vals,val_vco);
    Serial1.print("<Autotune_end>");
    Serial1.flush();
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
  const double tune_thresh = 1.0;
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
          SingleCountFrequencyDelta(50,freq,dintegrator,level,0); 
          integrator = (float) dintegrator;
          dev_cents = 1200 * log((integrator + freq)/freq)/log(2);

          pot_val_change = int(integrator/fabs(integrator));
          DebugPrint("INIT_POT_VAL_CHANGE",double(integrator),2);
          
          Serial1.print("<INIT POT VAL CHANGE:");
          Serial1.print(pot_val_change);
          Serial1.print(">"); 
          Serial1.flush();
                             
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
    Serial1.print(charnumintegrations);
    Serial1.flush();

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
        
        
        Serial1.print("<POT NO CHG:m=");
        Serial1.print(m);
        Serial1.print(">");
        
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
           
            Serial1.print("<VALCALC:");
            Serial1.print(String(pot_val_temp,5));
            Serial1.print(">");
            Serial1.flush();
           
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
        
          Serial1.print("<POT INC:m=");
          Serial1.print(m);
          Serial1.print(" VAL=");
          Serial1.print(pot_val_change);
          Serial1.print(">");
          Serial1.flush();
          
        }
        else if (((pot_val_change/abs(pot_val_change)) == -1) && (curr_pot_vals[m] > 0)) 
        {


          if ((m <= (max_pot*val_vco +2)) && newmethod ) 
          {
            rstep = pow(10,m - max_pot*val_vco +1);
            pot_val_temp = ((1/0.28e-6)/(freq*freq*rstep))*integrator;
            //pot_val_temp = ((1/0.28e-6)/(freq*freq*10))*integrator;
            pot_val_change = min(int(pot_val_temp - 0.5),-1);
            
            Serial1.print("<VALCALC:");
            Serial1.print(String(pot_val_temp,5));
            Serial1.print(">");
            Serial1.flush();
            
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
      
          
          Serial1.print("<POT DEC:m=");
          Serial1.print(m);
          Serial1.print(" VAL=");
          Serial1.print(pot_val_change);
          Serial1.print(">");
          Serial1.flush();
      
        }
        else if (((pot_val_change/abs(pot_val_change)) == -1) && (curr_pot_vals[m] == 0)) 
        {
      
          DebugPrint("POT_DEC_M_MINPOS",double(m),2);
          DebugPrint("POT_DEC_VAL",double(pot_val_change),2);
      
      
          Serial1.print("<POT DEC but MIN POS:m=");
          Serial1.print(m);
          Serial1.print(">");
          Serial1.flush();
      
          if(fabs(integrator_1) > fabs(integrator_2) && (integrator_count > 1))
          //not going in the good direction, bumping
          {
            pot_val_change = 1;
            DebugPrint("POT_REV_MIN_M",double(m),2);
            DebugPrint("POT_REV_VAL",double(pot_val_change),2);
      
      
        
            Serial1.print("<POT REVERSE AT MIN POS:m=");
            Serial1.print(m);
            Serial1.print(">");
            Serial1.flush();
        
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
      
          
          Serial1.print("<POT INC but MAX POS:m=");
          Serial1.print(m);
          Serial1.print(">");
          Serial1.flush();
          

          if(fabs(integrator_1) > fabs(integrator_2) && (integrator_count > 1))
          {
            //not going in the good direction, bumping
    
            pot_val_change = -1;
            DebugPrint("POT_REV_MAX_M",double(m),2);
            DebugPrint("POT_INC_VAL",double(pot_val_change),2);
      
            Serial1.print("<POT REVERSE AT MAX POS:m=");
            Serial1.print(m);
            Serial1.print(">");
            Serial1.flush();
        
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


            if ((curr_pot_vals[m+1] <= 1) && (m == 1 + val_vco*max_pot))
            {
              // When reaching high frequencies, we increase pot for m==2 only when its value is greater than 1.    
          
              // We are above frequency, we have not increased pot2, so we can only increase pot 1 and pot 0.
              
              Serial1.print("<SW-HIGHFREQ>");
              Serial1.flush();
              
              DebugPrint("SW_HIGHFREQ",double(m),2);
              pot_val_change = 1;
            }
            //else if ((curr_pot_vals[m] > 49) && m==val_vco*max_pot)
            else if ((curr_pot_vals[m] > 49) && m<=(val_vco*max_pot+1))
            {
              
              Serial1.print("<SW-M1/2-NORM_FROM_BELOW>");
              Serial1.flush();
              
              DebugPrint("SW_M1M2_NORM_FROM_BELOW",double(m),2);
              pot_val_change = 1;
              ptr[m+1]->increase(pot_val_change);
              curr_pot_vals[m+1]++;
              pot_val_change = -1;
            
            }
            //else if ((curr_pot_vals[m] < 49) && m==val_vco*max_pot)
            else if ((curr_pot_vals[m] <= 49) && m<=(val_vco*max_pot +1))
            {
              
              Serial1.println("<SW-M1/2-NORM_FROM_ABOVE>");
              Serial1.flush();
              
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
            
            Serial1.print("<SW-DEC POT:m=");
            Serial1.print(m);
            Serial1.print(">");
            Serial1.flush();
            
            
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
          
          Serial1.print("<SW-M1/2-NORM_FROM_ABOVE:");
          Serial1.print(m);
          Serial1.print(">");
          Serial1.flush();
         
          

          
        }
         
      }
    // test delay(1000);
    } //end if(integrator_count > 0)

    integrator_1 = integrator;

    if (newmethod) 
    {
      SingleCountFrequencyDelta(50,freq,dintegrator,level,0); 
      integrator = (float) dintegrator;
    }
    else 
    {
      integrator = MeasureFrequencyDelta(tunetime,samples_number,float(freq));
    }

    dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
    

    dtostrf(integrator, 10, 3, charintegrator);
    sprintf (charintegratorfmt, "<INTEG:%s>",charintegrator);
    Serial1.print(charintegratorfmt);
    

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
          SingleCountFrequencyDelta(50,freq,dintegrator,level,0); 
          integrator = (float) dintegrator;
        }
        else
        {
          integrator = MeasureFrequencyDelta(tunetime,samples_number,float(freq));
        }
            
           
        DebugPrint("INTEG_FIN_TUNED",double(integrator),2);
        PrintDigiPot(midi_to_pot,val_vco);
        Serial1.flush();
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
          
          Serial1.print("<P=");
          Serial1.print(p);
          Serial1.print(" VAL=");
          Serial1.print(String(states_integrator[p],5));
          Serial1.print(" POT=");
          Serial1.print(states[p][0]);
          Serial1.print(",");
          Serial1.print(states[p][1]);
          Serial1.print(",");       
          Serial1.print(states[p][2]); 
          Serial1.print(">");
          Serial1.flush();

          if (fabs(states_integrator[p]) < min_integrator) { 
            min_integrator = fabs(states_integrator[p]);
            best_index = p;
          }
        
        }
        Serial1.print("<BEST_POT_VALS:");
        Serial1.print(states[best_index][0]);
        Serial1.print(",");
        Serial1.print(states[best_index][1]);
        Serial1.print(",");       
        Serial1.print(states[best_index][2]);
        Serial1.print(">");
        Serial1.print("<BEST_P:");
        Serial1.print(best_index);
        Serial1.print(">");
        Serial1.flush();
        

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
        
        //checkserial();
        //sprintf (charpotvals, "<%02u,%02u,%02u>",curr_pot_vals[0],curr_pot_vals[1],curr_pot_vals[2]);
        //Serial1.print(charpotvals);            
        
if (newmethod)
{
  SingleCountFrequencyDelta(50,freq,dintegrator,level,0); 
  integrator = (float) dintegrator;
  
}
else
{
  integrator = MeasureFrequencyDelta(tunetime*2,samples_number,float(freq));
}

}

*/
void NewAutoTuneDAC(DigiPot *ptr[6], byte (&curr_pot_vals)[6], byte noteindex, double tunefrequency, double duty ,double freq, bool level, int val_vco, bool globaltune) 
{ 
  byte max_pot = 3;
  // if minfreq, set it right away

  if (freq == minfreq)
  {
    MaxVcoPots(ptr,curr_pot_vals,val_vco);
    DebugPrintToken("MINFREQ_SET",3);
    PWM_Note_Settings[noteindex][val_vco*max_pot] = 99;
    PWM_Note_Settings[noteindex][val_vco*max_pot +1] = 99;
    PWM_Note_Settings[noteindex][val_vco*max_pot +2] = 99;
     
    //if (globaltune != true) {InTuning = false;}
    return;
  }

  int num_integrations = 0;
  int max_integrations = 90;
  int states[max_integrations][3];
  float states_integrator[max_integrations];
 
  
  
  
  double dev_cents = 0.0;
  double dev_cents_back = 0.0;
  const double tune_thresh = 1.0;
  double delta_V = 0.0;
  
 
  int p;

  int bias;

  bool tuned = false;
  double dintegrator = 0.0;
  double dintegrator_p = 0.0;
  double dintegrator_p_1 = 0.0;
  
  
  float integrator = 0.0;
  float integrator_1 = 0.0;
  float min_integrator = 0.0;

  int integrator_count = 0;

  int best_index = 0;
  byte curr_pot_val_bck = 0;

  min_integrator = fabs(states_integrator[0]);
        
  for (p=0;p<=max_integrations;p++) 
  {

    if (globaltune)
    {
      //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
      //SingleCountFrequencyDelta(50,freq,dintegrator_p,level);
      //MIDI.sendNoteOn(70, 127, 1);
      SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,true); 
      //MIDI.sendNoteOn(71, 127, 1);
    
      integrator = (float) dintegrator;
      dev_cents = 1200 * log((integrator + tunefrequency)/tunefrequency)/log(2);
      delta_V = d_freq_to_d_volt(dintegrator_p,val_vco);
      Adjust_DAC(delta_V,val_vco);
      //vco formula global tune

    }
    else
    {
      //MIDI.sendNoteOn(72, 127, 1);
      SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,false); 
      //MIDI.sendNoteOn(73, 127, 1);
      //SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      integrator = (float) dintegrator_p;
      dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      delta_V = d_freq_to_d_volt(integrator,val_vco);
      Adjust_DAC(delta_V,val_vco);
      //vco formula single tune

    } // end if globaltune

    if (fabs(dev_cents) <= tune_thresh) 
    { 
      tuned = true;
      DebugPrint("THR_ATT",double(fabs(dev_cents)),3);
   
      states[p][0] = curr_pot_vals[val_vco*max_pot];
      states[p][1] = curr_pot_vals[val_vco*max_pot+1];
      states[p][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[p] = integrator;
      break;
    } //end tune thresh att

      states[p][0] = curr_pot_vals[val_vco*max_pot];
      states[p][1] = curr_pot_vals[val_vco*max_pot+1];
      states[p][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[p] = integrator;
        
  } // end for p < max_integrations

} // end NewAutotuneDAC

void NewAutoTune(DigiPot *ptr[6], byte (&curr_pot_vals)[6], byte noteindex, double tunefrequency, double freq, bool level, int val_vco, bool globaltune) 
{

  //static byte max_freq_pot[6] = {99,99,99,99,99,99};
  //static bool max_freq_vco0_set = false;
  //static bool max_freq_vco1_set = false;
  int m;
  int max_pot = 3;
  int pot_val_change;

  if (freq == minfreq)
  {
    MaxVcoPots(ptr,curr_pot_vals,val_vco);
    DebugPrintToken("MINFREQ_SET",3);
    PWM_Note_Settings[noteindex][val_vco*3] = 99;
    PWM_Note_Settings[noteindex][val_vco*3 +1] = 99;
    PWM_Note_Settings[noteindex][val_vco*3 +2] = 99;
     
    //if (globaltune != true) {InTuning = false;}
    return;
  }
  
  /*
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
  */
  int num_integrations = 0;
  int max_integrations = 90;
  int states[max_integrations][3];
  float states_integrator[max_integrations];
 
  
  
  pot_val_change = -1;
  double pot_val_temp = 0.0;
  double pot_val_temp2 = 0.0;
  
  double dev_cents = 0.0;
  double dev_cents_back = 0.0;
  const double tune_thresh = 1.0;
  
 
  m = val_vco*max_pot + max_pot -1;
  int rstep;
  int p;

  int bias;

  bool tuned = false;
  double dintegrator = 0.0;
  double dintegrator_p = 0.0;
  double dintegrator_p_1 = 0.0;
  
  
  float integrator = 0.0;
  float integrator_1 = 0.0;
  float min_integrator = 0.0;

  int integrator_count = 0;

  int best_index = 0;
  byte curr_pot_val_bck = 0;


  if (globaltune)
  {
    //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
    //SingleCountFrequencyDelta(50,freq,dintegrator_p,level);
    //MIDI.sendNoteOn(70, 127, 1);
    SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,true); 
    //MIDI.sendNoteOn(71, 127, 1);
   
    integrator = (float) dintegrator;
    dev_cents = 1200 * log((integrator + tunefrequency)/tunefrequency)/log(2);
  }
  else
  {
     //MIDI.sendNoteOn(72, 127, 1);
    SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,false); 
     //MIDI.sendNoteOn(73, 127, 1);
    //SingleCountFrequencyDelta(50,freq,dintegrator,level); 
    integrator = (float) dintegrator_p;
    dev_cents = 1200 * log((integrator + freq)/freq)/log(2);

  }
  
  
  pot_val_change = int(integrator/fabs(integrator));
  DebugPrint("INIT_DELTA",double(integrator),3);
  PrintDigiPot(curr_pot_vals,val_vco,2);


  while(!tuned) 
  {
    MIDI.read();
    if (num_integrations == max_integrations) 
    { 
      tuned = true;
      states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
      states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
      states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[num_integrations] = integrator;
      break;
    }
    if (fabs(dev_cents) <= tune_thresh) 
    { 
      tuned = true;
      DebugPrint("THR_ATT",double(fabs(dev_cents)),3);
      CountFrequencyDeltaGlobal(10,tunefrequency,dintegrator);
      
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
        DebugPrint("HP_VAL_OOB",double(curr_pot_vals[m]),3);
      
      }
      else if((pot_val_change<0) && (curr_pot_vals[m] == 0))
      {
        m--;
        // we can low m in all cases because we are in high pot check.
        integrator_count = 0;
        continue;
        DebugPrint("HP_VAL_OOB",double(curr_pot_vals[m]),3);
      }
      
      DebugPrint("HP_IDX_BEFORE",double(m),3);
      DebugPrint("HP_VAL_BEFORE",curr_pot_vals[m],3);
      DebugPrint("HP_DELTA_BEFORE",double(integrator),3);
      
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
      dev_cents_back = dev_cents; 
  
      if (globaltune)
      {
        dintegrator_p_1 = dintegrator_p;
        //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
        //SingleCountFrequencyDelta(50,freq,dintegrator_p,level);
        SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,true);  
        integrator = (float) dintegrator;
        dev_cents = 1200 * log((integrator + tunefrequency)/tunefrequency)/log(2);
      }
      else
      {
        //SingleCountFrequencyDelta(50,freq,dintegrator,level);
        SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,false);  
        integrator = (float) dintegrator_p;
        dev_cents = 1200 * log((integrator + freq)/freq)/log(2);

      }
      //SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      //integrator = (float) dintegrator;
      pot_val_change = int(integrator/fabs(integrator));
      //dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      
      DebugPrint("HP_IDX_AFTER",double(m),3); 
      DebugPrint("HP_VAL_AFTER ",curr_pot_vals[m],3);
      DebugPrint("HP_DELTA_AFTER ",double(integrator),3);
      DebugPrint("HP_DELTA_COMPONENT_AFTER", dintegrator_p,3);
      DebugPrint("HP_STEP_AFTER  ",pot_val_change,3);

      if (int(integrator/fabs(integrator)) != int(integrator_1/fabs(integrator_1)))
      {
        DebugPrint("HP_ZCROSS_VAL",double(curr_pot_vals[m]),3);

        // zero cross, check for change room in up or down direction for med/low pots.
        bias = 5000 - (100*(curr_pot_vals[m-1]) + curr_pot_vals[m-2]*10);  
        // approx. bias in ohms
        // if bias positive, there is room for going up (decrease freq)
        // if bias negative, there is room for going down (increase freq)
        if (globaltune)
        {
          pot_val_temp = -d_hertz_to_R(freq,val_vco)*dintegrator_p*1000; // * 1000 to convert kOhms in Ohms
          pot_val_temp2 = -d_hertz_to_R(freq,val_vco)*dintegrator_p_1*1000; // * 1000 to convert kOhms in Ohms
        }
        else
        {
          pot_val_temp = -d_hertz_to_R(freq,val_vco)*integrator*1000; // * 1000 to convert kOhms in Ohms
          pot_val_temp2 = -d_hertz_to_R(freq,val_vco)*integrator_1*1000; // * 1000 to convert kOhms in Ohms
        }
        

        DebugPrint("HP_STEP_VAL_AFTER", double(pot_val_temp),4);
        DebugPrint("HP_STEP_VAL_BEFORE", double(pot_val_temp2),4);
        DebugPrint("HP_BIAS", double(bias),4);

        // check for closest setting in terms of magnitude between two settings
        if(fabs(log(fabs(pot_val_temp/pot_val_temp2))) > 2) // one term is almost 8 times bigger than the other
        {
          // take the abs. val smallest one
          if(fabs(pot_val_temp) < fabs(pot_val_temp2))
          {
                DebugPrint("HP_NOCHNG_MAGN_VAL1", double(pot_val_temp),5);
                DebugPrint("HP_NOCHNG_MAGN_BIAS1", double(bias),5);
          }
          else
          {
                DebugPrint("HP_REV_MAGN_VAL1", double(pot_val_temp2),5);
                DebugPrint("HP_REV_MAGN_BIAS1", double(bias),5);
                integrator = integrator_1;
                dev_cents = dev_cents_back;
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
        else // use bias method.
        {        
          // pot_val_temp approx change in R (ohms) required.
          if (bias > 0)
          {
            if  (pot_val_temp > 0)
            {
              if (bias > pot_val_temp)
              {
                // bias is higher than pot_val temp, there is room for change using integrator
                // change nothing
                DebugPrint("HP_NOCHNG_VAL1", double(pot_val_temp),5);
                DebugPrint("HP_NOCHNG_BIAS1", double(bias),5);
                  
              }
              else if (bias < pot_val_temp) 
              {

                DebugPrint("HP_REV_VAL2", double(pot_val_temp),5);
                DebugPrint("HP_REV_BIAS2", double(bias),5);
                integrator = integrator_1;
                dev_cents = dev_cents_back;
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

                DebugPrint("HP_REV_VAL3", double(pot_val_temp2),5);
                DebugPrint("HP_REV_BIAS3", double(bias),5);
                integrator = integrator_1;
                dev_cents = dev_cents_back;
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

                DebugPrint("HP_NOCHNG_VAL4", double(pot_val_temp2),5);
                DebugPrint("HP_NOCHNG_BIAS4", double(bias),5);

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
                DebugPrint("HP_NOCHNG_VAL5", double(pot_val_temp),5);
                DebugPrint("HP_NOCHNG_BIAS5", double(bias),5);

              }
              else if (bias > pot_val_temp)
              {

                DebugPrint("HP_REV_VAL6", double(pot_val_temp),5);
                DebugPrint("HP_REV_BIAS6", double(bias),5);
                dev_cents = dev_cents_back;
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
                DebugPrint("HP_REV_VAL7", double(pot_val_temp2),5);
                DebugPrint("HP_REV_BIAS7", double(bias),5);

                integrator = integrator_1;
                dev_cents = dev_cents_back;
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
                DebugPrint("HP_NOCHNG_VAL8", double(pot_val_temp),5);
                DebugPrint("HP_NOCHNG_BIAS8", double(bias),5);

              }
            }  
          } // else if bias end
        } // else use bias method end.
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
        DebugPrint("HP_CONT_CHNG_VAL",double(curr_pot_vals[m]),3);
        integrator_count++;
      } 
      
      states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
      states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
      states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[num_integrations] = integrator;  
      num_integrations++; // increment num_integrations high pot
      DebugPrint("HP_NUM_INTEG_INC",double(num_integrations),3);
    
    } // end high pot check.
    
    else if (m < (max_pot*val_vco +2)) 
    { //start low pot check      
      
      if (globaltune)
      {
        //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
        SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,true); 
        integrator = (float) dintegrator;
        dev_cents = 1200 * log((integrator + tunefrequency)/tunefrequency)/log(2);
      }
      else
      {
        //SingleCountFrequencyDelta(50,freq,dintegrator,level);
        SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,false);  
        integrator = (float) dintegrator_p;
        dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      }
      
      
      
      //SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      //integrator = (float) dintegrator;
      //dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      rstep = pow(10,m - max_pot*val_vco +1);
      //pot_val_temp = ((1/0.28e-6)/(freq*freq*rstep))*integrator;  
      if (m == max_pot*val_vco)
      { 
        pot_val_temp = -(d_hertz_to_R(freq,val_vco)*integrator*1000/rstep)/2;
      }
      else
      {
        pot_val_temp = -(d_hertz_to_R(freq,val_vco)*integrator*1000/rstep);
        //decreased step size... pot_val_temp = -(d_hertz_to_R(freq,val_vco)*integrator*1000/rstep);
     
      }
       // * 1000 to convert kOhms in Ohms
     
      //pot_val_temp = ((1/0.28e-6)/(freq*freq*10))*integrator;
      curr_pot_val_bck = curr_pot_vals[m];
      
      DebugPrint("LP_IDX_BEFORE",double(m),3);
      DebugPrint("LP_VAL_BEFORE",curr_pot_vals[m],3);
      DebugPrint("LP_DELTA_BEFORE",double(integrator),3);
   
      //check bounds
      if((pot_val_temp>0) && (curr_pot_vals[m] == 99))
      {
        if (m == val_vco*max_pot) 
        { 
          DebugPrint("LP_VAL_OOB",double(curr_pot_vals[m]),3);
          tuned = true;
          states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
          states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
          states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
          states_integrator[num_integrations] = integrator;
          break;
        }
        m--;
        continue;
      }
      else if((pot_val_temp<0) && (curr_pot_vals[m] == 0))
      {
        if (m == val_vco*max_pot) 
        { 
          DebugPrint("LP_VAL_OOB",double(curr_pot_vals[m]),3);
          tuned = true;
          states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
          states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
          states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
          states_integrator[num_integrations] = integrator;
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
      dev_cents = dev_cents_back;  
      
      if (globaltune)
      {
        //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
        SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,true); 
        integrator = (float) dintegrator;
        dev_cents = 1200 * log((integrator + tunefrequency)/tunefrequency)/log(2);
      }
      else
      {
        //SingleCountFrequencyDelta(50,freq,dintegrator,level);
        SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,false);  
        integrator = (float) dintegrator_p;
        dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      }    
      
      //SingleCountFrequencyDelta(50,freq,dintegrator,level); 
      //integrator = (float) dintegrator;
      
      pot_val_change = int(integrator/fabs(integrator));
      //dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      
      DebugPrint("LP_IDX_AFTER",double(m),3);
      DebugPrint("LP_VAL_AFTER",curr_pot_vals[m],3);
      DebugPrint("LP_DELTA_AFTER",double(integrator),3);
      
      if (int(integrator/fabs(integrator)) != int(integrator_1/fabs(integrator_1)))
      {
        // zero cross, is integrator or integrator_1 closest ?
        DebugPrint("LP_ZCROSS_VAL",double(curr_pot_vals[m]),3);

        if (m == max_pot*val_vco +1) 
        {
          pot_val_temp = -(d_hertz_to_R(freq,val_vco)*integrator*1000/rstep)/2; // * 1000 to convert kOhms in Ohms
          pot_val_temp2 = -(d_hertz_to_R(freq,val_vco)*integrator_1*1000/rstep)/2; // * 1000 to convert kOhms in Ohms
          bias = 500 - (10*(curr_pot_vals[m-1]));

          DebugPrint("LP_STEP_VAL_AFTER", double(pot_val_temp),4);
          DebugPrint("LP_STEP_VAL_BEFORE", double(pot_val_temp2),4);
          DebugPrint("LP_BIAS", double(bias),4);
  
          // pot_val_temp approx change in R (ohms) required.
          if (bias > 0)
          {
            if  (pot_val_temp > 0)
            {
              if (bias > pot_val_temp)
              {
                // bias is higher than pot_val temp, there is room for change using integrator
                // change nothing
                DebugPrint("LP_NOCHNG_VAL1", double(pot_val_temp),5);
                DebugPrint("LP_NOCHNG_BIAS1", double(bias),5);
                  
              }
              else if (bias < pot_val_temp) 
              {

                DebugPrint("LP_REV_VAL2", double(pot_val_temp),5);
                DebugPrint("LP_REV_BIAS2", double(bias),5);
                integrator = integrator_1;
                dev_cents = dev_cents_back;
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

                DebugPrint("LP_REV_VAL3", double(pot_val_temp2),5);
                DebugPrint("LP_REV_BIAS3", double(bias),5);
                integrator = integrator_1;
                dev_cents = dev_cents_back;
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

                DebugPrint("LP_NOCHNG_VAL4", double(pot_val_temp2),5);
                DebugPrint("LP_NOCHNG_BIAS4", double(bias),5);

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
                DebugPrint("LP_NOCHNG_VAL5", double(pot_val_temp),5);
                DebugPrint("LP_NOCHNG_BIAS5", double(bias),5);

              }
              else if (bias > pot_val_temp)
              {

                DebugPrint("LP_REV_VAL6", double(pot_val_temp),5);
                DebugPrint("LP_REV_BIAS6", double(bias),5);
                integrator = integrator_1;
                dev_cents = dev_cents_back;
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
                DebugPrint("LP_REV_VAL7", double(pot_val_temp2),5);
                DebugPrint("LP_REV_BIAS7", double(bias),5);

                integrator = integrator_1;
                dev_cents = dev_cents_back;
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
                DebugPrint("LP_NOCHNG_VAL8", double(pot_val_temp),5);
                DebugPrint("LP_NOCHNG_BIAS8", double(bias),5);

              }
            }  
          } // else if bias end        
        } // end zero cross for middle pot m
     
        if (m == max_pot*val_vco)
        {
          if (fabs(integrator) <= fabs(integrator_1))
          {
            // Change nothing, current value closer.
            DebugPrint("LASTPOT_NOCHNG_VAL",double(curr_pot_vals[m]),3);

          }
          else
          {
              // revert
            pot_val_change = curr_pot_val_bck - int(curr_pot_vals[m]);
            curr_pot_vals[m] = curr_pot_val_bck;
            integrator = integrator_1;
            dev_cents = dev_cents_back;
            
            if (pot_val_change > 0) 
            { 
              ptr[m]->increase(pot_val_change); 
            }
            else if (pot_val_change < 0)
            { 
              ptr[m]->decrease(abs(pot_val_change)); 
            }

            DebugPrint("LASTPOT_REV_VAL",double(curr_pot_vals[m]),3);
          }
        
          DebugPrint("LASTPOT_TUNED",double(curr_pot_vals[m]),3);
          tuned = true;
          //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
          states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
          states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
          states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
          states_integrator[num_integrations] = integrator;
          break;       
 
        }

        //if (m == (val_vco*max_pot)) 
        //{ 
        //}
        // we decrease m and reset integrator count for this m
        m--;
        integrator_count = 0;
      } // end zero cross check.
      else 
      { 
        DebugPrint("LP_CONT_CHNG_VAL",double(curr_pot_vals[m]),3);
        integrator_count++; // zero not crossed, we keep on changing pot for this m
      } // end no zero cross check
      
      states[num_integrations][0] = curr_pot_vals[val_vco*max_pot];
      states[num_integrations][1] = curr_pot_vals[val_vco*max_pot+1];
      states[num_integrations][2] = curr_pot_vals[val_vco*max_pot+2];
      states_integrator[num_integrations] = integrator;
      num_integrations++; //increasing num_integrations low pot
      DebugPrint("LP_NUM_INTEG_INC",double(num_integrations),3);
      // check bounds.
    } // end low pot check             
  } // end while tuned
/*
Serial1.print("<COUNTDELTAGLOBAL>");

for(k = 0;k < 50;k++)
{
  CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
}
Serial1.print("<COUNTDELTA2>");
for(k = 0;k < 50;k++)
{
  double f1;
  double f2;
  double f_meas;
  double f1_meas;
  double f2_meas;
  double f_err;
  CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
}
*/
min_integrator = fabs(states_integrator[0]);
        
  for (p=0;p<=num_integrations;p++) 
  {


    //DebugPrint("P",double(p));
    //DebugPrint("VAL",double(states_integrator[p]));
    

    DebugPrintStr("<P=",2);
    DebugPrintStr(String(int(p)),2);
    DebugPrintStr(" VAL=",2);
    DebugPrintStr(String(int(states_integrator[p])),2);
    DebugPrintStr(" POT=",2);
    DebugPrintStr(String(int(states[p][0])),2);
    DebugPrintStr(",",2);
    DebugPrintStr(String(int(states[p][1])),2);
    DebugPrintStr(",",2);
    DebugPrintStr(String(int(states[p][2])),2);
    DebugPrintStr(">",2);
    
    if (fabs(states_integrator[p]) < min_integrator) 
    { 
      min_integrator = fabs(states_integrator[p]);
      best_index = p;
    }

  }
  
  DebugPrintStr("<BP_VALS:",2);
  DebugPrintStr(String(int(states[best_index][0])),2);
  DebugPrintStr(",",2);
  DebugPrintStr(String(int(states[best_index][1])),2);
  DebugPrintStr(",",2);
  DebugPrintStr(String(int(states[best_index][2])),2);
  DebugPrintStr(">",2);
  DebugPrintStr("<BP_IDX:",2);
  DebugPrintStr(String(int(best_index)),2);
  DebugPrintStr(">",2);
 
  
  int k = 0;
  for (m=val_vco*max_pot;m<val_vco*max_pot + max_pot;m++) 
  {
    pot_val_change = int(states[best_index][m - val_vco*max_pot]) - int(curr_pot_vals[m]);
    if (pot_val_change > 0) 
    { 
      curr_pot_vals[m] += pot_val_change;
      //for(k=0;k<pot_val_change;k++) 
      //{
        //delay(10);
      DebugPrint("BP_CHNG_VAL",double(pot_val_change),5);
      DebugPrint("BP_CHNG_IDX",double(m),5);
      
      ptr[m]->increase(pot_val_change);
      //}
        
    }
    else if (pot_val_change < 0) 
    {
      curr_pot_vals[m] += pot_val_change;
      //for(k=0;k<abs(pot_val_change);k++) 
      //{
        //delay(10);
      ptr[m]->decrease(abs(pot_val_change));
      DebugPrint("BP_CHNG_VAL",double(pot_val_change),5);
      DebugPrint("BP_CHNG_IDX",double(m),5);
      //}
                  
    }

    PWM_Note_Settings[noteindex][m] = curr_pot_vals[m];  
  } // end retune pots to best values

  //if(globaltune) {PWM_Note_Settings[noteindex][6] = 1;}
/*
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
  */
} // end NewAutoTune

void ADS()
{
  byte r = 1;
  byte s = 0;
  while(s<31)
  {
    pot12.decrease(r);
    r = byte(int(r*1.5 +0.5));
    s += r;
    delayMicroseconds(10000);
  }

}

void Release()
{
  byte r = 1;
  byte s = 0;
  while(s<31)
  {
    pot12.increase(r);
    r = byte(int(r*1.5 +0.5));
    s += r;
    delayMicroseconds(10000);
  }
}


void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  double f1 = 0.0;
  double f2 = 0.0;
  double f_meas = 0.0;
  double f1_meas = 0.0;
  double f2_meas = 0.0;
  double f_err = 0.0;
  double duty = 0.75;
  byte noteindex;
  byte midi_to_pot_G[6];
  // restrict OSC to notes of possible frequency range
  if (pitch < 48) {
    noteindex = 0;
    }
  else if (pitch > 96) {
    noteindex = 48;
  }
  else {
    noteindex = pitch - 48;
  }

//MIDI.sendNoteOn(41, 127, 1);
   
  if (InTuning)  {
    DebugPrintToken("IN_TUNING",1);
   //MIDI.sendNoteOn(42, 127, 1);
    return;}
  else if (PWM_Note_Settings[noteindex][6] == 0) 
  { DebugPrintToken("NOT_YET_TUNED",1);
    InTuning = true;
  //MIDI.sendNoteOn(43, 127, 1);
  }
  else
  {
    //MIDI.sendNoteOn(44, 127, 1);
    DebugPrintToken("ALREADY_TUNED",1);
    midi_to_pot_G[0] = PWM_Note_Settings[noteindex][0];
    midi_to_pot_G[1] = PWM_Note_Settings[noteindex][1];
    midi_to_pot_G[2] = PWM_Note_Settings[noteindex][2];
    midi_to_pot_G[3] = PWM_Note_Settings[noteindex][3];
    midi_to_pot_G[4] = PWM_Note_Settings[noteindex][4];
    midi_to_pot_G[5] = PWM_Note_Settings[noteindex][5];
    
    ChangeNote(midi_to_pot_G, midi_to_pot, pots, true, 0);
    ADS();
    Serial1.print("S");
    Serial1.print(String(float(noteindex),3));
    Serial1.print("Z");
    Serial1.flush();
    //pot12.decrease(30);
    return;
  }
  //MIDI.sendNoteOn(45, 127, 1);
  
  ADS();
  //pot12.decrease(30);
  notefreq = double(pgm_read_float(&(notes_freq[noteindex])));
  

  DebugPrint("NOTEFREQ",notefreq,2);

  MaxVcoPots(pots,midi_to_pot,0);
  MaxVcoPots(pots,midi_to_pot,1);
  //MIDI.sendNoteOn(46, 127, 1);
  GenerateArbitraryFreq(midi_to_pot,notefreq, duty, f1, f2);
  //MIDI.sendNoteOn(47, 127, 1);

  // f1 = high , f2 = low
  if ((f1 == minfreq) || (f1 == maxfreq))
  {
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f1,1,0,0);
      //MIDI.sendNoteOn(48, 127, 1);
 
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f2,0,1,1);
      //MIDI.sendNoteOn(49, 127, 1);
 
      CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //CountFrequencyDeltaGlobal(50,notefreq,f_err);
      //MIDI.sendNoteOn(50, 127, 1);
 
  }
  else if ((f2 == minfreq) || (f2 == maxfreq))
  {
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f2,0,1,0);
      //MIDI.sendNoteOn(51, 127, 1);
 
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f1,1,0,1);
      //MIDI.sendNoteOn(52, 127, 1);
 
      CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //MIDI.sendNoteOn(53, 127, 1);
 
  }
  else if (f1 <= f2)
  {
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f2,0,1,0);
      //MIDI.sendNoteOn(54, 127, 1);
 
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f1,1,0,1);
      //MIDI.sendNoteOn(55, 127, 1);
 
      CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //MIDI.sendNoteOn(56, 127, 1);
 

  }
  else if (f1 > f2)
  {
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f1,1,0,0);
      //MIDI.sendNoteOn(57, 127, 1);
 
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      NewAutoTune(pots,midi_to_pot,noteindex,notefreq,f2,0,1,1);
      //MIDI.sendNoteOn(58, 127, 1);
 
      CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //MIDI.sendNoteOn(59, 127, 1);
 
      //CountFrequencyDeltaGlobal(50,notefreq,f_err);
  }

  PWM_Note_Settings[noteindex][6] = 1;
  InTuning = false;
  //MIDI.sendNoteOn(60, 127, 1);
  
}


void handleNoteOff(byte channel, byte pitch, byte velocity) {

  // Disable VCA Gate
  DebugPrintToken("NOTE_OFF",2);
  Release();
  //pot12.increase(30);
}

void WriteEEPROMCoarsePotStepFrequencies(DigiPot *ptr[6], byte (&curr_pot_vals)[6], byte vcosel)
{

  byte vco_pin = 4;
  int k = 0;
  byte max_pot = 3;
  byte m;
  // max_pot*vcosel + 1;
  double f_meas;

  //int prev_data_offset = 686;
  int prev_data_offset = 0;
  
  int tuneblock_size = 100*(sizeof(f_meas));
  DebugPrint("WRITE COARSE VCO:",double(vcosel),2);
  digitalWrite(vco_pin, !vcosel);

  for(m=max_pot*vcosel + 1;m<=max_pot*vcosel + 2;m++)
  {
    DebugPrint("RESET POT:",double(m),2);
    for(k=0;k<100;k++)
    {
      ptr[m]->increase(1);
      delay(5);

    }
    curr_pot_vals[m] = 99;
  }
  

  for(m=max_pot*vcosel + 1;m<=max_pot*vcosel + 2;m++)
  {
    DebugPrint("WRITE COARSE POT:",double(m),2);
  
    for(k=99;k>=0;k--)
    {

      //DebugPrint("STEP:",double(k),2);
      CountFrequency(50,f_meas);
      //DebugPrint("FREQ:",f_meas,2);
      int addr = prev_data_offset + (2*vcosel + (m -(max_pot*vcosel +1)))*tuneblock_size + (99-k)*(sizeof(f_meas));
      DebugPrint("ADDR:",double(addr),2);
      EEPROM.put(addr,f_meas);
      //we write frequencies in increasing order
      //delay(1000);
      //DebugPrint("END_STEP",double(k),0);
      ptr[m]->decrease(1);
      curr_pot_vals[m]--;

    }
  }
  
}

void ReadAllCoarseFrequencies()
{
  int k;
  byte vco;

  //int prev_data_offset = 686;
  int prev_data_offset = 0;
  int tuneblock_size = 200*(sizeof(double));
  int addr;
  //double *freq;
  
  
  for (vco=0;vco<2;vco++)
  {
    for (k=0;k<200;k++) 
    {
      addr = prev_data_offset + tuneblock_size*vco + k*(sizeof(double));
      EEPROM.get(addr,*(*(coarse_freq +vco)+k));
      // we had written frequencies in increasing order
    }


  }
  // check that the coarse_freq array is correctly populated

  for (vco=0;vco<2;vco++)
  {
    for (k=0;k<200;k++) 
    {
      DebugPrint(String(k),coarse_freq[vco][k],2);
      // we had written frequencies in increasing order
    }


  }

}

void writeEEPROMpots (byte noteindex, byte pot2, byte pot1, byte pot0, float deviation, byte vcosel)
{

int tuneblock_size = 49*(sizeof(pot2)+sizeof(pot1)+sizeof(pot0)+sizeof(deviation));
int addr = vcosel*tuneblock_size + noteindex*(sizeof(pot2)+sizeof(pot1)+sizeof(pot0)+sizeof(deviation));

//Serial1.print("<addr:");
//Serial1.print(addr);
//Serial1.print(">");


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


//Serial1.print("<addr:");
//Serial1.print(addr);
//Serial1.print(">");



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


//Serial1.print("<tuneblock:");
//Serial1.print(tuneblock_size);
//Serial1.print(">");



for (k=1;k<49;k++) {

int addr = tuneblock_size*vcosel + k*(sizeof(byte)+sizeof(byte)+sizeof(byte)+sizeof(float));

//Serial1.print("<addr:");
//Serial1.print(addr);
//Serial1.print(">");


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
//Serial1.begin(9600,SERIAL_8E2);
Serial1.begin(9600);
//Serial1.println("<reset>");  
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
      //Serial1.println(rc);
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
        }
      }
      else {
        //Serial1.println("endmarker");
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

double CheckTuning(DigiPot *ptr[6], byte (&curr_pot_vals)[6], byte noteindex, int val_vco) 
{

double notefreq;
double component_freq = notefreq;
double integrator;
double component_integrator;

SetNotePots(pots,curr_pot_vals,0,vcosel);

notefreq = double(pgm_read_float(&(notes_freq[noteindex])));
//integrator = MeasureFrequencyDelta(2,10,notefreq);
SingleCountFrequencyDelta(10,notefreq,component_freq,integrator,component_integrator,LOW,true);

return integrator;

}


void setup() 
{

   DAC0.begin();
   DAC1.begin();
   DAC0.setValue(2047); //half deflection of 4095
   DAC1.setValue(2047); //half deflection of 4095
   
 
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
  bool checknotes_formula = false;
  bool checknotes_formula_DAC = false;
  bool checkpots = false;
  bool generatefreq = false;
  bool writecoarsefreqs = false;
  bool testcoarsefreqsselect = true;
  midimode = false;
  bool donothing = false;

  byte vco_pin = 4;
  byte max_pot = 3;
  byte notestart;
  byte noteend;
   
  //Serial1.begin(9600,SERIAL_8E2);
  if (!midimode)
  {
    Serial.begin(9600);
    Serial.print(charspeed);
    Serial.flush();   
    delay(3000);
  }
  StartAllPots(allpots);
  for (k=0;k<49;k++)
  {
    for(n=0;n<7;n++)
    {
      PWM_Note_Settings[k][n] = 0;
    }    
  }
  
  freqrefpin = 9;
  vcosel = 0;
  pinMode(vco_pin, OUTPUT);
  digitalWrite(vco_pin, !vcosel);

  // use default address 0
  //mcp.pinMode(0, OUTPUT);

  MaxVcoPots(pots,midi_to_pot,0);
  MaxVcoPots(pots,midi_to_pot,1);
  //DACTEST
  //pot1.decrease(99);
  //pot4.decrease(99);

  pot12.increase(99);
  delay(5000);

  pot12.decrease(3);
  

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
  
  if (donothing) 
  {
    Serial.begin(9600);
    DebugPrintStr("SERIAL 9600",0);
    return;
  }

  if (midimode) 
  { 

    // Connect the handleNoteOn function to the library,
    // so it is called upon reception of a NoteOn.
    MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function

    // Do the same for NoteOffs
    MIDI.setHandleNoteOff(handleNoteOff);
    
    MIDI.begin(MIDI_CHANNEL_OMNI);
    Serial.begin(9600);
    Serial1.begin(9600);
  
    return;
  }
  
  if (writecoarsefreqs)
  {

    WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,0);
    WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,1);
    ReadAllCoarseFrequencies();
    return;
    
  }

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
      Serial.flush();
    
      /*
      // not required for python curve fitting script 
      dtostrf(pot_devs[k], 6, 2, chardev);
      sprintf(chardevfmt,"%s", chardev);

      Serial1.print(chardevfmt);
      Serial1.print("]>");
      */
    
      //  delay(100); 
    } // end for all notes dumpeeprom
  //return;
  } // end dumpeeprom 

  double f1;
  double f2;
  double f_meas;
  double f1_meas;
  double f2_meas;
  double f_err;
  bool level = true;
  float duty;
  bool arb_vcosel = false;
  unsigned long tunestart;
  unsigned long tuneend;


  if (checkpots) { Check_all_pots_R(pots); return;}

  if (generatefreq) 
  {
    Serial.print("<generating freq start>");
    GenerateArbitraryFreq(midi_to_pot,400.0, 0.5, f1, f2);
    delay(120000);
    Serial.print("<generating freq end>");
    Serial.flush();
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
  Serial.flush();


  notefreq = pgm_read_float(&(notes_freq[notestart-1]));
  /*
  integrator = MeasureFrequencyDelta(2,10,notefreq);


  dtostrf(integrator, 6, 2, charintegrator);
  sprintf (charintegratorfmt, "<VALUE:%s>",charintegrator);
  Serial1.print(charintegratorfmt);
  */

  //FreqCount.begin(1000);



  for (k=notestart;k<noteend;k++) 
  {
    notefreq = pgm_read_float(&(notes_freq[k]));
    dtostrf(notefreq, 6, 2, charfreq);
    //Serial1.print(charstart);
    
    //Serial1.print(openchar);
    //Serial1.print(charfreq);
    //Serial1.print(closechar);
    checkserial();
    sprintf(printcharfreq,"<F%s>", charfreq);
    Serial.print(printcharfreq);
    Serial.flush();

    if (!checknotes && !checknotes_formula) 
    {
      //AutoTune(pots,midi_to_pot,notefreq,vcosel,false,false);
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
      f_meas = 0.0;
      f1_meas = 0.0;
      f2_meas = 0.0;
      f_err = 0.0;
      duty = 0.75;
      MaxVcoPots(pots,midi_to_pot,0);
      MaxVcoPots(pots,midi_to_pot,1);
      GenerateArbitraryFreq(midi_to_pot,notefreq, 0.75, f1, f2);
      //test delay(2000);
      Serial.print("<Generated_f1=");
      Serial.print(String(f1,3));
      Serial.print(">");
      Serial.print("<Generated_f2=");
      Serial.print(String(f2,3));
      Serial.print(">");
      Serial.flush();
      

      //digitalWrite(vco_pin,0);
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //if (duty > 0.5) { level = !level; arb_vcosel = !arb_vcosel;}
      tunestart = millis();
      // f1 = high , f2 = low
      if ((f1 == minfreq) || (f1 == maxfreq))
      {
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f1,1,0,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f2,0,1,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //CountFrequencyDeltaGlobal(50,notefreq,f_err);
      }
      else if ((f2 == minfreq) || (f2 == maxfreq))
      {
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f2,0,1,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f1,1,0,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      }
      else if (f1 <= f2)
      {
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f2,0,1,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f1,1,0,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
   

      }
      else if (f1 > f2)
      {
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f1,1,0,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTune(pots,midi_to_pot,notestart,notefreq,f2,0,1,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //CountFrequencyDeltaGlobal(50,notefreq,f_err);
      }

      
     /* 
      NewAutoTune(pots,midi_to_pot,f1,1,0);
      // test delay(100);
      CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      
      NewAutoTune(pots,midi_to_pot,f2,0,1);
      // test delay(100);
      CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      */
      //integrator = CountFrequencyDelta(6,1000,notefreq);
      
      Serial.print("<fmeas=");
      Serial.print(String(f_meas,3));
      Serial.print(">");
      /*
      Serial1.print("<f1_meas=");
      Serial1.print(String(f1_meas,3));
      Serial1.print(">");
      Serial1.print("<f2_meas=");
      Serial1.print(String(f2_meas,3));
      Serial1.print(">");
      */
      Serial.flush();
      /*
      Serial1.print("<f1err=");
      Serial1.print(String(f1_err,3));
      Serial1.print(">");
      Serial1.print("<f2err=");
      Serial1.print(String(f2_err,3));
      Serial1.print(">");
      */
      /*
      dtostrf(integrator, 10, 3, charintegrator);
      sprintf (charintegratorfmt, "<INTEG:%s>",charintegrator);
      Serial1.print(charintegratorfmt);
      */  

    } // end check notes formula


    else if (checknotes_formula_DAC)
    {

      f1 = 0.0;
      f2 = 0.0;
      f_meas = 0.0;
      f1_meas = 0.0;
      f2_meas = 0.0;
      f_err = 0.0;
      duty = 0.75;
      MaxVcoPots(pots,midi_to_pot,0);
      MaxVcoPots(pots,midi_to_pot,1);
      GenerateArbitraryFreqDAC(midi_to_pot,notefreq, 0.75, f1, f2);
      //test delay(2000);
      Serial.print("<Generated_f1=");
      Serial.print(String(f1,3));
      Serial.print(">");
      Serial.print("<Generated_f2=");
      Serial.print(String(f2,3));
      Serial.print(">");
      Serial.flush();
      

      //digitalWrite(vco_pin,0);
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //if (duty > 0.5) { level = !level; arb_vcosel = !arb_vcosel;}
      tunestart = millis();
      // f1 = high , f2 = low
      if ((f1 == minfreq) || (f1 == maxfreq))
      {
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //CountFrequencyDeltaGlobal(50,notefreq,f_err);
      }
      else if ((f2 == minfreq) || (f2 == maxfreq))
      {
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      }
      else if (f1 <= f2)
      {
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
   

      }
      else if (f1 > f2)
      {
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,1);
         tuneend = millis() - tunestart;
         Serial.print("<TUNETIME=");
         Serial.print(tuneend);
         Serial.print(">");
         Serial.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //CountFrequencyDeltaGlobal(50,notefreq,f_err);
      }

      
     /* 
      NewAutoTune(pots,midi_to_pot,f1,1,0);
      // test delay(100);
      CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      
      NewAutoTune(pots,midi_to_pot,f2,0,1);
      // test delay(100);
      CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      */
      //integrator = CountFrequencyDelta(6,1000,notefreq);
      
      Serial.print("<fmeas=");
      Serial.print(String(f_meas,3));
      Serial.print(">");
      /*
      Serial1.print("<f1_meas=");
      Serial1.print(String(f1_meas,3));
      Serial1.print(">");
      Serial1.print("<f2_meas=");
      Serial1.print(String(f2_meas,3));
      Serial1.print(">");
      */
      Serial1.flush();
      /*
      Serial1.print("<f1err=");
      Serial1.print(String(f1_err,3));
      Serial1.print(">");
      Serial1.print("<f2err=");
      Serial1.print(String(f2_err,3));
      Serial1.print(">");
      */
      /*
      dtostrf(integrator, 10, 3, charintegrator);
      sprintf (charintegratorfmt, "<INTEG:%s>",charintegrator);
      Serial1.print(charintegratorfmt);
      */  

    } // end check notes formula DAC
    // test 
    checkserial();
    delay(10);
    Serial.print(charend);
    Serial.flush();
    delay(10); 
    // test       
    
    //Serial1.print("T,");
    //Serial1.print(String(notefreq));
    //Serial1.print(",");
    //Serial1.print(String(midi_to_pot[0]));
    //Serial1.print(",");
    //Serial1.print(String(midi_to_pot[1]));
    //Serial1.print(",");
    //Serial1.print(String(midi_to_pot[2]));
    //Serial1.println("");
    bool getdeviation = false;
    float deviation;
    

    while(!getdeviation) 
    {
      recvWithEndMarker();
      //Serial1.println("devloop");
      if (newData == true) 
      {
        //Serial1.println("<yesdata>");
        //Serial1.print("<");
        //Serial1.print(receivedChars);
        //Serial1.println(">");
        deviation = atof(receivedChars);
        //sscanf( receivedChars, "%f\n", &deviation);
        newData = false;
        getdeviation = true;             
      }
      delay(10);
      //Serial1.println("<nodata>");
    }
    delay(1000);
    
    readEEPROMpots (k, &rpot2, &rpot1, &rpot0, &olddeviation, vcosel);

    if(fabs(olddeviation) > fabs(deviation)  && (!checknotes) && (!checknotes_formula)) 
    {

      writeEEPROMpots (k, midi_to_pot[vcosel*max_pot + 2], midi_to_pot[vcosel*max_pot +1], midi_to_pot[vcosel*max_pot], deviation, vcosel);
      //delay(1000);
      //Serial1.print("<new dev better>");
      //Serial1.flush();
    
    }
    else
    { 
        //Serial1.print("<old dev better>"); 
    }


    Serial.print(charackdev);
    Serial.flush();
    /*
    Serial1.print("<rpot2=");
    Serial1.print(rpot2);
    Serial1.print(">");

    Serial1.print("<rpot1=");
    Serial1.print(rpot1);
    Serial1.print(">");

    Serial1.print("<rpot0=");
    Serial1.print(rpot0);
    Serial1.print(">");
    */   
    /*
    dtostrf(olddeviation, 6, 2, chardev);
    sprintf(chardevfmt,"<%s>", chardev);
    Serial1.print("<olddev=>");
    Serial1.print(chardevfmt);

    dtostrf(deviation, 6, 2, chardev);
    sprintf(chardevfmt,"<%s>", chardev);
    Serial1.print("<newdev=>");
    Serial1.print(chardevfmt);
    
    */ 

    //AutoTune(pots,midi_to_pot,notefreq,0,true);

  } // end for notes


    //FreqCount.end();

  /*
    for (k=0;k<49;k++) 
    {
      notefreq = pgm_read_float(&(notes_freq[k]));
      AutoTune(pots,midi_to_pot,notefreq,1);
      

      Serial1.println("{");
      Serial1.println(String(midi_to_pot[3]));
      Serial1.println(",");
      Serial1.println(String(midi_to_pot[4]));
      Serial1.println(",");
      Serial1.println(String(midi_to_pot[5]));
      Serial1.println("},\n");
    }
    
    */
    

} // end setup


// flip the pin #0 up and down

void loop() {

  //delay(10);
  if (midimode)
  {
    MIDI.read();
    return;
  }

  if (Serial.available() > 0) {

  char inp = Serial.read();
  //Serial1.println(inp);

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
    PrintDigiPot(midi_to_pot,0,2); 
  }

    if (inp == 'd') {
    pot1.increase(1);
    (midi_to_pot[1])++;
    //intp1++;
    PrintDigiPot(midi_to_pot,0,2);
  }

    if (inp == 'f') {
    pot2.increase(1);
    (midi_to_pot[2])++;
    //intp2++;
    PrintDigiPot(midi_to_pot,0,2);
  }

    if (inp == 'x') {
    pot0.decrease(1);
    (midi_to_pot[0])--;
    //intp0--;
    PrintDigiPot(midi_to_pot,0,2);
  }

    if (inp == 'c') {
    pot1.decrease(1);
    (midi_to_pot[1])--;
    //intp1--;
    PrintDigiPot(midi_to_pot,0,2);
  }

    if (inp == 'v') {
    pot2.decrease(1);
    (midi_to_pot[2])--;
    //intp2--;
    PrintDigiPot(midi_to_pot,0,2);
  }

    if (inp == 't') {
    DAC0.setValue(4095);
  }

    if (inp == 'g') {
    DAC0.setValue(2047);
  }

    if (inp == 'b') {
    DAC0.setValue(0);
  }



    }

    else {

    if (inp == 's') {
    pot3.increase(1);
    (midi_to_pot[3])++;
    //intp3++;
    PrintDigiPot(midi_to_pot,1,2);
  }

    if (inp == 'd') {
    pot4.increase(1);
    (midi_to_pot[4])++;
    //intp4++;
    PrintDigiPot(midi_to_pot,1,2);
  }

    if (inp == 'f') {
    pot5.increase(1);
    (midi_to_pot[5])++;
    //intp5++;
    PrintDigiPot(midi_to_pot,1,2);
  }

    if (inp == 'x') {
    pot3.decrease(1);
    (midi_to_pot[3])--;
    //intp3--;
    PrintDigiPot(midi_to_pot,1,2); 
  }

    if (inp == 'c') {
    pot4.decrease(1);
    (midi_to_pot[4])--;
    //intp4--;
    PrintDigiPot(midi_to_pot,1,2);
  }

    if (inp == 'v') {
    pot5.decrease(1);
    (midi_to_pot[5])--;
    //intp5--;
    PrintDigiPot(midi_to_pot,1,2); 
  }

   if (inp == 't') {
    DAC1.setValue(4095);
  }

    if (inp == 'g') {
    DAC1.setValue(2047);
  }

    if (inp == 'b') {
    DAC1.setValue(0);
  }
      
    
    }
}

}


