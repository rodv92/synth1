//#include <FreqMeasure.h> 
#include <DigiPotX9Cxxx.h>
#include <Wire.h>
#include <MCP4725.h>
//#include <MemoryFree.h>
//#include <EEPROM.h>
#include <Arduino.h>
//#include <FreqCount.h>
//#include <eRCaGuy_Timer2_Counter.h>
//#include "wiring_private.h"
#include "pins_arduino.h"
#include <MIDI.h>
#include "SPIMemory.h"
#include <avr/dtostrf.h>


// GPIO MCP23017 I2C Expander constructor
Adafruit_MCP23017 mcp0;
//Adafruit_MCP23017 mcp1;
//Adafruit_MCP23017 mcp2;

// SPI W35Q32 Flash Memory constructors :
SPIFlash flash;

// DAC Constructors :
MCP4725 MCPDAC0(0x60);  
MCP4725 MCPDAC1(0x61);
MCP4725 MCPDAC2(0x62);  
MCP4725 MCPDAC3(0x63);


double DAC_gain[4];
double DAC_cor[4];


MIDI_CREATE_DEFAULT_INSTANCE();

byte DebugLevel = 2;

// Digi Pot constructors : (object constructor patched for MCP23017 support)
/*
DigiPot pot0(2,3,40);
DigiPot pot1(2,3,42);
DigiPot pot2(2,3,44);

DigiPot pot3(2,3,46);
DigiPot pot4(2,3,48);
DigiPot pot5(2,3,50);

DigiPot pot6(2,3,30);
DigiPot pot7(2,3,31);
DigiPot pot8(2,3,32);

DigiPot pot9(2,3,33);
DigiPot pot10(2,3,36);
DigiPot pot11(2,3,37);
*/

//VCO 0-0
DigiPot pot0(2,3,0,0,mcp0);
DigiPot pot1(2,3,1,0,mcp0);
DigiPot pot2(2,3,2,0,mcp0);

//VCO 0-1
DigiPot pot3(2,3,3,0,mcp0);
DigiPot pot4(2,3,4,0,mcp0);
DigiPot pot5(2,3,5,0,mcp0);

//VCO 1-0
DigiPot pot6(2,3,6,0,mcp0);
DigiPot pot7(2,3,7,0,mcp0);
DigiPot pot8(2,3,8,0,mcp0);

//VCO 1-1
DigiPot pot9(2,3,9,0,mcp0);
DigiPot pot10(2,3,10,0,mcp0);
DigiPot pot11(2,3,11,0,mcp0);



// VOLUME POT
DigiPot pot12(2,3,52);

// All osc pots : 0 to 5 : OSC1, 6 to 11: OSC2
DigiPot *pots[12] = { &pot0 , &pot1, &pot2, &pot3, &pot4, &pot5, &pot6, &pot7, &pot8, &pot9, &pot10, &pot11 };
// All pots :
DigiPot *allpots[13] = { &pot0 , &pot1, &pot2, &pot3, &pot4, &pot5, &pot6 , &pot7, &pot8, &pot9, &pot10, &pot11, &pot12};

//All DACs :
MCP4725 *allDACs[4] = { &MCPDAC0 , &MCPDAC1, &MCPDAC2, &MCPDAC3};

//


// now stored in FLASH
//double coarse_freq[4][200]; 

// to do : export to FLASH
int16_t sin_table[90]; // one degree resolution sin_table holding [0, Pi/2] interval

uint16_t DAC_table[4][360]; // full period DAC_table for both suboscs

// next two variables are initialized in InitSinTableOptimized
float sample_inc; // time interval in seconds of 1 sample of LFO. (LFO Freq dependent)
float inv_sample_inc_micros; // inverse time interval in µsec of 1 sample of LFO (LFO Freq dependent)
 
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

byte PWM_Note_Settings[49][14];
uint16_t PWM_DAC_Settings[49][4];


byte k;
byte n;
int subvco;
char inp;
bool InTuning = false;
bool PWMActive = false;
bool midimode;

byte midi_to_pot[12];
uint16_t DAC_states[4];
int freqrefpin;
double notefreq;
double minfreq = 130.8;
double maxfreq = 2093.0;

const byte numChars = 40;
char receivedChars[numChars]; // an array to store the received data
bool newData = false;

// Init all pots.
  void StartAllPots(DigiPot *ptr[13]) {

    byte m = 0;

    for(m = 0; m < 13; m++) {
  
        ptr[m]->begin();
        delay(5);
    
      }
    
  }

  
void MaxVcoPots(DigiPot *ptr[12], byte (&curr_pot_vals)[12], byte subvco) {

    byte m = 0;
    byte k = 0;
    byte max_pot = 3;

    for(m = max_pot*subvco; m < max_pot*subvco + max_pot; m++) 
    {
  
      for(k=0;k<100;k++)
      {
        ptr[m]->increase(1);
        delay(5);
      }
      curr_pot_vals[m] = 99;
     
    }
    
  }

void PrintDigiPot(byte (&curr_pot_vals)[12], byte subvco, byte msgdebuglevel)
{

  if (msgdebuglevel <= DebugLevel)
  {
    if (subvco == 0)
    {
      Serial.print("<P0= ");
      Serial.print(curr_pot_vals[0]);
      Serial.print(" P1= ");
      Serial.print(curr_pot_vals[1]);
      Serial.print(" P2= ");
      Serial.print(curr_pot_vals[2]);
      Serial.print(">");
    }
    else if (subvco == 1)
    {
      Serial.print("<P3= ");
      Serial.print(curr_pot_vals[3]);
      Serial.print(" P4= ");
      Serial.print(curr_pot_vals[4]);
      Serial.print(" P5= ");
      Serial.print(curr_pot_vals[5]);
      Serial.print(">");
    }
    else if (subvco == 2)
    {
      Serial.print("<P6= ");
      Serial.print(curr_pot_vals[6]);
      Serial.print(" P7= ");
      Serial.print(curr_pot_vals[7]);
      Serial.print(" P8= ");
      Serial.print(curr_pot_vals[8]);
      Serial.print(">");
    }
    else if (subvco == 3)
    {
      Serial.print("<P9= ");
      Serial.print(curr_pot_vals[9]);
      Serial.print(" P10= ");
      Serial.print(curr_pot_vals[10]);
      Serial.print(" P11= ");
      Serial.print(curr_pot_vals[11]);
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
double hertz_to_R(double freq, byte subvco) {

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


double hertz_to_R_DAC(double freq, double voltage, byte subvco) {

double R; // R in kOhms.

if(!subvco)
{
  R = 1.0/(0.0665E-6*freq - (1.0/33000.0)*(1.0-(voltage/3.0)));
}
else
{
  R = 1.0/(0.0665E-6*freq - (1.0/33000.0)*(1.0-(voltage/3.0)));
}
 
 //R = 1/(fC - (1/R_c)*(1 - Vc/3))
if(R < 0.0)  
  {
    R = R/0.0;
    DebugPrintToken("R_INF",4);
  } // assign NaN to R, unobtainable frequency through R without upping voltage.

return (R/1000.0);

}

/*
double d_hertz_to_R_v2(double freq, int subvco) {

double dR;
dR = -1/(pow(freq,2)*0.0665e-6);

return dR;

}
*/

double d_freq_to_d_volt(double delta_freq, byte subvco) {

double dV;

if(!subvco)
{
  dV = delta_freq * (33000*0.0665e-6)/0.32;
}
else
{
  dV = delta_freq * (33000*0.0665e-6)/0.32;
}


DebugPrint("delta_V",dV,5);

return dV;

}

uint16_t d_freq_to_DAC_steps(double delta_freq, byte subvco) {

uint16_t DAC_steps;
DAC_steps = delta_freq * DAC_gain[subvco];
DebugPrint("delta_DAC_steps",double(DAC_steps),5);

return DAC_steps;

}


void Set_DAC(uint16_t dac_steps, byte subvco) {

  DAC_states[subvco] = dac_steps;
  allDACs[subvco]->setValue(DAC_states[subvco]);

}

void Adjust_DAC(int16_t DAC_steps, byte subvco) 
{
 
  DebugPrint("dac_steps",double(DAC_steps),5);
  DAC_states[subvco] += DAC_steps;  
  DAC_states[subvco] = constrain(DAC_states[subvco],0,4095);

  allDACs[subvco]->setValue(DAC_states[subvco]);
      
}

void InitSinTableOptimized(float LFOFreq, int16_t SinTable[], uint16_t nb_samples)
{
  //compute SinTable on [0, Pi/2] interval, remaining interval [Pi/2 to 2*Pi] is obtained simply from 
  // [0, Pi/2]
  // nb_samples resolution to use for [0,Pi/2] for a resolution of 1 degree = nb_samples = 90

  uint16_t sample;
  float temp_sin;
  float inv_nb_sample;

  //these intervals are initialized there and used for DAC update based on LFO frequency
  sample_inc = 1.0/(LFOFreq*4.0*float(nb_samples)); // sample time increment in sec
  // calculate inverse of time interval. multiplication by inverse is way faster.
  inv_sample_inc_micros = LFOFreq*4.0*float(nb_samples)/1E6; // in 1/µsec
  
  // calculate inverse one time only. multiplication by inverse is way faster.

  inv_nb_sample = 1.0/float(nb_samples);
  for (sample = 0; sample < nb_samples; sample++)
  {
        
    temp_sin = sin(0.5*PI*sample*inv_nb_sample);
    // side effect of [0, Pi/2] only calculation is slight positive bias due to half step rounding.
    sin_table[sample] = (int(32767.0*temp_sin + 0.5));
    
  }

}

void InitSinTable(float LFOFreq, int16_t SinTable[], uint16_t nb_samples)
{
  // todo : use a flash mem 4096 values table with linear interpolation and a modulo 2*pi rounded integer
  // access
  uint16_t sample;
  float temp_sin;
  float inv_nb_sample;

  //these intervals are initialized there and used for DAC update based on LFO frequency
  sample_inc = 1.0/(LFOFreq*float(nb_samples)); // sample time increment in sec
  // calculate inverse of time interval. multiplication by inverse is way faster.
  inv_sample_inc_micros = LFOFreq*float(nb_samples)/1E6; // in 1/µsec
  
  // sin_table is not dependent on LFO frequency. 512 sample resolution full period
  // to do : calculate only half period, and go in reverse direction at PI, reduces table computation
  //time by half
  
  // calculate inverse one time only. multiplication by inverse is way faster.

  inv_nb_sample = 1.0/float(nb_samples);
  for (sample = 0; sample < nb_samples; sample++)
  {
        
    temp_sin = sin(2.0*PI*sample*inv_nb_sample);
    // use commercial half step rounding.
    sin_table[sample] = ((temp_sin > 0)? (int(32767.0*temp_sin + 0.5)):(int(32767.0*temp_sin - 0.5)));
    
  }

}

void Enable_PWM_Mod_by_LFO(float &PWMDepth, float DutyCenterVal, float LFOFreq, float notefreq) 
{
  //enables PWM for the oscillator by managing frequency of both subosc through DAC
  //We have to check that DutyDepth is not out of bounds of DAC range according to the noteoffset.
  float DACdtmp;
  int16_t DACd;
  float dVmax[2];
  float PWMDepthMax[2];
  float PWMDepthMaxPossible;
  //float vco_freq[2];
  float DutyCenterValCmp = 1 - DutyCenterVal;
  static float CurLFOFreq = 0;
  uint16_t sample;
  div_t divresult;

  if (LFOFreq != CurLFOFreq) 
  {
    CurLFOFreq = LFOFreq;
    InitSinTableOptimized(LFOFreq,sin_table,90);
  }

  //vco_freq[0] = notefreq/2*DutyCenterVal;
  //vco_freq[1] = notefreq/(2*(1.0 - DutyCenterVal));
  
  
  //DebugPrint("vco_freq[0]",vco_freq[0],4);
  //DebugPrint("vco_freq[1]",vco_freq[1],4);
  
  // get max voltage deflection from current offset, and get max possible DutyDepth;
  dVmax[0] = 3.0*(2047.0 - fabs(DAC_states[0]- 2047.0))/4095.0;
  dVmax[1] = 3.0*(2047.0 - fabs(DAC_states[1] - 2047.0))/4095.0;
  DebugPrint("DAC_states[0]",DAC_states[0],0);
  DebugPrint("DAC_states[1]",DAC_states[1],0);
 
  DebugPrint("dVmax[0]",dVmax[0],3);
  DebugPrint("dVmax[1]",dVmax[1],3);
  
  
  //min_dVmax = min(dVmax[0],dVmax[1]);

  PWMDepthMax[0] = (2*DutyCenterVal*dVmax[0])/(2*dVmax[0] + 3.0*33000*0.0665E-6*notefreq/DutyCenterVal);
  PWMDepthMax[1] = (2*DutyCenterValCmp*dVmax[1])/(2*dVmax[1] + 3.0*33000*0.0665E-6*notefreq/DutyCenterValCmp);
  DebugPrint("PWMDepthMax[0]",PWMDepthMax[0],3);
  DebugPrint("PWMDepthMax[1]",PWMDepthMax[1],3);
  


  PWMDepthMaxPossible = min(PWMDepthMax[0],PWMDepthMax[1]);

  PWMDepth = min(PWMDepth,PWMDepthMaxPossible);
  DebugPrint("PWMDepth",PWMDepth,3);
  

/*
  for (sample = 0; sample < 512; sample++)
  {
  
    Vd = 3.0*33000*0.0665E-6*vco_freq[0]*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[sample])) -1.0/(2.0*DutyCenterVal));
    DAC_table[0][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[0]),0,4095);
    // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
    // are always equal to 1/f_note in PWM)
    Vd = 3.0*33000*0.0665E-6*vco_freq[1]*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[(256 + sample) % 512])) -1.0/(2.0*DutyCenterVal));
    DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[1]),0,4095);
  

  }
  */




// compute and initialize dac_tables (both subosc) for a full period, using the sintable for [0, Pi/2]
 for (sample = 0; sample < 360; sample++)
  {
  
    divresult = div(sample,90);
    switch (divresult.quot)
    {
    case 0:
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = DAC_gain[0]*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal)) + DAC_cor[0];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[0]),0,4095);
      DAC_table[0][sample] = constrain(DACd + DAC_states[0],0,4095);
      
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = DAC_gain[1]*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp)) + DAC_cor[1];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[1]),0,4095);
      DAC_table[1][sample] = constrain(DACd + DAC_states[1],0,4095);
      
      break;
    
    case 1:
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = DAC_gain[0]*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal)) + DAC_cor[0];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[0]),0,4095);
      DAC_table[0][sample] = constrain(DACd + DAC_states[0],0,4095);
     
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = DAC_gain[1]*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp)) + DAC_cor[1];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[1]),0,4095);
      DAC_table[1][sample] = constrain(DACd + DAC_states[1],0,4095);
     
      break;

    case 2:

      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = DAC_gain[0]*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal)) + DAC_cor[0];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[0]),0,4095);
      DAC_table[0][sample] = constrain(DACd + DAC_states[0],0,4095);
    
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = DAC_gain[1]*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp)) + DAC_cor[1];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[1]),0,4095);
      DAC_table[1][sample] = constrain(DACd + DAC_states[1],0,4095);
     
      break;

    case 3:

      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = DAC_gain[0]*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal)) + DAC_cor[0];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[0]),0,4095);
      DAC_table[0][sample] = constrain(DACd + DAC_states[0],0,4095);
      
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = DAC_gain[1]*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp)) + DAC_cor[1];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[1]),0,4095);
      DAC_table[1][sample] = constrain(DACd + DAC_states[1],0,4095);
   
      break;
    }

  }

  PWMActive = true;
  //timer2.setup();

}

void Disable_PWM_Mod_by_LFO()
{
  Set_DAC(DAC_states[0],0);
  Set_DAC(DAC_states[1],1);
  //timer2.unsetup();
}


double d_hertz_to_R(double freq, byte subvco) {
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
void saw_freq_duty_to_R_DAC(double freq, double voltage, double duty, double &R1 , double &R2, double &f1, double &f2, uint8_t vco) {

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


R1 = hertz_to_R_DAC(f1,voltage,0);
R2 = hertz_to_R_DAC(f2,voltage,1);

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
  if(R != R) {val_pot100K = 199;return;} // check for NaN, if Nan, the two 100K digipots are maxed out. 
  //(unobtainable low frequency through R alone)
  if (subvco == 0) 
  {

    if(R > (R1O0Kb_total_R_vco0 + R1OOK_total_R_vco0 + R1K_total_R_vco0 + trim_pot_R_vco0))
      {val_pot100K = 199;
      DebugPrintToken("ABOVE POSSIBLE R vco0",2);
      return;
      } // R value is above max R obtainable with this setup, max out pots.

    // R is in kOhms
    // we substract wiper resistance and trim pot resistance.
    if (R > R1O0Kb_total_R_vco0) // in this case we use all the steps of pot1 (+99)
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


    if(R > (R1OOK_total_R_vco1 + R1O0Kb_total_R_vco1 + R1K_total_R_vco1 + trim_pot_R_vco1))
      {val_pot100K = 199;
      DebugPrintToken("ABOVE POSSIBLE R vco1",2);
      return;} // R value is above max R obtainable with this setup, max out pots.

    if(R > R1O0Kb_total_R_vco1) // in this case we use all the steps of pot4
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
//byte vco selects the VCO if val_saw is true (0 or 1), else it selects the subvco ( 0 to 3 ) 
void ChangeNote(byte pot_vals[12], byte (&curr_pot_vals)[12], DigiPot *ptr[12], boolean val_saw, byte vco) {

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

  
  for(m = vco*max_pot; m < max_pot + vco*max_pot; m++) {

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

double ReadCoarseFreq(uint8_t step, uint8_t subvco)

{
  uint16_t prev_data_offset = 0;
  uint16_t addr;
  uint16_t tuneblock_size = 200*(sizeof(double));
  
  addr = prev_data_offset + tuneblock_size*subvco + step*(sizeof(double));
  return flash.readFloat(addr);

}


void GenerateArbitraryFreqDAC(byte (&curr_pot_vals)[12], double freq, double duty, double &f1, double &f2, uint8_t vco)
{

  double Rvco1;
  double Rvco2;

  double tmp_f1a;
  double tmp_f2a;
  double tmp_f1b;
  double tmp_f2b;
  
  uint8_t allR100steps_1;
  //uint8_t R100_1;
  //uint8_t R100b_1;
  uint8_t R1_1 = 99;
  
  uint8_t allR100steps_2;
  //uint8_t R100_2;
  //uint8_t R100b_2;
  uint8_t R1_2 = 99;

  uint8_t idx0;
  uint8_t idx1;
 

  byte midi_to_pot_G[12];
  // Get Closest Coarse frequency, find k.
  //coarse_freq[0][k];


  saw_freq_duty_to_R_DAC(freq, 1.5, duty, Rvco1 , Rvco2, f1, f2, vco);

  DebugPrint("duty",duty,2);
  DebugPrint("R1",Rvco1,2);
  DebugPrint("R2",Rvco2,2);

  //Check for R value special cases
  
  // Set the fours DACs to center value (1.5V)
  Set_DAC(2047,0);
  Set_DAC(2047,1);
  Set_DAC(2047,2);
  Set_DAC(2047,3);
  


  // estimating coarse pot R step
  if (isinf(Rvco1)) 
  { 
    allR100steps_1 = 199;
  }
  else
  {
    R_to_pot_DAC(Rvco1, allR100steps_1, 0);
  }
  
  if (isinf(Rvco2)) 
  { 
    allR100steps_2 = 199;
  }
  else
  {
    R_to_pot_DAC(Rvco2, allR100steps_2, 1);
  }
  
  DebugPrint("allR100steps_0", double(allR100steps_1), 2);
  DebugPrint("allR100steps_1",double(allR100steps_2), 2);


  idx0 = 199 - allR100steps_1;
  idx1 = 199 - allR100steps_2;

  DebugPrint("idx0_est",double(idx0),2);
  DebugPrint("idx1_est",double(idx1),2);


  //coarse_freq table lookup index guess...
  //tmp_f1a = coarse_freq[2*vco][idx0];
  //tmp_f2a = coarse_freq[2*vco + 1][idx1];

  // same but using flash memory chip.
  tmp_f1a = ReadCoarseFreq(idx0,2*vco);
  tmp_f2a = ReadCoarseFreq(idx1,2*vco +1);
  


  DebugPrint("f0_est",tmp_f1a,2);
  DebugPrint("f1_est",tmp_f2a,2);


  //try to bracket the frequency between two steps without reading the whole array using estimated R step.
  // using sliding window, then select idx based on closest coarse frequency to required component frequency
  
  // frequencies are sorted in increasing order in coarse_freq
  //VCO 0
  if (tmp_f1a < f1) 
  {
    if (idx0 < 199)
    {
      //tmp_f1b = coarse_freq[2*vco][++idx0];
      tmp_f1b =  ReadCoarseFreq(++idx0,2*vco);
      while ((tmp_f1b < f1) && (idx0 < 199))
      {
        //tmp_f1b = coarse_freq[2*vco][++idx0];
        //tmp_f1a = coarse_freq[2*vco][idx0-1];
        tmp_f1b = ReadCoarseFreq(++idx0,2*vco);
        tmp_f1a = ReadCoarseFreq(idx0 - 1,2*vco);
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
      //tmp_f1b = coarse_freq[2*vco][--idx0];
      tmp_f1b = ReadCoarseFreq(--idx0,2*vco);
      while ((tmp_f1b > f1) && (idx0 > 0))
      {
        //tmp_f1b = coarse_freq[2*vco][--idx0];
        //tmp_f1a = coarse_freq[2*vco][idx0+1];
        tmp_f1b = ReadCoarseFreq(--idx0,2*vco);
        tmp_f1a = ReadCoarseFreq(idx0 + 1, 2*vco);
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
      //tmp_f2b = coarse_freq[2*vco + 1][++idx1];
      tmp_f2b = ReadCoarseFreq(++idx1,2*vco + 1);
      while ((tmp_f2b < f2) && (idx1 < 199))
      {
        //tmp_f2b = coarse_freq[2*vco + 1][++idx1];
        //tmp_f2a = coarse_freq[2*vco + 1][idx1-1];
        tmp_f2b = ReadCoarseFreq(++idx1,2*vco + 1);
        tmp_f2a = ReadCoarseFreq(idx1 - 1,2*vco + 1);
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
      //tmp_f2b = coarse_freq[2*vco + 1][--idx1];
      tmp_f2b = ReadCoarseFreq(--idx1,2*vco + 1);
      while ((tmp_f2b > f2) && (idx1 > 0))
      {
        //tmp_f2b = coarse_freq[2*vco +1][--idx1];
        //tmp_f2a = coarse_freq[2*vco +1][idx1+1];
        tmp_f2b = ReadCoarseFreq(--idx1,2*vco + 1);
        tmp_f2a = ReadCoarseFreq(idx1 + 1,2*vco + 1);
        
      }
      if (((tmp_f2a-f2) <= (f2-tmp_f2b)) && (idx1 >= 0))
      {
        idx1++;;
        // set index to closest frequency
      }
    }
  }
  
  DebugPrint("idx0_real",double(idx0),2);
  DebugPrint("idx1_real",double(idx1),2);

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
  idx0 = 199 - idx0;
  idx1 = 199 - idx1;


  midi_to_pot_G[6*vco + 0] = R1_1;

  if (idx0 > 99) 
  { 
    midi_to_pot_G[6*vco + 1] = 99;
    midi_to_pot_G[6*vco + 2] = idx0 - 99;
  }
  else
  {
    midi_to_pot_G[6*vco + 1] = idx0;
    midi_to_pot_G[6*vco + 2] = 0;
  }


  midi_to_pot_G[6*vco + 3] = R1_2;

  if (idx1 > 99) 
  { 
    midi_to_pot_G[6*vco + 4] = 99;
    midi_to_pot_G[6*vco + 5] = idx1 - 99;
  }
  else
  {
    midi_to_pot_G[6*vco + 4] = idx1;
    midi_to_pot_G[6*vco + 5] = 0;
  }
  
  DebugPrintToken("NEW:",2); 
  PrintDigiPot(midi_to_pot_G,2*vco,2);
  PrintDigiPot(midi_to_pot_G,2*vco + 1,2);
  DebugPrintToken("CUR:",2); 
  PrintDigiPot(curr_pot_vals,2*vco,2);
  PrintDigiPot(curr_pot_vals,2*vco + 1,2);
  


  /*
   for(k=6;k<12;k++){
   midi_to_pot[k] = 99;
   }
  */  

   //Change OSC freq with saw
   
  MCPDAC0.setValue(2047);
  DAC_states[0] = 2047;
  MCPDAC1.setValue(2047);
  DAC_states[1] = 2047;

  MCPDAC2.setValue(2047);
  DAC_states[2] = 2047;
  MCPDAC3.setValue(2047);
  DAC_states[3] = 2047;


  ChangeNote(midi_to_pot_G, curr_pot_vals, pots, true, vco);

  DebugPrintToken("NEW_CUR:",2); 
  PrintDigiPot(curr_pot_vals,2*vco,2);
  PrintDigiPot(curr_pot_vals,2*vco + 1,2);


}
/*
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



   midi_to_pot_G[0] = R1_1; 
   midi_to_pot_G[1] = R10_1; 
   midi_to_pot_G[2] = R100_1; 
   midi_to_pot_G[3] = R1_2; 
   midi_to_pot_G[4] = R10_2; 
   midi_to_pot_G[5] = R100_2; 

  
  // for(k=6;k<12;k++){
  //  midi_to_pot[k] = 99;
  //  }
   

   //Change OSC freq with saw
   ChangeNote(midi_to_pot_G, curr_pot_vals, pots, true, 0);

}
*/

void Check_all_pots_R(DigiPot *ptr[12]) 
{

 byte m = 0;
 byte k = 0;
 byte max_pot = 3;
 byte subvco;
 byte vco_pin = 4;
 pinMode(vco_pin, OUTPUT);
   
for(subvco = 0; subvco < 4; subvco++) 
  {
   if (subvco == 0) 
   {
    digitalWrite(vco_pin, HIGH);
   }
   else if (subvco == 1)
   {
    digitalWrite(vco_pin, LOW); 
   }

    DebugPrint("MIN_ALL_POTS_VCO",double(subvco),6);


      for(m = max_pot*subvco; m < max_pot + max_pot*subvco; m++) {
        
        for(k=0;k<100;k++)
        {
      
          ptr[m]->decrease(1);
          delay(5);
      
        }
    
      
      }

      DebugPrint("MIN_ALL_POTS_VCO_DONE",double(subvco),6);

      delay(20000);


      for(m = max_pot*subvco; m < max_pot + max_pot*subvco; m++) 
      {
        
        DebugPrint("MAX_POT_VCO",double(subvco),6);
        DebugPrint("MAX_POT_VCO_M",double(m),6);


        for(k=0;k<100;k++)
        {
      
          ptr[m]->increase(1);
          delay(5);
      
        }
         
        delay(20000);
        DebugPrint("MAX_ALL_POTS_VCO_DONE",double(subvco),6);
      

        for(k=0;k<100;k++)
        {
      
          ptr[m]->decrease(1);
          delay(5);
      
        }

      }

      for(m = max_pot*subvco; m < max_pot + max_pot*subvco; m++) 
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
  unsigned long start;

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
	start = timer2.get_count();
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
*/

void CountFrequency(byte samplesnumber, double &f_meas, byte subvco)
{
  uint8_t freq_meas_pin;
  if ((subvco == 0) || (subvco == 1))
  {
    freq_meas_pin = 34;
  }
  else if ((subvco == 1) || (subvco == 2))
  {
    freq_meas_pin = 35;
  }
  pinMode(freq_meas_pin, INPUT);
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

  //timer2.setup();
  
  while (count < samplesnumber)
  {
   
    pulsehigh = pulseIn(freq_meas_pin,HIGH,100000);
    pulselow = pulseIn(freq_meas_pin,LOW,100000);

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
 
  //timer2.unsetup();
  
  if ((countlow >0) && (counthigh>0))
  {
    f_total_measured = 2000000.0/((sumhigh/counthigh) + (sumlow/countlow));
    //DebugPrint("ftm",f_total_measured,0);
    //f_err_calc = tunefrequency*1.55005013e-03 -1.45261187e-01;
    f_err_calc = f_total_measured*1.36503806e-03 -7.58951531e-02;
    f_meas = f_total_measured + f_err_calc;  
  }

    DebugPrint("f_total_meas",f_meas,4);
  
}

void CountFrequencyDeltaGlobal(byte samplesnumber,float tunefrequency, double &f_err, uint8_t vco) 
{

  uint8_t freq_meas_pin;
  
  if (vco == 0)
  {
    freq_meas_pin = 34;
  }
  else if (vco == 1)
  {
    freq_meas_pin = 35;
  }
  pinMode(freq_meas_pin, INPUT);
 
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

  //timer2.setup();
  
  while (count < samplesnumber)
  {
   
    pulsehigh = pulseIn(freq_meas_pin,HIGH,100000);
    pulselow = pulseIn(freq_meas_pin,LOW,100000);

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
  //timer2.unsetup();
  
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


void CountFrequencyDelta2(byte samplesnumber,float tunefrequency, double f1, double f2, double &f_meas, double &f1_meas, double &f2_meas, double &f_err, uint8_t vco) 
{
  uint8_t freq_meas_pin;
  
  if (vco == 0)
  {
    freq_meas_pin = 34;
  }
  else if (vco == 1)
  {
    freq_meas_pin = 35;
  }
  pinMode(freq_meas_pin, INPUT);
 

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
  //timer2.setup();
  while (count < samplesnumber)
  {
   
    pulsehigh = pulseIn(freq_meas_pin,HIGH,100000);
    pulselow = pulseIn(freq_meas_pin,LOW,100000);

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
  //timer2.unsetup();
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

void SingleCountFrequencyDelta(byte samplesnumber,double f_global, double f_component, double &f_global_err, double &f_component_err, bool low_or_high, bool globaltune, byte subvco) 
{

  uint8_t freq_meas_pin;
  
  if ((subvco == 0) || (subvco == 1))
  {  
    freq_meas_pin = 34;  
  }
  else if ((subvco == 2) || (subvco == 3))
  {
    freq_meas_pin = 35;
  }

  pinMode(freq_meas_pin, INPUT);
  
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
    //timer2.setup();
    //MIDI.sendNoteOn(82, 127, 1);
   
    while (count < samplesnumber)
    {
    
      pulsehigh = pulseIn(freq_meas_pin,HIGH,100000);
      pulselow = pulseIn(freq_meas_pin,LOW,100000);

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

    //timer2.unsetup();
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
    //timer2.setup();
    //MIDI.sendNoteOn(84, 127, 1);
   
    while (count < samplesnumber)
    {
   
      pulse = pulseIn(freq_meas_pin,low_or_high,100000);
    
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
    //timer2.unsetup();

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


void Generate_dV_to_dHz_Table(byte (&curr_pot_vals)[12],double center_freq, uint8_t subvco)
{
  double f1;
  double f2;
  int i;
  double f_meas;
  double base_f_meas;
  double duty;
  byte vco_pin = 4;

  f1 = 0.0;
  f2 = 0.0;
  f_meas = 0.0;
  duty = 0.5;

  GenerateArbitraryFreqDAC(curr_pot_vals, center_freq, duty, f1, f2, int(subvco/2));
  DebugPrint("dV_dHz DAC:",double(subvco),0);
  //delay(10000);
  digitalWrite(vco_pin, !subvco);
  Set_DAC(0,subvco);
  delay(10);
  CountFrequency(100,f_meas, subvco);
  base_f_meas = f_meas;
  char charfmeas[8];
  DebugPrint("base_f_meas:", base_f_meas, 0);

  for (i=0;i<4095;i++)
  {
    Set_DAC(i,subvco);
    CountFrequency(10,f_meas, subvco);
    f_meas -= base_f_meas;
    dtostrf(f_meas, 7, 2, charfmeas);
    DebugPrintStr("[",0);
    DebugPrintStr(String(i),0);
    DebugPrintStr(",",0);
    Serial.print(charfmeas);
    Serial.flush();
    DebugPrintStr("]",0);
    DebugPrintStr("\n",0);
    
  }


}

void NewAutoTuneDAC(DigiPot *ptr[12], byte (&curr_pot_vals)[12], byte noteindex, double tunefrequency, double duty ,double freq, bool level, byte subvco, bool globaltune) 
{ 
  byte max_pot = 3;
  // if minfreq, set it right away

/*
  if (freq == minfreq)
  {
    MaxVcoPots(ptr,curr_pot_vals,subvco);
    DebugPrintToken("MINFREQ_SET",3);
    PWM_Note_Settings[noteindex][subvco*max_pot] = 99;
    PWM_Note_Settings[noteindex][subvco*max_pot +1] = 99;
    PWM_Note_Settings[noteindex][subvco*max_pot +2] = 99;
     
    //if (globaltune != true) {InTuning = false;}
    return;
  }
*/
  //int num_integrations = 0;
  int max_integrations = 90;
  int states[max_integrations][3];
  float states_integrator[max_integrations];
 
  
  
  
  double dev_cents = 0.0;
  //double dev_cents_back = 0.0;
  const double tune_thresh = 2.5;
  uint16_t DAC_steps = 0;
  
 
  int p;

  //int bias;

  bool tuned = false;
  double dintegrator = 0.0;
  double dintegrator_p = 0.0;
  //double dintegrator_p_1 = 0.0;
  
  
  float integrator = 0.0;
  //float integrator_1 = 0.0;
  //float min_integrator = 0.0;

  //int integrator_count = 0;

  //int best_index = 0;
  byte curr_pot_val_bck = 0;

  //min_integrator = fabs(states_integrator[0]);
        
  for (p=0;p<=max_integrations;p++) 
  {

    if (globaltune)
    {
      //CountFrequencyDeltaGlobal(50,tunefrequency,dintegrator);
      //SingleCountFrequencyDelta(50,freq,dintegrator_p,level);
      //MIDI.sendNoteOn(70, 127, 1);
      SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,true,subvco); 
      //MIDI.sendNoteOn(71, 127, 1);
      DebugPrint("global_err",dintegrator,3);
      integrator = (float) dintegrator;
      dev_cents = 1200 * log((integrator + tunefrequency)/tunefrequency)/log(2);
      DAC_steps = d_freq_to_DAC_steps(dintegrator_p,subvco);
      Adjust_DAC(DAC_steps,subvco);
      //vco formula global tune

    }
    else
    {
      //MIDI.sendNoteOn(72, 127, 1);
      SingleCountFrequencyDelta(10,tunefrequency,freq,dintegrator,dintegrator_p,level,false,subvco); 
      //MIDI.sendNoteOn(73, 127, 1);
      //SingleCountFrequencyDelta(50,freq,dintegrator,level);
      DebugPrint("comp_err",dintegrator_p,3);       
      integrator = (float) dintegrator_p;
      dev_cents = 1200 * log((integrator + freq)/freq)/log(2);
      DAC_steps = d_freq_to_DAC_steps(integrator,subvco);
      Adjust_DAC(DAC_steps,subvco);
      //vco formula single tune

    } // end if globaltune

    if (fabs(dev_cents) <= tune_thresh) 
    { 
      tuned = true;
      DebugPrint("THR_ATT",double(fabs(dev_cents)),3);
   
      states[p][0] = curr_pot_vals[subvco*max_pot];
      states[p][1] = curr_pot_vals[subvco*max_pot+1];
      states[p][2] = curr_pot_vals[subvco*max_pot+2];
      states_integrator[p] = integrator;
      break;
    } //end tune thresh att

      states[p][0] = curr_pot_vals[subvco*max_pot];
      states[p][1] = curr_pot_vals[subvco*max_pot+1];
      states[p][2] = curr_pot_vals[subvco*max_pot+2];
      states_integrator[p] = integrator;
        
  } // end for p < max_integrations

  PWM_Note_Settings[noteindex][subvco*max_pot] = curr_pot_vals[subvco*max_pot];
  PWM_DAC_Settings[noteindex][subvco*max_pot] = DAC_states[subvco];


} // end NewAutotuneDAC

void Attack_Decay_Sustain()
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
  uint8_t i;
  byte noteindex;
  byte midi_to_pot_G[12];
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
    for(i=0;i<12;i++)
    {
      midi_to_pot_G[i] = PWM_Note_Settings[noteindex][i];
    }
    
    ChangeNote(midi_to_pot_G, midi_to_pot, pots, true, 0); // Change note for VCO 0 (subosc 0&1)
    ChangeNote(midi_to_pot_G, midi_to_pot, pots, true, 1); // Change note for VCO 1 (subosc 2&3)
    Set_DAC(PWM_DAC_Settings[0][noteindex],0);
    Set_DAC(PWM_DAC_Settings[1][noteindex],1);
    Set_DAC(PWM_DAC_Settings[2][noteindex],2);
    Set_DAC(PWM_DAC_Settings[3][noteindex],3);
    
    
    Attack_Decay_Sustain();
    Serial1.print("S");
    Serial1.print(String(float(noteindex),3));
    Serial1.print("Z");
    Serial1.flush();
    //pot12.decrease(30);
    return;
  }
  //MIDI.sendNoteOn(45, 127, 1);
  
  Attack_Decay_Sustain();
  //pot12.decrease(30);
  notefreq = double(pgm_read_float(&(notes_freq[noteindex])));
  

  DebugPrint("NOTEFREQ",notefreq,2);

  MaxVcoPots(pots,midi_to_pot,0);
  MaxVcoPots(pots,midi_to_pot,1);
  //MIDI.sendNoteOn(46, 127, 1);
  

  // Tune OSCs sequentially.
  for (i=0;i<2;i++)
  {
    // Set pots for coarse tuning and set both DACs to half deflection setpoint (2047) 
    GenerateArbitraryFreqDAC(midi_to_pot,notefreq, duty, f1, f2, i); // O is for VCO 0 (not subvco)
  
    //MIDI.sendNoteOn(47, 127, 1);

    // f1 = high , f2 = low
    if ((f1 == minfreq) || (f1 == maxfreq))
    {
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f1,1,0,2*i);
        //MIDI.sendNoteOn(48, 127, 1);
  
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f2,0,1,2*i + 1);
        //MIDI.sendNoteOn(49, 127, 1);
  
        //TO DO : split tuning for both VCO in a loop
        CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        //CountFrequencyDeltaGlobal(50,notefreq,f_err);
        //MIDI.sendNoteOn(50, 127, 1);
  
    }
    else if ((f2 == minfreq) || (f2 == maxfreq))
    {
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f2,0,1,2*i);
        //MIDI.sendNoteOn(51, 127, 1);
  
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f1,1,0,2*i + 1);
        //MIDI.sendNoteOn(52, 127, 1);
  
        CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        //MIDI.sendNoteOn(53, 127, 1);
  
    }
    else if (f1 <= f2)
    {
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f2,0,1,2*i);
        //MIDI.sendNoteOn(54, 127, 1);
  
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f1,1,0,2*i + 1);
        //MIDI.sendNoteOn(55, 127, 1);
  
        CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        //MIDI.sendNoteOn(56, 127, 1);
  
    }
    else if (f1 > f2)
    {
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f1,1,0,2*i);
        //MIDI.sendNoteOn(57, 127, 1);
  
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreq,duty,f2,0,1,2*i + 1);
        //MIDI.sendNoteOn(58, 127, 1);
  
        CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        //MIDI.sendNoteOn(59, 127, 1);
  
        //CountFrequencyDeltaGlobal(50,notefreq,f_err);
    }
  } // end tune OSC sequentially

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

void WriteEEPROMCoarsePotStepFrequencies(DigiPot *ptr[12], byte (&curr_pot_vals)[12], byte subvco)
{

  byte vco_pin = 4;
  int k = 0;
  byte max_pot = 3;
  byte m;
  // max_pot*subvco + 1;
  double f_meas;

  //base flash address;
  uint16_t prev_data_offset = 0;
  
  // tuneblock is for ONE potentiometer.
  int tuneblock_size = 100*(sizeof(f_meas));
  //DebugPrint("WRITE COARSE VCO:",double(subvco),2);
  digitalWrite(vco_pin, !subvco);

  flash.eraseSection(prev_data_offset + subvco*2*tuneblock_size,2*tuneblock_size);  

  for(m=max_pot*subvco + 1;m<=max_pot*subvco + 2;m++)
  {
    //DebugPrint("RESET POT:",double(m),2);
    for(k=0;k<100;k++)
    {
      ptr[m]->increase(1);
      delay(5);

    }
    curr_pot_vals[m] = 99;
  }

  
  for(m=max_pot*subvco + 1;m<=max_pot*subvco + 2;m++)
  {
    //DebugPrint("WRITE COARSE POT:",double(m),2);
  
    for(k=99;k>=0;k--)
    {

      DebugPrint("STEP:",double(k),2);
      Serial.println("");
      Serial.flush();
      CountFrequency(50,f_meas, subvco);
      //DebugPrint("FREQ:",f_meas,2);
      int addr = prev_data_offset + (2*subvco + (m -(max_pot*subvco +1)))*tuneblock_size + (99-k)*(sizeof(f_meas));
      DebugPrint("ADDR:",double(addr),2);
      flash.writeFloat(addr,f_meas);
      //EEPROM.put(addr,f_meas);
      //we write frequencies in increasing order
      //delay(1000);
      //DebugPrint("END_STEP",double(k),2);
      ptr[m]->decrease(1);
      curr_pot_vals[m]--;

    }
  }
  
}


/*
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
      DebugPrint(String(k),coarse_freq[vco][k],5);
      // we had written frequencies in increasing order
    }


  }

}
*/
/*
void writeEEPROMpots (byte noteindex, byte pot2, byte pot1, byte pot0, float deviation, byte subvco)
{

int tuneblock_size = 49*(sizeof(pot2)+sizeof(pot1)+sizeof(pot0)+sizeof(deviation));
int addr = subvco*tuneblock_size + noteindex*(sizeof(pot2)+sizeof(pot1)+sizeof(pot0)+sizeof(deviation));

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
*/

/*
void readEEPROMpots (byte noteindex, byte *pot2, byte *pot1, byte *pot0, float *deviation, byte subvco)
{

int tuneblock_size = 49*(sizeof(*pot2)+sizeof(*pot1)+sizeof(*pot0)+sizeof(*deviation));
int addr = subvco*tuneblock_size + noteindex*(sizeof(*pot2)+sizeof(*pot1)+sizeof(*pot0)+sizeof(*deviation));


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
*/

/*
void dumpEEPROMpots (byte *all_pot_vals, float *all_pot_devs, byte subvco)
{
byte k;
all_pot_vals += 3;
all_pot_devs++;
int tuneblock_size = 49*(sizeof(byte)+sizeof(byte)+sizeof(byte)+sizeof(float));


//Serial1.print("<tuneblock:");
//Serial1.print(tuneblock_size);
//Serial1.print(">");



for (k=1;k<49;k++) {

int addr = tuneblock_size*subvco + k*(sizeof(byte)+sizeof(byte)+sizeof(byte)+sizeof(float));

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
*/


/*
void checkserial()
{
if (!Serial) {
delay(10);
//Serial1.begin(9600,SERIAL_8E2);
Serial1.begin(9600);
//Serial1.println("<reset>");  
}

}
*/

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = 'Z';
  char rc;
 

  if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();
      //delay(10);
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

/// if val_saw is true, vco selects the vco (0 or 1) else, it selects the subvco (0 to 3)
void SetNotePots(DigiPot *ptr[12], byte (&pot_vals)[12], bool val_saw, byte vco) {

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

  for(m = vco*max_pot; m < max_pot + vco*max_pot; m++) {

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

double CheckTuning(DigiPot *ptr[6], byte (&curr_pot_vals)[12], byte noteindex, byte subvco) 
{

double notefreq;
double component_freq = notefreq;
double integrator;
double component_integrator;

SetNotePots(pots,curr_pot_vals,0,subvco);

notefreq = double(pgm_read_float(&(notes_freq[noteindex])));
//integrator = MeasureFrequencyDelta(2,10,notefreq);
SingleCountFrequencyDelta(10,notefreq,component_freq,integrator,component_integrator,LOW,true,subvco);

return integrator;

}


void setup() 
{

  flash.begin();
  MCPDAC0.begin();
  MCPDAC1.begin();
  MCPDAC2.begin();
  MCPDAC3.begin();
  Set_DAC(2047,0);
  Set_DAC(2047,1);
  Set_DAC(2047,2);
  Set_DAC(2047,3);

  DAC_gain[0] = -9.169;
  DAC_gain[1] = -9.184;
  // modify these values after calibration
  DAC_gain[2] = -9.169;
  DAC_gain[3] = -9.184;
  
  DAC_cor[0] = 5.25;
  DAC_cor[1] = 1.673;
  // modify these values after calibration
  DAC_cor[2] = 5.25;
  DAC_cor[3] = 1.673;


  //char charstart[4] = "<F>";
  char charend[4] = "<E>";
  char charackdev[4] = "<D>";

  //char openchar = '<';
  //char closechar = '>';


  char charspeed[14] = "<SERIAL 9600>";
  char charfreq[8];
  char printcharfreq[10];
    
  //bool dumpeeprom = false;
  bool checknotes = false;
  bool checknotes_formula_DAC = true;
  bool checkPWM_LFO = true;
  bool checkpots = false;
  bool generatefreq = false;
  bool generatedVdHzTable = false;
  bool writecoarsefreqs = false;
  bool readcoarsefreqs = false;
  midimode = false;
  bool donothing = false;

  byte vco_pin = 4;
  byte max_pot = 3;
  byte notestart;
  byte noteend;

  long currentsample = 0;
  long prevsample = 0;
      
   
  //Serial1.begin(9600,SERIAL_8E2);
  if (!midimode)
  {
    //Serial.begin(2400,SERIAL_8E2);
    Serial.begin(9600);
    Serial.print(charspeed);
    Serial.flush();   
    delay(3000);
  }
  StartAllPots(allpots);
  for (k=0;k<49;k++)
  {
    for(n=0;n<13;n++)
    {
      PWM_Note_Settings[k][n] = 0;
    }    
  }
  
  freqrefpin = 9;
  subvco = 0;
  // select subvco 0 by default
  pinMode(vco_pin, OUTPUT);
  digitalWrite(vco_pin, !subvco);

  // use default address 0
  //mcp.pinMode(0, OUTPUT);

  MaxVcoPots(pots,midi_to_pot,0);
  MaxVcoPots(pots,midi_to_pot,1);

  MaxVcoPots(pots,midi_to_pot,2);
  MaxVcoPots(pots,midi_to_pot,3);
  
  //DACTEST
  //pot1.decrease(99);
  //pot4.decrease(99);

  pot12.increase(99);
  delay(5000);

  //pot12.decrease(30);
  

  byte pot_vals[49][12];
  float pot_devs[49];

  byte rpot2;
  byte rpot1;
  byte rpot0;
  float olddeviation;
  char chardev[30];
  char chardevfmt[32];

  byte i;
  for (i=0;i<12;i++)
  {
    pot_vals[0][i] = 99;
  }
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
    // TO DO : Convert ReadAllCoarseFreq to flash read
    //ReadAllCoarseFrequencies();
    return;
    
  }

  if (readcoarsefreqs)
  {

    // TO DO : Convert ReadAllCoarseFreq to flash read
    //ReadAllCoarseFrequencies();
    return;
    
  }
  

/*
  if (dumpeeprom) 
  {   
    //dumpEEPROMpots((byte *)pot_vals,(float *)pot_devs,subvco);

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
    
      
      // not required for python curve fitting script 
      //dtostrf(pot_devs[k], 6, 2, chardev);
      //sprintf(chardevfmt,"%s", chardev);

      //Serial1.print(chardevfmt);
      //Serial1.print("]>");
      
    
      //  delay(100); 
    } // end for all notes dumpeeprom
  //return;
  } // end dumpeeprom 
*/
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
    GenerateArbitraryFreqDAC(midi_to_pot,400.0, 0.5, f1, f2,0); // 0 is for VCO 0 not subvco.
    delay(120000);
    Serial.print("<generating freq end>");
    Serial.flush();
    return;
  }

  notestart = 1;
  noteend = 49;


  midi_to_pot[subvco*max_pot] = pot_vals[notestart - 1][2];
  midi_to_pot[subvco*max_pot +1] = pot_vals[notestart - 1][1];
  midi_to_pot[subvco*max_pot +2] = pot_vals[notestart - 1][0];


  Serial.print("<");
  Serial.print(midi_to_pot[subvco*max_pot +2]);
  Serial.print(",");
  Serial.print(midi_to_pot[subvco*max_pot +1]);
  Serial.print(",");
  Serial.print(midi_to_pot[subvco*max_pot]);
  Serial.print(">");
  Serial.flush();

  float integrator;
  char charintegratorfmt[13];
  char charintegrator[7];


  SetNotePots(pots,midi_to_pot,0,subvco);


  Serial.print("<midi_to_pot:");
  Serial.print(midi_to_pot[subvco*max_pot +2]);
  Serial.print(midi_to_pot[subvco*max_pot +1]);
  Serial.print(midi_to_pot[subvco*max_pot]);
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
  if ((checknotes_formula_DAC) || (generatedVdHzTable)) 
  {
    // Not necessary, now in flash memory.
    //ReadAllCoarseFrequencies(); // populating global coarse_freq array fromEEPROM
  }  
  
  for (k=notestart;k<noteend;k++) 
  {
    notefreq = pgm_read_float(&(notes_freq[k]));
    dtostrf(notefreq, 6, 2, charfreq);
    //Serial1.print(charstart);
    
    //Serial1.print(openchar);
    //Serial1.print(charfreq);
    //Serial1.print(closechar);
    //checkserial();
    sprintf(printcharfreq,"<F%s>", charfreq);
    Serial.print(printcharfreq);
    Serial.flush();

    if (generatedVdHzTable)
    {
      MaxVcoPots(pots,midi_to_pot,0);
      MaxVcoPots(pots,midi_to_pot,1);
      DebugPrintStr("POTS MAXED",0);
      delay(10000);
      Generate_dV_to_dHz_Table(midi_to_pot,notefreq,0);
      Generate_dV_to_dHz_Table(midi_to_pot,notefreq,1);
      continue;
    }

    if (!checknotes && !checknotes_formula_DAC) 
    {
      //AutoTune(pots,midi_to_pot,notefreq,subvco,false,false);
    }
    else if (checknotes)
    {
      midi_to_pot[subvco*max_pot] = pot_vals[k][2];
      midi_to_pot[subvco*max_pot +1] = pot_vals[k][1];
      midi_to_pot[subvco*max_pot +2] = pot_vals[k][0];

      CheckTuning(pots,midi_to_pot,k,subvco);
    }
    

    else if (checknotes_formula_DAC)
    {

      f1 = 0.0;
      f2 = 0.0;
      f_meas = 0.0;
      f1_meas = 0.0;
      f2_meas = 0.0;
      f_err = 0.0;
      duty = 0.5;
      MaxVcoPots(pots,midi_to_pot,0);
      MaxVcoPots(pots,midi_to_pot,1);
      GenerateArbitraryFreqDAC(midi_to_pot,notefreq, duty, f1, f2, 0); // 0 is for VCO 0, not subvco
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

      // to do : check if that section is still required.
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
         // TO DO : manage both OSCs
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
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
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
      }
      // end check if that section is still required.
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
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
   

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
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
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
      Serial.flush();
     
    } // end check notes formula DAC
    // test 
    if (!checkPWM_LFO)
    {
      //checkserial();
      //delay(10);
      Serial.print(charend);
      Serial.flush();
      delay(10); 
    }
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

    if (checkPWM_LFO)
    {
      float PWMDepth = 0.1;
      Enable_PWM_Mod_by_LFO(PWMDepth,0.5,0.1,notefreq);
      //int i;
      /*
      delay(5);
      DebugPrintToken("DACTABLE_0:",3);
      for (i=0; i<360; i++)
      {
        DebugPrint(String(i),DAC_table[0][i],3);
        delay(100);       
      }
      delay(5);
      DebugPrintToken("DACTABLE_1:",3);
      for (i=0; i<360; i++)
      {
        DebugPrint(String(i),DAC_table[1][i],3);
        delay(100);       
      }
      */
      //checkserial();
      //delay(10);
      Serial.print(charend);
      Serial.flush();
      //delay(10); 
  
    }  

    bool getdeviation = false;
    float deviation;
    bool switchvco = false;
      
    while(!getdeviation) 
    {
      recvWithEndMarker();
      //Serial1.println("devloop");
      if (checkPWM_LFO)
      {
        
        // PWM mode free running. (do not update DAC state needlessly)
        // to do : LFO 'handlenoteon' trigger mode.
        currentsample = micros();
        //currentsample = long(timer2.get_micros()*inv_sample_inc_micros + 0.5);
        if(currentsample != prevsample)
        {
          
          MCPDAC0.setValue(DAC_table[0][currentsample % 360]);
          //delayMicroseconds(1000);
          MCPDAC1.setValue(DAC_table[1][currentsample % 360]);
        /*
        if ((currentsample % 720) == 0)
        {
          DebugPrint("VCO:",double(switchvco),3);
          digitalWrite(vco_pin, switchvco);
          switchvco = !switchvco;
        } 
        */


        }
        
        prevsample = currentsample;
      }

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
      //delay(10);
      //Serial1.println("<nodata>");
    }

    if (checkPWM_LFO) {Disable_PWM_Mod_by_LFO();}
    delay(1000);
    
    //readEEPROMpots (k, &rpot2, &rpot1, &rpot0, &olddeviation, subvco);

    if ((fabs(olddeviation) > fabs(deviation))  && !checknotes && !checknotes_formula_DAC) 
    {

      //writeEEPROMpots (k, midi_to_pot[subvco*max_pot + 2], midi_to_pot[subvco*max_pot +1], midi_to_pot[subvco*max_pot], deviation, subvco);
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
  static long currentsample = 0;
  static long prevsample = 0;
  
  if (PWMActive)
  {
   // PWM mode free running. (do not update DAC state needlessly)
   // to do : LFO 'handlenoteon' trigger mode.
   //currentsample = long(timer2.get_micros()*inv_sample_inc_micros + 0.5);
   currentsample = micros();
   if(currentsample != prevsample)
   {
     
     MCPDAC0.setValue(DAC_table[0][currentsample % 360]);
     MCPDAC1.setValue(DAC_table[1][currentsample % 360]);
   
   }

   prevsample = currentsample;
  }

  if (midimode)
  {
    MIDI.read();
    return;
  }

  if (Serial.available() > 0) {

  char inp = Serial.read();
  //Serial1.println(inp);

    if (inp == '0') {
      subvco = 0;
      Serial.println("VCOSEL 0");
      digitalWrite(4,HIGH);
    }
    else if(inp == '1') {
      subvco = 1;
      Serial.println("VCOSEL 1");
      digitalWrite(4,LOW);
    }

    if (!subvco) {

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
    MCPDAC0.setValue(4095);
  }

    if (inp == 'g') {
    MCPDAC0.setValue(2047);
  }

    if (inp == 'b') {
    MCPDAC0.setValue(0);
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
    MCPDAC1.setValue(4095);
  }

    if (inp == 'g') {
    MCPDAC1.setValue(2047);
  }

    if (inp == 'b') {
    MCPDAC1.setValue(0);
  }
      
    
    }
}

}


