//#include <FreqMeasure.h> 
#include <DigiPotX9Cxxx.h>
#include <Wire.h>
extern TwoWire Wire;
extern TwoWire Wire1;
#include <MCP4725.h>
#include <MCP4728.h>
//#include <MemoryFree.h>
//#include <EEPROM.h>
#include <Arduino.h>
//#include <FreqCount.h>
//#include <eRCaGuy_Timer2_Counter.h>
#include <tc_lib.h>

using namespace arduino_due;

// action_tc2 declaration
action_tc_declaration(2);
action_tc_declaration(3);


//#include "wiring_private.h"
#include "pins_arduino.h"

//TODO : include native USB-HID Midi (MIDIUSB)
#include <MIDI.h>
#include <midi_Defs.h>

#include "SPIMemory.h"
#include <avr/dtostrf.h>


// using a 1.8K resistor in series on the low current pins
// using a 560 resistor in series on the high current pins

uint8_t vco0_pin = 5; // high current pin
uint8_t vco1_pin = 6; // high current pin
uint8_t gatepin = 7; // high current pin, for Attack Decay ok
uint8_t hspin = 8; // high current pin, for hardsync on/off ok
uint8_t flashcspin = 9; // high current pin, flash CS pin ok
uint8_t ldacpin = 14; // high current pin MCP4728 ldac pin ok
uint8_t swvco_meas_pin = 46; // high current pin, for switching incoming SQ signal between OSC1/OSC2 ok
uint8_t freqmeaspin = A7; // low current pin, used to measure sq signal frequency for OSC1/OSC2 ok
// plus 4 pins for I2C wire and wire1 
//SCL1 high current pin // for MCP4728 DAC OK
//SDA1 low current pin // for MCP4728 DAC OK
//SCL low current pin // for MCP23017 Demux ok
//SDA low current pin // for MCP23017 Demux ok
//plus 4 pins for 4 last digipots.

// plus pins 2 and 3 for digipot up/down
// pin 2 low current INC ok
// pin 3 high current UD ok

//total required pins 17 + SPI pins
// 12 high current
// 5 low current

// GPIO MCP23017 I2C Expander constructor
Adafruit_MCP23017 mcp0;
//Adafruit_MCP23017 mcp1;
//Adafruit_MCP23017 mcp2;

// SPI W35Q32 Flash Memory constructors :
SPIFlash flash(flashcspin);

// DAC Constructors :
//MCP4725 MCPDAC0(0x60,&Wire);  
//MCP4725 MCPDAC1(0x61,&Wire);
//MCP4725 MCPDAC2(0x60,&Wire1);  
//MCP4725 MCPDAC3(0x61,&Wire1);
MCP4728 MCPdac;


double DAC_gain[4];
double DAC_cor[4];
uint8_t adaptive_index[4] = {0,0,0,0};


double DAC_gain_adaptive[200][4];
// DAC_gain_corrected for LFO corrections for each note first index : noteindex, second index, subvco for each gain corrected, last position = tuned
double DAC_gain_corrected_lfo[49][4];
// DACoffset for LFO corrections for each note first index : noteindex, second index, subvco for each offset, last position = tuned
int16_t DACoffset[49][4];
  

// store the targets for PWM. first index is subvco, second index is min and max frequency targets in that order.
double LFO_MinMax_Freq_Targets[4][2];
// store the frequency deltas by components for PWM. first index is subvco, second index is min and max frequency targets in that order.
double LFO_Delta_Freq_Targets[4][2];



MIDI_CREATE_INSTANCE(HardwareSerial,Serial,MIDI);

byte DebugLevel = 1;
uint16_t DebugDelay = 0; // slow messaging speed and processes to aid real time debug (ms)

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
/*
//VCO 0-0
DigiPot pot0(10,11,31);
DigiPot pot1(10,11,33);
DigiPot pot2(10,11,35);

//VCO 0-1
DigiPot pot3(10,11,37);
DigiPot pot4(10,11,39);
DigiPot pot5(10,11,41);

//VCO 1-0
DigiPot pot6(10,11,43);
DigiPot pot7(10,11,45);
DigiPot pot8(10,11,30);

//VCO 1-1
DigiPot pot9(10,11,32);
DigiPot pot10(10,11,34);
DigiPot pot11(10,11,36);

// ATTACK POT
DigiPot pot12(10,11,38);
// DECAY POT
DigiPot pot13(10,11,40);

// MIXER1 POT
DigiPot pot14(10,11,42);
// MIXER2 POT
DigiPot pot15(10,11,44);


// MIXER3 POT
DigiPot pot16(50,52,24);
// MIXER4 POT
DigiPot pot17(50,52,26);


// PHASE1 POT
DigiPot pot18(50,52,22);
// PHASE2 POT
DigiPot pot19(50,52,28);
*/



DigiPot pot0(2,3,0,0,mcp0);
DigiPot pot1(2,3,48); // mcp0 pin 1 is fried.
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

// ATTACK POT
DigiPot pot12(2,3,12,0,mcp0);
// DECAY POT
DigiPot pot13(2,3,13,0,mcp0);

// MIXER POT
DigiPot pot14(2,3,14,0,mcp0);
// MIXER 2 POT
DigiPot pot15(2,3,15,0,mcp0);



// MIXER3 POT
DigiPot pot16(26,28,34);
// MIXER4 POT
DigiPot pot17(26,28,36);


// PHASE1 POT
DigiPot pot18(26,28,30);
// PHASE2 POT
DigiPot pot19(26,28,32);




// All osc pots : 0 to 5 : OSC1, 6 to 11: OSC2
DigiPot *pots[12] = { &pot0, &pot1, &pot2, &pot3, &pot4, &pot5, &pot6, &pot7, &pot8, &pot9, &pot10, &pot11 };
// All pots :
DigiPot *allpots[20] = { &pot0, &pot1, &pot2, &pot3, &pot4, &pot5, &pot6 ,&pot7, &pot8, &pot9, &pot10, &pot11, &pot12, &pot13, &pot14, &pot15, &pot16, &pot17, &pot18, &pot19};
//DigiPot *allpots[16] = { &pot0 , &pot1, &pot2, &pot3, &pot4, &pot5, &pot6 , &pot7, &pot8, &pot9, &pot10, &pot11, &pot12, &pot13, &pot14, &pot15};

//All DACs :
//MCP4725 *allDACs[4] = { &MCPDAC0 , &MCPDAC1, &MCPDAC2, &MCPDAC3};

//


// now stored in FLASH
double coarse_freq[4][200]; 

// to do : export to FLASH
int16_t sin_table[360]; // one degree resolution sin_table holding [0, 2*Pi] interval

uint16_t DAC_table[4][360]; // full period DAC_table for both suboscs
uint16_t AD_table[2][500];
uint16_t R_table[2][500];

// next two variables are initialized in InitSinTableOptimized
float sample_inc; // time interval in seconds of 1 sample of LFO. (LFO Freq dependent)
float inv_sample_inc_micros; // inverse time interval in µsec of 1 sample of LFO (LFO Freq dependent)

float sample_adsr_inc[8];// time interval in seconds for attack,decay,release, sustain(if looped). first 4 values = A D R S (DAC0),
// next 4, A D R S (DAC1)
float inv_sample_adsr_inc_micros[8]; // same as above, inverse time interval
 
uint8_t ADSR_DAC_State[2];
uint8_t ADSR_Sustain_Level[2];

uint16_t ADSR_nb_samples[2];
bool ADSR_looped = true;

bool Gate_DAC = false;
uint32_t gate_period_hundredth_micros = 0.0;

// semitone shift of OSC2
int16_t osc2noteshift;
int8_t cents_detune[2] = {0,0};

// ringmod octave setting for DAC1.
int8_t ringmod_octave = 1;

// global variables used by LFO and handlenoteon
double duty[2] = {0.44,0.44};
double LFOFreq[2] = {0.8,0.8};
double PWMDepth[2] = {0.05,0.05};
double notefreqs[2];
uint8_t noteindexes[2];
static uint8_t totalerrorsamples[2] = {0,0};

// VCA attack/decay
int16_t attack;
int16_t decay;

int16_t cur_attack;
int16_t cur_decay;

int16_t chng_attack;
int16_t chng_decay;

// OSC1/2 volumes. mix3 and 4 are for SQ WAVE outputs.
int16_t mix1; // osc1
int16_t mix2; // osc2
int16_t mix3; // osc1 sq
int16_t mix4; // osc2 sq

int16_t cur_mix1 = 99;
int16_t cur_mix2 = 99;
int16_t cur_mix3 = 99;
int16_t cur_mix4 = 99;

int16_t chng_mix1;
int16_t chng_mix2;
int16_t chng_mix3;
int16_t chng_mix4;

// osc1 and 2 sq wave phase shifts

int16_t phase1; // osc1
int16_t phase2; // osc2

int16_t cur_phase1;
int16_t cur_phase2;

int16_t chng_phase1;
int16_t chng_phase2;



volatile uint32_t CaptureCountA, CaptureCountB, Period, Duty;
volatile boolean CaptureFlag;

enum ADSR_States {ADSR_Disable = 0, ADSR_Disabled, ADSR_Off, ADSR_AttackInit, ADSR_Attack, ADSR_DecayInit, ADSR_Decay, ADSR_Sustain, ADSR_ReleaseInit, ADSR_Release};


const double notes_freq[49] = {

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

uint8_t PWM_Note_Settings[49][14];
uint16_t PWM_DAC_Settings[49][4];
double PWM_Real_Duty[49][2];

byte k;
byte n;
uint8_t subvco;

char inp;
char potlow_inc[2] = "s";
char potmid_inc[2] = "d";
char pothigh_inc[2] =  "s";



bool InTuning = false;
bool InitialTune = true;
bool PWMActive = false;
bool midimode;

byte midi_to_pot[12];
uint16_t DAC_states[6];
uint16_t DAC_states_det_backup[49][6];
  
//int freqrefpin;



//double minfreq = 130.8;
double minfreq = 90.0;
double maxfreq = 2093.0;

const byte numChars = 40;
char receivedChars[numChars]; // an array to store the received data
bool newData = false;

// Init all pots.
  void StartAllPots(DigiPot *ptr[20]) {

    uint8_t m = 0;
    //SerialUSB.print("OKSP");
    //Serial.flush();
    //ptr[0]->beginMCP();

    for(m = 0; m < 20; m++) {
  
        ptr[m]->begin();
        delay(10);
        //SerialUSB.print(m);
        //Serial.flush();
    
      }
    
  }

void MaxRemPots(DigiPot *ptr[20]) 
{

  uint8_t m = 0;
  uint8_t k = 0;

  for(m = 12; m < 20; m++) 
  {
    for(k=0;k<100;k++)
      {

        ptr[m]->increase(1);
        delay(2);

      }

  }

}


  
void MaxVcoPots(DigiPot *ptr[12], byte (&curr_pot_vals)[12], byte subvco) {

    byte m = 0;
    byte k = 0;
    byte max_pot = 3;

    for(m = max_pot*subvco; m < max_pot*subvco + max_pot; m++) 
    {
  //    ptr[m]->increase(99);
      //SerialUSB.print(m);
      //Serial.flush();
  
      for(k=0;k<100;k++)
      {
        //SerialUSB.print(k);
        //SerialUSB.flush();
        ptr[m]->increase(1);
        delay(2);
      }


      for(k=0;k<50;k++)
      {
        //SerialUSB.print(k);
        //SerialUSB.flush();
        ptr[m]->decrease(1);
        delay(2);
      }


      for(k=0;k<100;k++)
      {
        //SerialUSB.print(k);
        //SerialUSB.flush();
        ptr[m]->increase(1);
        delay(2);
      }

  
      //SerialUSB.print(m);
      curr_pot_vals[m] = 99;
     
    }
    
  }

void PrintDACGainAdaptive(byte msgdebuglevel)
{

  uint8_t m;
  uint8_t i;
  if (DebugDelay > 0) {delay(DebugDelay);}
  
  if (msgdebuglevel <= DebugLevel)
  {
    for (m=0;m < 4;m++)
    {
      SerialUSB.print("SUBVCO: ");
      SerialUSB.print(m);
      SerialUSB.print("\n");

      for (i=0;i< adaptive_index[m];i++)
      {
        SerialUSB.print(DAC_gain_adaptive[i][m]);
        SerialUSB.print("\n");
        SerialUSB.flush();
      }
    }
  }   
}


void PrintDigiPot(byte (&curr_pot_vals)[12], byte subvco, byte msgdebuglevel)
{

  if (DebugDelay > 0) {delay(DebugDelay);}

  if (msgdebuglevel <= DebugLevel)
  {
    if (subvco == 0)
    {
      SerialUSB.print("<P0= ");
      SerialUSB.print(curr_pot_vals[0]);
      SerialUSB.print(" P1= ");
      SerialUSB.print(curr_pot_vals[1]);
      SerialUSB.print(" P2= ");
      SerialUSB.print(curr_pot_vals[2]);
      SerialUSB.print(">");
    }
    else if (subvco == 1)
    {
      SerialUSB.print("<P3= ");
      SerialUSB.print(curr_pot_vals[3]);
      SerialUSB.print(" P4= ");
      SerialUSB.print(curr_pot_vals[4]);
      SerialUSB.print(" P5= ");
      SerialUSB.print(curr_pot_vals[5]);
      SerialUSB.print(">");
    }
    else if (subvco == 2)
    {
      SerialUSB.print("<P6= ");
      SerialUSB.print(curr_pot_vals[6]);
      SerialUSB.print(" P7= ");
      SerialUSB.print(curr_pot_vals[7]);
      SerialUSB.print(" P8= ");
      SerialUSB.print(curr_pot_vals[8]);
      SerialUSB.print(">");
    }
    else if (subvco == 3)
    {
      SerialUSB.print("<P9= ");
      SerialUSB.print(curr_pot_vals[9]);
      SerialUSB.print(" P10= ");
      SerialUSB.print(curr_pot_vals[10]);
      SerialUSB.print(" P11= ");
      SerialUSB.print(curr_pot_vals[11]);
      SerialUSB.print(">");
    }


    SerialUSB.flush();
  }
}

void DebugPrint(String token, double value, byte msgdebuglevel)
{
 
 if (DebugDelay > 0) {delay(DebugDelay);}
  
 if (msgdebuglevel <= DebugLevel)
  {
  SerialUSB.print("<");
  SerialUSB.print(token);
  SerialUSB.print("=");
  SerialUSB.print(String(value,5));
  SerialUSB.print(">");
  SerialUSB.flush();
  }

}
void DebugPrintToken(String token, byte msgdebuglevel)
{

 if (DebugDelay > 0) {delay(DebugDelay);}
  
 if (msgdebuglevel <= DebugLevel)
  { 
  SerialUSB.print("<");
  SerialUSB.print(token);
  SerialUSB.print(">");
  SerialUSB.flush();
  }
}

void DebugPrintStr(String token, byte msgdebuglevel)
{
  
  if (DebugDelay > 0) {delay(DebugDelay);}
  
  if (msgdebuglevel <= DebugLevel)
  {
  SerialUSB.print(token);
  SerialUSB.flush();
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

uint16_t d_freq_to_DAC_steps(double delta_freq, double prev_delta_freq, uint8_t subvco, bool adaptive) {

uint16_t i;
int16_t DAC_steps;
double DAC_steps_temp;
double DAC_gain_corrected;

if ((delta_freq != prev_delta_freq) && (adaptive))
{
  DAC_gain_corrected = (prev_delta_freq * DAC_gain[subvco])/(prev_delta_freq - delta_freq);
  DebugPrint("dac_gain_corrected_subvco", double(subvco),4);
  DebugPrint("dac_gain_corrected_val", DAC_gain_corrected,4);
  DAC_steps_temp = delta_freq * -DAC_gain_corrected;
  //DAC_gain[subvco] = DAC_gain_corrected;
  
  // store the DAC_gain_corrected values to check for fluctuations (should have none if tuning is linearly dependent
  // on the DAC, whatever tuning frequency it is)
  
  DAC_gain_adaptive[adaptive_index[subvco]][subvco] = DAC_gain_corrected;
  adaptive_index[subvco]++;

  for (i=0;i<4;i++)
  {
    DebugPrint("\nadaptive_indexes", adaptive_index[i],5);
  }

}
else
{  
  DAC_steps_temp = delta_freq * -(DAC_gain[subvco]);
}

if (DAC_steps_temp > 0) { DAC_steps_temp += 0.5;}
else { DAC_steps_temp -= 0.5;}

DAC_steps = constrain(int(DAC_steps_temp),-4096,4096);
//DebugPrint("double_factor",double(globaltune +1),2);

DebugPrint("delta_DAC_steps",double(DAC_steps),2);
DebugPrintToken("\n",2);

return DAC_steps;

}


void Set_DAC(uint16_t dac_steps, uint8_t subvco) 
{

  if (subvco < 4)
  {
    DAC_states[subvco] = dac_steps;
    MCPdac.analogWrite(subvco,dac_steps);
  }
  else if (subvco == 4) 
  {
    analogWrite(DAC0, dac_steps);
  }
  else if (subvco == 5)
  {
    analogWrite(DAC1, dac_steps);
  }

  /*
  if ((subvco == 0) || (subvco == 1))
  {
    allDACs[subvco]->setValue(DAC_states[subvco]);
  }
  else if (subvco == 2)
  {
    analogWrite(DAC0, dac_steps);
  }
  else if (subvco == 3)
  {
    analogWrite(DAC1, dac_steps);
  }
  */
}

void Adjust_DAC(int16_t dac_steps, uint8_t subvco) 
{

  DebugPrint("dac_steps",double(dac_steps),5);
  DebugPrint("subvco",double(subvco),5);
  
  DAC_states[subvco] += dac_steps;
  DebugPrint("DAC_states[subvco]",double(DAC_states[subvco]),5);  
  DAC_states[subvco] = constrain(DAC_states[subvco],0,4095);
  
  if (subvco < 4) 
  {
    MCPdac.analogWrite(subvco,DAC_states[subvco]);
  }
  else if (subvco == 4)
  {
    analogWrite(DAC0, DAC_states[subvco]);
  }
  else if (subvco == 5)
  {
    analogWrite(DAC1, DAC_states[subvco]);
  }
       
}

void InitADSR(uint8_t adsr_sel, uint16_t attack_millis, uint16_t decay_millis,uint16_t release_millis, uint8_t sustain_level, uint8_t gain)
{
  
  uint16_t ad_millis;
  uint16_t nb_samples = 500;
  uint16_t nb_samples_attack;
  uint16_t nb_samples_decay;
  uint16_t atkindex = 0;
  uint16_t decindex = 0;
  uint16_t relindex = 0;
  
  // attack and decay share the total number of samples to lower memory usage and make triggering easier
  ad_millis = attack_millis + decay_millis;
  
  sample_adsr_inc[2*adsr_sel] = 1.0/(nb_samples*float(ad_millis/1E3));
  sample_adsr_inc[2*adsr_sel +1] = 1.0/(nb_samples*float(release_millis/1E3));

  nb_samples_attack = int(attack_millis / (attack_millis + decay_millis) + 0.5);
  nb_samples_decay = nb_samples - nb_samples_attack;

  inv_sample_adsr_inc_micros[2*adsr_sel] = (nb_samples*float(ad_millis/1E3))/1E3;
  inv_sample_adsr_inc_micros[2*adsr_sel +1] = (nb_samples*float(release_millis/1E3))/1E3;
  
  ADSR_Sustain_Level[2*adsr_sel] = sustain_level*gain;

  // compute linear Attack slope
  for (atkindex=0;atkindex<nb_samples_attack;atkindex++)
  {
    // linear up slope up to max gain level
    AD_table[2*adsr_sel][atkindex] =  int(4095*gain*atkindex/nb_samples_attack +0.5);
  }
  for (decindex=atkindex;decindex<nb_samples;decindex++)
  {
    //linear down slope to the sustain level * gain
    AD_table[2*adsr_sel][decindex] = int(((4095*gain) - (4095*gain*sustain_level))*(4095*gain*(nb_samples_decay - decindex)/nb_samples_decay) + 4095*gain*sustain_level + 0.5);
  }

    for (relindex=0;relindex<nb_samples;relindex++)
  {
    //linear down slope from sustain level * gain to the 0 level
    R_table[2*adsr_sel][relindex] = int((4095*gain*sustain_level)*((nb_samples - relindex)/nb_samples) + 0.5);
  }

  
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
  sample_inc = 2.0/(LFOFreq*1.0*float(nb_samples)); // sample time increment in sec
  // calculate inverse of time interval. multiplication by inverse is way faster.
  inv_sample_inc_micros = LFOFreq*0.5*float(nb_samples)/1E6; // in 1/µsec
  
  // calculate inverse one time only. multiplication by inverse is way faster.

  inv_nb_sample = 1.0/float(nb_samples);
  for (sample = 0; sample < nb_samples; sample++)
  {
        
    temp_sin = sin(2*PI*sample*inv_nb_sample);
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


// TO DO : make calculations of offsets external and pass offset[2]
void Enable_PWM_Mod_by_LFO(uint8_t Action, double &PWMDepth, double DutyCenterVal, double LFOFreq, float notefreq, uint8_t vco, bool order) 
{
  //enables PWM for the oscillator by managing frequency of both timing lines through two DAC
  //We have to check that DutyDepth is not out of bounds of DAC range, taking into account the DACs offsets
  //used for fine frequency tuning
  // Action : 0x1 : backup initial DAC_states ( when called inside handlenoteon usually)
  // Action : 0x2 : restore DAC_states (on subsequent calls for LFO retuning)
  // Action : 0x4 : generate re-offseted DACtables (offset calculated in loop() and stored globally)
  // Action : 0x8 : generate gaincorrected DACtables (gain correction calculated in loop() and stored globally)

  float DACdtmp;
  float dfreq;
  float targetfreq;
  float invDutyCenterVal;
  int16_t DACd;
  int16_t DACf;
  uint8_t i;
  static uint16_t DAC_states_backup[4] = {0,0,0,0};
  //static bool firstcall = true;
  float dVmax[2];
  float PWMDepthMax[2];
  float PWMDepthMaxPossible;
  //float vco_freq[2];
  float DutyCenterValCmp = 1 - DutyCenterVal;
  static float CurLFOFreq = 0;
  uint16_t sample;
  //div_t divresult;

  if (Action & 0x1)
  {
    for (i=0;i<4;i++)
    {
      DAC_states_backup[i] = DAC_states[i];
    }
  }

  if (Action & 0x2)
  {
    for (i=0;i<4;i++)
    {
      DAC_states[i] = DAC_states_backup[i];
    }
  }

  if (Action & 0x4)
  {
      
  //DebugPrint("f_offset[0]",f_offset[0],1);
  //DebugPrint("f_offset[1]",f_offset[1],1);
  
  DebugPrint("DACoffset[0]",DACoffset[noteindexes[vco]][!order],2);
  DebugPrint("DACoffset[1]",DACoffset[noteindexes[vco]][order],2);
  }

  if (LFOFreq != CurLFOFreq) 
  {
    CurLFOFreq = LFOFreq;
    InitSinTableOptimized(LFOFreq,sin_table,360);
    
    
  }

  //vco_freq[0] = notefreq/2*DutyCenterVal;
  //vco_freq[1] = notefreq/(2*(1.0 - DutyCenterVal));
  
  
  //DebugPrint("vco_freq[0]",vco_freq[0],4);
  //DebugPrint("vco_freq[1]",vco_freq[1],4);
  
  // get max voltage deflection from current offset, and get max possible DutyDepth;
  dVmax[0] = 3.0*(2047.0 - fabs(DAC_states[vco*2 + !order]- 2047.0))/4095.0;
  dVmax[1] = 3.0*(2047.0 - fabs(DAC_states[vco*2 + order] - 2047.0))/4095.0;
  DebugPrint("DAC_states[0]",DAC_states[vco*2 + !order],1);
  DebugPrint("DAC_states[1]",DAC_states[vco*2 + order],1);
 
  DebugPrint("dVmax[0]",dVmax[0],3);
  DebugPrint("dVmax[1]",dVmax[1],3);
  
  
  //min_dVmax = min(dVmax[0],dVmax[1]);
  // we calculate the max PWM Depth so as not to clip the DAC and the wavetable, if the requested
  // PWMDdepth is unattainable, we will provide the Max PWMDepth possible.
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
    DAC_table[0][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2]),0,4095);
    // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
    // are always equal to 1/f_note in PWM)
    Vd = 3.0*33000*0.0665E-6*vco_freq[1]*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[(256 + sample) % 512])) -1.0/(2.0*DutyCenterVal));
    DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2 +1]),0,4095);
  

  }
  */




// compute and initialize dac_tables (both subosc) for a full period, using the sintable for [0, Pi/2]
 for (sample = 0; sample < 360; sample++)
  {

      invDutyCenterVal = 1.0/(2.0*DutyCenterVal);
      targetfreq = notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[sample]/32767.0)));
      //dfreq = notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[sample]/32767.0)) -invDutyCenterVal);
      dfreq = targetfreq - notefreq*invDutyCenterVal;

      if (Action & 0x8) {DACdtmp = -DAC_gain_corrected_lfo[noteindexes[vco]][!order]*dfreq;}
      else {DACdtmp = -DAC_gain[vco*2 +!order]*dfreq;}
      
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      
      // if clipping happen nonetheless, we soft clip the DAC values.
      if (Action & 0x4) 
      {
        DACf = DACd + DAC_states[vco*2 + !order] + abs(sin_table[sample]/32767.0)*DACoffset[noteindexes[vco]][!order];
        DAC_table[vco*2 + !order][sample] = constrain(DACf,0,4095);
      }
      else
      {
        DACf = DACd + DAC_states[vco*2 + !order];
        DAC_table[vco*2 + !order][sample] = constrain(DACf,0,4095);
      }
      
      
      //if ((DAC_table[vco*2 + !order][sample] == 0) || (DAC_table[vco*2 + !order][sample] == 4095))
      //{
      //  DebugPrintStr("LFO DAC CLIP!\n",3);
      //}
      
      if (sample == 89)
      {
        LFO_MinMax_Freq_Targets[vco*2 + !order][0] = targetfreq;
        LFO_Delta_Freq_Targets[vco*2 + !order][0] = dfreq;
      
      }
      else if (sample == 269)
      {
        LFO_MinMax_Freq_Targets[vco*2 + !order][1] = targetfreq;
        LFO_Delta_Freq_Targets[vco*2 + !order][1] = dfreq;
      }
      
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      invDutyCenterVal = 1.0/(2.0*DutyCenterValCmp);
      targetfreq = notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[sample]/32767.0)));
      dfreq = targetfreq - notefreq*invDutyCenterVal;

      if (Action & 0x8) {DACdtmp = -DAC_gain_corrected_lfo[noteindexes[vco]][order]*dfreq;}
      else {DACdtmp = -DAC_gain[vco*2 + order]*dfreq;}

      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      if (Action & 0x4) 
      {
        DACf = DACd + DAC_states[vco*2 +order] + abs(sin_table[sample]/32767.0)*DACoffset[noteindexes[vco]][order];
        DAC_table[vco*2 +order][sample] = constrain(DACf,0,4095);
      }
      else
      {
        DACf = DACd + DAC_states[vco*2 +order];
        DAC_table[vco*2 +order][sample] = constrain(DACf,0,4095);
      }
      
      //if ((DAC_table[vco*2 + order][sample] == 0) || (DAC_table[vco*2 + order][sample] == 4095))
      //{
      //  DebugPrintStr("LFO DAC CLIP!\n",3);
      //}
      
    
      if (sample == 89)
      {
        LFO_MinMax_Freq_Targets[vco*2 + order][0] = targetfreq;
        LFO_Delta_Freq_Targets[vco*2 + order][0] = dfreq;
     
      }
      else if (sample == 269)
      {
        LFO_MinMax_Freq_Targets[vco*2 + order][1] = targetfreq;
        LFO_Delta_Freq_Targets[vco*2 + order][1] = dfreq;
     
      }
      
  
    //divresult = div(sample,90);
    //switch (divresult.quot)
    //switch(3)
    /*
    {
    case 0:
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = -DAC_gain[vco*2 +!order]*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));// - DAC_cor[vco*2 + !order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2]),0,4095);
      DAC_table[vco*2 + !order][sample] = constrain(DACd + DAC_states[vco*2 + !order],0,4095);
      
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = -DAC_gain[vco*2 + order]*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));// - DAC_cor[vco*2 + order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2 +1]),0,4095);
      DAC_table[vco*2 +order][sample] = constrain(DACd + DAC_states[vco*2 +order],0,4095);
      
      break;
    
    case 1:
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = -DAC_gain[vco*2 + !order]*notefreq*(1.0/(2.0*(DutyCenterVal + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));// - DAC_cor[vco*2 + !order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2]),0,4095);
      DAC_table[vco*2 + !order][sample] = constrain(DACd + DAC_states[vco*2 + !order],0,4095);
     
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = -DAC_gain[vco*2 + order]*notefreq*(1.0/(2.0*(DutyCenterValCmp - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));// - DAC_cor[vco*2 + order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2 +1]),0,4095);
      DAC_table[vco*2 + order][sample] = constrain(DACd + DAC_states[vco*2 + order],0,4095);
     
      break;

    case 2:

      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = -DAC_gain[vco*2 + !order]*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));// - DAC_cor[vco*2 + !order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2]),0,4095);
      DAC_table[vco*2 + !order][sample] = constrain(DACd + DAC_states[vco*2 +!order],0,4095);
    
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = -DAC_gain[vco*2 + order]*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));// - DAC_cor[vco*2 + order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2 +1]),0,4095);
      DAC_table[vco*2 + order][sample] = constrain(DACd + DAC_states[vco*2 + order],0,4095);
     
      break;

    case 3:

      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));
      DACdtmp = -DAC_gain[vco*2 + !order]*notefreq*(1.0/(2.0*(DutyCenterVal - PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterVal));// - DAC_cor[vco*2 +!order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[0][sample] = constrain(int(corfac*Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2]),0,4095);
      DAC_table[vco*2 + !order][sample] = constrain(DACd + DAC_states[vco*2 +!order],0,4095);
      
      // the next suboscillator has 180° phase shift to the first (The sum of subosc periods
      // are always equal to 1/f_note in PWM)
      //Vd = 3.0*33000*0.0665E-6*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));
      DACdtmp = -DAC_gain[vco*2 + order]*notefreq*(1.0/(2.0*(DutyCenterValCmp + PWMDepth*sin_table[89 - divresult.rem]/32767.0)) -1.0/(2.0*DutyCenterValCmp));// - DAC_cor[vco*2 + order];
      DACd = (DACdtmp > 0)?(int(DACdtmp +0.5)):(int(DACdtmp - 0.5));
      //DAC_table[1][sample] = constrain(int(Vd*4095.0/3.0 + 0.5 + DAC_states[vco*2 +1]),0,4095);
      DAC_table[vco*2 + order][sample] = constrain(DACd + DAC_states[vco*2 + order],0,4095);
   
      break;
    }
    */

  }

  //firstcall = false;
  PWMActive = false;
  //timer2.setup();

}

void Disable_PWM_Mod_by_LFO(uint8_t vco)
{
  PWMActive = false;
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



void R_to_pot_DAC(double R, byte &val_pot100K, uint8_t subvco)
{

//input : R is the resistance to distribute among the two 100K digipots.
//input : subvco is the subvco selector (0 or 1)
//output : &val_pot100K, reference to the serially aggregated steps of the two 100K digipots,
// 0 to 99 = pot1/4, 100 to 199 = pot2/5. with pot1/4 being maxed out.

  double Rtmp;
  // subvco0 (TR1 pin)
  //const double R1OOK_total_R_vco0 = 102.1;
  const double R1OOK_total_R_subvco0 = 103.2;
  const double R1O0Kb_total_R_subvco0 = 101.2;
  const double R1K_total_R_subvco0 = 1.055;
  const double trim_pot_R_subvco0 = 6.30; // with added wiper_R*3

  // subvco1 (TR2 pin)
  //const double R1OOK_total_R_subvco1 = 105.3;
  const double R1OOK_total_R_subvco1 = 104.3;
  const double R1O0Kb_total_R_subvco1 = 101.1;
  const double R1K_total_R_subvco1 = 1.033;
  const double trim_pot_R_subvco1 = 1.11; // with added wiper_R*3


  // subvco2 (TR1 pin)
  const double R1OOK_total_R_subvco2 = 96.2;
  const double R1O0Kb_total_R_subvco2 = 95.5;
  const double R1K_total_R_subvco2 = 1.055;
  const double trim_pot_R_subvco2 = 0.17; // with added wiper_R*3


  // subvco3 (TR2 pin)
  const double R1OOK_total_R_subvco3 = 92.3;
  const double R1O0Kb_total_R_subvco3 = 98.3;
  const double R1K_total_R_subvco3 = 1.033;
  const double trim_pot_R_subvco3 = 4.79; // with added wiper_R*3


  //const double wiper_R = 0.12;
  if(R != R) {val_pot100K = 199;return;} // check for NaN, if Nan, the two 100K digipots are maxed out. 
  //(unobtainable low frequency through R alone)
  if (subvco == 0) 
  {

    if(R > (R1O0Kb_total_R_subvco0 + R1OOK_total_R_subvco0 + R1K_total_R_subvco0 + trim_pot_R_subvco0))
      {val_pot100K = 199;
      DebugPrintToken("ABOVE POSSIBLE R vco0",5);
      return;
      } // R value is above max R obtainable with this setup, max out pots.

    // R is in kOhms
    // we substract wiper resistance and trim pot resistance.
    if (R > R1O0Kb_total_R_subvco0) // in this case we use all the steps of pot1 (+99)
    {
      Rtmp = R - R1O0Kb_total_R_subvco0 - R1K_total_R_subvco0 - trim_pot_R_subvco0;
      val_pot100K = (byte) int(Rtmp/(R1OOK_total_R_subvco0/99) + 0.5) + 99;
      //(byte) int(Rtmp/(R1OOKb_total_R_subvco0/99) + 0.5) + 99;
    
    }
    else // only pot2 is required, pot1 stays at step 0.
    { 
      Rtmp = R - R1K_total_R_subvco0 - trim_pot_R_subvco0;
      val_pot100K = (byte) int(Rtmp/(R1O0Kb_total_R_subvco0/99) + 0.5);
    }
    
  }
  
  else if (subvco == 1) 
  {


    if(R > (R1OOK_total_R_subvco1 + R1O0Kb_total_R_subvco1 + R1K_total_R_subvco1 + trim_pot_R_subvco1))
      {val_pot100K = 199;
      DebugPrintToken("ABOVE POSSIBLE R vco1",5);
      return;} // R value is above max R obtainable with this setup, max out pots.

    if(R > R1O0Kb_total_R_subvco1) // in this case we use all the steps of pot4
    {
      Rtmp = R - R1O0Kb_total_R_subvco1 - R1K_total_R_subvco1 - trim_pot_R_subvco1;
      val_pot100K = (byte) int(Rtmp/(R1OOK_total_R_subvco1/99) + 0.5) + 99;
      //(byte) int(Rtmp/(R1OOKb_total_R_subvco0/99) + 0.5) + 99;
  
    }
    else // only pot5 is required, pot4 stays at step 0.
    {
      Rtmp = R - R1K_total_R_subvco1 - trim_pot_R_subvco1;
      val_pot100K = (byte) int(Rtmp/(R1O0Kb_total_R_subvco1/99) + 0.5);
    }

  }
  else if (subvco == 2) 
  {


    if(R > (R1OOK_total_R_subvco2 + R1O0Kb_total_R_subvco2 + R1K_total_R_subvco2 + trim_pot_R_subvco2))
      {val_pot100K = 199;
      DebugPrintToken("ABOVE POSSIBLE R vco1",5);
      return;} // R value is above max R obtainable with this setup, max out pots.

    if(R > R1O0Kb_total_R_subvco2) // in this case we use all the steps of pot4
    {
      Rtmp = R - R1O0Kb_total_R_subvco2 - R1K_total_R_subvco2 - trim_pot_R_subvco2;
      val_pot100K = (byte) int(Rtmp/(R1OOK_total_R_subvco2/99) + 0.5) + 99;
      //(byte) int(Rtmp/(R1OOKb_total_R_subvco0/99) + 0.5) + 99;
  
    }
    else // only pot5 is required, pot4 stays at step 0.
    {
      Rtmp = R - R1K_total_R_subvco2 - trim_pot_R_subvco2;
      val_pot100K = (byte) int(Rtmp/(R1O0Kb_total_R_subvco2/99) + 0.5);
    }
    
  }

  else if (subvco == 3) 
  {


    if(R > (R1OOK_total_R_subvco3 + R1O0Kb_total_R_subvco3 + R1K_total_R_subvco3 + trim_pot_R_subvco3))
      {val_pot100K = 199;
      DebugPrintToken("ABOVE POSSIBLE R vco1",5);
      return;} // R value is above max R obtainable with this setup, max out pots.

    if(R > R1O0Kb_total_R_subvco3) // in this case we use all the steps of pot4
    {
      Rtmp = R - R1O0Kb_total_R_subvco3 - R1K_total_R_subvco3 - trim_pot_R_subvco3;
      val_pot100K = (byte) int(Rtmp/(R1OOK_total_R_subvco3/99) + 0.5) + 99;
      //(byte) int(Rtmp/(R1OOKb_total_R_subvco0/99) + 0.5) + 99;
  
    }
    else // only pot5 is required, pot4 stays at step 0.
    {
      Rtmp = R - R1K_total_R_subvco3 - trim_pot_R_subvco3;
      val_pot100K = (byte) int(Rtmp/(R1O0Kb_total_R_subvco3/99) + 0.5);
    }
    
  }


}
//Splits R to the settings of the three serial pots (100K, 10K, 1K)
/*
void R_to_pot(double R, byte &val_pot100K, byte &val_pot10K, byte &val_pot1K, byte subvco) {

  double Rtmp;
  const double R1OOK_total_R_vco0 = 96.77;
  const double R1OK_total_R_vco0 = 10.08;
  const double R1K_total_R_vco0 = 1.055;
  const double trim_pot_R_vco0 = 1.765;

  const double R1OOK_total_R_subvco1 = 97.26;
  const double R1OK_total_R_subvco1 = 10.42;
  const double R1K_total_R_subvco1 = 1.033;
  const double trim_pot_R_subvco1 = 1.349;


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
  
    Rtmp = R - trim_pot_R_subvco1;

    if(Rtmp > R1OOK_total_R_subvco1) 
    {

      val_pot100K = 99;
      Rtmp = Rtmp - R1OOK_total_R_subvco1;
      val_pot10K = (byte) constrain(int(Rtmp/(R1OK_total_R_subvco1/99)),0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_subvco1/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_subvco1/99) + 0.5);
    }
    else 
    {
      val_pot100K = (byte) int(int(Rtmp/(R1OOK_total_R_subvco1/99))/10)*10;
      Rtmp = Rtmp - val_pot100K*(R1OOK_total_R_subvco1/99);
      val_pot10K = (byte) constrain(int(Rtmp/(R1OK_total_R_subvco1/99)),0,99);
      Rtmp = Rtmp - val_pot10K*(R1OK_total_R_subvco1/99);
      val_pot1K = (byte) int(Rtmp/(R1K_total_R_subvco1/99) + 0.5);
      
    }

  }

}
*/

//Change OSC frequency for TRI/SAW or SINE. Supply pot vals in this order for osc1 and osc2 osc1:100K,10K,1K,osc2:100K,10K,1K.
//Digitpot *ptr[] : array of pointers of digipots objects in the same order as pot vals.
//boolean val_saw : 0: generate sine. 1: generate saw/tri
//byte vco selects the VCO if val_saw is true (0 or 1), else it selects the subvco ( 0 to 3 ) 
void ChangeNote(byte pot_vals[12], byte (&curr_pot_vals)[12], DigiPot *ptr[12], boolean val_saw, byte vco) {

  uint8_t m = 0;
  //uint8_t i = 0;
  uint8_t max_pot;
  int16_t pot_val_change;
  //static byte curr_pot_vals[6] = {99,99,99,99,99,99};
  DebugPrintStr("BEFORE CHNG\n",5);
  PrintDigiPot(curr_pot_vals,vco*2,5);
  PrintDigiPot(curr_pot_vals,vco*2 +1,5);
  
  if (!val_saw) 
  {
    max_pot = 3;  
  }
  else 
  {
    max_pot = 6;
  }
  
  for(m = vco*max_pot; m < max_pot + vco*max_pot; m++) 
  {

    if (pot_vals[m] > 99) { pot_vals[m] = 99; }
    
    pot_val_change = pot_vals[m] - curr_pot_vals[m];
    DebugPrint("POT_M\n",double(m),5);
    DebugPrint("POT_VAL_CHANGE\n",double(pot_val_change),5);
  
    //delay(5000);
    if (pot_val_change > 0) 
    {
      /*
      for(i=0;i<pot_val_change;i++)
      {
        ptr[m]->increase(1);
        delay(2);
      }
      */
       //delay(2);
       ptr[m]->increase(pot_val_change);
       

    }
    else if (pot_val_change < 0) 
    {

     /*
     for(i=0;i<abs(pot_val_change);i++)
      {
        ptr[m]->decrease(1);
        delay(2);
      }
      */


      ptr[m]->decrease(abs(pot_val_change));
      //delay(2);
    }

    curr_pot_vals[m] = pot_vals[m];
    DebugPrintStr("POT_CHANGED\n",5);

  }


  DebugPrintStr("AFTER CHNG\n",5);
  PrintDigiPot(curr_pot_vals,vco*2,5);
  PrintDigiPot(curr_pot_vals,vco*2 +1,5);
  


}

double ReadCoarseFreq(uint8_t step, uint8_t subvco)

{
  uint16_t prev_data_offset = 0;
  uint16_t addr;
  uint16_t tuneblock_size = 200*(sizeof(double));
  double coarsefreq;
  
  addr = prev_data_offset + tuneblock_size*subvco + step*(sizeof(double));
  coarsefreq = flash.readFloat(addr); 
  DebugPrint("COARSE FREQ=", coarsefreq, 5);
  DebugPrintToken("\n",5);
  return coarsefreq;

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

  DebugPrint("duty\n",duty,2);
  DebugPrint("R1\n",Rvco1,2);
  DebugPrint("R2\n",Rvco2,2);
  DebugPrint("f1\n",f1,2);
  DebugPrint("f2\n",f2,2);



  //Check for R value special cases
  
  // Set the fours DACs to center value (1.5V)
  
  if (vco == 0)
  {
    Set_DAC(2047,0);
    Set_DAC(2047,1);
  }
  else if (vco == 1)
  {
    Set_DAC(2047,2);
    Set_DAC(2047,3);
  }


  // estimating coarse pot R step
  if (isinf(Rvco1)) 
  { 
    allR100steps_1 = 199;
  }
  else
  {
    R_to_pot_DAC(Rvco1, allR100steps_1, 2*vco);
  }
  
  if (isinf(Rvco2)) 
  { 
    allR100steps_2 = 199;
  }
  else
  {
    R_to_pot_DAC(Rvco2, allR100steps_2, 2*vco + 1);
  }
  
  DebugPrint("allR100steps_0", double(allR100steps_1), 5);
  DebugPrint("\nallR100steps_1",double(allR100steps_2), 5);
  DebugPrintToken("\n",2);

  idx0 = 199 - allR100steps_1;
  idx1 = 199 - allR100steps_2;

  DebugPrint("idx0_est",double(idx0),5);
  DebugPrint("\nidx1_est",double(idx1),5);
  DebugPrintToken("\n",5);


  //coarse_freq table lookup index guess...
  //tmp_f1a = coarse_freq[2*vco][idx0];
  //tmp_f2a = coarse_freq[2*vco + 1][idx1];

  // same but using flash memory chip.
  tmp_f1a = ReadCoarseFreq(idx0,2*vco);
  tmp_f2a = ReadCoarseFreq(idx1,2*vco +1);
  


  DebugPrint("\nf0_est",tmp_f1a,5);
  DebugPrint("\nf1_est",tmp_f2a,5);
  DebugPrintToken("\n",5);

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
  
  DebugPrint("idx0_real",double(idx0),4);
  DebugPrint("\nidx1_real",double(idx1),4);

  DebugPrint("\ntune freq f1\n",f1,4);
  DebugPrint("\ntune freq f2\n",f2,4);

  DebugPrint("\nSelected CoarseFreq f1:",ReadCoarseFreq(idx0,2*vco),4);
  DebugPrint("\nSelected CoarseFreq f2:",ReadCoarseFreq(idx1,2*vco + 1),4);
  
  DebugPrintToken("\n",5);

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

  //first subvco
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

  //second subvco
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
  
  DebugPrintToken("NEW:",5); 
  PrintDigiPot(midi_to_pot_G,2*vco,5);
  PrintDigiPot(midi_to_pot_G,2*vco + 1,5);
  DebugPrintToken("\nCUR:",5); 
  PrintDigiPot(curr_pot_vals,2*vco,5);
  PrintDigiPot(curr_pot_vals,2*vco + 1,5);
  DebugPrintToken("\n",5);



  /*
   for(k=6;k<12;k++){
   midi_to_pot[k] = 99;
   }
  */  

   //Change OSC freq with saw
  //Set_DAC(2047,0);
  //Set_DAC(2047,1);
  //Set_DAC(2047,2);
  //Set_DAC(2047,3);
  
  /*
  MCPDAC0.setValue(2047);
  DAC_states[0] = 2047;
  MCPDAC1.setValue(2047);
  DAC_states[1] = 2047;

  MCPDAC2.setValue(2047);
  DAC_states[2] = 2047;
  MCPDAC3.setValue(2047);
  DAC_states[3] = 2047;
  */

  ChangeNote(midi_to_pot_G, curr_pot_vals, pots, true, vco);

  DebugPrintToken("\nNEW_CUR:",5); 
  PrintDigiPot(curr_pot_vals,2*vco,5);
  PrintDigiPot(curr_pot_vals,2*vco + 1,5);
  DebugPrintToken("\n",5);

}

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
    digitalWrite(vco0_pin, HIGH);
   }
   else if (subvco == 1)
   {
    digitalWrite(vco0_pin, LOW); 
   }

   else if (subvco == 2) 
   {
    digitalWrite(vco1_pin, HIGH);
   }
   else if (subvco == 3)
   {
    digitalWrite(vco1_pin, LOW); 
   }

    DebugPrint("\nMIN_ALL_POTS_VCO",double(subvco),6);
    DebugPrintToken("\n",6);


      for(m = max_pot*subvco; m < max_pot + max_pot*subvco; m++) {
        
        for(k=0;k<100;k++)
        {
      
          ptr[m]->decrease(1);
          delay(5);
      
        }
    
      
      }

      DebugPrint("MIN_ALL_POTS_VCO_DONE",double(subvco),6);
      DebugPrintToken("\n",6);


      delay(20000);


      for(m = max_pot*subvco; m < max_pot + max_pot*subvco; m++) 
      {
        
        DebugPrint("MAX_POT_VCO",double(subvco),6);
        DebugPrint("\nMAX_POT_VCO_M",double(m),6);
        DebugPrintToken("\n",6);



        for(k=0;k<100;k++)
        {
      
          ptr[m]->increase(1);
          delay(5);
      
        }
         
        delay(20000);
        DebugPrint("\nMAX_ALL_POTS_VCO_DONE",double(subvco),6);
        DebugPrintToken("\n",6);

      

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

void SetupFreqMeasure()
{
  PMC->PMC_PCER0 |= PMC_PCER0_PID28;                       // Timer Counter 0 channel 1 IS TC1

  TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1   // capture mode, MCK/2, clk on rising edge
                              | TC_CMR_ABETRG              // TIOA is used as the external trigger
                              | TC_CMR_LDRA_RISING         // load RA on rising edge of trigger input
                              | TC_CMR_LDRB_FALLING;       // load RB on falling edge of trigger input

  TC0->TC_CHANNEL[1].TC_IER |= TC_IER_LDRAS | TC_IER_LDRBS; // Trigger interruption on Load RA and load RB
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Software trigger and enable

  NVIC_DisableIRQ(TC1_IRQn);
  NVIC_ClearPendingIRQ(TC1_IRQn);
  NVIC_SetPriority(TC1_IRQn, 0);                      // Give TC1 interrupt the highest urgency
  NVIC_SetPriority(SysTick_IRQn, 15);                 // SysTick interrupt will not interrupt TC1 interrupt
  NVIC_SetPriority(UART_IRQn, 14);                    // UART interrupt will not interrupt TC1 interrupt

  NVIC_EnableIRQ(TC1_IRQn);                     
}


void TC1_Handler() {

  static uint32_t _CaptureCountA;

  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;       // Read and Clear status register


  //if (status & TC_SR_LOVRS) abort();  // We are loosing some edges

  if (status & TC_SR_LDRAS) {  // If ISR is triggered by LDRAS then ....
    CaptureCountA = (uint32_t) TC0->TC_CHANNEL[1].TC_RA;        // get data from capture register A for TC0 channel 1
    Period = CaptureCountA - _CaptureCountA;
    _CaptureCountA = CaptureCountA;
  }
  else { /*if ((status & TC_SR_LDRBS) == TC_SR_LDRBS)*/  // If ISR is triggered by LDRBS then ....
    CaptureCountB = (uint32_t) TC0->TC_CHANNEL[1].TC_RB;         // get data from caputre register B for TC0 channel 1
    Duty = CaptureCountB - _CaptureCountA;
    CaptureFlag = true;                      // set flag indicating a new capture value is present
  }

}

void CountFrequencyDelta2(byte samplesnumber,float tunefrequency, double fhigh, double flow, double &f_meas, double &fhigh_meas, double &flow_meas, double &f_err, uint8_t vco) 
{

 
  //uint8_t count = 0;
  DebugPrintStr("\nCountFrequencyDelta2", 6);
  pinMode(swvco_meas_pin,OUTPUT);
  pinMode(A7,INPUT);
  SetupFreqMeasure();

  if (vco == 0) 
  {
    digitalWrite(swvco_meas_pin,HIGH);
  }
  else if (vco == 1)
  {
    digitalWrite(swvco_meas_pin,LOW);
  }
  //pinMode(freq_meas_pin, INPUT);
  //digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

  double f_total_measured = 0.0;
  double t_temp_total = 0.0;
  double t_temp_high = 0.0;
  double t_temp_low = 0.0;
  
  //double f_err_calc = 0.0;
  
  //float integrator_temp = 0.0;
  //unsigned long sumlow = 0;   
  //unsigned long sumhigh = 0;   
  

  //byte counthigh = 0;
  //byte countlow = 0;
  //byte count = 0;
  uint8_t totalpulses = 0;
  uint32_t sumpulsestotal = 0;
  uint32_t sumpulseshigh = 0;
  
  uint16_t millistimeout = 1500;
  uint16_t millispassed = 0;
  uint16_t millisstart = 0;
  
  //unsigned long pulsehigh = 0;
  //unsigned long pulselow = 0;

  //timer2.setup();
  millisstart = millis();
  while (totalpulses < samplesnumber) 
  {
    delay(2);
    if (millispassed > millistimeout) {break;}

    if ( CaptureFlag == true) 
    {
      millisstart = millis();
      //SerialUSB.println("capture"); 
      //Frequency = _F_CPU / Period  ; //  (Mck/2 is TC1 clock) F in Hz
      //_Duty = (Duty * 100.00) / Period;
      CaptureFlag = false;
      if (totalpulses > 1) 
      {
        sumpulsestotal += Period;
        sumpulseshigh += Duty;
        //SerialUSB.println(sumpulseshigh);
        DebugPrint("\nPeriod:", Period, 6);
        //SerialUSB.println(totalpulses); 
      }
      totalpulses++;
    }
    millispassed = millis() - millisstart;
  }
 
  //timer2.unsetup();
  
  if (sumpulseshigh >0)
  {


    t_temp_total = double(sumpulsestotal)/double(totalpulses -2);
    t_temp_high  = double(sumpulseshigh)/double(totalpulses -2);
    t_temp_low = t_temp_total - t_temp_high;
    f_total_measured = double((F_CPU/t_temp_total)/2);
    f_meas = f_total_measured; //+ f_err_calc; 
    fhigh_meas =  double((F_CPU/t_temp_high)/4);
    flow_meas =  double((F_CPU/t_temp_low)/4);
    
  }

    DebugPrint("\nf_total_meas",f_meas,5);
    //SerialUSB.println("");
    DebugPrint("\nfhigh_meas",fhigh_meas,5);
    //SerialUSB.println("");
    DebugPrint("\nflow_meas",flow_meas,5);
    //SerialUSB.println("");
    DebugPrintToken("\n",5);

    //DebugPrint("totalpulses",double(totalpulses),5);
  
} // end CountFrequencyDelta2

void CountFrequency(uint8_t samplesnumber, double &f_meas, uint8_t subvco)
{
  
  //uint8_t count = 0;
  pinMode(swvco_meas_pin,OUTPUT);
  pinMode(A7,INPUT);
  SetupFreqMeasure();

  if ((subvco == 0) || (subvco == 1))
  {
    digitalWrite(swvco_meas_pin,HIGH);
  }
  else if ((subvco == 2) || (subvco == 3))
  {
    digitalWrite(swvco_meas_pin,LOW);
  }
  //pinMode(freq_meas_pin, INPUT);
  //digitalWrite(34, HIGH);
  //FreqCount.begin(gatetime);

  double f_total_measured = 0.0;
  double f_temp = 0.0;
  //double f_err_calc = 0.0;
  
  //float integrator_temp = 0.0;
  //unsigned long sumlow = 0;   
  //unsigned long sumhigh = 0;   
  

  //byte counthigh = 0;
  //byte countlow = 0;
  //byte count = 0;
  uint8_t totalpulses = 0;
  uint32_t sumpulses = 0;
  uint16_t millistimeout = 1500;
  uint16_t millispassed = 0;
  uint16_t millisstart = 0;
  
  //unsigned long pulsehigh = 0;
  //unsigned long pulselow = 0;

  //timer2.setup();
  millisstart = millis();
  while (totalpulses < samplesnumber) 
  {
    delay(2);
    if (millispassed > millistimeout) {break;}
   /*
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
    */

    if ( CaptureFlag == true) 
    {
      //Frequency = _F_CPU / Period  ; //  (Mck/2 is TC1 clock) F in Hz
      //_Duty = (Duty * 100.00) / Period;
      CaptureFlag = false;
      if (totalpulses > 1) 
      {
        sumpulses += Period;
        //SerialUSB.println(Period);
        //SerialUSB.println(sumpulses);
        //SerialUSB.println(totalpulses); 
      }
      totalpulses++;
    }
    millispassed = millis() - millisstart;
  }
 
  //timer2.unsetup();
  
  if (sumpulses >0)
  {
    f_temp = double(sumpulses)/double(totalpulses -2);
    f_total_measured = double((F_CPU/f_temp)/2);
    //DebugPrint("ftm",f_total_measured,0);
    //f_err_calc = tunefrequency*1.55005013e-03 -1.45261187e-01;
    //f_err_calc = f_total_measured*1.36503806e-03 -7.58951531e-02;
    f_meas = f_total_measured; //+ f_err_calc;  
  }

    DebugPrint("\nf_total_meas",f_meas,5);
    DebugPrintToken("\n",5);

    //SerialUSB.println("");
    //DebugPrint("totalpulses",double(totalpulses),5);
  
}

void CountFrequencyDeltaGlobal(byte samplesnumber,float tunefrequency, double &f_err, uint8_t vco) 
{
    double f_meas = 0;
    CountFrequency(samplesnumber,f_meas,int(vco/2)*2); 
    f_err = f_meas - tunefrequency;
    DebugPrint("\nf_total_meas",f_meas,4);
    DebugPrint("\nf_total_gen",tunefrequency,4);
    DebugPrintToken("\n",4);


}

/*
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
    
    Serial1.print("<ph=");
    Serial1.print(pulsehigh);
    Serial1.print(">");
    Serial1.print("<pl=");
    Serial1.print(pulselow);
    Serial1.print(">");
    Serial1.flush();
    
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
    
    Serial1.print("<countlow=");
    Serial1.print(countlow);
    Serial1.print(">");    
    Serial1.print("<counthigh=");
    Serial1.print(counthigh);
    Serial1.print(">");
    
    
    DebugPrint("f_glob_tot_meas",f_meas,5);

    
    
    Serial1.print("<f1m=");
    Serial1.print(String(f1_measured,5));
    Serial1.print(">");
    Serial1.print("<f2m=");
    Serial1.print(String(f2_measured,5));
    Serial1.print(">");
    Serial1.flush();  
    
}
*/

void SingleCountFrequencyDelta(byte samplesnumber,double f_global, double f_component, double &f_global_err, double &f_component_err, bool low_or_high, bool globaltune, uint8_t vco) 
{

  double f1 = 0;
  double f2 = 0;

  double f_meas = 0;
  double f1_meas = 0;
  double f2_meas = 0;
  double f_err = 0;
  
  double f_total_measured = 0.0;
  double f_total_measured_compensated = 0.0;
  //double f1_measured = 0.0;
  //double f2_measured = 0.0;
  double f_err_calc = 0.0;
  
  //float integrator_temp = 0.0;
  
  double f_measured = 0.0;
   //MIDI.sendNoteOn(80, 127, 1);

   //MIDI.sendNoteOn(81, 127, 1);
    //timer2.setup();
    //MIDI.sendNoteOn(82, 127, 1);
  CountFrequencyDelta2(samplesnumber,f_global,f1, f2, f_meas, f1_meas, f2_meas, f_err, vco);

  if (low_or_high)
  {
    f_measured = f1_meas;        
  }
  else
  {
    f_measured = f2_meas;
  }
      
         

    
  if (globaltune)
  {
    f_component_err = f_measured - f_component;
    f_total_measured = f_meas;
    f_err_calc = 0.0;
    f_total_measured_compensated = f_total_measured + f_err_calc;
    f_global_err = f_total_measured_compensated - f_global;

    DebugPrint("\nf_total_meas",f_total_measured_compensated,4);
    DebugPrint("\nf_total_gen",f_global,4);
    DebugPrintToken("\n",4);


  } // end if (globaltune)
  
  else
  {
    
      f_component_err = f_measured - f_component;
      f_global_err = 0.0;

      DebugPrint("\nf_sng_cmp_meas",f_measured,4);
      DebugPrint("\nf_sng_cmp_trg",f_component,4);
      DebugPrintToken("\n",5);

  } // end else (!globaltune)

}


void Generate_dV_to_dHz_Table(byte (&curr_pot_vals)[12],double center_freq, uint8_t subvco)
{
  double f1;
  double f2;
  uint32_t baseaddr = 16384;
  uint32_t subvcoblock = 4096;
  uint32_t addr;
  uint16_t i;
  double f_meas;
  double base_f_meas;
  double duty;



  f1 = 0.0;
  f2 = 0.0;
  f_meas = 0.0;
  duty = 0.5;

  GenerateArbitraryFreqDAC(curr_pot_vals, center_freq, duty, f1, f2, int(subvco/2));

  DebugPrint("\ndV_dHz DAC:",double(subvco),1);
  DebugPrintToken("\n",1);

  //delay(10000);

  if (subvco < 2)
  {
    digitalWrite(vco0_pin, !(subvco % 2));
  }
  else if (subvco >= 2)
  {
    digitalWrite(vco1_pin, !(subvco % 2));
  }

  Set_DAC(0,subvco);
  delay(10);
  CountFrequency(100,f_meas, subvco);
  base_f_meas = f_meas;
  char charfmeas[8];

  DebugPrint("\nbase_f_meas:", base_f_meas, 1);
  DebugPrintToken("\n",1);


  for (i=0;i<4095;i += 8)
  {
    addr = baseaddr + subvco*subvcoblock + sizeof(f_meas)*(i/8);
    Set_DAC(i,subvco);
    delay(10);
    CountFrequency(100,f_meas, subvco);
    f_meas -= base_f_meas;
    dtostrf(f_meas, 7, 2, charfmeas);
    //flash.writeFloat(addr,f_meas);
    DebugPrintStr("[",1);
    DebugPrintStr(String(i),1);
    DebugPrintStr(",",1);
    SerialUSB.print(charfmeas);
    SerialUSB.flush();
    DebugPrintStr("],",1);
    DebugPrintStr("\n",1);
    
  }


}


void NewAutoTuneDAC2(DigiPot *ptr[12], byte (&curr_pot_vals)[12], byte noteindex, double tunefrequency, double duty ,double fhigh, double flow, byte vco, bool order) 
{ 

  //Two components parallel tuning with weighted subosc error compensation
  // Tune both osc at the same time based on fhigh_meas_err and flow_meas_err
  // tune both osc at the same time based on f_total_err, using duty cycle as weighting for the period error.
  // adaptive : use adaptive correction of DAC frequency to step gain between iterations.
  bool adaptive = false;
  uint16_t stabilization_delay = 5; //(ms)
  byte max_pot = 6;
  double f_meas = 0.0;
  double fhigh_meas = 0.0;
  double flow_meas = 0.0;
  double fhigh_err = 0.0;
  double flow_err = 0.0;
  double f_err = 0.0;
  
  uint8_t max_tunesteps = 10;
  float f_err_history[2][max_tunesteps];
  float dev_cents_history[2][max_tunesteps];
  uint8_t tuned_at_step[2] = {0,0};
  double dutycor = duty;
  double dev_cents[2] = {0.0,0.0};
  const double tune_thresh = 2.5;
  uint16_t DAC_steps[2] = {0,0};
  
 
  uint8_t p;
  uint8_t m;

  bool tuned[2] = {false,false};
  //double dintegrator = 0.0;
  //double dintegrator_p = 0.0;
  
  
  //float integrator = 0.0;
  //float integrator_1 = 0.0;
  //float min_integrator = 0.0;

  //int integrator_count = 0;

  //int best_index = 0;
  //byte curr_pot_val_bck = 0;


  if (duty >= 0.5) { dutycor = duty;}
  else if (duty < 0.5) { dutycor = 1 - duty;}
  //min_integrator = fabs(f_err_history[0]);
        
  for (p=0;p<max_tunesteps;p++) 
  {

   
      // measure both frequencies errors
      // f_err is : f_measured - f_target;

      delay(stabilization_delay);
      CountFrequencyDelta2(10,tunefrequency,fhigh,flow,f_meas,fhigh_meas,flow_meas,f_err,vco);
      fhigh_err = fhigh_meas - fhigh;
      flow_err = flow_meas - flow;
       
      DebugPrint("fhigh_err",fhigh_err,2);
      DebugPrintToken("\n",2);
      DebugPrint("flow_err",flow_err,2);
      DebugPrintToken("\n",2);
      DebugPrint("fhigh",fhigh,2);
      DebugPrintToken("\n",2);
      DebugPrint("flow",flow,2);
      DebugPrintToken("\n",2);

      
      DebugPrint("vco",double(vco),5);
      DebugPrintToken("\n",5);
      
      //integrator = (float) dintegrator;
      if (!tuned[0])
      {
        dev_cents[0] = 1200 * log(fhigh_meas/fhigh)/log(2);
        f_err_history[0][p] = fhigh_err;
        dev_cents_history[0][p] = dev_cents[0];
      }
      else
      {
        f_err_history[0][p] = 0.0;
        dev_cents_history[0][p] = 0.0;
      }
      
       if (!tuned[1])
      {
        dev_cents[1] = 1200 * log((flow_meas)/flow)/log(2);
        f_err_history[1][p] = flow_err;
        dev_cents_history[1][p] = dev_cents[1];
      }
      else
      {
        f_err_history[1][p] = 0.0;
        dev_cents_history[1][p] = 0.0;
      }
      
      if (p > 0)
      {
        if (!tuned[0]) {DAC_steps[0] = d_freq_to_DAC_steps(f_err_history[0][p],f_err_history[0][p-1],2*vco + !order, adaptive);}
        if (!tuned[1]) {DAC_steps[1] = d_freq_to_DAC_steps(f_err_history[1][p],f_err_history[1][p-1],2*vco + order, adaptive);}
      }
      else
      {
        if (!tuned[0]) {DAC_steps[0] = d_freq_to_DAC_steps(f_err_history[0][p],f_err_history[0][p],2*vco + !order, adaptive);}
        if (!tuned[1]) {DAC_steps[1] = d_freq_to_DAC_steps(f_err_history[1][p],f_err_history[1][p],2*vco + order, adaptive);}
      }
      
      if (!tuned[0]) {Adjust_DAC(DAC_steps[0],2*vco + !order);}
      if (!tuned[1]) {Adjust_DAC(DAC_steps[1],2*vco + order);}
      
      //vco formula global tune
   
    if ((fabs(dev_cents[0]) <= tune_thresh) && (!tuned[0])) 
    { 
      tuned[0] = true;
      tuned_at_step[0] = p;
      DebugPrint("THR_ATT",double(fabs(dev_cents[0])),2);
      DebugPrintToken("\n",2);
    } //end tune thresh att

    if ((fabs(dev_cents[1]) <= tune_thresh) && (!tuned[1])) 
    { 
      tuned[1] = true;
      tuned_at_step[1] = p;
      DebugPrint("THR_ATT",double(fabs(dev_cents[1])),2);
      DebugPrintToken("\n",2);
    } //end tune thresh att

    if (tuned[0] && tuned[1]) { break;}

  } // end for p < max_tunesteps

  // now proportional error tuning.
  DebugPrintToken("CENTS DEV TUNE HIST:\n",1);

  for (m=0;m<2;m++)
  {
    DebugPrint("VCO:",double(vco*2 + m),1);
    DebugPrintToken("\n",1);
    for (p=0;p<=tuned_at_step[m];p++) 
    {
      DebugPrint( "\n", dev_cents_history[m][p],1);
      
    }
  
    tuned_at_step[m] = 0;

  }
 
  for(m = vco*max_pot; m < vco*max_pot + max_pot; m ++)
  {
    PWM_Note_Settings[noteindex][m] = curr_pot_vals[m];
  }

  PWM_DAC_Settings[noteindex][2*vco] = DAC_states[2*vco];
  PWM_DAC_Settings[noteindex][2*vco +1] = DAC_states[2*vco +1];
  CountFrequencyDelta2(10,tunefrequency,fhigh,flow,f_meas,fhigh_meas,flow_meas,f_err,vco);
  PWM_Real_Duty[noteindex][vco] = (1/(2*fhigh_meas))/(1/f_meas);
  

} // end NewAutotuneDAC2


struct ctx
{
  ctx() {onoff=false; DACId=5; counter=0;}
  bool onoff;
  uint8_t DACId;
  uint32_t counter;
};

struct ctx_ad
{
  ctx_ad() {DACId=4; counter=0; is_attackdecay = true;}
  uint8_t DACId;
  bool is_attackdecay;
  uint16_t counter;
};

struct ctx_r
{
  ctx_r() {DACId=4; counter=0; is_attackdecay = false;}
  uint8_t DACId;
  bool is_attackdecay;
  uint16_t counter;
};


ctx action_ctx;
ctx_ad action_ctx_ad;
ctx_r action_ctx_r;

// flip the gate DAC up 4095 and down 0
void UpdateGateDac(void* a_ctx)
{

  ctx* the_ctx=reinterpret_cast<ctx*>(a_ctx);

  Set_DAC(((the_ctx->onoff)? 4095: 0),the_ctx->DACId);
  the_ctx->onoff=!(the_ctx->onoff);
  the_ctx->counter++;

}

void UpdateADDac(void* a_ctx)
{

  ctx_ad* the_ctx=reinterpret_cast<ctx_ad*>(a_ctx);
  if (the_ctx->counter > 499) 
  {
     the_ctx->counter = 0;
     action_tc3.stop();
     return;
  }

  Set_DAC(AD_table[0][the_ctx->counter],the_ctx->DACId);
  the_ctx->counter++;

}

void UpdateRDac(void* a_ctx)
{

  ctx_r* the_ctx=reinterpret_cast<ctx_r*>(a_ctx);
  if (the_ctx->counter > 499) 
  {
     the_ctx->counter = 0;
     action_tc3.stop(); 
     return;
  }

  Set_DAC(R_table[0][the_ctx->counter],the_ctx->DACId);
  the_ctx->counter++;

}


void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  double f1 = 0.0;
  double f2 = 0.0;
  double f_meas = 0.0;
  double f1_meas = 0.0;
  double f2_meas = 0.0;
  double f_err = 0.0;

  bool order[2] = {0,0};
  double real_duty[2] = {0.0,0.0};
  double osc2notefreq;
  uint8_t i;
  uint8_t midi_to_pot_G[12];

  //MIDI.sendNoteOn(40, 127, 1);
  //osc2noteshift = 0;

  // restrict OSC to notes of possible frequency range
  if (pitch < 48) {
    noteindexes[0] = 0;
    }
  else if (pitch > 96) {
    noteindexes[0] = 48;
  }
  else {
    noteindexes[0] = pitch - 48;
  }

  noteindexes[1] = noteindexes[0] + osc2noteshift;
  noteindexes[1] = constrain(noteindexes[1],0,48);
  if (noteindexes[0] != (noteindexes[1] - osc2noteshift)) { noteindexes[1] = noteindexes[0];}

  //MIDI.sendNoteOn(41, 127, 1);
   
  if (InTuning)  {
    DebugPrintToken("IN_TUNING",1);
   //MIDI.sendNoteOn(42, 127, 1);
    return;}
  else if ((PWM_Note_Settings[noteindexes[0]][12] == 0) || (PWM_Note_Settings[noteindexes[1]][13] == 0))
  { DebugPrintToken("NOT_YET_TUNED",1);
    DebugPrint("TUNE STATUS VCO0", double(PWM_Note_Settings[noteindexes[0]][12]),4);
    DebugPrint("TUNE STATUS VCO1", double(PWM_Note_Settings[noteindexes[1]][13]),4);
    InTuning = true;
  //MIDI.sendNoteOn(43, 127, 1);
  }
  else
  {
    //MIDI.sendNoteOn(44, 127, 1);
    DebugPrintToken("ALREADY_TUNED",1);
  
    if (cents_detune[0] == 0)
    {
        notefreqs[0] = notes_freq[noteindexes[0]];
        PWM_DAC_Settings[noteindexes[0]][0] = DAC_states_det_backup[noteindexes[0]][0];
        PWM_DAC_Settings[noteindexes[0]][1] = DAC_states_det_backup[noteindexes[0]][1];
    }
    else
    // to do : update DAC_states ! check DAC clipping before
    {
      notefreqs[0] = exp(cents_detune[0]*log(2)/1200 + log(notes_freq[noteindexes[0]]));
      PWM_DAC_Settings[noteindexes[0]][0] = DAC_states_det_backup[noteindexes[0]][0] + DAC_gain[0]*exp(cents_detune[0]*log(2)/1200);
      PWM_DAC_Settings[noteindexes[0]][1] = DAC_states_det_backup[noteindexes[0]][1] + DAC_gain[1]*exp(cents_detune[0]*log(2)/1200);
    }
    
    if (cents_detune[1] == 0)
    {
      notefreqs[1] = notes_freq[noteindexes[1]]; 
      PWM_DAC_Settings[noteindexes[1]][2] = DAC_states_det_backup[noteindexes[1]][2];
      PWM_DAC_Settings[noteindexes[1]][3] = DAC_states_det_backup[noteindexes[1]][3];
    
    }
    else
    // to do : update DAC_states ! check DAC clipping before
    {
      notefreqs[1] = exp(cents_detune[1]*log(2)/1200 + log(notes_freq[noteindexes[1]]));
      PWM_DAC_Settings[noteindexes[1]][2] = DAC_states_det_backup[noteindexes[1]][2] + DAC_gain[2]*exp(cents_detune[1]*log(2)/1200);
      PWM_DAC_Settings[noteindexes[1]][3] = DAC_states_det_backup[noteindexes[1]][3] + DAC_gain[3]*exp(cents_detune[1]*log(2)/1200);
     
    }
    

    //#define CALLBACK_PERIOD 10000000 // hundreths of usecs. (1e-8 secs.)
    //#define DELAY_TIME 2000 // msecs.

    action_tc2.stop();
    if (ringmod_octave < 0) {ringmod_octave = int(pow(2,ringmod_octave));}
    gate_period_hundredth_micros = long(1E8/(ringmod_octave*notefreqs[0]));
    action_tc2.start(gate_period_hundredth_micros,UpdateGateDac,&action_ctx);

    Gate_DAC = true;

    for(i=0;i<6;i++)
    {
      midi_to_pot_G[i] = PWM_Note_Settings[noteindexes[0]][i];
      midi_to_pot_G[i+6] = PWM_Note_Settings[noteindexes[1]][i+6];
      
    }
    
    ChangeNote(midi_to_pot_G, midi_to_pot, pots, true, 0); // Change note for VCO 0 (subosc 0&1)
    ChangeNote(midi_to_pot_G, midi_to_pot, pots, true, 1); // Change note for VCO 1 (subosc 2&3)
    DebugPrint("DAC0",double(PWM_DAC_Settings[noteindexes[0]][0]),2);
    DebugPrintStr("\n",2);
    DebugPrint("DAC1",double(PWM_DAC_Settings[noteindexes[0]][1]),2);
    DebugPrintStr("\n",2);
    DebugPrint("DAC2",double(PWM_DAC_Settings[noteindexes[1]][2]),2);
    DebugPrintStr("\n",2);
    DebugPrint("DAC3",double(PWM_DAC_Settings[noteindexes[1]][3]),2);
    DebugPrintStr("\n",2);
    
    
    Set_DAC(PWM_DAC_Settings[noteindexes[0]][0],0);
    Set_DAC(PWM_DAC_Settings[noteindexes[0]][1],1);
    Set_DAC(PWM_DAC_Settings[noteindexes[1]][2],2);
    Set_DAC(PWM_DAC_Settings[noteindexes[1]][3],3);

    
    if (!InitialTune)
    {
      if (duty[0] <= 0.5) {order[0] = true;} else {order[0] = false;}
      if (duty[1] <= 0.5) {order[1] = true;} else {order[1] = false;}

      //double f_errors[2][2] = { {0.0,0.0} ,{0.0,0.0} };
      

      // TO DO : make calculations of offsets external and pass offset[2]
      //void Enable_PWM_Mod_by_LFO(uint8_t Action, double &PWMDepth, double DutyCenterVal, double LFOFreq, float notefreq, double errors[2][2], uint8_t vco, bool order) 
      
      //if (DAC_gain_corrected_lfo[noteindexes[1]][4] && DACoffset[noteindexes[1]][4])
      //Enable_PWM_Mod_by_LFO(1,PWMDepth[0],PWM_Real_Duty[noteindexes[0]][0],LFOFreq[0],notefreqs[0],0,order[0]);
      for (i=0;i<2;i++)
      {
        if (DAC_gain_corrected_lfo[noteindexes[i]][2+i] == 1.0) 
        {
          DebugPrintStr("\nRECALL_LFO",2);
          Enable_PWM_Mod_by_LFO(13,PWMDepth[i],PWM_Real_Duty[noteindexes[i]][i],LFOFreq[i],notefreqs[i],i,order[i]);
        }
        else
        {
          DebugPrintStr("\nTUNE_LFO",2);
          Enable_PWM_Mod_by_LFO(1,PWMDepth[i],PWM_Real_Duty[noteindexes[i]][i],LFOFreq[i],notefreqs[i],i,order[i]);
          totalerrorsamples[i] = 0;  
        }
      }
    }

    /*
    Serial1.print("S");
    Serial1.print(String(float(noteindex),3));
    Serial1.print("Z");
    Serial1.flush();
    */
    //pot12.decrease(30);
    digitalWrite(gatepin, HIGH);
    action_ctx_ad.counter = 0;
    action_tc3.start(sample_adsr_inc[0]*1E5 ,UpdateADDac,&action_ctx_ad);

    return;
  } // end if already tuned
  
  // TO DO :manage both suboscs.
  
  if ((ADSR_DAC_State[0] != ADSR_Disable) && (ADSR_DAC_State[0] != ADSR_Disabled))
  {
    ADSR_DAC_State[0] = ADSR_AttackInit;
  }

  if ((ADSR_DAC_State[1] != ADSR_Disable) && (ADSR_DAC_State[1] != ADSR_Disabled))
  {
    ADSR_DAC_State[1] = ADSR_AttackInit;
  }
  
  //pot12.decrease(30);
  //notefreqs[0] = double(pgm_read_float(&(notes_freq[noteindex])));
  //notefreqs[1] = double(pgm_read_float(&(notes_freq[osc2noteindex])));
  
  
  if (cents_detune[0] == 0)
    {notefreqs[0] = notes_freq[noteindexes[0]];}
  else
    {notefreqs[0] = exp(cents_detune[0]*log(2)/1200 + log(notes_freq[noteindexes[0]]));}
  
  if (cents_detune[1] == 0)
    {notefreqs[1] = notes_freq[noteindexes[1]];}
  else
    {notefreqs[1] = exp(cents_detune[1]*log(2)/1200 + log(notes_freq[noteindexes[1]]));}
  

  DebugPrint("NOTEFREQ0",notefreqs[0],2);
  DebugPrint("NOTEFREQ1",notefreqs[1],2);
  
  // faster_tune_test
  //MaxVcoPots(pots,midi_to_pot,0);
  //MaxVcoPots(pots,midi_to_pot,1);
  //MaxVcoPots(pots,midi_to_pot,2);
  //MaxVcoPots(pots,midi_to_pot,3);
  
  //MIDI.sendNoteOn(46, 127, 1);
  

  // Tune OSCs sequentially.
  for (i=0;i<2;i++)
  {
    // Set pots for coarse tuning and set both DACs to half deflection setpoint (2047) 
    if (PWM_Note_Settings[noteindexes[i]][12 + i] == 1) 
      { 
        DebugPrint("ALREADY TUNED VCO:",double(i),5);
        continue;
      }

    DebugPrint("\nTUNE VCO:",double(i),5);
    DebugPrint("\nBEFORE ARB FREQ:",double(i),5);
    //CountFrequencyDelta2(10,notefreqs[i],f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
       
    GenerateArbitraryFreqDAC(midi_to_pot,notefreqs[i], duty[i], f1, f2, i); // O is for VCO 0 (not subvco)
    
    DebugPrint("\nAFTER ARB FREQ:",double(i),4);
    //delay(50);
    //CountFrequencyDelta2(10,notefreqs[i],f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
    //DebugPrint("\nf1_meas=",f1_meas,4);
    //DebugPrint("\nf2_meas=",f2_meas,4);
    


    //MIDI.sendNoteOn(47, 127, 1);
  
    //MIDI.sendNoteOn(47, 127, 1);

    // f1 = high , f2 = low
    /*
    if ((f1 == minfreq) || (f1 == maxfreq))
    {

       DebugPrintStr("TUNE CASE 1:",2);
    
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreqs[i],duty,f1,1,2*i,0);
        //MIDI.sendNoteOn(48, 127, 1);
  
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreqs[i],duty,f2,0,2*i + 1,1);
        //MIDI.sendNoteOn(49, 127, 1);
  
        //TO DO : split tuning for both VCO in a loop
        CountFrequencyDelta2(10,notefreqs[i],f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        //CountFrequencyDeltaGlobal(50,notefreq,f_err);
        //MIDI.sendNoteOn(50, 127, 1);
  
    }
    */
    /*
    else if ((f2 == minfreq) || (f2 == maxfreq))
    {


        DebugPrintStr("TUNE CASE 2:",2);
    
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreqs[i],duty,f2,1,2*i,0);
        //MIDI.sendNoteOn(51, 127, 1);
  
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        NewAutoTuneDAC(pots,midi_to_pot,noteindex,notefreqs[i],duty,f1,0,2*i + 1,1);
        //MIDI.sendNoteOn(52, 127, 1);
  
        CountFrequencyDelta2(10,notefreqs[i],f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        //MIDI.sendNoteOn(53, 127, 1);
  
    }
    */
    if (f1 <= f2)
    {


        DebugPrintStr("TUNE CASE 3:",2);
    
        //NewAutoTuneDAC(pots,midi_to_pot,noteindexes[i],notefreqs[i],duty,f2,1,2*i,0);
        NewAutoTuneDAC2(pots,midi_to_pot,noteindexes[i],notefreqs[i],duty[i],f1,f2,i,0);

        DebugPrint("REAL_DUTY", PWM_Real_Duty[noteindexes[i]][i], 2);
        DebugPrintStr("\n",2);

        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        //NewAutoTuneDAC(pots,midi_to_pot,noteindexes[i],notefreqs[i],duty,f1,0,2*i + 1,1);
        
        //CountFrequencyDelta2(10,notefreqs[i],f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        
    }
    else if (f1 > f2)
    {

        // when duty < 0.5
        DebugPrintStr("TUNE CASE 4:\n",2);
        //NewAutoTuneDAC(pots,midi_to_pot,noteindexes[i],notefreqs[i],duty,f1,1,2*i,0);
        NewAutoTuneDAC2(pots,midi_to_pot,noteindexes[i],notefreqs[i],duty[i],f1,f2,i,1);

        DebugPrint("REAL_DUTY", PWM_Real_Duty[noteindexes[i]][i], 2);
        DebugPrintStr("\n",2);

        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        //NewAutoTuneDAC(pots,midi_to_pot,noteindexes[i],notefreqs[i],duty,f2,0,2*i + 1,1);
        
        //CountFrequencyDelta2(10,notefreqs[i],f1,f2,f_meas,f1_meas,f2_meas,f_err,i);
        
    }

  PWM_Note_Settings[noteindexes[i]][12 + i] = 1;
  
  DAC_states_det_backup[noteindexes[i]][2*i] = PWM_DAC_Settings[noteindexes[i]][2*i];
  DAC_states_det_backup[noteindexes[i]][2*i + 1] = PWM_DAC_Settings[noteindexes[i]][2*i + 1];  
    
  
  //DebugPrint("TUNE WRITE STATUS VCO", double(PWM_Note_Settings[noteindexes[i]][12 +i]),4);
  
  } // end tune OSC sequentially

  //PWM_Note_Settings[noteindexes[0]][12] = 1;
  //DebugPrint("TUNE STATUS VCO0", double(PWM_Note_Settings[noteindexes[0]][12]),4);
  //DebugPrint("TUNE STATUS VCO1", double(PWM_Note_Settings[noteindexes[1]][13]),4);


  InTuning = false;
  digitalWrite(gatepin, HIGH);
  action_ctx_ad.counter = 0;
  action_tc3.start(sample_adsr_inc[0]*1E5 ,UpdateADDac,&action_ctx_ad);

 
  //MIDI.sendNoteOn(60, 127, 1);
  
} // End handleNoteOn

void handleNoteOff(byte channel, byte pitch, byte velocity) {

  // Disable VCA Gate
  DebugPrintToken("NOTE_OFF",2);
  digitalWrite(gatepin, LOW);
  action_ctx_ad.counter = 0;
  action_tc3.stop();
  action_tc3.start(sample_adsr_inc[1]*1E5 ,UpdateRDac,&action_ctx_r);
  


  if ((ADSR_DAC_State[0] != ADSR_Disable) && (ADSR_DAC_State[0] != ADSR_Disabled))
  {
    ADSR_DAC_State[0] = ADSR_ReleaseInit;
    DebugPrint("to_ADSR_RELEASE_INIT",double(0),2);
  }

  if ((ADSR_DAC_State[1] != ADSR_Disable) && (ADSR_DAC_State[1] != ADSR_Disabled))
  {
    DebugPrint("to_ADSR_RELEASE_INIT",double(1),2);
    ADSR_DAC_State[1] = ADSR_ReleaseInit;
  }
 
  //pot12.increase(30);
} // end handleNoteOff

void TuneAll()
{
  InitialTune = true;
  uint8_t k;

  for (k = 0;k < 49;k++)
  {
    // reset tuned flag for all notes.
    PWM_Note_Settings[k][12] = 0;
    PWM_Note_Settings[k][13] = 0;
    handleNoteOn(0,k+48,127);
    delay(100);
    handleNoteOff(0,k+48,127);
  }

  InitialTune = false;
  PrintDACGainAdaptive(1);

} // end TuneAll

void handleControlChange(byte channel, byte ccnum, uint8_t val)
{

  // https://www.midi.org/specifications-old/item/table-3-control-change-messages-data-bytes-2
  // Add RPN support for OSC 01 to 23 tune.
  switch (ccnum)
  {  

    case 73:

      // attack pot. LSB 0 to 127.
      if (val > 99) { attack = 99;}
      else { attack = val;}

      chng_attack = attack - cur_attack;
      if (chng_attack > 0) {
      pot12.increase(chng_attack);
      }
      else {
      pot12.decrease(abs(chng_attack));
      }
      cur_attack = attack;
      break;

    case 75:

      // decay pot. LSB 0 to 127.
      if (val > 99) { decay = 99;}
      else { decay = val;}

      chng_decay = decay - cur_decay;
      if (chng_decay > 0) {
      pot13.increase(chng_decay);
      }
      else {
      pot13.decrease(abs(chng_decay));
      }
      cur_decay = decay;
      break;
    
    case 76:

      // semitone pot shift for OSC2 -12 to +12 LSB 0 to 127.
      if (val > 23) { osc2noteshift = 12;}
      else { osc2noteshift =  val  - 12;}
      break;
    

    case 77: 
    
      // hardsync pot. LSB 0 to 63 and 64 to 127. (> 64 hardsync disabled)
      if (val > 64) { digitalWrite(hspin,HIGH);}
      else { digitalWrite(hspin,LOW);}
      break;

    case 78:
     // osc1 square wave volume pot. LSB 0 to 99.

      if (val > 99) { mix3 = 99;}
      else { mix3 = val;}

      chng_mix3 = mix3 - cur_mix3;
      if (chng_mix3 > 0) {
      pot16.increase(chng_mix3);
      }
      else if (chng_mix3 < 0) {
      pot16.decrease(abs(chng_mix3));
      }
      cur_mix3 = mix3;
      break;
    
    case 79:
     // osc2 square wave volume pot. LSB 0 to 99.

      if (val > 99) { mix4 = 99;}
      else { mix4 = val;}

      chng_mix4 = mix4 - cur_mix4;
      if (chng_mix4 > 0) {
      pot17.increase(chng_mix4);
      }
      else if (chng_mix4 < 0) {
      pot17.decrease(abs(chng_mix4));
      }
      cur_mix4 = mix4;
      break;

    case 80:

      // osc1 volume pot. LSB 0 to 99.
      if (val > 99) { mix1 = 99;}
      else { mix1 = val;}

      chng_mix1 = mix1 - cur_mix1;
      if (chng_mix1 > 0) {
      pot14.increase(chng_mix1);
      }
      else if (chng_mix1 < 0) {
      pot14.decrease(abs(chng_mix1));
      }
      cur_mix1 = mix1;
      break;


    case 81:

      // osc2 volume pot. LSB 0 to 127.
      if (val > 99) { mix2 = 99;}
      else { mix2 = val;}

      chng_mix2 = mix2 - cur_mix2;
      if (chng_mix2 > 0) {
      pot15.increase(chng_mix2);
      }
      else if (chng_mix2 < 0) {
      pot15.decrease(abs(chng_mix2));
      }
      cur_mix2 = mix2;
      break;
    
    case 84:
      TuneAll();
      break;

    case 82:

      cents_detune[0] = val - 64;
      break;

    case 83:

      cents_detune[1] = val - 64;
      break;

    case 85:
      
      duty[0] = float(val/255.0);
      TuneAll();
      break;

    case 86:
  
      duty[1] = float(val/255.0);
      TuneAll;
      break;

    case 87:

      ringmod_octave = (val - 64);
      break;


    case 88:

     // osc1 square wave volume pot. LSB 0 to 99.

      if (val > 99) { phase1 = 99;}
      else { phase1 = val;}

      chng_phase1 = phase1 - cur_phase1;
      if (chng_phase1 > 0) {
      pot18.increase(chng_phase1);
      }
      else if (chng_phase1 < 0) {
      pot18.decrease(abs(chng_phase1));
      }
      cur_phase1 = phase1;
      break;

    case 89:

     // osc2 square wave volume pot. LSB 0 to 99.

      if (val > 99) { phase2 = 99;}
      else { phase2 = val;}

      chng_phase2 = phase2 - cur_phase2;
      if (chng_phase2 > 0) {
      pot19.increase(chng_phase2);
      }
      else if (chng_phase2 < 0) {
      pot19.decrease(abs(chng_phase2));
      }
      cur_phase2 = phase2;
      break;

  } // end switch case ccnum
} // end handleControlChange

void WriteEEPROMCoarsePotStepFrequencies(DigiPot *ptr[12], byte (&curr_pot_vals)[12], uint8_t subvco)
{

  int k = 0;
  uint8_t max_pot = 3;
  uint8_t m;
  // max_pot*subvco + 1;
  double f_meas;

  //base flash address;
  uint16_t prev_data_offset = 0;
  
  // tuneblock is for ONE potentiometer.
  int tuneblock_size = 100*(sizeof(f_meas));
  //DebugPrint("WRITE COARSE VCO:",double(subvco),2);

  // make sure DAC is at middle setting.
  Set_DAC(2047,subvco);
  
  if (subvco < 2)
  {
    digitalWrite(vco0_pin, !(subvco % 2));
  }
  else if (subvco >= 2)
  {
    digitalWrite(vco1_pin, !(subvco % 2));
  }

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

  
  DebugPrint("SUBVCO:",double(subvco),2);
      
  for(m=max_pot*subvco + 2;m>=max_pot*subvco + 1;m--)
  {
    //DebugPrint("WRITE COARSE POT:",double(m),2);
  
    for(k=99;k>=0;k--)
    {

      PrintDigiPot(curr_pot_vals,subvco,4);
      DebugPrint("STEP:",double(k),2);
      SerialUSB.println("");
      SerialUSB.flush();
      CountFrequency(100, f_meas, subvco);
      //DebugPrint("FREQ:",f_meas,2);
      int addr = prev_data_offset + (2*subvco + (-m +(max_pot*subvco +2)))*tuneblock_size + (99-k)*(sizeof(f_meas));
      DebugPrint("ADDR:",double(addr),2);
      DebugPrint("FREQ:",double(f_meas),2);
      SerialUSB.println("");
      SerialUSB.flush();
      flash.writeFloat(addr,f_meas);
      delay(500);
      //EEPROM.put(addr,f_meas);
      //we write frequencies in increasing order
      //delay(1000);
      //DebugPrint("END_STEP",double(k),2);
      ptr[m]->decrease(1);
      if (k > 0) {curr_pot_vals[m]--;}

    }
  }

  // reset all pots to max values (to decrease current on timing lines)
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

  
}



void ReadAllCoarseFrequencies(uint8_t subvco)
{
  int k;
  uint8_t m;
  uint8_t max_pot = 3;

  //int prev_data_offset = 686;
  int prev_data_offset = 0;
  int tuneblock_size = 100*(sizeof(double));
  int addr;
  //double *freq;
  DebugPrintStr("READ COARSE FREQS",4);
  
  
  for(m=max_pot*subvco + 2;m>=max_pot*subvco + 1;m--)
  {
    //DebugPrint("WRITE COARSE POT:",double(m),2);
  
    for(k=99;k>=0;k--)
    {

      addr = prev_data_offset + (2*subvco + (-m +(max_pot*subvco +2)))*tuneblock_size + (99-k)*(sizeof(double));
      coarse_freq[subvco][k] = flash.readFloat(addr);
      //DebugPrint(String(k),coarse_freq[subvco][k],4);

      DebugPrint("STEP:",double(k),2);
      DebugPrint("ADDR:",double(addr),2);
      DebugPrint("FREQ:",coarse_freq[subvco][k],4);
      SerialUSB.println("");
      SerialUSB.flush();

      //EEPROM.get(addr,*(*(coarse_freq +vco)+k));
      // we had written frequencies in increasing order
    } // end for k=99


  } // end for m=
 
} // end readallcoarsefreq


void recvWithEndMarker(char endChar) 
{
  static uint8_t ndx = 0;
  char rc;
 

  if (SerialUSB.available() > 0) {
    while (SerialUSB.available() > 0 && newData == false) {
      rc = SerialUSB.read();
      //delay(10);
      //Serial1.println(rc);
      if (rc != endChar) {
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

  double notefreq;
  char charspeed[16] = "<SERIAL 9600>";
  char charfreq[8];
  char printcharfreq[10];
  Serial.begin(115200);
  if (DebugLevel != 0)
  {
    SerialUSB.begin(115200);

    while (!SerialUSB) 
    {
      digitalWrite(13,LOW);
      delay(1000);
      digitalWrite(13,HIGH);
      // wait for serial port to connect
    }
    
    SerialUSB.print("SERIAL 115200\n");
    //Serial.print(charspeed);
  }

  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);

  pinMode(gatepin, OUTPUT);
  pinMode(hspin,OUTPUT);
  
  // disable HardSync at startup
  digitalWrite(hspin, HIGH);
  SerialUSB.print("GATE/HS PIN SET\n");
   
  bool flash_status;
  SPI.begin();
  SerialUSB.print("SPI SET\n");
  
  //DebugPrintStr(String(flash.begin(32*1024)),4);
  //pinMode(53,OUTPUT);
  //digitalWrite(53,HIGH);
  flash_status = flash.begin(MB(32));
  SerialUSB.print("FLASH SET\n");
   
  Wire.begin();
  Wire1.begin();
  SerialUSB.print("WIRE BEGIN\n");
   
  /*
    pot0.begin();
    pot1.begin();
    pot2.begin();
    pot3.begin();
    pot4.begin();
    pot5.begin();
    pot6.begin();
    pot7.begin();
    pot8.begin();
    pot9.begin();
    pot10.begin();
    pot11.begin();
    pot12.begin();
  */
  MCPdac.attach(Wire1, 14);
  MCPdac.readRegisters();

  MCPdac.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD);
  MCPdac.selectPowerDown(MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM);
  MCPdac.selectGain(MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1);

  MCPdac.readRegisters();
  SerialUSB.print("MCP4728 SET\n");
   


  //MCPDAC0.begin();
  //MCPDAC1.begin();
  //MCPDAC2.begin();
  //MCPDAC3.begin();
  Set_DAC(2047,0);
  Set_DAC(2047,1);
  Set_DAC(2047,2);
  Set_DAC(2047,3);
  MCPdac.enable(true);
  SerialUSB.print("DAC HALF-SET\n");
   
  // previous
  /*
  DAC_gain[0] = -8.863;
  DAC_gain[1] = -8.807;
  DAC_gain[2] = -8.813;
  DAC_gain[3] = -8.831;
  
  DAC_cor[0] = 29.37;
  DAC_cor[1] = 42.77;
  DAC_cor[2] = 8.76;
  DAC_cor[3] = 52.26;
  */
  //old (before adding indcuctors)
  /*
  DAC_gain[0] = -8.8473;
  DAC_gain[1] = -8.8702;
  DAC_gain[2] = -8.7908;
  DAC_gain[3] = -8.8144;
  
  DAC_cor[0] = 7.4650;
  DAC_cor[1] = 19.5841;
  DAC_cor[2] = 36.2866;
  DAC_cor[3] = 31.6796;
  */

/*
  DAC_gain[0] = -9.1085;
  DAC_gain[1] = -9.1340;
  DAC_gain[2] = -9.116;
  DAC_gain[3] = -9.1385;
*/  
  

  DAC_gain[0] = -9.10277;
  DAC_gain[1] = -9.10866;
  DAC_gain[2] = -9.11105;
  DAC_gain[3] = -9.12003;

  DAC_cor[0] = 0.3729;
  DAC_cor[1] = -0.6215;
  DAC_cor[2] = 0.957;
  DAC_cor[3] = 3.0808;

 

  ADSR_nb_samples[0] = 1000;
  ADSR_nb_samples[1] = 1000;

  ADSR_DAC_State[0] = ADSR_Disabled;
  ADSR_DAC_State[1] = ADSR_Disabled;
  


  //char charstart[4] = "<F>";
  char charend[4] = "<E>";
  char charackdev[4] = "<D>";

  //char openchar = '<';
  //char closechar = '>';

   
  //bool dumpeeprom = false;
  bool checknotes = false;
  bool checknotes_formula_DAC = false;
  bool checkPWM_LFO = false;
  bool checkpots = false;
  bool generatefreq = false;
  bool generatedVdHzTable = false;
  bool writecoarsefreqs = false; // also read coarse freqs at the end.
  bool readcoarsefreqs = false;
  midimode = true;
  bool donothing = true;

  

  uint8_t max_pot = 3;
  uint8_t notestart;
  uint8_t noteend;

  osc2noteshift = 0;

  long currentsample = 0;
  long prevsample = 0;
    
    
    //Serial1.begin(9600);
    
  //Serial1.begin(9600,SERIAL_8E2);
  if (!midimode)
  {
    //Serial.begin(2400,SERIAL_8E2);
    //Serial.begin(9600,SERIAL_8E1);
    SerialUSB.print(charspeed);
    SerialUSB.flush();   
    delay(3000);
    SerialUSB.print("5");
    SerialUSB.flush();
    //SerialUSB.print(String(flash_status));
    //SerialUSB.print(String(flash.getManID()));
  }
  
  DebugPrintToken("WILL_START_POTS\n",1);
  delay(5000);
  StartAllPots(allpots);
  
  DebugPrintToken("POTS_STARTED\n",1);

  for (k=0;k<49;k++)
  {
    for(n=0;n<14;n++)
    {
      PWM_Note_Settings[k][n] = 0;
    }

    for(n=0;n<4;n++)
    {
      DACoffset[k][n] = 0.0;
      DAC_gain_corrected_lfo[k][n] = 0.0;
      DAC_states_det_backup[k][n] = 0;
      DebugPrint("\nDAC_gain_corrected_lfo[k][n]",DAC_gain_corrected_lfo[k][n],5);

    
    }    
  }

for (k=0;k<200;k++)
{
  for(n=0;n<4;n++)
  {
    DAC_gain_adaptive[k][n] = 0.0;
  } 

}
  DebugPrintToken("NOTES_SETTINGS_ZEROED\n",1);
  
  //freqrefpin = 9;
  subvco = 0;
  // select subvco 0 by default
  pinMode(vco0_pin, OUTPUT);
  pinMode(vco1_pin, OUTPUT);
  
  digitalWrite(vco0_pin, !subvco);
  digitalWrite(vco1_pin, !subvco);

  // use default address 0
  //mcp.pinMode(0, OUTPUT);
  DebugPrintToken("WILL_MAX_POTS\n",1);
  delay(5000);
  
  MaxVcoPots(pots,midi_to_pot,0);
  MaxVcoPots(pots,midi_to_pot,1);

  MaxVcoPots(pots,midi_to_pot,2);
  MaxVcoPots(pots,midi_to_pot,3);

  MaxRemPots(allpots);
  
  
  DebugPrintToken("ALL_POTS_MAXED\n",1);

  //DACTEST
  //pot1.decrease(99);
  //pot4.decrease(99);

  //pot12.increase(99);
  delay(500);
  //SerialUSB.print("8");
  //Serial.flush();
  //pot12.decrease(30);
 // void InitADSR(uint8_t adsr_sel, uint16_t attack_millis, uint16_t decay_millis,uint16_t release_millis, uint8_t sustain_level, uint8_t gain)
  InitADSR(0,10,30,30,0.5,1);
  //InitADSR(1,10,30,30,0.5,1);
  

  byte pot_vals[49][12];
  float pot_devs[49];

  byte rpot2;
  byte rpot1;
  byte rpot0;
  float olddeviation;
  char chardev[30];
  char chardevfmt[32];

  uint8_t i;
  for (i=0;i<12;i++)
  {
    pot_vals[0][i] = 99;
  }
  pot_devs[0] = 0.0;


  DebugPrintToken("POTS_VALS_MAXED\n",1);


  if (midimode) 
  { 

    TuneAll();
    // Connect the handleNoteOn function to the library,
    // so it is called upon reception of a NoteOn.
    MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
    // Do the same for NoteOffs
    MIDI.setHandleNoteOff(handleNoteOff);

    MIDI.setHandleControlChange(handleControlChange);
    
    MIDI.begin(0);
    Serial.begin(115200);
    return;
  }

  if (donothing) 
  {
    //Serial.begin(9600);
    //DebugPrintStr("SERIAL 9600",0);
    DebugPrintToken("DO_NOTHING_RETURN\n",1);
    return;
  }
  
  if (writecoarsefreqs)
  {

    flash.eraseSection(0,4096);
    flash.eraseSection(4096,4096);  
    WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,0);
    WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,1);
    WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,2);
    WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,3);
    // TO DO : Convert ReadAllCoarseFreq to flash read
    ReadAllCoarseFrequencies(0);
    ReadAllCoarseFrequencies(1);
    ReadAllCoarseFrequencies(2);
    ReadAllCoarseFrequencies(3);
    
    return;
    
  }

  if (readcoarsefreqs)
  {

    // TO DO : Convert ReadAllCoarseFreq to flash read
    ReadAllCoarseFrequencies(0);
    ReadAllCoarseFrequencies(1);
    ReadAllCoarseFrequencies(2);
    ReadAllCoarseFrequencies(3);
    
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
      SerialUSB.print(printcharfreq);    
      SerialUSB.print(pot_vals[k][2]);
      SerialUSB.print(",");
      SerialUSB.print(pot_vals[k][1]);
      SerialUSB.print(",");
      SerialUSB.print(pot_vals[k][0]);
      SerialUSB.print("]>");
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
    SerialUSB.print("<generating freq start>");
    GenerateArbitraryFreqDAC(midi_to_pot,400.0, 0.5, f1, f2,0); // 0 is for VCO 0 not subvco.
    delay(120000);
    SerialUSB.print("<generating freq end>");
    SerialUSB.flush();
    return;
  }

  notestart = 20;
  noteend = 21;

  for (subvco = 0; subvco < 4; subvco++)
  {
    midi_to_pot[subvco*max_pot +2] = pot_vals[notestart - 1][subvco*max_pot + 2];
    midi_to_pot[subvco*max_pot +1] = pot_vals[notestart - 1][subvco*max_pot + 1];
    midi_to_pot[subvco*max_pot] = pot_vals[notestart - 1][subvco*max_pot];


    SerialUSB.print("<");
    SerialUSB.print(midi_to_pot[subvco*max_pot +2]);
    SerialUSB.print(",");
    SerialUSB.print(midi_to_pot[subvco*max_pot +1]);
    SerialUSB.print(",");
    SerialUSB.print(midi_to_pot[subvco*max_pot]);
    SerialUSB.print(">");
    SerialUSB.flush();

  

    float integrator;
    char charintegratorfmt[13];
    char charintegrator[7];


    SetNotePots(pots,midi_to_pot,0,subvco);
  
  }

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
    ReadAllCoarseFrequencies(0); // populating global coarse_freq array from flash
    ReadAllCoarseFrequencies(1); // populating global coarse_freq array from flash
    ReadAllCoarseFrequencies(2); // populating global coarse_freq array from flash
    ReadAllCoarseFrequencies(3); // populating global coarse_freq array from flash

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
    SerialUSB.print(printcharfreq);
    SerialUSB.flush();

    if (generatedVdHzTable)
    {
      MaxVcoPots(pots,midi_to_pot,0);
      MaxVcoPots(pots,midi_to_pot,1);
      MaxVcoPots(pots,midi_to_pot,2);
      MaxVcoPots(pots,midi_to_pot,3);
    
      DebugPrintStr("POTS MAXED",2);
      delay(10000);
      Generate_dV_to_dHz_Table(midi_to_pot,notefreq,0);
      Generate_dV_to_dHz_Table(midi_to_pot,notefreq,1);
      Generate_dV_to_dHz_Table(midi_to_pot,notefreq,2);
      Generate_dV_to_dHz_Table(midi_to_pot,notefreq,3);
      
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
      SerialUSB.print("<Generated_f1=");
      SerialUSB.print(String(f1,3));
      SerialUSB.print(">");
      SerialUSB.print("<Generated_f2=");
      SerialUSB.print(String(f2,3));
      SerialUSB.print(">");
      SerialUSB.flush();
      

      //digitalWrite(vco_pin,0);
      //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
      //if (duty > 0.5) { level = !level; arb_vcosel = !arb_vcosel;}
      tunestart = millis();
      // f1 = high , f2 = low

      // to do : check if that section is still required.
      if ((f1 == minfreq) || (f1 == maxfreq))
      {
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,1);
         tuneend = millis() - tunestart;
         SerialUSB.print("<TUNETIME=");
         SerialUSB.print(tuneend);
         SerialUSB.print(">");
         SerialUSB.flush();
         // TO DO : manage both OSCs
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
         //CountFrequencyDeltaGlobal(50,notefreq,f_err);
      }
      else if ((f2 == minfreq) || (f2 == maxfreq))
      {
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,1);
         tuneend = millis() - tunestart;
         SerialUSB.print("<TUNETIME=");
         SerialUSB.print(tuneend);
         SerialUSB.print(">");
         SerialUSB.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
      }
      // end check if that section is still required.
      else if (f1 <= f2)
      {
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,1);
         tuneend = millis() - tunestart;
         SerialUSB.print("<TUNETIME=");
         SerialUSB.print(tuneend);
         SerialUSB.print(">");
         SerialUSB.flush();
         CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,0);
   

      }
      else if (f1 > f2)
      {
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,0,0);
         //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
         //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,1,1);
         tuneend = millis() - tunestart;
         SerialUSB.print("<TUNETIME=");
         SerialUSB.print(tuneend);
         SerialUSB.print(">");
         SerialUSB.flush();
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
      
      SerialUSB.print("<fmeas=");
      SerialUSB.print(String(f_meas,3));
      SerialUSB.print(">");
      SerialUSB.println("");
      SerialUSB.println("");
      SerialUSB.flush();
     
    } // end check notes formula DAC
    // test 
    if (!checkPWM_LFO)
    {
      //checkserial();
      //delay(10);
      SerialUSB.print(charend);
      SerialUSB.flush();
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
      //float PWMDepth = 0.1;
      //Enable_PWM_Mod_by_LFO(false,PWMDepth[0],0.5,0.1,notefreq,f_errors,0,false);
      //Enable_PWM_Mod_by_LFO(false,PWMDepth[1],0.5,0.1,notefreq,f_errors,1,false);
      
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
      SerialUSB.print(charend);
      SerialUSB.flush();
      //delay(10); 
  
    }  

    bool getdeviation = false;
    float deviation;
    bool switchvco = false;
      
    while(!getdeviation) 
    {
      recvWithEndMarker('Z');
      //Serial1.println("devloop");
      if (checkPWM_LFO)
      {
        
        // PWM mode free running. (do not update DAC state needlessly)
        // to do : LFO 'handlenoteon' trigger mode.
        currentsample = long(micros()*inv_sample_inc_micros + 0.5);
        //currentsample = long(timer2.get_micros()*inv_sample_inc_micros + 0.5);
        if(currentsample != prevsample)
        {
          
          Set_DAC(DAC_table[0][currentsample % 360],0);
          Set_DAC(DAC_table[1][currentsample % 360],1);
          Set_DAC(DAC_table[2][currentsample % 360],2);
          Set_DAC(DAC_table[3][currentsample % 360],3);
          
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

    if (checkPWM_LFO) {Disable_PWM_Mod_by_LFO(0);Disable_PWM_Mod_by_LFO(1);}
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


    SerialUSB.print(charackdev);
    SerialUSB.flush();
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


void loop() 
{
  
  // LFO freq errors at both extremes of PWM
  static double fhe1[2] = {0.0,0.0};
  static double fhe2[2] = {0.0,0.0};
  static double fle1[2] = {0.0,0.0};
  static double fle2[2] = {0.0,0.0};
  static double f_errors[4][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  static double f_offset[4] = {0.0,0.0,0.0,0.0};
  
  // temporary, put as global.
  
  //static bool DACGainAdjusted = false;
  //static bool OffsetAdjusted = false;

  //delay(10);
  double notefreq;
  static long currentsample = 0;
  static long prevsample = 0;
  static long ADSR_currentsample[2] = {0,0};
  static long ADSR_prevsample[2] = {0,0};
  static long ADSR_basesample[2] = {0,0};
  
  uint8_t k;
  uint8_t tmp_ADSR_DAC_State;


  //static uint32_t Gate_prevsample = 0;
  static uint32_t Gate_basesample = 0;
  //uint32_t Gate_currentsample = 0;
  //static bool Gate_Level = false;

  if (midimode)
  {
    //pinMode(LED_BUILTIN, OUTPUT);
    MIDI.read();
    //digitalWrite(LED_BUILTIN, HIGH);
    //delay(500);
    //digitalWrite(LED_BUILTIN, LOW);
    
    //MIDI.sendNoteOn(80, 127, 1);

    /*
    if (Gate_DAC)
    {
      UpdateGateDac(Gate_basesample,5);
    }
    */

    if (!Gate_DAC)
    { 
      for (k=1;k<2;k++)
      {
        tmp_ADSR_DAC_State = ADSR_DAC_State[k];
        switch(tmp_ADSR_DAC_State)
        {
          case ADSR_Disabled:
            //DebugPrint("\nADSR_DISABLED",double(k),2);
            break;


          case ADSR_Sustain:
            DebugPrint("\nADSR_SUST",double(k),2);
            if (ADSR_looped)
            {
              DebugPrint("\nADSR_LOOPED",double(k),2);
              ADSR_currentsample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4 + 3] + 0.5) - ADSR_basesample[k];
              DebugPrint("\nADSR_LOOPED_CUR_SAMPLE",double(ADSR_currentsample[k]),3);
              
              if(ADSR_currentsample[k] >= ADSR_nb_samples[k])
              {
                ADSR_DAC_State[k] = ADSR_ReleaseInit;
                DebugPrint("\nADSR_SUST_LOOP_END",double(ADSR_currentsample[k]),3);
              }
            }
            break;

          case ADSR_Off:
            DebugPrint("\nADSR_OFF",double(k),2);
            break;

          case ADSR_Disable:
            DebugPrint("\nADSR_TO_DISABLE",double(k),2);
            Set_DAC(4095,4 + k);
            ADSR_DAC_State[k] = ADSR_Disabled;
            break;

          case ADSR_AttackInit:
            DebugPrint("\nADSR_ATTACKINIT",double(k),2);
            Set_DAC(0,4 + k);
            ADSR_basesample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4] + 0.5);
            ADSR_prevsample[k] = 0;
            ADSR_DAC_State[k] = ADSR_Attack;
            break;

          case ADSR_Attack:
            DebugPrint("\nADSR_ATTACK",double(k),2);
            ADSR_currentsample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4] + 0.5) - ADSR_basesample[k];
            DebugPrint("\nADSR_ATTACK_CUR_SAMPLE",double(ADSR_currentsample[k]),3);
            DebugPrint("\nADSR_NB_SAMPLES",double(ADSR_nb_samples[k]),3);
            if(ADSR_currentsample[k] >= ADSR_nb_samples[k])
            {
              Set_DAC(4095,4 +k);
              ADSR_DAC_State[k] = ADSR_DecayInit;
              DebugPrint("\nADSR_ATTACK_CUR_SAMPLE_OVF",double(ADSR_currentsample[k]),3);
            }
            if(ADSR_currentsample[k] != ADSR_prevsample[k])
            {
              Set_DAC(int(4095*ADSR_currentsample[k]/ADSR_nb_samples[k] +0.5),4+k);
              ADSR_prevsample[k] = ADSR_currentsample[k];    
            }
            break;

          case ADSR_DecayInit:
            DebugPrint("\nADSR_DECAYINIT",double(k),2);
            ADSR_basesample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4 + 1] + 0.5);
            ADSR_prevsample[k] = 0;
            ADSR_DAC_State[k] = ADSR_Decay;
            break;

          case ADSR_Decay:
            DebugPrint("\nADSR_DECAY",double(k),2);
            ADSR_currentsample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4 + 1] + 0.5) - ADSR_basesample[k];
            DebugPrint("\nADSR_DECAY_CUR_SAMPLE",double(ADSR_currentsample[k]),3);
            if(ADSR_currentsample[k] >= ADSR_nb_samples[k])
            {
              Set_DAC(int(ADSR_Sustain_Level[k]/255 + 0.5)*4095,4+k);
              DebugPrint("\nADSR_DECAY_CUR_SAMPLE_OVF",double(ADSR_currentsample[k]),3);
              ADSR_DAC_State[k] = ADSR_Sustain;
              ADSR_basesample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4 +3] + 0.5);
              ADSR_prevsample[k] = 0;
            }
            if(ADSR_currentsample[k] != ADSR_prevsample[k])
            {
              Set_DAC(int(4095*ADSR_currentsample[k]/ADSR_nb_samples[k] +0.5),4+k);
              ADSR_prevsample[k] = ADSR_currentsample[k];    
            }
            break;

          case ADSR_ReleaseInit:
            DebugPrint("\nADSR_RELEASEINIT",double(k),2); 
            ADSR_basesample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4 +2] + 0.5);
            ADSR_prevsample[k] = 0;
            ADSR_DAC_State[k] = ADSR_Release;
            break;

          case ADSR_Release:
            DebugPrint("\nADSR_RELEASE",double(k),2);
            ADSR_currentsample[k] = long(micros()*inv_sample_adsr_inc_micros[k*4 + 2] + 0.5) - ADSR_basesample[k];
            DebugPrint("\nADSR_RELEASE_CUR_SAMPLE",double(ADSR_currentsample[k]),3);
            if(ADSR_currentsample[k] >= ADSR_nb_samples[k])
            {
              DebugPrint("\nADSR_RELEASE_CUR_SAMPLE_OVF",double(ADSR_currentsample[k]),3);
              Set_DAC(0,4);
              if (!ADSR_looped) 
              {
                ADSR_DAC_State[k] = ADSR_Off;
                DebugPrint("\nADSR_OFF",double(k),3);
              }
              else 
              {
                ADSR_DAC_State[k] = ADSR_AttackInit;
                DebugPrint("\nADSR_LOOP_RESET_TO_ATK",double(k),3);
              } // loop enveloppe
            }
            if(ADSR_currentsample[k] != ADSR_prevsample[k])
            {
              Set_DAC(int(4095*(ADSR_Sustain_Level[k]/255)*ADSR_currentsample[k]/ADSR_nb_samples[k] +0.5),4 +k);
              ADSR_prevsample[k] = ADSR_currentsample[k];
            }
            break;
        } // end switch ADSR_states
      } // end ADSR loop
    } // end if !Gate_DAC

     
    if (PWMActive)
    {
      // PWM mode free running. (do not update DAC state needlessly)
      // to do : LFO 'handlenoteon' trigger mode.
      //currentsample = long(timer2.get_micros()*inv_sample_inc_micros + 0.5);
      double fm[2] = {0.0,0.0};
      double fhm[2] = {0.0,0.0};
      double flm[2] = {0.0,0.0};
      double fe[2] = {0.0,0.0};
      double lftmh[2] = {0.0,0.0};
      double lftml[2] = {0.0,0.0};
      uint8_t i;

      bool order[2] = {false,false};

      if (duty[0] <= 0.5) {order[0] = false;} else {order[0] = true;}
      if (duty[1] <= 0.5) {order[1] = false;} else {order[1] = true;}

      currentsample = long(micros()*inv_sample_inc_micros + 0.5);

      // We have to update Gate_DAC a second time because of timer overshoots
      /*
      if (Gate_DAC)
      {
        UpdateGateDac(Gate_basesample,5);
      }
      */
      if(currentsample != prevsample)
      {

        Set_DAC(DAC_table[0][currentsample % 360],0);
      /*
          if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
      */
        Set_DAC(DAC_table[1][currentsample % 360],1);
         /*
         if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
          */
        Set_DAC(DAC_table[2][currentsample % 360],2);
          /*
          if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
          */
        Set_DAC(DAC_table[3][currentsample % 360],3);
         /*
         if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
        */
        for(i=0;i<2;i++)
        {

          // a third time for good measure ;)
          /*
          if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
          */
          
          if ((DACoffset[noteindexes[i]][2+i] == 1.0) && (DAC_gain_corrected_lfo[noteindexes[i]][2+i] == 1.0))
          {
            //DebugPrint("\nLFO ALREADY TUNED - VCO:\n",double(i),1);
            /*
            if (Gate_DAC)
            {
              UpdateGateDac(Gate_basesample,5);
            }
            */
            continue;
          }
        
          if (currentsample % 360 == 89)
          {
            /*
            if (Gate_DAC)
            {
              UpdateGateDac(Gate_basesample,5);
            }
            */
            //for(i=0;i<2;i++)
            //{

            DebugPrint("\n HIGH DAC - VCO:\n",double(i),1);

            DebugPrint("\nDACoffset[noteindexes[0]][2]",DACoffset[noteindexes[0]][2],5);
            DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[0]][2]",DAC_gain_corrected_lfo[noteindexes[0]][2],5);

            DebugPrint("\nDACoffset[noteindexes[1]][2]",DACoffset[noteindexes[1]][3],5);
            DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[1]][3]",DAC_gain_corrected_lfo[noteindexes[1]][3],5);

            CountFrequencyDelta2(10,notefreqs[0],LFO_MinMax_Freq_Targets[2*i + order[i]][1],LFO_MinMax_Freq_Targets[2*i + !order[i]][1],fm[i],fhm[i],flm[i],fe[i],i);
            //CountFrequencyDelta2(100,notefreqs[1],LFO_MinMax_Freq_Targets[2 + order[1]][1],LFO_MinMax_Freq_Targets[2 + !order[1]][1],fm[1],fhm[1],flm[1],fe[1],1);
            
            lftmh[i] = LFO_MinMax_Freq_Targets[2 + order[i]][1];
            lftml[i] = LFO_MinMax_Freq_Targets[2 + !order[i]][1];
            fhe1[i] = fhm[i] - lftmh[i];
            fle1[i] = flm[i] - lftml[i];
            f_errors[2*i][1] = fhe1[i];
            f_errors[2*i +1][1] = fle1[i];
            
            DebugPrint("\nTOT_ERR_SAMPLES:", double(totalerrorsamples[i]),1);
            DebugPrintStr("\nHIGH DAC WATERMARK:\n",1);
            DebugPrint("HIGH_STATE_MEAS:", fhm[i], 2);
            DebugPrint("LOW_STATE_MEAS:", flm[i], 2);
            DebugPrintStr("\n",2);
            DebugPrint("HIGH_STATE_TRG:", lftmh[i],2);
            DebugPrint("LOW_STATE_TRG:", lftml[i],2);
            DebugPrintStr("\n",2);
            DebugPrint("HIGH_STATE_ERR:", fhe1[i], 2);
            DebugPrint("LOW_STATE_ERR:", fle1[i], 2);
            DebugPrintStr("\n",2);
            totalerrorsamples[i]++;
        
          //} // end for(i=0;i<2;i++) - vco select
          /*
            if (Gate_DAC)
              {
                UpdateGateDac(Gate_basesample,5);
              }
          */

            //DAC_gain_corrected = (prev_delta_freq * DAC_gain[subvco])/(prev_delta_freq - delta_freq);
          }  // if (currentsample % 360 == 89)
          else if (currentsample % 360 == 269)
          {
            // this loop works only if both lfos have same phase and freq....
            //for(i=0;i<2;i++)
            //{

            /*
            if (Gate_DAC)
            {
              UpdateGateDac(Gate_basesample,5);
            }
            */
                  
            DebugPrint("LOW DAC - VCO:\n",double(i),1);  
            CountFrequencyDelta2(10,notefreqs[i],LFO_MinMax_Freq_Targets[2*i + order[i]][0],LFO_MinMax_Freq_Targets[2*i + !order[i]][0],fm[i],fhm[i],flm[i],fe[i],i);
            lftmh[i] = LFO_MinMax_Freq_Targets[2*i + order[i]][0];
            lftml[i] = LFO_MinMax_Freq_Targets[2*i + !order[i]][0];
            fhe2[i] = fhm[i] - lftmh[i];
            fle2[i] = flm[i] - lftml[i];
            f_errors[2*i][0] = fhe2[i];
            f_errors[2*i + 1][0] = fle2[i];

            DebugPrint("\nDACoffset[noteindexes[0]][2]",DACoffset[noteindexes[0]][2],5);
            DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[0]][2]",DAC_gain_corrected_lfo[noteindexes[0]][2],5);

            DebugPrint("\nDACoffset[noteindexes[1]][2]",DACoffset[noteindexes[1]][3],5);
            DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[1]][3]",DAC_gain_corrected_lfo[noteindexes[1]][3],5);

            
          
            DebugPrint("\nTOT_ERR_SAMPLES:", double(totalerrorsamples[i]),1);
            DebugPrintStr("\nLOW DAC WATERMARK:\n",1);
            DebugPrint("HIGH_STATE_MEAS:", fhm[i], 2);
            DebugPrint("LOW_STATE_MEAS:", flm[i], 2);
            DebugPrintStr("\n",2);
            DebugPrint("HIGH_STATE_TRG:", lftmh[i], 2);
            DebugPrint("LOW_STATE_TRG:", lftml[i], 2);
            DebugPrintStr("\n",2);

            DebugPrint("HIGH_STATE_ERR:", fhe2[i], 2);
            DebugPrint("LOW_STATE_ERR:", fle2[i], 2);
            DebugPrintStr("\n",2);
            
            totalerrorsamples[i]++;
            
            //} //end for i....
            
            //for(i=0;i<2;i++)
            //{
            
              //test only....

              //f_errors[subvco for given vco][low/high_pwm_trip]
              
              //Enable_PWM_Mod_by_LFO(true,PWMDepth[0],duty[0],LFOFreq[0],notefreqs[0],f_errors,0,true);
            DebugPrint("\nTOT_ERR_SAMPLES0:", totalerrorsamples[0], 5);
            DebugPrint("\nTOT_ERR_SAMPLES1:", totalerrorsamples[1], 5);
            DebugPrint("\nDACOFFSETSET0:", DACoffset[noteindexes[0]][2], 5);
            DebugPrint("\nDACOFFSETSET1:", DACoffset[noteindexes[1]][3], 5);


            if ((totalerrorsamples[i] >= 2) && (DACoffset[noteindexes[i]][2+i] == 0))
            {

              DebugPrint("\nOFFSET SECTION - VCO:\n",double(i),1);  
            
              if ((f_errors[2*i + !order[i]][1] != 0.0) && (f_errors[2*i + !order[i]][0] != 0.0))
                {
                  f_offset[2*i] = (f_errors[2*i + !order[i]][1] + f_errors[2*i + !order[i]][0])/2;
                  DACoffset[noteindexes[i]][!order[i]] = int(f_offset[2*i]*-DAC_gain[2*i + !order[i]]);
                  DebugPrint("f_errors[!order][1]", f_errors[2*i + !order[i]][1],5);
                  DebugPrint("f_offset[!order][0]", f_errors[2*i + !order[i]][0],5);
          
                }

                if ((f_errors[2*i + order[i]][1] != 0.0) && (f_errors[2*i + order[i]][0] != 0.0))
                {
                  f_offset[2*i +1] = (f_errors[2*i + order[i]][1] + f_errors[2*i + order[i]][0])/2;
                  DACoffset[noteindexes[i]][order[i]] = int(f_offset[2*i + 1]*-DAC_gain[2*i + order[1]]);
                
                  // errors[1][1] et errors[0][0]
                  DebugPrint("f_errors[order][1]", f_errors[2*i + order[i]][1],5);
                  DebugPrint("f_offset[order][0]", f_errors[2*i + order[i]][0],5);
          
                }    
      
              Enable_PWM_Mod_by_LFO(6,PWMDepth[i],duty[i],LFOFreq[i],notefreqs[i],i,true);


              DebugPrint("\nDAC OFFSET SET!\n",double(i),1);
              DACoffset[noteindexes[i]][2 + i] = 1.0;
              

              DebugPrint("\nDACoffset[noteindexes[0]][2]",DACoffset[noteindexes[0]][2],5);
              DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[0]][2]",DAC_gain_corrected_lfo[noteindexes[0]][2],5);

              DebugPrint("\nDACoffset[noteindexes[1]][2]",DACoffset[noteindexes[1]][3],5);
              DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[1]][3]",DAC_gain_corrected_lfo[noteindexes[1]][3],5);

              /*
              if (Gate_DAC)
              {
                UpdateGateDac(Gate_basesample,5);
              }
              */
           
              //OffsetAdjusted = true;
              //totalcorrections++;
            } // end if totalerrorsamples > 2 && ....
          //} // end for i....


            //for(i=0;i<2;i++)
            //{
            if ((totalerrorsamples[i] >= 4) && (DACoffset[noteindexes[i]][2+i] == 1.0) && (DAC_gain_corrected_lfo[noteindexes[i]][2+i] == 0.0))
            {
              
              DebugPrint("\nGAIN CORRECTION SECTION - VCO:\n",double(i),1);  
            
              double prev_errors[4];
              double current_errors[4];
              double DAC_gain_corrected_lfo_tmp[8];
                  // these are previous values
              prev_errors[0] = LFO_Delta_Freq_Targets[2*i + order[i]][1];
              prev_errors[1] = LFO_Delta_Freq_Targets[2*i + !order[i]][1];
              prev_errors[2] = LFO_Delta_Freq_Targets[2*i + order[i]][0];
              prev_errors[3] = LFO_Delta_Freq_Targets[2*i + !order[i]][0];
                
              DebugPrint("\nprev_errors[0]", prev_errors[0],5);
              DebugPrint("\nprev_errors[1]", prev_errors[1],5);
              DebugPrint("\nprev_errors[2]", prev_errors[2],5);
              DebugPrint("\nprev_errors[3]", prev_errors[3],5);
              
                
              // these are new values
              current_errors[0] = prev_errors[0] + fhe1[i];
              current_errors[1] = prev_errors[1] + fle1[i];
              current_errors[2] = prev_errors[2] + fhe1[i];
              current_errors[3] = prev_errors[3] + fle1[i];

              DebugPrint("\ncurrent_errors[0]", current_errors[0],5);
              DebugPrint("\ncurrent_errors[1]", current_errors[1],5);
              DebugPrint("\ncurrent_errors[2]", current_errors[2],5);
              DebugPrint("\ncurrent_errors[3]", current_errors[3],5);
                
              if (current_errors[0] > 0) {DAC_gain_corrected_lfo_tmp[0] = (prev_errors[0] * DAC_gain[2*i + order[i]])/current_errors[0];}
              else {DAC_gain_corrected_lfo_tmp[0] = (current_errors[0] * DAC_gain[2*i + order[i]])/prev_errors[0];} 
              
              if (current_errors[1] > 0) {DAC_gain_corrected_lfo_tmp[1] = (prev_errors[1] * DAC_gain[2*i+ !order[i]])/current_errors[1];}
              else {DAC_gain_corrected_lfo_tmp[1] = (current_errors[1] * DAC_gain[2*i + !order[i]])/prev_errors[1];}
              
              if (current_errors[2] > 0) {DAC_gain_corrected_lfo_tmp[2] = (prev_errors[2] * DAC_gain[2 + order[1]])/current_errors[2];}
              else {DAC_gain_corrected_lfo_tmp[2] = (current_errors[2] * DAC_gain[2*i + order[i]])/prev_errors[2];}

              if (current_errors[3] > 0) {DAC_gain_corrected_lfo_tmp[3] = (prev_errors[3] * DAC_gain[2 + !order[1]])/current_errors[3];}
              else {DAC_gain_corrected_lfo_tmp[3] = (current_errors[3] * DAC_gain[2*i + !order[i]])/prev_errors[3];}

              DebugPrint("\nDAC_gain_corrected_lfo_tmp[0]", DAC_gain_corrected_lfo_tmp[0],3);
              DebugPrint("\nDAC_gain_corrected_lfo_tmp[1]", DAC_gain_corrected_lfo_tmp[1],3);
              DebugPrint("\nDAC_gain_corrected_lfo_tmp[2]", DAC_gain_corrected_lfo_tmp[2],3);
              DebugPrint("\nDAC_gain_corrected_lfo_tmp[3]", DAC_gain_corrected_lfo_tmp[3],3);


              // computing the gain correction (based on the average correction of both PWM high and low trips)
              DAC_gain_corrected_lfo[noteindexes[i]][order[1]] = (DAC_gain_corrected_lfo_tmp[0] + DAC_gain_corrected_lfo_tmp[2])/2;
              DAC_gain_corrected_lfo[noteindexes[i]][!order[1]] = (DAC_gain_corrected_lfo_tmp[1] + DAC_gain_corrected_lfo_tmp[3])/2;
              DebugPrint("\nDAC_GAIN_COR1:", DAC_gain_corrected_lfo[noteindexes[i]][order[1]],2);
              DebugPrint("\nDAC_GAIN_COR2:", DAC_gain_corrected_lfo[noteindexes[i]][!order[1]],2);

              Enable_PWM_Mod_by_LFO(14,PWMDepth[i],duty[i],LFOFreq[i],notefreqs[i],i,true);
              DAC_gain_corrected_lfo[noteindexes[i]][2+i] = 1.0;
              DebugPrint("\nDAC GAIN CORRECTION SET!\n",double(i),1);
              DebugPrint("\nDACoffset[noteindexes[i]][2+i]",DACoffset[noteindexes[i]][2+i],2);
              DebugPrint("\nDAC_gain_corrected_lfo[noteindexes[i]][2+i]",DAC_gain_corrected_lfo[noteindexes[i]][2+i],2);


              totalerrorsamples[i] = 0;
              DebugPrint("\nTOTAL_ERROR_SAMPLES_RESET FOR VCO:!\n",double(i),1);
              
              /*
              if (Gate_DAC)
              {
                UpdateGateDac(Gate_basesample,5);
              }
              */

              //DACGainAdjusted = true;
            } //if ((totalerrorsamples >= 4) && (OffsetAdjusted) && (!DACGainAdjusted))
            
          //} // end for i ...
          
            /*
            if ((totalerrorsamples >= 8) && (OffsetAdjusted) && (DACGainAdjusted))
            {

              Enable_PWM_Mod_by_LFO(14,PWMDepth[1],duty[1],LFOFreq[1],notefreqs[0],f_errors,1,true);
              //totalerrorsamples = 0;
              OffsetAdjusted = false;
              DACGainAdjusted = false;
            }
            */

            /*
            if (Gate_DAC)
            {
              UpdateGateDac(Gate_basesample,5);
            }
            */

          } //else if (currentsample % 360 == 269)
            /*
            if (Gate_DAC)
            {
              UpdateGateDac(Gate_basesample,5);
            }
            */
        
        } // end for i.... (VCO)  
        prevsample = currentsample;
        /*
        if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
        */
      } // if(currentsample != prevsample)
         /*
         if (Gate_DAC)
          {
            UpdateGateDac(Gate_basesample,5);
          }
         */
    
    } // if (PWMActive)  
  } // end if midimode

  if (!midimode)
  {
  
    recvWithEndMarker(':');
    if (newData) 
    {
      newData = false;
      //Serial1.println(inp);

      if (!(strcmp(receivedChars , "0")))
      {
        subvco = 0;
        SerialUSB.println("VCOSEL 0");
        digitalWrite(vco0_pin,HIGH);
      }
      else if (!(strcmp(receivedChars , "1"))) 
      {
        subvco = 1;
        SerialUSB.println("VCOSEL 1");
        digitalWrite(vco0_pin,LOW);
      }

      else if (!(strcmp(receivedChars , "2"))) 
      {
        subvco = 2;
        SerialUSB.println("VCOSEL 2");
        digitalWrite(vco1_pin,HIGH);
      }
      else if (!(strcmp(receivedChars , "3"))) 
      {
        subvco = 3;
        SerialUSB.println("VCOSEL 3");
        digitalWrite(vco1_pin,LOW);
      }

      if (!(strcmp(receivedChars, "help")))
      {
        SerialUSB.println("syntax is 'command:' send command with colon ':'");
        SerialUSB.println("available commands:");
        SerialUSB.println("help: print help");
        SerialUSB.println("n: select VCO number n, valid values for n are {0,1,2,3}");
        SerialUSB.println("freqmon: prints sampled frequency of current subvco for 200 cycles, 50 samples pc.");
        SerialUSB.println("writecoarse: steps all pots R100K pots and write corresponding osc freqs to flash");
        SerialUSB.println("readcoarse: pots steps to frequency table print out");
        SerialUSB.println("s: R1K pot value increment for prev. selected vco");
        SerialUSB.println("d: mid R100K pot value increment for prev. selected vco");
        SerialUSB.println("f: high R100K pot value increment for prev. selected vco");
        SerialUSB.println("x: R1K pot value decrement for prev. selected vco");
        SerialUSB.println("c: mid R100K pot value decrement for prev. selected vco");
        SerialUSB.println("v: high R100K pot value decrement for prev. selected vco");
        SerialUSB.println("t: DAC level set to high for prev. selected vco");
        SerialUSB.println("g: DAC level set to mid for prev. selected vco");
        SerialUSB.println("b: DAC level set to low for prev. selected vco");
        
        
      }

      if (!(strcmp(receivedChars, "freqmon")))
      {
        uint8_t count = 0;
        double f_meas = 0;
        
        while (count++ < 50)
        { 
          f_meas = 0;
          CountFrequency(220,f_meas,subvco);
          delay(500);
        }
      
      }

      if (!(strcmp(receivedChars, "freqmonsaw")))
      {
        uint8_t count = 0;
        double f_meas = 0;
        double f1 = 0;
        double f2 = 0;
        double f_err = 0;
        
        while (count++ < 50)
        { 
          f_meas = 0;
          f1 = 0;
          f2 = 0;
          f_err = 0;
          CountFrequencyDelta2(10,100,100,100,f_meas,f1,f2,f_err,int(subvco/2));
          DebugPrint("f_meas", f_meas, 2);
          DebugPrint("f1", f1, 2);
          DebugPrint("f2", f2, 2);
          
          delay(500);
        }
      
      }


      if (!(strcmp(receivedChars, "writecoarse")))
      {
        flash.eraseSection(0,4096);
        flash.eraseSection(4096,4096);  

        WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,0);
        WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,1);
        WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,2);
        WriteEEPROMCoarsePotStepFrequencies(pots,midi_to_pot,3);
        ReadAllCoarseFrequencies(0);
        ReadAllCoarseFrequencies(1);
        ReadAllCoarseFrequencies(2);
        ReadAllCoarseFrequencies(3);
        
      }


      if (!(strcmp(receivedChars, "readcoarse")))
      {
        ReadAllCoarseFrequencies(0);
        ReadAllCoarseFrequencies(1);
        ReadAllCoarseFrequencies(2);
        ReadAllCoarseFrequencies(3);
      }
    

      if (!(strcmp(receivedChars, "gendactable")))
      {
        static uint8_t notestart = 20;
        static uint8_t noteend = 21;
        static char charfreq[8];
        static char printcharfreq[10];
        uint32_t addr;
        //uint32_t baseaddr = 16384;
        //flash.eraseSection(16384,4096*4);

        for (k=notestart;k<noteend;k++) 
        {
          //notefreq = pgm_read_float(&(notes_freq[k]));
          notefreq = notes_freq[k];
          dtostrf(notefreq, 6, 2, charfreq);
          sprintf(printcharfreq,"<F%s>", charfreq);
          SerialUSB.print(printcharfreq);
          SerialUSB.flush();

          MaxVcoPots(pots,midi_to_pot,0);
          MaxVcoPots(pots,midi_to_pot,1);
          MaxVcoPots(pots,midi_to_pot,2);
          MaxVcoPots(pots,midi_to_pot,3);
          PrintDigiPot(midi_to_pot,0,1);
          PrintDigiPot(midi_to_pot,1,1);
          PrintDigiPot(midi_to_pot,2,1);
          PrintDigiPot(midi_to_pot,3,1);
           
          DebugPrintStr("POTS MAXED",2);
          delay(10000);
          //flash.eraseSection(16384,4096);
          //Generate_dV_to_dHz_Table(midi_to_pot,notefreq,0);
          //flash.eraseSection(20480,4096);
          //Generate_dV_to_dHz_Table(midi_to_pot,notefreq,1);
          //flash.eraseSection(24576,4096);
          Generate_dV_to_dHz_Table(midi_to_pot,notefreq,2);
          //flash.eraseSection(28672,4096);
          Generate_dV_to_dHz_Table(midi_to_pot,notefreq,3);
          
        }
      }


      if (!(strcmp(receivedChars, "checknotesdac")))
    {
      
      double f1 = 0.0;
      double f2 = 0.0;
      double f_meas = 0.0;
      double f1_meas = 0.0;
      double f2_meas = 0.0;
      double f_err = 0.0;
      double duty = 0.5;
      uint32_t tunestart;
      uint32_t tuneend;
      MaxVcoPots(pots,midi_to_pot,0);
      MaxVcoPots(pots,midi_to_pot,1);
      MaxVcoPots(pots,midi_to_pot,2);
      MaxVcoPots(pots,midi_to_pot,3);
      

      static uint8_t notestart = 1;
      static uint8_t noteend = 49;
      static char charfreq[8];
      static char printcharfreq[10];
        
      for (k=notestart;k<noteend;k++) 
      {
        notefreq = notes_freq[k];
        dtostrf(notefreq, 6, 2, charfreq);
        sprintf(printcharfreq,"<F%s>", charfreq);
        SerialUSB.print(printcharfreq);
        SerialUSB.flush();
          

        GenerateArbitraryFreqDAC(midi_to_pot,notefreq, duty, f1, f2, int(subvco/2)); // 0 is for VCO 0, not subvco
        //test delay(2000);
        SerialUSB.print("<Generated_f1=");
        SerialUSB.print(String(f1,3));
        SerialUSB.print(">");
        SerialUSB.print("<Generated_f2=");
        SerialUSB.print(String(f2,3));
        SerialUSB.print(">");
        SerialUSB.flush();
        

        //digitalWrite(vco_pin,0);
        //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
        //if (duty > 0.5) { level = !level; arb_vcosel = !arb_vcosel;}
        tunestart = millis();
        // f1 = high , f2 = low
        // to do : check if that section is still required.
        if ((f1 == minfreq) || (f1 == maxfreq))
        {
        //SerialUSB.println("freq_case1");
        //Serial.flush();
          
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,int(subvco/2)*2,0);
          //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,int(subvco/2)*2 + 1,1);
          tuneend = millis() - tunestart;
          SerialUSB.print("<TUNETIME=");
          SerialUSB.print(tuneend);
          SerialUSB.print(">");
          SerialUSB.flush();
          // TO DO : manage both OSCs
          CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,int(subvco/2));
          //CountFrequencyDeltaGlobal(50,notefreq,f_err);
        }
        else if ((f2 == minfreq) || (f2 == maxfreq))
        {
        //SerialUSB.println("freq_case2");
        //Serial.flush();
        
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,int(subvco/2)*2 + 1,0);
          //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,int(subvco/2)*2,1);
          tuneend = millis() - tunestart;
          SerialUSB.print("<TUNETIME=");
          SerialUSB.print(tuneend);
          SerialUSB.print(">");
          SerialUSB.flush();
          CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,int(subvco/2));
        }
        // end check if that section is still required.
        else if (f1 <= f2)
        {
          //SerialUSB.println("freq_case3");
          //Serial.flush();
        
          /// last three params : level, subvco, global tune
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,int(subvco/2)*2 + 1,0);
          //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,int(subvco/2)*2,1);
          tuneend = millis() - tunestart;
          SerialUSB.print("<TUNETIME=");
          SerialUSB.print(tuneend);
          SerialUSB.print(">");
          SerialUSB.flush();
          CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,int(subvco/2));
    

        }
        else if (f1 > f2)
        {
          //SerialUSB.println("freq_case4");
          //Serial.flush();
        
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f1,1,int(subvco/2)*2,0);
          //CountFrequencyDelta2(50,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err);
          //NewAutoTuneDAC(pots,midi_to_pot,notestart,notefreq,duty,f2,0,int(subvco/2)*2 + 1,1);
          tuneend = millis() - tunestart;
          SerialUSB.print("<TUNETIME=");
          SerialUSB.print(tuneend);
          SerialUSB.print(">");
          SerialUSB.flush();
          CountFrequencyDelta2(10,notefreq,f1,f2,f_meas,f1_meas,f2_meas,f_err,int(subvco/2));
          //CountFrequencyDeltaGlobal(50,notefreq,f_err);
        }

        SerialUSB.print("<fmeas=");
        SerialUSB.print(String(f_meas,3));
        SerialUSB.print(">");
        SerialUSB.println("");
        SerialUSB.println("");
        SerialUSB.flush();
      }
    } // end check notes formula DAC
   

      if (subvco == 0) 
      {
        if (!(strcmp(receivedChars , "s"))) 
        {
          pots[0]->increase(1);
          (midi_to_pot[0])++;
          //intp0++;
          PrintDigiPot(midi_to_pot,0,2); 
        }

        if (!(strcmp(receivedChars , "d"))) 
        {
          pots[1]->increase(1);
          (midi_to_pot[1])++;
          //intp1++;
          PrintDigiPot(midi_to_pot,0,2);
        }

        if (!(strcmp(receivedChars , "f"))) 
        {
          pots[2]->increase(1);
          (midi_to_pot[2])++;
          //intp2++;
          PrintDigiPot(midi_to_pot,0,2);
        }

        if (!(strcmp(receivedChars , "x"))) 
        {
          pots[0]->decrease(1);
          (midi_to_pot[0])--;
          //intp0--;
          PrintDigiPot(midi_to_pot,0,2);
        }

        if (!(strcmp(receivedChars , "c"))) 
        {
          pots[1]->decrease(1);
          (midi_to_pot[1])--;
          //intp1--;
          PrintDigiPot(midi_to_pot,0,2);
        }

        if (!(strcmp(receivedChars , "v"))) 
        {
          pots[2]->decrease(1);
          (midi_to_pot[2])--;
          //intp2--;
          PrintDigiPot(midi_to_pot,0,2);
        }

        if (!(strcmp(receivedChars , "t"))) 
        {
          MCPdac.analogWrite(0,4095);
          SerialUSB.println("DAC0 high");
      
        }

        if (!(strcmp(receivedChars , "g"))) 
        {
          MCPdac.analogWrite(0,2047);
          SerialUSB.println("DAC0 mid");
        }

        if (!(strcmp(receivedChars , "b"))) 
        {
          MCPdac.analogWrite(0,0);
          SerialUSB.println("DAC0 low");
      
        }
      } // end if (!subvco)

      else if (subvco == 1)
      {

        if (!(strcmp(receivedChars , "s"))) 
        {
          pots[3]->increase(1);
          (midi_to_pot[3])++;
          //intp3++;
          PrintDigiPot(midi_to_pot,1,2);
        }

        if (!(strcmp(receivedChars , "d"))) 
        {
          pots[4]->increase(1);
          (midi_to_pot[4])++;
          //intp4++;
          PrintDigiPot(midi_to_pot,1,2);
        }

        if (!(strcmp(receivedChars , "f"))) 
        {
          pots[5]->increase(1);
          (midi_to_pot[5])++;
          //intp5++;
          PrintDigiPot(midi_to_pot,1,2);
        }

        if (!(strcmp(receivedChars , "x"))) 
        {
          pots[3]->decrease(1);
          (midi_to_pot[3])--;
          //intp3--;
          PrintDigiPot(midi_to_pot,1,2); 
        }

        if (!(strcmp(receivedChars , "c"))) 
        {
          pots[4]->decrease(1);
          (midi_to_pot[4])--;
          //intp4--;
          PrintDigiPot(midi_to_pot,1,2);
        }

        if (!(strcmp(receivedChars , "v"))) 
        {
          pots[5]->decrease(1);
          (midi_to_pot[5])--;
          //intp5--;
          PrintDigiPot(midi_to_pot,1,2); 
        }

        if (!(strcmp(receivedChars , "t"))) 
        {
          MCPdac.analogWrite(1,4095);
          SerialUSB.println("DAC1 high");
        }

          if (!(strcmp(receivedChars , "g"))) 
        {
          MCPdac.analogWrite(1,2047);
          SerialUSB.println("DAC1 mid");
      
        }

          if (!(strcmp(receivedChars , "b"))) 
        {
          MCPdac.analogWrite(1,0);
          SerialUSB.println("DAC1 low");
      
        } 
      } // end ... else (subvco)
    
    else if (subvco == 2)
      {

        if (!(strcmp(receivedChars , "s"))) 
        {
          pots[6]->increase(1);
          (midi_to_pot[6])++;
          //intp3++;
          PrintDigiPot(midi_to_pot,2,2);
        }

        if (!(strcmp(receivedChars , "d"))) 
        {
          pots[7]->increase(1);
          (midi_to_pot[7])++;
          //intp4++;
          PrintDigiPot(midi_to_pot,2,2);
        }

        if (!(strcmp(receivedChars , "f"))) 
        {
          pots[8]->increase(1);
          (midi_to_pot[8])++;
          //intp5++;
          PrintDigiPot(midi_to_pot,2,2);
        }

        if (!(strcmp(receivedChars , "x"))) 
        {
          pots[6]->decrease(1);
          (midi_to_pot[6])--;
          //intp3--;
          PrintDigiPot(midi_to_pot,2,2); 
        }

        if (!(strcmp(receivedChars , "c"))) 
        {
          pots[7]->decrease(1);
          (midi_to_pot[7])--;
          //intp4--;
          PrintDigiPot(midi_to_pot,2,2);
        }

        if (!(strcmp(receivedChars , "v"))) 
        {
          pots[8]->decrease(1);
          (midi_to_pot[8])--;
          //intp5--;
          PrintDigiPot(midi_to_pot,2,2); 
        }

        if (!(strcmp(receivedChars , "t"))) 
        {
          MCPdac.analogWrite(2,4095);
          SerialUSB.println("DAC2 high");
      
        }

          if (!(strcmp(receivedChars , "g"))) 
        {
          MCPdac.analogWrite(2,2047);
          SerialUSB.println("DAC2 mid");
      
        }

          if (!(strcmp(receivedChars , "b"))) 
        {
          MCPdac.analogWrite(2,0);
          SerialUSB.println("DAC2 low");
      
        } 
      } // end ... else (subvco)

    else if (subvco == 3)
      {
        if (!(strcmp(receivedChars , "s"))) 
        {
          pots[9]->increase(1);
          (midi_to_pot[9])++;
          //intp3++;
          PrintDigiPot(midi_to_pot,3,2);
        }

        if (!(strcmp(receivedChars , "d"))) 
        {
          pots[10]->increase(1);
          (midi_to_pot[10])++;
          //intp4++;
          PrintDigiPot(midi_to_pot,3,2);
        }

        if (!(strcmp(receivedChars , "f"))) 
        {
          pots[11]->increase(1);
          (midi_to_pot[11])++;
          //intp5++;
          PrintDigiPot(midi_to_pot,3,2);
        }

        if (!(strcmp(receivedChars , "x"))) 
        {
          pots[9]->decrease(1);
          (midi_to_pot[9])--;
          //intp3--;
          PrintDigiPot(midi_to_pot,3,2); 
        }

        if (!(strcmp(receivedChars , "c"))) 
        {
          pots[10]->decrease(1);
          (midi_to_pot[10])--;
          //intp4--;
          PrintDigiPot(midi_to_pot,3,2);
        }

        if (!(strcmp(receivedChars , "v"))) 
        {
          pots[11]->decrease(1);
          (midi_to_pot[11])--;
          //intp5--;
          PrintDigiPot(midi_to_pot,3,2); 
        }

        if (!(strcmp(receivedChars , "t"))) 
        {
          MCPdac.analogWrite(3,4095);
          SerialUSB.println("DAC3 max");
        }

          if (!(strcmp(receivedChars , "g"))) 
        {
          MCPdac.analogWrite(3,2047);
          SerialUSB.println("DAC3 mid");
        }

          if (!(strcmp(receivedChars , "b")))
        {
          MCPdac.analogWrite(3,0);
          SerialUSB.println("DAC3 low");
        } 
      } // end ... else (subvco)
    } // end if (serial.available)
  } // end else (midimode)
} // end loop()


