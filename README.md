# synth1

Project philosophy :

Arduino Due driven dual XR-2206 MIDI Analog Synthesizer
Projected to use modular approach based on standard eurorack for housing instead of a monolythic PCB.
All control done through Digital Potentiometers driving analog components, or DACs with a selection of wavetables or custom user loadable wavetables based on math
functions. All control wrapped into standard MIDI CC control.


Project Manifest and current support :

Dual XR2206 VCO With 4 DAC acting as pitch/pwm LFOs (controlling the fine frequency of oscillation of the individual suboscillators in self-keying mode)
Coarse frequency control is achieved thru 2 X9C104 (100K ohms digital potentiometers) by suboscillator and a 20k trim potentiometer for base (C3 Note frequency)
Fine tuning precision is projected to be remain under 2 musical cents.
Frequency Range is from C3 to C7 at least for both oscillators.
Total fine tuning range is fixed at +/- 225 Hz around coarse note frequency.

Uses Arduino MIDI USB library.
Uses MCP23017 GPIO expander.
Uses AD333 and a 4 channel XOR gate for control signal routing (tri/saw or square output to the output stage) and hardsync.
Uses an autotune helper and frequency counting based on PulseIn (to measure frequency and pulse width).
555 timer analog Attack/Decay enveloppe generator and OTA based VCA.  

Projected support :
Additional 4 general purpose DAC based LFOs/ADSR
Digital control 4pole LP,BP,HP filters (one per XR2206).
Mixing stage of both oscillators post-filter.
WaveShapping from sine to triangle based on vactrol control. (resistive opto-isolator) using XR-2206 waveshapping pins.
THD/Symmetry control based on vactrol control. (using XR-2206 symmetry adjust pins)
AM/vibrato/Gate control using XR2206 native AM pin
Diode/transformer based RINGMOD.

For the PCB board prototype advancement part of the project, please refer to : https://easyeda.com/rodv92/synth1



