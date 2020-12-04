#!/usr/bin/env python
  
from waveform_analysis._common import parabolic
import numpy as np
from numpy.fft import rfft
from numpy import asarray, argmax, mean, diff, log, copy
from waveform_analysis._common import find
from scipy.signal import correlate, kaiser, decimate
from scipy import signal
from scipy.io import wavfile
import matplotlib.pyplot as plt
import os
import serial
import time
import math
import alsaaudio as audio
import sys
from subprocess import Popen

startMarker = 60
endMarker = 62

def recvFromArduino():
  global startMarker, endMarker
  #print("inwait:" + str(ser.inWaiting()))
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  time.sleep(0.05)
  # wait for the start character
  while  ord(x) != startMarker: 
    time.sleep(0.001)
    #print(ser.inWaiting())
    if(ser.inWaiting()):
        x = ser.read()
        #print(x)
  # save data until the end marker is found
  while ord(x) != endMarker:
    time.sleep(0.001)
    if ord(x) != startMarker:
      ck = ck + x.decode('utf-8') 
      byteCount += 1
    if(ser.inWaiting()):
        x = ser.read()
  
  return(ck)

def freq_from_hps(signal, fs):
    """
    Estimate frequency using harmonic product spectrum

    Low frequency noise piles up and overwhelms the desired peaks

    Doesn't work well if signal doesn't have harmonics
    """
    signal = asarray(signal) + 0.0

    N = len(signal)
    signal -= mean(signal)  # Remove DC offset

    # Compute Fourier transform of windowed signal
    windowed = signal * kaiser(N, 100)

    # Get spectrum
    X = log(abs(rfft(windowed)))

    # Remove mean of spectrum (so sum is not increasingly offset
    # only in overlap region)
    X -= mean(X)

    # Downsample sum logs of spectra instead of multiplying
    hps = copy(X)
    for h in range(2, 9):  # TODO: choose a smarter upper limit
        dec = decimate(X, h, zero_phase=True)
        hps[:len(dec)] += dec

    # Find the peak and interpolate to get a more accurate peak
    i_peak = argmax(hps[:len(dec)])
    i_interp = parabolic(hps, i_peak)[0]

    # Convert to equivalent frequency
    return fs * i_interp / N  # Hz



#Output Device
#out = audio.PCM(audio.PCM_PLAYBACK,channels=1,rate=48000,format=audio.PCM_FORMAT_S16_LE,periodsize=1000,device='plughw:CARD=Intel,DEV=0')
#out = audio.PCM(audio.PCM_PLAYBACK,channels=1,rate=48000,format=audio.PCM_FORMAT_S16_LE,periodsize=1000,device='pulse')
#periodsize=1000
#out.setchannels(channels)
#out.setrate(framerate)
#out.setformat(audioformat)
#out.setperiodsize(periodsize)

#splitting the bytearray into period sized chunks
#list1 = [allData[i:i+periodsize] for i in range(0, len(allData), periodsize)]




count = int(sys.argv[1])
noteend = int(sys.argv[2])
print("tune from: " + sys.argv[1] + " to: " + sys.argv[2])
#ser =  serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser = serial.Serial()
ser.port = "/dev/ttyACM0"
ser.baudrate = 9600
ser.timeout = None
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
#ser.parity = serial.PARITY_EVEN
#ser.stopbits = serial.STOPBITS_TWO
ser.setDTR(False)
time.sleep(0.5)
ser.setRTS(False)
time.sleep(0.5)
ser.open()
time.sleep(2)

line = ""   # read a '\n' terminated line

while (count<noteend):

    line = ""
    fund_freq = 0
    #line = ser.readline()   # read a '\n' terminated line
    #line = line.decode('utf-8')
    time.sleep(0.01)

    if(ser.inWaiting()):
        #time.sleep(0.2)
        line = recvFromArduino()   # read a '\n' terminated line
        print(line)
        #print(line[:1])
        #try:
        #    line = line.decode('utf-8', errors='replace')
        #    line = line.strip('\n')
        #    #print(line)
        #except UnicodeDecodeError: # catch error and ignore it
        #    print('DECODE ERROR')
        #    line = ""
    #else:
        #line = ""
    #print(line)
    if (line[:1] == 'F'):
        
        #print("match")
        fund_freq = 0
        note_freq = (line[1:])
        #note_freq = line
        note_freq = float(note_freq)
        print(note_freq)
        #print('note_freq ' + tune_vals[0])
        #print ('pot0 ' + tune_vals[1])
        #print ('pot1 ' + tune_vals[2])
        #print ('pot2 ' + tune_vals[3])
        #Writing the output
        #refwavename = 'note' + str(count) + '.wav'
        #refwavenamepath = '/home/rod/Documents/CLASS/LTSPICE/SYNTH1/REF/' + refwavename

        
        #p = Popen(['aplay', refwavenamepath],shell = False) # something long running
        # ... do other stuff while subprocess is running
        #print("pid is:")
        #print(str(p.pid))      
        
        while True:
            line = ""
            if(ser.inWaiting()):
                #time.sleep(0.2)
                line = recvFromArduino()   # read a '\n' terminated line
                print(line)
            time.sleep(0.01)
            if (line[:1] == 'E'):
                print("tune end")
                break



        #time.sleep(3)    
        filename = '/home/rod/Documents/CLASS/LTSPICE/SYNTH1/WAV/tunetest_' + str(count) + '.wav'
        os.system('arecord -f S16_LE -d 10 -r 48000 --device="plughw:CARD=Intel,DEV=0" ' + filename)
        #os.system('arecord -f S16_LE -d 3 -r 44100 --device="plughw:CARD=Intel,DEV=0" ' + filename + ' > /dev/null 2>&1')
        count += 1

        # downloaded from http://vocaroo.com/i/s1KZzNZLtg3c

        # Parameters
        time_start = 0  # seconds
        time_end = 10  # seconds
        filter_stop_freq = 50  # Hz
        filter_pass_freq = 100  # Hz
        filter_order = 1001

        # Load data
        fs, audio = wavfile.read(filename)
        #audio_l = audio_both[:, 0]
        #audio = audio_l.astype(float)

        # High-pass filter
        nyquist_rate = fs / 2.
        desired = (0, 0, 1, 1)
        bands = (0, filter_stop_freq, filter_pass_freq, nyquist_rate)
        filter_coefs = signal.firls(filter_order, bands, desired, nyq=nyquist_rate)

        # Examine our high pass filter
        #w, h = signal.freqz(filter_coefs)
        #f = w / 2 / np.pi * fs  # convert radians/sample to cycles/second
        #plt.plot(f, 20 * np.log10(abs(h)), 'b')
        #plt.ylabel('Amplitude [dB]', color='b')
        #plt.xlabel('Frequency [Hz]')
        #plt.xlim((0, 300))

        #print(audio)
        # Apply high-pass filter
        filtered_audio = signal.filtfilt(filter_coefs, [1], audio)

        # Only analyze the audio between time_start and time_end
        time_seconds = np.arange(filtered_audio.size, dtype=float) / fs
        audio_to_analyze = filtered_audio[(time_seconds >= time_start) &
                                          (time_seconds <= time_end)]

        #fundamental_frequency = freq_from_hps(audio_to_analyze, fs)



        fund_freq = freq_from_hps(audio_to_analyze, fs)
        print("Fundamental is {} Hz".format(fund_freq))
        print("Note freq is {} Hz".format(note_freq))
        cents_dev = 1200 * math.log2(note_freq/fund_freq)
        print("Deviation is {} Cents".format(cents_dev))
        cents_dev_str = str(cents_dev) + 'Z'
        cents_dev_ascii = cents_dev_str.encode('ascii')
        #bytecents = bytearray(cents_dev_ascii,'ascii')
        print(cents_dev_ascii)
        ser.write(cents_dev_ascii)
        ser.flush()
        #for i in cents_dev_ascii:
            #print(i)
            #time.sleep(0.01)
            #ser.write(i)
        line = ""
        while True:
            if(ser.inWaiting()):
                #time.sleep(0.2)
                line = recvFromArduino()   # read a '\n' terminated line
                print(line)
            time.sleep(0.001)
            if (line[:1] == 'D'):
                print("dev ack")
                break

