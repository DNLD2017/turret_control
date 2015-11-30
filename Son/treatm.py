import urllib2, urllib
import os
import wave
import struct
import numpy as np
import pylab as pl
import plot
import operator
import matplotlib.pyplot as plt
import alsaaudio


def lireFic(name):

    tempb = wave.Wave_read(name)
    
    num_channels = tempb.getnchannels()
    num_frames = tempb.getnframes()
    total_samples = num_frames * num_channels
    
    
    
    fs = tempb.getframerate()
    ns = tempb.getnframes()
    nch = tempb.getnchannels()
    nbit = tempb.getsampwidth()*8
    print fs, ns, nch, nbit
    tempb = tempb.readframes(ns)
    signal = np.array(struct.unpack("%dh"%(total_samples),tempb))
    signal = signal/2.0**(nbit-1)
    return signal, fs,ns,num_channels

def estimationEchelleTemps(ns,fs,num_channels):
    dt = 1.0/float(fs)
    duree = float(num_channels*ns)*dt
    t=np.linspace(0,duree,num_channels*ns)
    return dt,t

def affichageTemps(t,s):
  
    pl.plot(t, s)
    
def getNextPow2(num):
    i=0
    while(num>0):
        num = num/2
        i=i+1
    return i+1
    
def fastFourrierTransf(s):
    pow2 = getNextPow2(len(s))
    signal = abs(np.fft.fft(s,2**pow2))
    return signal/max(signal),pow2

def TFCT(s,fs,ns,nfft,astep):
    nwin=1+(ns-nfft)/astep
    is0=np.array(range(0,ns-nfft+1,astep))
    is1=is0+nfft
    win=np.hanning(nfft)
    tfr=np.zeros((nwin,nfft/2),float)
    
    print nwin,len(is0), len(is1)    
    
    for i in range(nwin):
        s_bloc=s[is0[i]:is1[i]]
        spectre_bloc=np.abs(np.fft.fft(s_bloc*win))
        spectre_bloc[0]=0.0
        tfr[i,:]=spectre_bloc[0:(nfft/2)]
    tfr_min=np.min(tfr)
    tfr_max=np.max(tfr)
    tfr=np.round(255.0*(tfr-tfr_min)/(tfr_max-tfr_min))
    return tfr

def  Signalwav(w1,fs):
    data0, fs0, ns0 =lireFic(w1)
    data1, fs1, ns1 =lireFic("jog_lfm_2013_ch1.wav")
    data2, fs2, ns2 =lireFic("jog_lfm_2013_ch2.wav")
    data3, fs3, ns3 = lireFic("jog_lfm_2013_ch3.wav")
    data4, fs4, ns4 =lireFic("jog_lfm_2013_ch4.wav")
    
    n=max(ns0,ns1,ns2,ns3,ns4)
   
   
    f0=np.fft.fft(data0, n)
    f1=np.fft.fft(data1, n)
    f2=np.fft.fft(data2, n)
    f3=np.fft.fft(data3, n)
    f4=np.fft.fft(data4, n)
   
    
    
    
    dt= 1.0/float(fs)
    duree=float(ns0)*dt
    t_inter=np.linspace(0.0,duree,ns0)
    
    inter_coor0=np.abs(np.fft.ifft(f0*np.conj(f1)))
    inter_coor1=np.abs(np.fft.ifft(f0*np.conj(f2)))
    inter_coor2=np.abs(np.fft.ifft(f0*np.conj(f3)))
    inter_coor3=np.abs(np.fft.ifft(f0*np.conj(f4)))
    
    max_coor0=max(inter_coor0)
    max_coor1=max(inter_coor1)
    max_coor2=max(inter_coor2)
    max_coor3=max(inter_coor3)
    
    ind0=np.where(inter_coor0-max_coor0)
    ind00=ind0[0][0]
    t_detect0=float(ind00)/fs
    
    ind1=np.where(inter_coor1-max_coor1)
    ind01=ind1[0][0]
    t_detect1=float(ind01)/fs
    
    ind2=np.where(inter_coor2-max_coor2)
    ind02=ind2[0][0]
    t_detect2=float(ind02)/fs
    
    ind3=np.where(inter_coor3-max_coor3)
    ind03=ind3[0][0]
    t_detect3=float(ind03)/fs
    
    return "t0 : "+str(t_detect0)+"t1 : "+str(t_detect1)+"t2 : "+str(t_detect2)+"t3 : "+str(t_detect3)
    
    
    
def detectFreq(data1,f,alpha):
    data=[];
    dataf=[];
    freq=[];
   
    for  i in range(len(data1)):
        if data1[i]>alpha: 
            data.append(data1[i])
            dataf.append(data1[i])
            freq.append(f[i])
        else:
            data.append(0)
            
    return data,freq
    
def detecHarmoniques(freq,seuil):
    m=[];
    f=[];
    freq=np.abs(freq)
    for  i in range(len(freq)):
        for  j in range(len(freq)):
            if(i!=j):
                if(abs(freq[i]-freq[j])<=seuil):
                   
                    m.append(freq[j])
               
            
        if m==[]:
            f.append(freq[i])
        else :
           
            f.append(np.mean(m))
            m=[];
    print("1")
    f=np.round(f)
    f=list(set(f)) 
    f=np.sort(f)
    return f
    
def detectDrone (f,seuil):

    modulo=[]
    for i in range(len(f)):
        modulo.append(f[i]%f[i])
    if np.mean(modulo)<seuil:
        return True
    else:
        return False
                
                
def verifMethodeSeuil1():
    pas=np.linspace(0.3,0.6,4)
    print(pas)
    pas2=np.linspace(1,10,10)
    print(pas2)
    seuil1=[]
    seuil2=[]
    seuil3=[]
    nombreVerifOk=[]
    filenames=["drone1wav.wav","drone2wav.wav","drone3wav.wav","drone4wav.wav","drone5wav.wav"]
    compteur=0
    indiceCompteur=[]
    l=0
    
    for i in range(5):
        for s1 in pas:
            for s2 in pas2:
                for s3 in pas2:
                    print(s1,s2,s3)
                    data0, fs0, ns0,num = lireFic(filenames[i])
                    dt,t = estimationEchelleTemps(ns0, fs0,num)
                    data1, ls = fastFourrierTransf(data0)
                    f=np.linspace(-fs0/2,fs0/2,2**ls)
                    f=np.fft.fftshift(f)
                    data1,freq=detectFreq(data1,f,s1)
                    f3=detecHarmoniques(freq,s2)
                    b=detectDrone(f3,s3)
                    if (b==True) :
                        
                        seuil1.append(s1)
                        seuil2.append(s2)
                        seuil3.append(s3)
                        l+=1
                        compteur+=1
                    else :
                        nombreVerifOk.append(compteur)
                        indiceCompteur.append(l)
                        compteur=0
    return seuil1,seuil2,seuil3,nombreVerifOk,indiceCompteur
    
def record():                    
    inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE)

    inp.setrate(44100)
    inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    inp.setperiodsize(1024)

    w = wave.open('test.wav', 'w+')
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(44100)

    while True:
        l, data = inp.read()
        a = numpy.fromstring(data, dtype='int16')
        print numpy.abs(a).mean()
        w.writeframes(data)

def liveTreat():
    inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE)

    inp.setrate(44100)
    inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    inp.setperiodsize(1024)

    w = wave.open('test.wav', 'w+')
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(44100)

    compteur=0
    
    while True:
        l, data = inp.read()
        a = numpy.fromstring(data, dtype='int16')
        print numpy.abs(a).mean()
        w.writeframes(data)
        compteur+=1
        if (compteur==20):
            data0, fs0, ns0,num = lireFic('test.wav')
            dt,t = estimationEchelleTemps(ns0, fs0,num)
            affichageTemps(t, data0)
            data1, ls = fastFourrierTransf(data0)
            f=np.linspace(-fs0/2,fs0/2,2**ls)
            f=np.fft.fftshift(f)
            data1,freq=detectFreq(data1,f,0.3)
            f3=detecHarmoniques(freq,5)
            b=detectDrone(f3,5)
            print b
            affichageTemps(f, data1)
            
seuil1,seuil2,seuil3,nombreVerifOk,indiceCompteur=verifMethodeSeuil1()   
i=np.argmax(nombreVerifOk)
j=indiceCompteur[i]
s1=seuil1[j] 
s2=seuil2[j] 
s3=seuil3[j]
print(s1,s2,s3) 

    
    
#main
#if __name__=='__main__':

    
    
#chargement du fichier


#filename="drone4wav.wav"
#data0, fs0, ns0,num = lireFic(filename)
#dt,t = estimationEchelleTemps(ns0, fs0,num)
#affichageTemps(t, data0)
#data1, ls = fastFourrierTransf(data0)
#f=np.linspace(-fs0/2,fs0/2,2**ls)
#f=np.fft.fftshift(f)
#data1,freq=detectFreq(data1,f,0.3)
#f3=detecHarmoniques(freq,5)
#b=detectDrone(f3,5)
#print b
#affichageTemps(f, data1)
#data3=TFCT(data0, fs0, ns0, 1024, 512)
#pl.figure()
#pl.xlim(-200,800)
#pl.imshow(data3)
#pl.show()
