import apreshttp
import datetime
import pytest
import os
import time


API_ROOT = "192.168.1.1"
API_KEY = "18052021"
ApRES = apreshttp.API(API_ROOT)
ApRES.setKey(API_KEY)

polarization = "Tx1H-Rx1H"
download = True
prefix = "test"
n_attenuator = 1
n_subburst = 10
attenuators = [21,0,0,0]
gains = [-4,-14,-14,-14]
tx = [1,1,0,0,0,0,0,0]
rx = [1,1,0,0,0,0,0,0]

polarization = polarization
DownloadFile = download
Prefix = prefix
ApRES.radar.config.set(nAtts = n_attenuator,
                            nBursts = n_subburst,
                            rfAttnSet = tuple(attenuators),
                            afGainSet = tuple(gains),
                            txAnt = tuple(tx),
                            rxAnt = tuple(rx))

filename = "TEST.dat"
ApRES.radar.burst(filename)
ApRES.radar.results(wait = True)
if DownloadFile == True:
    if not os.path.exists(filename):
        ApRES.data.download("Survey/" + filename)



# API_ROOT = "192.168.1.1"
# API_KEY = "18052021"
# api = apreshttp.API(API_ROOT)
# api.setKey(API_KEY)
# api.radar.config.set(nAtts = 1,
#                     nBursts = 1,
#                     rfAttnSet=21,
#                     afGainSet=-14,
#                     txAnt=(1,1,0,0,0,0,0,0),
#                     rxAnt=(1,1,0,0,0,0,0,0))
# filename = "httpBurst_"+datetime.datetime.now().strftime("%Y%m%d_%H-%M-%S")+".dat"
# print("Burst initiated")
# t0 = time.perf_counter()
# api.radar.burst(filename)
# api.radar.results(wait = True)
# print(f"Burst completed in: {round(time.perf_counter() - t0,1)} seconds")
# t0 = time.perf_counter()
# if not os.path.exists(filename):
#     print("Downloading the burst file")
#     api.data.download("Survey/" + filename)
#     print(f"Download Completed in {round(time.perf_counter() - t0,1)} seconds")
# with pytest.raises(apreshttp.RadarBusyException):
#     api.radar.burst(filename)

# attenuator = [0,0,0,0]
# gain = [-6,-6,-6,-6]
# txAnt=[0,0,0,0,0,0,0,0]
# rxAnt=[0,0,0,0,0,0,0,0]
# a = "ApRESburst,3,5,10,21,-14,-4,6,mimo,1,5,1,5"
# comma_splt = a.split(",")
# ApRESconf = comma_splt[1:]
# "3,5,10,21,-14,-4,6,mimo,1,1,5,5"
# nAtt = int(ApRESconf[0])
# for i in range(int(nAtt)):
#     attenuator[i] = int(ApRESconf[i+1])
#     gain[i] = int(ApRESconf[i+1+int(nAtt)])
# BursMode = ApRESconf[(2*nAtt)+1]
# iTx = [(2*nAtt)+2,(2*nAtt)+3]
# iRx = [(2*nAtt)+2]
# if BursMode == "mono":
#     nt1 = int(ApRESconf[(2*nAtt)+2])
#     nr1 = int(ApRESconf[(2*nAtt)+3])
#     txAnt[nt1-1] = 1
#     rxAnt[nr1-1] = 1
# elif BursMode == "mimo":
#     nt1 = int(ApRESconf[(2*nAtt)+2])
#     nt2 = int(ApRESconf[(2*nAtt)+3])
#     nr1 = int(ApRESconf[(2*nAtt)+4])
#     nr2 = int(ApRESconf[(2*nAtt)+5])
#     txAnt[nt1-1] = 1
#     txAnt[nt2-1] = 1
#     rxAnt[nr1-1] = 1
#     rxAnt[nr2-1] = 1

# print(nAtt)
# print(attenuator)
# print(gain)
# print(BursMode)
# print(tuple(txAnt))
# print(tuple(rxAnt))