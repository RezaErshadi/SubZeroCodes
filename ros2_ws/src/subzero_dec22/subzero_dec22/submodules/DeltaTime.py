import datetime

def DeltaTime(t0):
    now =  datetime.datetime.now()
    d = now - t0
    return round(d.seconds + (d.microseconds)/1000000,1)