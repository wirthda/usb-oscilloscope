import numpy as np

def maximum(data):
    return np.max(data)

def minimum(data):
    return np.min(data)

def mean(data):
    return np.mean(data[0:(np.floor(len(data)/period_value(data))*period_value(data))])

def peak_peak(data):
    return np.max(data) - np.min(data)

def period_value(data):
    meanval = np.mean(data)
    crossings = np.where((data[:-1] < meanval) & (data[1:] >= meanval))[0]
    if len(crossings) >= 2:
        return np.mean(np.diff(crossings)) #Gibt den Mittelwert über mehrere Perioden
    else:
        return None

def period(data, abtastzeit):
    pv = period_value(data)
    if pv is None:
        return None
    return pv * abtastzeit
    # if p<1e-6:
    #     p = p*1_000_000
    #     return(str(p) + " uHz")
    # elif p<1e-3:
    #     p = p*1_000
    #     return(str(p) + " mHz")

def frequency(data, abtastzeit):
    perv = period_value(data)
    per = perv*abtastzeit
    if per is None or per == 0:
        return None
    return 1 / per

def top(data, percent=0.1):
    sorted_data = np.sort(data)
    N = len(data)
    cnt = max(int(N * percent), 1)
    return np.mean(sorted_data[-cnt:])

def base(data, percent=0.1):
    sorted_data = np.sort(data)
    N = len(data)
    cnt = max(int(N * percent), 1)
    return np.mean(sorted_data[:cnt])

def amplitude(data):
    return top(data) - base(data)

def dc_rms(data):
    return np.sqrt(np.mean(np.square(data[0:(np.floor(len(data)/period_value(data))*period_value(data))])))

def ac_rms(data):
    return np.std(data[0:(np.floor(len(data)/period_value(data))*period_value(data))], ddof=0)