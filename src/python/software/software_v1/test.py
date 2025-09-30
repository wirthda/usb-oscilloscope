import numpy as np
from scipy import stats

def period_value(data):
    meanval = np.mean(data)
    crossings = np.where((data[:-1] < meanval) & (data[1:] >= meanval))[0]
    if len(crossings) >= 2:
        return np.mean(np.diff(crossings)) #Gibt den Mittelwert über mehrere Perioden
    else:
        return None
    
def mean(data):
    return np.mean(data[0:int((np.floor(len(data)/period_value(data))*period_value(data)))])


data = [1, 1, 2, 5, 6, 1, 1, 2, 5, 6, 1, 1, 2, 5, 6, 1, 1, 2, 5, 6, 7]

erg = mean(data)
print(erg)