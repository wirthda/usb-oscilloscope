import numpy as np
from scipy import stats

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

def frequency(data, abtastzeit):
    perv = period_value(data)
    per = perv*abtastzeit
    if per is None or per == 0:
        return None
    return 1 / per

def dach(signal_data, upper_part_threshold_percentage=0.9, mode_uniqueness_ratio=0.1):
    if not isinstance(signal_data, np.ndarray) or signal_data.ndim != 1:
        raise ValueError("signal_data muss ein 1D NumPy-Array sein.")
    if len(signal_data) == 0:
        return np.nan # Oder einen Fehler auslösen, je nach gewünschtem Verhalten

    min_val = np.min(signal_data)
    max_val = np.max(signal_data)

    # Sonderfall: Wenn alle Werte gleich sind, ist das Dach dieser Wert
    if min_val == max_val:
        return min_val

    # 1. Den "oberen Wellenformteil" definieren
    # Schwellenwert: min_val + Prozentsatz des Peak-to-Peak-Bereichs
    threshold = min_val + upper_part_threshold_percentage * (max_val - min_val)
    upper_waveform_part = signal_data[signal_data >= threshold]

    # Wenn keine Datenpunkte im oberen Teil sind (z.B. sehr verrauschtes Signal oder Schwellenwert zu hoch)
    if len(upper_waveform_part) == 0:
        # Fallback auf das Maximum, da kein "oberer Teil" vorhanden ist, um einen Modus zu finden.
        return max_val

    # 2. Den Modus des oberen Wellenformteils berechnen
    # `stats.mode` gibt ein ModeResult-Objekt mit .mode und .count zurück
    mode_result = stats.mode(upper_waveform_part, keepdims=False) # keepdims=False für skalaren Modus

    dach_value = mode_result.mode
    mode_count = mode_result.count

    # 3. Prüfen, ob der Modus "eindeutig definiert" ist
    # Wenn die Modus-Häufigkeit einen geringen Anteil der oberen Samples ausmacht, ist er nicht eindeutig genug.
    # Oder wenn es generell sehr wenige Samples im oberen Teil gibt.
    if len(upper_waveform_part) < 2 or (mode_count / len(upper_waveform_part)) < mode_uniqueness_ratio:
        # Wenn nicht eindeutig, wird das Dach auf das Maximum des *gesamten* Signals gesetzt
        dach_value = max_val

    return dach_value

def base(signal_data, lower_part_threshold_percentage=0.9, mode_uniqueness_ratio=0.1):
    if not isinstance(signal_data, np.ndarray) or signal_data.ndim != 1:
        raise ValueError("signal_data muss ein 1D NumPy-Array sein.")
    if len(signal_data) == 0:
        return np.nan # Oder einen Fehler auslösen, je nach gewünschtem Verhalten

    min_val = np.min(signal_data)
    max_val = np.max(signal_data)

    # Sonderfall: Wenn alle Werte gleich sind, ist das Dach dieser Wert
    if min_val == max_val:
        return min_val

    # 1. Den "oberen Wellenformteil" definieren
    # Schwellenwert: min_val + Prozentsatz des Peak-to-Peak-Bereichs
    threshold = min_val + lower_part_threshold_percentage * (max_val - min_val)
    lower_waveform_part = signal_data[signal_data <= threshold]

    # Wenn keine Datenpunkte im oberen Teil sind (z.B. sehr verrauschtes Signal oder Schwellenwert zu hoch)
    if len(lower_waveform_part) == 0:
        # Fallback auf das Maximum, da kein "oberer Teil" vorhanden ist, um einen Modus zu finden.
        return min_val

    # 2. Den Modus des oberen Wellenformteils berechnen
    # `stats.mode` gibt ein ModeResult-Objekt mit .mode und .count zurück
    mode_result = stats.mode(lower_waveform_part, keepdims=False) # keepdims=False für skalaren Modus

    base_value = mode_result.mode
    mode_count = mode_result.count

    # 3. Prüfen, ob der Modus "eindeutig definiert" ist
    # Wenn die Modus-Häufigkeit einen geringen Anteil der oberen Samples ausmacht, ist er nicht eindeutig genug.
    # Oder wenn es generell sehr wenige Samples im oberen Teil gibt.
    if len(lower_waveform_part) < 2 or (mode_count / len(lower_waveform_part)) < mode_uniqueness_ratio:
        # Wenn nicht eindeutig, wird das Dach auf das Maximum des *gesamten* Signals gesetzt
        dach_value = min_val

    return base_value

def amplitude(data):
    return dach(data) - base(data)

def dc_rms(data):
    return np.sqrt(np.mean(np.square(data[0:(np.floor(len(data)/period_value(data))*period_value(data))])))

def ac_rms(data):
    return np.std(data[0:(np.floor(len(data)/period_value(data))*period_value(data))], ddof=0)