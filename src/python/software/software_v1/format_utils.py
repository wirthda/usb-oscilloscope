def format_value(val, unit_base):
    if unit_base == 's':
        if val < 1e-6:
            return f"{val*1e9:.0f} ns"
        elif val < 1e-3:
            return f"{val*1e6:.1f} µs"
        elif val < 1:
            return f"{val*1e3:.1f} ms"
        else:
            return f"{val:.2f} s"
        
    if unit_base == 'V':
        if val < 1:
            return f"{val*1e3:.0f} mV"
        
        else:
            return f"{val:.2f} V"
    
    if unit_base == 'Hz':
        if val < 1e3:
            return f"{val:.2f} Hz"
        elif val < 1e6:
            return f"{val/1e3:.2f} kHz"
        else:
            return f"{val/1e6:.2f} MHz"