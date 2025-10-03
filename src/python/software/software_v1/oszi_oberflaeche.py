import sys
import os
import threading
import queue
from statemachine import StateMachine, State
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pickle
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar
from scipy.interpolate import make_interp_spline, interp1d


from usb_oszi_pc_fsm import fsm_pc
from PySide6.QtWidgets import QApplication, QWidget, QMainWindow, QGridLayout, QPushButton, QLabel, QComboBox, QStackedWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QDial, QGroupBox, QSpinBox
from PySide6.QtCore import Qt, QObject, QTimer, Signal, Slot, QThread

from measurement_functions import *
from format_utils import format_value

class UsbOsziGUI(StateMachine):
     idle = State("Idle", initial=True)     #Warten bis Run gedrückt
     acquiring = State("Acquiring")         #Warten bis Daten vorhanden
     processing = State("Processing")       #Verarbeitung und Darstellung der Daten

     start_acquisition = idle.to(acquiring)
     data_received = acquiring.to(processing)
     processing_done_continue = processing.to(acquiring)
     processing_done_stop = processing.to(idle)
     reset = (
        idle.to(idle) | acquiring.to(idle) | processing.to(idle)
    )


class MatplotlibWidget(FigureCanvas):
    cursor_diff_changed = Signal(float,float)
    def __init__(self):
        self.figure, self.ax = plt.subplots()
        super().__init__(self.figure)

        # ... alle bisherigen Initialisierungen ... #
        self.crosshair1_visible = False
        self.crosshair2_visible = False
        self.dragging_cursor = None
        self.cursor_x1 = None  
        self.cursor_y1 = None  
        self.cursor_x2 = None
        self.cursor_y2 = None

        # Hauptkurve und Cursor werden jetzt als persistent angelegt (1x)
        self.line = self.ax.plot([], [], color='C0')[0]  # initial leeres Datenarray!
        self.vline1 = self.ax.axvline(x=0, color='orange', linestyle='--', linewidth=1, visible=False)
        self.hline1 = self.ax.axhline(y=0, color='orange', linestyle='--', linewidth=1, visible=False)
        self.coord_text1 = self.ax.text(0, 0, '', color='black', visible=False,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", lw=0.5, alpha=0.8)
        )
        self.vline2 = self.ax.axvline(x=0, color='orange', linestyle='--', linewidth=1, visible=False)
        self.hline2 = self.ax.axhline(y=0, color='orange', linestyle='--', linewidth=1, visible=False)
        self.coord_text2 = self.ax.text(0, 0, '', color='black', visible=False,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", lw=0.5, alpha=0.8)
        )
        self.zero_line = self.ax.axhline(y=0, color="gray", linestyle="--", linewidth=1, zorder=1)

        self.data_x = np.array([])
        self.data_y = None

        # ... Event-Handler weiter wie gehabt ...
        self.figure.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.figure.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.figure.canvas.mpl_connect('button_release_event', self.on_mouse_release)
    
    def get_time_label(self,val):
        return format_value(val, "s") + "/div"
        
    def get_v_label(self, val):
        return format_value(val, "V") + "/div" 

    # --- Skalierung (Achsensetting nur wenn nötig aufrufen, bei Basisänderung!) ---
    def update_axes_scaling(self, num_xdiv, num_ydiv, timebase, vdiv, xoffset, yoffset):
        x_min = 0 + xoffset
        x_max = timebase * num_xdiv + xoffset
        y_center = yoffset
        y_min = y_center - (num_ydiv/2) * vdiv
        y_max = y_center + (num_ydiv/2) * vdiv

        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_xticks(np.linspace(x_min, x_max, num_xdiv + 1))
        self.ax.set_yticks(np.linspace(y_min, y_max, num_ydiv + 1))
        self.ax.grid(True, which="major", axis="both")
        self.ax.set_xlabel(self.get_time_label(timebase))
        self.ax.set_ylabel(self.get_v_label(vdiv))
        if self.data_x.any():
            self.update_cursor1(x_min)
            self.update_cursor2(x_max)
        self.figure.canvas.draw_idle()

    # --- Performantes Live-Plotting, ohne clear und Neuanlage ---
    def plot_data(self, data, samplingrate, plot_style): #t für Testzwecke hinzugefügt
        # if not data:
        #     print("No data to plot.")
        #     return

        delta_time = 1 / samplingrate
        t = np.arange(len(data)) * delta_time

        self.data_x = t
        self.data_y = data

        if plot_style == "linear":
            self.line.set_linestyle('-')
            self.line.set_marker('')
            self.line.set_drawstyle('default')
            self.line.set_data(t, data)
        elif plot_style == "points":
            self.line.set_linestyle('None')
            self.line.set_marker('o')
            self.line.set_data(t, data)
        elif plot_style == "steps":
            self.line.set_linestyle('-')
            self.line.set_drawstyle('steps-post')
            self.line.set_marker('')
            self.line.set_data(t, data)
        elif plot_style == "spline":
            # Berechne glatten Spline
            t_fine = np.linspace(t[0], t[-1], len(t)*10)
            y_fine = make_interp_spline(t, data, k=3)(t_fine)
            self.line.set_linestyle('-')
            self.line.set_marker('')
            self.line.set_drawstyle('default')
            self.line.set_data(t_fine, y_fine)

        # --- Cursor-Initialisierung falls nötig ---
        # Setze Defaultposition ans Plotfenster-Anfang/Ende, falls noch None
        if self.cursor_x1 is None:
            x_min, x_max = self.ax.get_xlim()  # aktueller sichtbarer Plotbereich
            # Finde alle x-Indizes, die im Sichtbereich liegen:
            indices_in_window = np.where((self.data_x >= x_min) & (self.data_x <= x_max))[0]
            if len(indices_in_window) > 0:
                idx_first_visible = indices_in_window[0]
            else:
                idx_first_visible = -1  # fallback
            self.cursor_x1 = self.data_x[idx_first_visible]
            self.cursor_y1 = self.data_y[idx_first_visible]

        if self.cursor_x2 is None:
            x_min, x_max = self.ax.get_xlim()  # aktueller sichtbarer Plotbereich
            # Finde alle x-Indizes, die im Sichtbereich liegen:
            indices_in_window = np.where((self.data_x >= x_min) & (self.data_x <= x_max))[0]
            if len(indices_in_window) > 0:
                idx_last_visible = indices_in_window[-1]
            else:
                idx_last_visible = -1  # fallback
            self.cursor_x2 = self.data_x[idx_last_visible]
            self.cursor_y2 = self.data_y[idx_last_visible]

        # Cursor-Objekte verschieben, Text erneuern
        self.vline1.set_xdata([self.cursor_x1])
        self.hline1.set_ydata([self.cursor_y1])
        self.coord_text1.set_position((self.cursor_x1, self.cursor_y1))
        self.coord_text1.set_text(f"x={self.cursor_x1:.3e}\ny={self.cursor_y1:.3e}")

        self.vline2.set_xdata([self.cursor_x2])
        self.hline2.set_ydata([self.cursor_y2])
        self.coord_text2.set_position((self.cursor_x2, self.cursor_y2))
        self.coord_text2.set_text(f"x={self.cursor_x2:.3e}\ny={self.cursor_y2:.3e}")

        # delta t update
        self.update_delta_t_textbox()

        # Kein ax.clear(), keine neue Linie/cursor—nur updaten!
        self.figure.canvas.draw_idle()
        
    def show_crosshair(self):
        self.crosshair1_visible = True
        self.vline1.set_visible(True)
        self.hline1.set_visible(True)
        self.coord_text1.set_visible(True)
        self.crosshair2_visible = True
        self.vline2.set_visible(True)
        self.hline2.set_visible(True)
        self.coord_text2.set_visible(True)
        self.draw_idle()

    def hide_crosshair(self):
        self.crosshair1_visible = False
        self.vline1.set_visible(False)
        self.hline1.set_visible(False)
        self.coord_text1.set_visible(False)
        self.crosshair2_visible = False
        self.vline2.set_visible(False)
        self.hline2.set_visible(False)
        self.coord_text2.set_visible(False)
        if hasattr(self, "delta_t_text") and self.delta_t_text is not None:
                self.delta_t_text.remove()
                self.delta_t_text = None
        self.draw_idle()

    def update_cursor1(self, mouse_x):
        if self.crosshair1_visible is True:
            idx = np.argmin(np.abs(self.data_x - mouse_x))
            x_val = self.data_x[idx]
            y_val = self.data_y[idx]
            self.cursor_x1 = x_val
            self.cursor_y1 = y_val
            self.vline1.set_xdata([x_val])
            self.hline1.set_ydata([y_val])
            self.coord_text1.set_text(f"x={x_val:.2e}\ny={y_val:.2e}")
            self.coord_text1.set_position((x_val, y_val))
            self.update_delta_t_textbox()
    
    def update_cursor2(self, mouse_x):
        if self.crosshair2_visible is True:
            idx = np.argmin(np.abs(self.data_x - mouse_x))
            x_val = self.data_x[idx]
            y_val = self.data_y[idx]
            self.cursor_x2 = x_val
            self.cursor_y2 = y_val
            self.vline2.set_xdata([x_val])
            self.hline2.set_ydata([y_val])
            self.coord_text2.set_text(f"x={x_val:.2e}\ny={y_val:.2e}")
            self.coord_text2.set_position((x_val, y_val))
            self.update_delta_t_textbox()
    
    def update_delta_t_textbox(self):
        
        if self.cursor_x1 is not None and self.cursor_x2 is not None and self.crosshair1_visible is True:
            delta_x = abs(self.cursor_x2 - self.cursor_x1)
            delta_y = abs(self.cursor_y2 - self.cursor_y1)
            self.cursor_diff_changed.emit(delta_x, delta_y)
        self.draw_idle()

    def on_mouse_press(self, event):
        if event.button == 1 and event.inaxes == self.ax and self.data_x.any():
            mouse_x = event.xdata
            # Bestimme Index und y-Wert des Mauspunktes
            idx_click = np.argmin(np.abs(self.data_x - mouse_x))
            click_x = self.data_x[idx_click]
            # Cursor-Initialisierung sicherstellen!
            if self.cursor_x1 is None or self.cursor_x2 is None:
                self.cursor_x1, self.cursor_y1 = self.data_x[0], self.data_y[0]
                self.cursor_x2, self.cursor_y2 = self.data_x[500], self.data_y[500]
            #Distanz berechnen
            dist1 = abs(self.cursor_x1 - click_x)
            dist2 = abs(self.cursor_x2 - click_x)
            # Entscheide, welcher Cursor gezogen wird
            if dist1 <= dist2:
                self.dragging_cursor = 1
                self.update_cursor1(click_x)
            else:
                self.dragging_cursor = 2
                self.update_cursor2(click_x)

    def on_mouse_move(self, event):
        if self.dragging_cursor == 1 and event.inaxes == self.ax and self.data_x is not None:
            self.update_cursor1(event.xdata)
        elif self.dragging_cursor == 2 and event.inaxes == self.ax and self.data_x is not None:
            self.update_cursor2(event.xdata)

    def on_mouse_release(self, event):
        self.dragging_cursor = None

UNIT_BASES = {
    "Amplitude" : "V",
    "Max" : "V",
    "Min" : "V",
    "Top" : "V",
    "Base" : "V",
    "Mittelwert" : "V",
    "Peak-Peak" : "V",
    "DC RMS" : "V",
    "AC RMS" : "V",
    "Periode" : "s",
    "Frequenz" : "Hz",
}

class MeasureWorker(QObject):
    finished = Signal(list)

    def __init__(self, data_buffer, abtastzeit, checks):
        super().__init__()
        self.data_buffer = data_buffer
        self.abtastzeit = abtastzeit
        self.checks = checks

    @Slot()
    def process(self):
        messwerte = []
        
        for checkbox, (name, func) in self.checks:
            if checkbox.isChecked():
                try:
                    result = func(self.data_buffer) 
                    if result is not None:
                        einheit = UNIT_BASES.get(name, "")
                        if einheit: 
                            txt = format_value(result, einheit)
                            messwerte.append(f"{name}: {txt}")
                        else:
                            messwerte.append(f"{name}: {result:.3f}")

                    else:
                        messwerte.append(f"{name}: -")
                except Exception as e:
                    messwerte.append(f"{name}: Fehler!")

        self.finished.emit(messwerte)  # Ergebnisse zur GUI senden  

class StatusProvider(QObject):
    status_update = Signal(object)

    def __init__(self, status_queue):
        super().__init__()
        self.status_queue = status_queue
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        print("StatusProvider-Thread wurde gestartet")
        while self._running:
            try:
                msg = self.status_queue.get(timeout=0.1)
                print("StatusProvider: Sende Nachricht", msg)
                self.status_update.emit(msg)
            except queue.Empty:
                continue
            except Exception as e:
                print("Exception in StatusProvider:", e)
                break

class CustomStepSpinBox(QSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        # 1. Definieren der benutzerdefinierten Wertesequenz
        self._values_map = self._generate_value_map()

        # 2. Konfigurieren des internen Bereichs der SpinBox
        # Der interne Wert ist der Index in unserer _values_map
        self.setMinimum(0)
        self.setMaximum(len(self._values_map) - 1)
        self.setSingleStep(1) # Jeder Schritt bewegt sich zum nächsten Index

        # 3. Wichtig: Deaktiviert die Verfolgung der Tastatureingabe in Echtzeit,
        # damit valueFromText nur aufgerufen wird, wenn die Bearbeitung abgeschlossen ist.
        self.setKeyboardTracking(False)

        # Setze den Startwert auf den ersten Wert in der Liste
        self.setValue(len(self._values_map)-1) # Setzt den internen Index auf 0 (entspricht 1k)

    def _generate_value_map(self) -> list[int]:
        """Erzeugt die Liste der gewünschten Werte."""
        values = []

        # 1k, 2k, ..., 10k (Schritte von 1k)
        #values.extend(range(1, 10_001, 1_000))     #Die niedrigst mögliche Abtastfrequenz des ADCs ist 20 kHz

        # 20k, 30k, ..., 90k (Schritte von 10k)
        # Beginnen bei 20k, da 10k bereits enthalten ist
        values.extend(range(20_000, 90_001, 10_000))

        # 100k, 200k, ..., 900k (Schritte von 100k)
        values.extend(range(100_000, 900_001, 100_000))

        # 1M, 2M, ..., 10M (Schritte von 1M)
        values.extend(range(1_000_000, 10_000_001, 1_000_000))

        # Sicherstellen, dass alle Werte eindeutig sind und sortiert sind
        return sorted(list(set(values)))

    def _format_value(self, val: int) -> str:
        """Hilfsfunktion zum Formatieren eines numerischen Wertes in k/M-Notation."""
        if val >= 1_000_000:
            # Für Werte im Millionenbereich (1M, 2M, ...)
            # Zeigt "X.0M" als "XM" an, sonst "X.YM"
            if val % 1_000_000 == 0:
                return f"{val // 1_000_000} MHz"
            else:
                return f"{val / 1_000_000:.1f} MHz"
        elif val >= 1_000:
            # Für Werte im Tausenderbereich (1k, 2k, ...)
            # Zeigt "X.0k" als "Xk" an, sonst "X.Yk"
            if val % 1_000 == 0:
                return f"{val // 1_000} kHz"
            else:
                return f"{val / 1_000:.1f} kHz"
        else:
            # Sollte bei dieser Werteliste nicht vorkommen, da Minimum 1k ist
            return str(val)

    def textFromValue(self, index: int) -> str:
        """
        Konvertiert den internen Index in den anzuzeigenden Text.
        """
        # Sicherstellen, dass der Index im gültigen Bereich liegt
        if 0 <= index < len(self._values_map):
            actual_value = self._values_map[index]
            return self._format_value(actual_value)
        return "" # Leerer String bei ungültigem Index

    def valueFromText(self, text: str) -> int:
        """
        Konvertiert den vom Benutzer eingegebenen Text (z.B. "50k", "2M")
        zurück in den internen Index des nächstgelegenen Wertes in unserer Liste.
        """
        text = text.strip().replace(',', '.') # Komma durch Punkt für Dezimalzahlen ersetzen

        parsed_num = 0.0
        try:
            if text.endswith('k') or text.endswith('K'):
                parsed_num = float(text[:-1]) * 1_000
            elif text.endswith('m') or text.endswith('M'):
                parsed_num = float(text[:-1]) * 1_000_000
            else:
                parsed_num = float(text)
        except ValueError:
            # Bei ungültiger Eingabe den aktuellen internen Wert beibehalten
            return self.value()

        # Finden des Index des nächstgelegenen Wertes in _values_map
        # Wir durchlaufen die Liste und finden den Wert, der am nächsten an parsed_num liegt.
        # Bei Gleichstand wird aufgerundet (d.h. der höhere Wert gewählt).

        # Wenn der geparste Wert kleiner oder gleich dem ersten Wert ist, wähle den ersten Index.
        if parsed_num <= self._values_map[0]:
            return 0
        # Wenn der geparste Wert größer oder gleich dem letzten Wert ist, wähle den letzten Index.
        if parsed_num >= self._values_map[-1]:
            return len(self._values_map) - 1

        # Ansonsten, finde das Intervall, in das parsed_num fällt, und wähle den näheren Wert.
        for i in range(len(self._values_map) - 1):
            lower_val = self._values_map[i]
            upper_val = self._values_map[i+1]

            if lower_val <= parsed_num <= upper_val:
                # parsed_num liegt zwischen lower_val und upper_val
                # Wenn parsed_num näher an lower_val ist (oder genau in der Mitte, dann wird aufgerundet)
                if parsed_num - lower_val < upper_val - parsed_num:
                    return i # Näher an lower_val
                else:
                    return i + 1 # Näher an upper_val oder genau in der Mitte (dann upper_val)
        
        # Fallback (sollte bei korrekter Logik nicht erreicht werden)
        return self.value()

    def get_current_value(self) -> int:
        """Gibt den aktuell angezeigten numerischen Wert zurück (nicht den internen Index)."""
        current_index = self.value()
        if 0 <= current_index < len(self._values_map):
            return self._values_map[current_index]
        return 0 # Oder einen geeigneten Standardwert

    def set_value_from_external(self, external_val: int):
        """Setzt den Wert der SpinBox basierend auf einem externen numerischen Wert."""
        # Finde den Index des nächstgelegenen Wertes für external_val
        # Dies nutzt die gleiche Logik wie valueFromText, aber direkt mit einem numerischen Wert
        
        if external_val <= self._values_map[0]:
            self.setValue(0)
            return
        if external_val >= self._values_map[-1]:
            self.setValue(len(self._values_map) - 1)
            return

        for i in range(len(self._values_map) - 1):
            lower_val = self._values_map[i]
            upper_val = self._values_map[i+1]

            if lower_val <= external_val <= upper_val:
                if external_val - lower_val < upper_val - external_val:
                    self.setValue(i)
                else:
                    self.setValue(i + 1)
                return
        
        # Fallback, falls der Wert außerhalb des Bereichs liegt (sollte durch die ersten Checks abgefangen werden)
        self.setValue(0)


class Info(QWidget):
    def __init__(self):
            super().__init__()
            self.initUI()
        
    def initUI(self):
        layout = QVBoxLayout()
        self.info_label = QLabel("Here you can find some information about the GUI")
        layout.addWidget(self.info_label)
        layout.addStretch(1) # Fügt einen dehnbaren Leerraum hinzu
        self.setLayout(layout) # Das Layout dem Widget zuweisen


class General(QWidget):
    data_received_signal = Signal(object)
    status_update_signal = Signal(dict)

    def __init__(self, command_queue):
        super().__init__()
        self.command_queue = command_queue
        self.current_timebase = 1e-6
        self.current_timebase_idx = 0
        self.current_vdiv = 1
        self.current_vdiv_idx = 0
        self.current_yoffset = 0
        self.current_yoffset_idx = 0
        self.current_xoffset = 0
        self.current_xoffset_idx = 0
        self.max_samplerate = 10_000_000
        self.is_run_mode = False # Flag, um den Betriebsmodus (Single/Run) zu speichern

        self.data_buffer = []
        self.samplingrate = 10_000_000
        self.reference_voltage = 0
        self.num_xdiv = 10
        self.num_ydiv = 8

        self.timebase_values = [
            0.1e-6, 0.2e-6, 0.5e-6,   # 0.1, 0.2, 0.5 µs/div
            1e-6, 2e-6, 5e-6,         # 1, 2, 5 µs/div
            10e-6, 20e-6, 50e-6,      # 10, 20, 50 µs/div
            100e-6, 200e-6, 500e-6,   # 100, 200, 500 µs/div
            1e-3, 2e-3, 5e-3,         # 1, 2, 5 ms/div
            10e-3, 20e-3, 50e-3,      # 10, 20, 50 ms/div
            100e-3, 200e-3, 500e-3,   # 100, 200, 500 ms/div
            1.0                       # 1 s/div
        ]
        self.vdiv_values = [
            0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000
        ]  # in V/div, wie echte Oszilloskope

        # Variable zum Speichern des letzten FSM-Status
        self.last_fsm_state = "Idle"
        
        # Timer für die Statusanzeige
        self.status_clear_timer = QTimer(self)
        self.status_clear_timer.setSingleShot(True) # Timer läuft nur einmal ab
        self.status_clear_timer.timeout.connect(self.revert_status_label)

         # Dateiname für die Einstellungen
        self.settings_file = 'gui_settings.pkl'

        # Einstellungen beim Start der GUI laden
        self.load_settings()

        self.gui_fsm = UsbOsziGUI()
        self.matplotlib_widget = MatplotlibWidget()
        self.toolbar = NavigationToolbar(self.matplotlib_widget, self)

        self.initUI()

        self.data_received_signal.connect(self.process_and_plot_data)
        self.status_update_signal.connect(self.handle_fsm_status_update)
        
        self.update_gui_state()
    
    def initUI(self):
        """Layout der GUI"""

        self.setWindowTitle("Oszilloskop")

        self.trigger_combobox = QComboBox()
        self.trigger_combobox.addItems(["rising edge", "falling edge"])
        self.single = QPushButton("Single",self)
        self.run = QPushButton("Run", self)
        self.stop = QPushButton("Stop",self)
        self.reset = QPushButton("Reset", self)
        self.interp_combo = QComboBox()
        self.interp_combo.addItems(["linear","spline","points","steps"])
        self.status_label = QLabel("Status: Bereit")
        self.measure = QPushButton("Measure", self)
        self.toogle_crosshair_button = QPushButton("Cursor")
        self.delta_x_text = QLabel()
        self.delta_y_text = QLabel()
        self.measurement_results_label = QLabel()
        self.measurement_results_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        
        self.overflow_label = QLabel("Overflow")
        self.overflow_label.setAlignment(Qt.AlignCenter)
        self.overflow_label.setStyleSheet("background-color: lightgrey;")

        self.triggered_label = QLabel("Trig´d")
        self.triggered_label.setAlignment(Qt.AlignCenter)
        self.triggered_label.setStyleSheet("background-color: lightgrey;")


        self.toogle_crosshair_button.setStyleSheet("")
        self.run.setStyleSheet("")
        self.stop.setStyleSheet("")

        self.probe_combo = QComboBox()
        self.probe_combo.addItems(["1:1", "10:1"])

        self.select_measure_checkbox_group = QGroupBox('select measurement:', parent =self)
        self.checkbox_layout = QVBoxLayout() # Layout für die Checkboxen innerhalb der GroupBox

        self.measure_amplitude = QCheckBox('amplitude')
        self.measure_max = QCheckBox('max')
        self.measure_min = QCheckBox('min')
        self.measure_dach = QCheckBox('dach')
        self.measure_base = QCheckBox('base')
        self.measure_mean = QCheckBox('mean')
        self.measure_peak_peak = QCheckBox('peak-peak')
        self.measure_DCrms = QCheckBox('DC RMS')
        self.measure_ACrms = QCheckBox('AC RMS')
        self.measure_period = QCheckBox('period')
        self.measure_frequency = QCheckBox('frequency')

        self.checkbox_layout.addWidget(self.measure_amplitude)
        self.checkbox_layout.addWidget(self.measure_max)
        self.checkbox_layout.addWidget(self.measure_min)
        self.checkbox_layout.addWidget(self.measure_dach)
        self.checkbox_layout.addWidget(self.measure_base)
        self.checkbox_layout.addWidget(self.measure_mean)
        self.checkbox_layout.addWidget(self.measure_peak_peak)
        self.checkbox_layout.addWidget(self.measure_DCrms)
        self.checkbox_layout.addWidget(self.measure_ACrms)
        self.checkbox_layout.addWidget(self.measure_period)
        self.checkbox_layout.addWidget(self.measure_frequency)

        self.select_measure_checkbox_group.setLayout(self.checkbox_layout)
        self.select_measure_checkbox_group.setVisible(False)
       
        self.timebase_dial = QDial()
        self.timebase_dial.setMinimum(0)
        self.timebase_dial.setMaximum(len(self.timebase_values)-1)
        self.timebase_dial.setValue(self.current_timebase_idx)
        self.on_timebase_dial_change(self.current_timebase_idx)
        self.timebase_dial.setWrapping(False)
        self.timebase_dial.setFixedSize(120, 120)      # großer Dial

        self.vdiv_dial = QDial()
        self.vdiv_dial.setMinimum(0)
        self.vdiv_dial.setMaximum(len(self.vdiv_values)-1)
        self.vdiv_dial.setValue(self.current_vdiv_idx)  # Wert auf letzen Wert setzen oder Standardwert
        self.on_vdiv_dial_change(self.current_vdiv_idx)
        self.vdiv_dial.setWrapping(False)
        self.vdiv_dial.setFixedSize(120, 120)      # großer Dial

        self.yoffset_dial = QDial()
        self.yoffset_dial.setRange(-100,100)
        self.yoffset_dial.setMinimum(-100)
        self.yoffset_dial.setMaximum(100)
        self.yoffset_dial.setValue(self.current_yoffset_idx)
        self.on_yoffset_changed(self.current_yoffset_idx)
        self.yoffset_dial.setWrapping(False)
        self.yoffset_dial.setFixedSize(60, 60)      # großer Dial

        self.xoffset_dial = QDial()
        self.xoffset_dial.setRange(0,2000)
        self.xoffset_dial.setMinimum(0)
        self.xoffset_dial.setMaximum(2000)
        self.xoffset_dial.setValue(self.current_xoffset_idx)
        self.on_xoffset_changed(self.current_xoffset_idx)
        self.xoffset_dial.setWrapping(False)
        self.xoffset_dial.setFixedSize(60, 60)         # kleiner Dial

                # Die Differenz in der Höhe:
        diff = self.timebase_dial.height() - self.xoffset_dial.height()
        oben = diff // 2
        unten = diff - oben

        # Kleiner Dial vertikal mittig „aufpolstern“
        xoffset_wrapper = QWidget()
        xoffset_layout = QVBoxLayout()
        xoffset_layout.setContentsMargins(0, 0, 0, 0)
        if oben > 0:
            xoffset_layout.addSpacing(oben)
        xoffset_layout.addWidget(self.xoffset_dial)
        if unten > 0:
            xoffset_layout.addSpacing(unten)
        xoffset_wrapper.setLayout(xoffset_layout)

        # Horizontales Layout für x-Achse
        dial_row_widget_x = QWidget()
        dial_row_layout_x = QHBoxLayout()
        dial_row_layout_x.setContentsMargins(0, 0, 0, 0)
        dial_row_layout_x.setSpacing(10)  # Optional: Abstand zwischen den Dials
        dial_row_layout_x.addWidget(self.timebase_dial)
        dial_row_layout_x.addWidget(xoffset_wrapper)
        dial_row_widget_x.setLayout(dial_row_layout_x)

                # Die Differenz in der Höhe:
        diff = self.vdiv_dial.height() - self.yoffset_dial.height()
        oben = diff // 2
        unten = diff - oben

        # Kleiner Dial vertikal mittig „aufpolstern“
        yoffset_wrapper = QWidget()
        yoffset_layout = QVBoxLayout()
        yoffset_layout.setContentsMargins(0, 0, 0, 0)
        if oben > 0:
            yoffset_layout.addSpacing(oben)
        yoffset_layout.addWidget(self.yoffset_dial)
        if unten > 0:
            yoffset_layout.addSpacing(unten)
        yoffset_wrapper.setLayout(yoffset_layout)

        # Horizontales Layout für die y-Achse
        dial_row_widget_y = QWidget()
        dial_row_layout_y = QHBoxLayout()
        dial_row_layout_y.setContentsMargins(0, 0, 0, 0)
        dial_row_layout_y.setSpacing(10)
        dial_row_layout_y.addWidget(self.vdiv_dial)
        dial_row_layout_y.addWidget(yoffset_wrapper)
        dial_row_widget_y.setLayout(dial_row_layout_y)

        grid = QGridLayout()
        grid.addWidget(self.toolbar, 24, 0, 1, 17) 
        grid.addWidget(self.matplotlib_widget, 0, 0, 23, 19)
        grid.addWidget(self.overflow_label, 25, 0, 1,2)
        grid.addWidget(self.triggered_label,25, 2, 1,2)

        grid.addWidget(QLabel("timbase"), 0, 23)
        grid.addWidget(dial_row_widget_x, 1, 23, 3, 4)

        grid.addWidget(self.single, 0, 29)
        grid.addWidget(self.run, 0, 27)
        grid.addWidget(self.stop, 0, 28)
        grid.addWidget(self.reset, 25, 29)

        grid.addWidget(QLabel("measure"), 3, 27)
        grid.addWidget(self.measure, 5, 27)
        grid.addWidget(self.toogle_crosshair_button, 4, 27)
        grid.addWidget(self.delta_x_text, 4, 29)
        grid.addWidget(self.delta_y_text, 5, 29)
        grid.addWidget(self.select_measure_checkbox_group, 7, 23, 12,3)
        grid.addWidget(self.measurement_results_label, 7, 27, 12,2)


        grid.addWidget(QLabel("trigger typ"), 4, 23)
        grid.addWidget(self.trigger_combobox, 5, 23, 1, 2)

        grid.addWidget(QLabel("vertical"), 19,23)
        grid.addWidget(dial_row_widget_y, 20, 23, 3, 4)

        grid.addWidget(QLabel("interpolation"), 1, 27)
        grid.addWidget(self.interp_combo,2,27,1,2)

        grid.addWidget(QLabel("probe"), 24, 17)
        grid.addWidget(self.probe_combo, 25,17)

        grid.addWidget(QLabel("status label"), 24, 23)
        grid.addWidget(self.status_label, 25, 23, 1, 5)

        self.setLayout(grid)
        
        self.single.clicked.connect(self.on_single_clicked)
        self.run.clicked.connect(self.on_run_clicked)
        self.stop.clicked.connect(self.on_stop_clicked)
        self.reset.clicked.connect(self.on_reset_clicked)
        self.trigger_combobox.currentIndexChanged.connect(self.on_trigger_combobox_changed)
        self.measure.clicked.connect(self.on_measure_clicked)
        self.toogle_crosshair_button.clicked.connect(self.on_toogle_crosshair_button_clicked)

        self.measure_amplitude.stateChanged.connect(self.on_measure_changed)
        self.measure_max.stateChanged.connect(self.on_measure_changed)
        self.measure_min.stateChanged.connect(self.on_measure_changed)
        self.measure_dach.stateChanged.connect(self.on_measure_changed)
        self.measure_base.stateChanged.connect(self.on_measure_changed)
        self.measure_mean.stateChanged.connect(self.on_measure_changed)
        self.measure_peak_peak.stateChanged.connect(self.on_measure_changed)
        self.measure_DCrms.stateChanged.connect(self.on_measure_changed)
        self.measure_ACrms.stateChanged.connect(self.on_measure_changed)
        self.measure_period.stateChanged.connect(self.on_measure_changed)
        self.measure_frequency.stateChanged.connect(self.on_measure_changed)
        self.timebase_dial.valueChanged.connect(self.on_timebase_dial_change)
        self.vdiv_dial.valueChanged.connect(self.on_vdiv_dial_change)
        self.yoffset_dial.valueChanged.connect(self.on_yoffset_changed)
        self.xoffset_dial.valueChanged.connect(self.on_xoffset_changed)

        self.matplotlib_widget.cursor_diff_changed.connect(self.set_cursor_diff)


    def handle_status(self, msg):

                print(f"GUI received message from FSM: {msg}")
                if msg["type"] == "FSM_STATE":
                    self.status_update_signal.emit(msg)
                elif msg["type"] == "RAW_DATA":
                    self.data_buffer = msg["data"]
                    self.data_received_signal.emit(self.data_buffer)

                elif msg["type"] == "OVERFLOW":
                    self.overflow_label.setStyleSheet("color: black;" "background-color: #FF4E4E;")
                elif msg["type"] == "NO_OVERFLOW":
                    self.overflow_label.setStyleSheet("color:black;" "background-color: lightgrey;")

                elif msg["type"] == "TRIGGER_TIMEOUT":
                    self.triggered_label.setStyleSheet("color: black;" "background-color: lightgrey;")
                elif msg["type"] == "TRIGGERED":
                    self.triggered_label.setStyleSheet("color:black;" "background-color: limegreen;")

                elif msg["type"] == "INFO":
                    self.status_label.setText(f"Info: {msg['message']}")
                    self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
                elif msg["type"] == "GET_FREQUENCY":
                    str = msg["FREQUENCY"]
                    if str.startswith("SAMPLING_FREQUENCY:"):
                        freq_str = str.split(":")[1].strip()
                        self.samplingrate = int(freq_str)
                        self.status_label.setText(f"Info: sampling rate set to {self.samplingrate}.")
                        self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
                
                elif msg["type"] == "GET_REF_VOLTAGE":
                    str = msg["REF"]
                    if str.startswith("REFERENCE_VOLTAGE:"):
                        ref_str = str.split(":")[1].strip()
                        self.reference_voltage = int(ref_str)
                        self.status_label.setText(f"Info: reference voltage set to {self.reference_voltage}.")
                        self.status_clear_timer.start(3000) # 3 Sekunden anzeigen   

                elif msg["type"] == "ERROR":
                    self.status_label.setText(f"FEHLER: {msg['message']}")
                    #self.status_clear_timer.start(5000) # 5 Sekunden anzeigen
                    self.gui_fsm.reset() # GUI in Idle setzen bei Fehler
                    self.update_gui_state()
                    self.run.setStyleSheet("")
                    self.stop.setStyleSheet("")
                    self.toogle_crosshair_button.setStyleSheet("")
                

    @Slot(dict)
    def handle_fsm_status_update(self, status_message):
        fsm_state = status_message["state"]
        self.last_fsm_state = fsm_state # Letzten FSM-Status speichern

        # Nur aktualisieren, wenn keine temporäre Nachricht angezeigt wird
        if not self.status_clear_timer.isActive():
           self.status_label.setText(f"FSM Status: {fsm_state}")

        current_gui_fsm_state_id = self.gui_fsm.current_state.id

        # GUI FSM Zustandsübergänge basierend auf USB-FSM Status
        if fsm_state in ["acquiring_start", "acquiring_stopp", "transmission"]:
            # Wenn USB-FSM in einem Akquisitionszustand ist, sollte GUI-FSM acquiring sein.
            # Dies deckt Übergänge von idle -> acquiring (Start) oder processing -> acquiring (Continue) ab.
            if current_gui_fsm_state_id == self.gui_fsm.idle.id:
                self.gui_fsm.start_acquisition() 
            elif current_gui_fsm_state_id == self.gui_fsm.processing.id:
                # Dies bedeutet, dass der USB-FSM nach der Verarbeitung zu wait_uC zurückgekehrt ist,
                # also soll die Akquisition fortgesetzt werden.
                if self.is_run_mode:
                    self.gui_fsm.processing_done_continue()
                else:
                    self.gui_fsm.processing_done_stop()
                    self.stop.setStyleSheet("")

        elif fsm_state == "wait_for_ack":
            # Wenn USB-FSM progressing ist, sollte GUI-FSM processing sein.
            if current_gui_fsm_state_id == self.gui_fsm.acquiring.id:
                self.gui_fsm.data_received()

        elif fsm_state == "init":
            # Wenn USB-FSM idle ist, sollte GUI-FSM auch idle sein.
            # Dies deckt Übergänge von acquiring -> idle oder processing -> idle (Stop) ab.
            self.run.setStyleSheet("")
            self.stop.setStyleSheet("")
            self.matplotlib_widget.hide_crosshair()
            self.toogle_crosshair_button.setStyleSheet("")
            self.select_measure_checkbox_group.setVisible(False)
            self.measure.setStyleSheet("")
            
            if current_gui_fsm_state_id != self.gui_fsm.idle.id:
                self.gui_fsm.reset() # reset kann von jedem Zustand nach idle wechseln

        elif fsm_state == "idle":
            self.run.setStyleSheet("")
            self.single.setStyleSheet("")
            self.stop.setStyleSheet("color: black;" "background-color: #FF4E4E;")
            
            if current_gui_fsm_state_id != self.gui_fsm.idle.id:
                self.gui_fsm.reset() # reset kann von jedem Zustand nach idle wechseln

        self.update_gui_state()


    @Slot(object)
    def process_and_plot_data(self, data):
        print("GUI: Processing and plotting data...")
        plot_style = self.interp_combo.currentText()
        #Testsignal
        # freq = 100
        # ampli = 1
        # phase = 0
        # offset = 0
        # self.samplingrate = 1000
        # delta_time = 1/1000
        # duration = 3
        # #t = np.arange(0, duration+delta_time/2, delta_time)
        # t = np.arange(0, 100000/self.samplingrate, delta_time)
        # data = ampli*np.sin(2*np.pi*freq*t+phase)+offset

        selected_ratio = self.probe_combo.currentText()
        if (selected_ratio == "10:1"):
            data_corr = data.copy() * 10
        else:
            data_corr = data.copy()

        self.matplotlib_widget.plot_data(data_corr, self.samplingrate, plot_style)
        self.command_queue.put({
            "type": "PROCESSING_COMPLETE",
            "continue_acquisition": self.is_run_mode
        })
        self.on_measure_changed()
    

    def calc_optimal_sampling(self):
        duration = self.current_timebase * self.num_xdiv
        n_pixels = self.matplotlib_widget.size().width()
        overhead = 2
        fs_ideal = n_pixels * overhead / duration
        fs_ideal = min(fs_ideal, self.max_samplerate)
        return fs_ideal
    

    def send_sampling_to_uc(self, sampling_freq):
        self.command_queue.put({"type": "CONFIGURE_FREQ", "FREQ" : str(sampling_freq)})   


    def on_single_clicked(self):
        print("GUI: single button pressed.")
        if self.gui_fsm.current_state == self.gui_fsm.idle:
            self.is_run_mode = False # Setze Modus auf Single
            self.command_queue.put({"type": "START_ACQUISITION"})
            self.gui_fsm.start_acquisition()
            self.update_gui_state()
            self.single.setStyleSheet("color: black;" "background-color: lightblue;")
            self.stop.setStyleSheet("")
            self.status_label.setText("Status: single pressed: aquisition started.")
            self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
        else:
            print(f"GUI: Single-Button im Zustand {self.gui_fsm.current_state.id} nicht erlaubt.")


    def on_run_clicked(self):
        print("GUI: run button pressed.")
        if self.gui_fsm.current_state == self.gui_fsm.idle:
            self.is_run_mode = True # Setze Modus auf Run
            self.command_queue.put({"type": "START_ACQUISITION"})
            self.gui_fsm.start_acquisition()
            self.update_gui_state()
            self.run.setStyleSheet("color: black;" "background-color: limegreen;") 
            self.stop.setStyleSheet("")           
            self.status_label.setText("Status: run pressed: aquisition started.")
            self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
        else:
            print(f"GUI: Run-Button im Zustand {self.gui_fsm.current_state.id} nicht erlaubt.")


    def on_stop_clicked(self):
        print("GUI: stop button pressed.")
        # Stop kann (fast) immer geklickt werden
        self.is_run_mode = False
        self.command_queue.put({"type": "STOP_ACQUISITION"})         
        self.status_label.setText("Status: Acquisition stopped.")
        self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
    

    def on_reset_clicked(self):
        print("GUI: reset button pressed.")
        self.command_queue.put({"type": "RESET"})
        self.gui_fsm.reset()
        self.update_gui_state()
        self.status_label.setText("Status: application reset")
        self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
        for cb in [
            self.measure_amplitude, self.measure_max, self.measure_min, self.measure_dach, self.measure_base,
            self.measure_mean, self.measure_peak_peak, self.measure_DCrms, self.measure_ACrms,
            self.measure_period, self.measure_frequency
        ]:
            cb.setChecked(False)
        self.measurement_results_label.clear()
    

    def on_timebase_dial_change(self, idx):
        self.current_timebase_idx = idx
        self.current_timebase = self.timebase_values[idx]
        self.matplotlib_widget.update_axes_scaling(
                self.num_xdiv, self.num_ydiv,
                self.current_timebase, self.current_vdiv, self.current_xoffset, self.current_yoffset
            )


    def on_vdiv_dial_change(self, idx):
        self.current_vdiv_idx = idx
        self.current_vdiv = self.vdiv_values[idx]
        # Achsen & Grid neu setzen bei Änderung der Amplitude:
        self.matplotlib_widget.update_axes_scaling(
            self.num_xdiv, self.num_ydiv,
            self.current_timebase, self.current_vdiv, self.current_xoffset, self.current_yoffset
        )
        has_data = (
            isinstance(self.data_buffer, (list, np.ndarray)) and len(self.data_buffer) > 0
            and not (
                isinstance(self.data_buffer, np.ndarray)
                and self.data_buffer.size == 0
            )
        )
        if has_data:
            if self.num_ydiv*self.current_vdiv+self.current_yoffset*self.current_vdiv < maximum(self.data_buffer) or self.num_ydiv*self.current_vdiv+self.current_yoffset*self.current_vdiv < abs(minimum(self.data_buffer)):
                self.status_label.setText(f"Warning: Values ​​are outside the displayed range")
                self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
    
    def on_yoffset_changed(self, value):
        self.current_yoffset_idx = value
        self.current_yoffset = value/100*self.current_vdiv*self.num_ydiv/2
        self.matplotlib_widget.update_axes_scaling(self.num_xdiv, self.num_ydiv, self.current_timebase, self.current_vdiv, self.current_xoffset, self.current_yoffset)
        has_data = (
            isinstance(self.data_buffer, (list, np.ndarray)) and len(self.data_buffer) > 0
            and not (
                isinstance(self.data_buffer, np.ndarray)
                and self.data_buffer.size == 0
            )
        )
        if has_data:
            if self.num_ydiv*self.current_vdiv+self.current_yoffset*self.current_vdiv < maximum(self.data_buffer) or self.num_ydiv*self.current_vdiv+self.current_yoffset*self.current_vdiv < abs(minimum(self.data_buffer)):
                self.status_label.setText(f"Warning: Values ​​are outside the displayed range")
                self.status_clear_timer.start(3000) # 3 Sekunden anzeigen
    
    def on_xoffset_changed(self, value):
        self.current_xoffset_idx = value
        if len(self.data_buffer):
            laenge = len(self.data_buffer)
        else:
            laenge = 200_000
        self.current_xoffset = (1/self.samplingrate)*laenge*(value/2000)
        self.matplotlib_widget.update_axes_scaling(self.num_xdiv, self.num_ydiv, self.current_timebase, self.current_vdiv, self.current_xoffset, self.current_yoffset)


    def on_measure_clicked(self):
        if self.select_measure_checkbox_group.isVisible():
            self.select_measure_checkbox_group.setVisible(False)
            self.measure.setStyleSheet("")
        else:
            self.select_measure_checkbox_group.setVisible(True)
            self.measure.setStyleSheet("color: black;" "background-color: limegreen;")


    def on_trigger_combobox_changed(self):
        selected_text = self.trigger_combobox.currentText()
        print(f"GUI: Trigger combobox changed to: {selected_text}")
        # Nur senden, wenn die GUI-FSM im Idle-Zustand ist
        if self.gui_fsm.current_state == self.gui_fsm.idle:
            command_to_uc = ""
            if selected_text == "rising edge":
                command_to_uc = "CONFIGURE_TRIG_RISE_EDGE"
            elif selected_text == "falling edge":
                command_to_uc = "CONFIGURE_TRIG_FALL_EDGE"
            
            if command_to_uc:
                self.command_queue.put({"type": "SET_TRIGGER_EDGE", "edge": command_to_uc})

            else:
                print("GUI: Unknown trigger edge selected.")
        else:
            print(f"GUI: Cannot set trigger edge in current state: {self.gui_fsm.current_state.id}")
            self.status_label.setText(f"Warning: trigger can only set in idle state.")
            self.status_clear_timer.start(5000) # 5 Sekunden anzeigen
    

    def on_measure_changed(self):
        self.measure.setEnabled(False)
        if not hasattr(self, "data_buffer") or self.data_buffer is None or len(self.data_buffer) == 0:
            self.measurement_results_label.setText("Keine Daten")
            return
        if self.samplingrate:
            abtastzeit = 1/self.samplingrate
        else:
            print("Fehler beim Setzen der Abtastfrequenz")
        messwerte = []
        # Mapping Checkbox zu Funktion & Name
        checks = [
            (self.measure_amplitude,    ("Amplitude", amplitude)),
            (self.measure_max,          ("Max", maximum)),
            (self.measure_min,          ("Min", minimum)),
            (self.measure_dach,          ("Top", dach)),
            (self.measure_base,         ("Base", base)),
            (self.measure_mean,         ("Mittelwert", mean)),
            (self.measure_peak_peak,    ("Peak-Peak", peak_peak)),
            (self.measure_DCrms,        ("DC RMS", dc_rms)),
            (self.measure_ACrms,        ("AC RMS", ac_rms)),
            (self.measure_period,   ("Periode", lambda data: period(data, abtastzeit))),
            (self.measure_frequency,("Frequenz", lambda data: frequency(data, abtastzeit))),
        ]
        # Worker und Thread erzeugen
        self.worker = MeasureWorker(self.data_buffer, abtastzeit, checks)
        self.thread = QThread()
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.process)
        self.worker.finished.connect(self.show_results)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()
    
    @Slot(list)
    def show_results(self, messwerte):
        self.measurement_results_label.setText("\n\n".join(messwerte))
        self.measure.setEnabled(True)
    

    def on_toogle_crosshair_button_clicked(self):
        # Sicher prüfen, ob Daten vorhanden sind (egal ob None, list oder np.array)
        has_data = (
            isinstance(self.data_buffer, (list, np.ndarray)) and len(self.data_buffer) > 0
            and not (
                isinstance(self.data_buffer, np.ndarray)
                and self.data_buffer.size == 0
            )
        )
        if has_data:
            if self.matplotlib_widget.crosshair1_visible:
                self.matplotlib_widget.hide_crosshair()
                self.toogle_crosshair_button.setStyleSheet("")
            else:
                self.matplotlib_widget.show_crosshair()
                self.toogle_crosshair_button.setStyleSheet("color: black;" "background-color: limegreen;")

        else:
            self.status_label.setText("Info: Cursors can only be used if data are available")
            self.status_clear_timer.start(3000) # 3 Sekunden anzeigen

    @Slot(float, float)
    def set_cursor_diff(self, dx, dy):
        text_x = f"Δx = {format_value(dx, "s")}"
        text_y = f"Δy = {format_value(dy, "V")}"
        self.delta_x_text.setText(text_x)
        self.delta_y_text.setText(text_y)
       


    @Slot() #Slot für den Timer
    def revert_status_label(self):
        """Setzt den Status-Label auf den letzten bekannten FSM-Status zurück."""
        self.status_label.setText(f"FSM Status: {self.last_fsm_state}")
        self.status_clear_timer.stop() # Timer stoppen, da er SingleShot ist

    def update_gui_state(self):
        # Steuert die Aktivierung/Deaktivierung von Buttons basierend auf dem GUI-FSM-Zustand
        current_gui_state = self.gui_fsm.current_state

        self.single.setEnabled(current_gui_state == self.gui_fsm.idle)
        self.run.setEnabled(current_gui_state == self.gui_fsm.idle)
        self.stop.setEnabled(current_gui_state in [self.gui_fsm.acquiring, self.gui_fsm.processing])
        
        print(f"GUI State: {current_gui_state.id}")

    def stop_my_timer(self):
         print("Timer gestoppt")
         self.status_clear_timer.stop()
    
    def save_settings(self):
        """Speichert die aktuellen GUI-Einstellungen in einer Datei."""
        settings = {
            'vdiv_idx': self.current_vdiv_idx,
            'timebase_idx': self.current_timebase_idx,
            'xoffset_idx': self.current_xoffset_idx,
            'yoffset_idx': self.current_yoffset_idx
        }
        try:
            # 'wb' steht für 'write binary'
            with open(self.settings_file, 'wb') as f:
                pickle.dump(settings, f)
            print(f"Einstellungen erfolgreich in '{self.settings_file}' gespeichert.")
        except Exception as e:
            print(f"Fehler beim Speichern der Einstellungen: {e}")

    def load_settings(self):
        """Lädt die GUI-Einstellungen aus einer Datei."""
        if os.path.exists(self.settings_file):
            try:
                # 'rb' steht für 'read binary'
                with open(self.settings_file, 'rb') as f:
                    settings = pickle.load(f)
                # Die .get()-Methode wird verwendet, um Standardwerte zu setzen,
                # falls ein Schlüssel in der geladenen Datei fehlt.
                self.current_vdiv_idx = settings.get('vdiv_idx', self.current_vdiv_idx)
                self.current_vdiv = self.vdiv_values[self.current_vdiv_idx]
                self.current_timebase_idx = settings.get('timebase_idx', self.current_timebase_idx)
                self.current_timebase = self.timebase_values[self.current_timebase_idx]

                self.current_xoffset_idx = settings.get('xoffset_idx', self.current_xoffset_idx)
                self.current_yoffset_idx = settings.get('yoffset_idx', self.current_yoffset_idx)
        
                print(f"Einstellungen erfolgreich aus '{self.settings_file}' geladen.")
            except Exception as e:
                print(f"Fehler beim Laden der Einstellungen (Datei möglicherweise beschädigt): {e}. Verwende Standardwerte.")
                # Löschen Sie die beschädigte Datei, damit beim nächsten Start eine neue Datei mit Standardwerten erstellt wird.
                os.remove(self.settings_file)
        else:
            print(f"Einstellungsdatei '{self.settings_file}' nicht gefunden. Verwende Standardwerte.")


class Debugging(QWidget):
    def __init__(self, command_queue):
        super().__init__()
        self.command_queue = command_queue
        self.current_samplingfrequency = 10_000_000

        self.initUI()
    
    def initUI(self):
        layout = QGridLayout()
        self.samplingfrequency_spinbox = CustomStepSpinBox()

        self.reference_voltage_spinbox = QSpinBox()
        self.reference_voltage_spinbox.setMinimum(0)
        self.reference_voltage_spinbox.setMaximum(4095)
        self.reference_voltage_spinbox.setSingleStep(1)
        self.reference_voltage_spinbox.setKeyboardTracking(False)
        self.reference_voltage_spinbox.setValue(4095)
        self.reference_voltage_spinbox.setSuffix(" mV")

        self.actual_samplingfrequency = QLabel()
        
        self.transmission_time_label = QLabel()
        self.conversion_time_label = QLabel()

        layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        layout.addWidget(QLabel("sampling frequency"),0,0)
        layout.addWidget(self.samplingfrequency_spinbox, 1,0)
        layout.addWidget(self.actual_samplingfrequency, 2,0)
        layout.setHorizontalSpacing(50)
        layout.addWidget(QLabel("reference voltage"), 0, 1)
        layout.addWidget(self.reference_voltage_spinbox, 1, 1)
        layout.addWidget(QLabel("timing data"), 4, 0)
        layout.addWidget(QLabel("transmission time"), 5,0)
        layout.addWidget(self.transmission_time_label, 5,1)
        layout.addWidget(QLabel("conversion time"), 6, 0)
        layout.addWidget(self.conversion_time_label, 6,1)
        self.setLayout(layout) # Das Layout dem Widget zuweisen

        self.samplingfrequency_spinbox.valueChanged.connect(self.on_samplingfrequency_spinbox_changed)
        self.reference_voltage_spinbox.valueChanged.connect(self.on_reference_voltage_spinbox_changed)
    
    def send_sampling_to_uc(self, sampling_freq):
        self.command_queue.put({"type": "CONFIGURE_FREQ", "FREQ" : str(sampling_freq)})  

    def send_reference_voltage_to_uc(self, reference_voltage):
        self.command_queue.put({"type": "CONFIGURE_REF_VOLT", "REF" : str(reference_voltage)})
    
    def on_samplingfrequency_spinbox_changed(self,idx):
        self.samplingfrequency_spinbox.textFromValue(idx)
        value = self.samplingfrequency_spinbox.get_current_value()
        self.send_sampling_to_uc(value)
    
    def on_reference_voltage_spinbox_changed(self):
        ref_value = self.reference_voltage_spinbox.value()
        self.send_reference_voltage_to_uc(ref_value)
    
    
    def handle_status(self, msg):
        
        print(f"Debugging received message from FSM: {msg}")
        if msg["type"] == "Transmission_Time":
            #self.status_label.setText(f"Info: {msg['message']}")
            self.transmission_time_label.setText(f"{msg['time']} s")
        elif msg["type"] == "Conversion_Time":
            self.conversion_time_label.setText(f"{msg['time']} s")
        elif msg["type"] == "GET_FREQUENCY":
            str = msg["FREQUENCY"]
            if str.startswith("SAMPLING_FREQUENCY:"):
                freq_str = str.split(":")[1].strip()
                self.actual_samplingfrequency.setText(freq_str+" Hz")           
                    
        
class StackedWidgetApp(QMainWindow):
    windowClosing = Signal()
    def __init__(self, stop_event, status_queue):
        super().__init__()
        self.status_provider = StatusProvider(status_queue)
        self.thread = threading.Thread(target=self.status_provider.run, daemon=True)
        self.thread.start()
        self.stop_event = stop_event
        self.initUI()

    def initUI(self):
        self.setWindowTitle("USB oscilloscope")
        self.setGeometry(100, 100, 1500, 900)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        nav_layout = QVBoxLayout()
        self.btn_page1 = QPushButton("General")
        self.btn_page2 = QPushButton("Info")
        self.btn_page3 = QPushButton("Debugging")

        nav_layout.addWidget(self.btn_page1)
        nav_layout.addWidget(self.btn_page2)
        nav_layout.addWidget(self.btn_page3)
        nav_layout.addStretch(1)

        self.stacked_widget = QStackedWidget()

        self.page1_instance = General(command_queue)
        self.page2_instance = Info()
        self.page3_instance = Debugging(command_queue)
        self.windowClosing.connect(self.page1_instance.save_settings)
        self.windowClosing.connect(self.page1_instance.stop_my_timer)
        self.status_provider.status_update.connect(self.page1_instance.handle_status)
        self.status_provider.status_update.connect(self.page3_instance.handle_status)


        # Seiten zum StackedWidget hinzufügen
        self.stacked_widget.addWidget(self.page1_instance) # Index 0
        self.stacked_widget.addWidget(self.page2_instance) # Index 1
        self.stacked_widget.addWidget(self.page3_instance) # Index 2

        # Verbindungen für die Buttons
        self.btn_page1.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(0))
        self.btn_page2.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))
        self.btn_page3.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(2))

        # Layouts zum Hauptlayout hinzufügen
        main_layout.addLayout(nav_layout)
        main_layout.addWidget(self.stacked_widget)
        main_layout.setStretch(1, 1) # StackedWidget nimmt den restlichen Platz ein

        self.setLayout(main_layout)


    def closeEvent(self, event):
        print("GUI wird geschlossen, FSM-Thread wird gestoppt...")
        self.windowClosing.emit()
        self.status_provider.stop()
        self.thread.join()
        self.stop_event.set() # Signalisiert dem USB-FSM-Thread das Beenden
        event.accept()


if __name__ == "__main__":
    # Queues für die Kommunikation zwischen Threads
    command_queue = queue.Queue() # GUI -> USB-FSM
    status_queue = queue.Queue()  # USB-FSM -> GUI_General
    stop_event = threading.Event()

    # 1. USB-Kommunikations-FSM in einem separaten Thread starten
    fsm_thread = threading.Thread(
        target=fsm_pc,
        args=(command_queue, status_queue, stop_event),
        daemon=True # Stellt sicher, dass der Thread beendet wird, wenn das Hauptprogramm endet
    )
    fsm_thread.start()

    # 2. GUI im Hauptthread starten
    app = QApplication(sys.argv)
    ex = StackedWidgetApp(stop_event, status_queue)
    ex.show()

    # 3. GUI-Event-Loop starten
    sys.exit(app.exec())

    # Dieser Teil wird erst erreicht, wenn die GUI geschlossen wird
    # Warten auf den FSM-Thread, um sauber zu beenden
    fsm_thread.join(timeout=2)
    if fsm_thread.is_alive():
        print("FSM thread did not terminate gracefully.")
    else:
        print("FSM thread terminated.")
    print("Programm wurde sauber beendet.")