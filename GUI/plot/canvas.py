import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

class Canvas(FigureCanvasQTAgg):
    def __init__(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(nrows=2, sharex=True)
        super().__init__(self.fig)
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.plot_lines_ax1 = {}
        self.plot_lines_ax2 = {}
        self.fig.tight_layout()

    def add_data_ax1(self, x, y, label, style):
        if label in self.plot_lines_ax1:
            line = self.plot_lines_ax1[label]
            line.set_xdata(np.append(line.get_xdata(), x))
            line.set_ydata(np.append(line.get_ydata(), y))
        else:
            self.plot_lines_ax1[label], = self.ax1.plot(x, y, style, label=label)
            self.ax1.legend()
        self.ax1.relim(); self.ax1.autoscale_view(True, True, True); self.draw_idle()

    def add_data_ax2(self, x, y, label, style):
        if label in self.plot_lines_ax2:
            line = self.plot_lines_ax2[label]
            line.set_xdata(np.append(line.get_xdata(), x))
            line.set_ydata(np.append(line.get_ydata(), y))
        else:
            self.plot_lines_ax2[label], = self.ax2.plot(x, y, style, label=label)
            self.ax2.legend()

    def draw_ax2(self):
        self.ax2.relim(); self.ax2.autoscale_view(True, True, True); self.draw_idle()

    def clear_plots(self):
        self.ax1.clear(); self.ax2.clear()
        self.ax1.grid(True); self.ax2.grid(True)
        self.plot_lines_ax1.clear(); self.plot_lines_ax2.clear()
        self.ax1.legend(); self.ax2.legend()
        self.draw_idle()
        print("Plots cleared")

    def save_only_second_plot(self, filename):
        self.ax1.set_visible(False)
        self.fig.tight_layout()
        try:
            self.fig.savefig(filename, bbox_inches='tight')
        except ValueError as e:
            print(f"Error saving plot: {e}")
        self.ax1.set_visible(True)
        self.draw_idle()
        self.fig.tight_layout()
        print(f"Second plot saved as {filename}")
