import tkinter as tk
from tkinter import ttk
from controls import PIDPhi, PDPhi, PIDPhiInterpolated, Lyapunov, FuzzyLyapunov
import metrics

import matplotlib

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from graphs import generate_figure
from simulation import simulate
from bikemodel import MeijaardModel
from graphs import plot_params

matplotlib.use("TkAgg")
LARGE_FONT = ("Verdana", 12)


class AutocycleGUI(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.iconbitmap(self, default='autocycle_4yC_icon.ico')
        tk.Tk.wm_title(self, "Autocycle Simulator")

        container = tk.Frame(self)

        container.pack(side="top", fill="both", expand=True)

        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        for F in (StartPage, GraphPage, InfoPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky='nsew')

        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()


class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = ttk.Label(self, text="Welcome to the Autocycle Simulator!", font=LARGE_FONT)
        label.pack(pady=10, padx=10)
        button = ttk.Button(self, text="Make a Graph",
                            command=lambda: controller.show_frame(GraphPage))
        button.pack()

        button2 = ttk.Button(self, text="More Info",
                             command=lambda: controller.show_frame(InfoPage))
        button2.pack()


class GraphPage(tk.Frame):

    def __init__(self, parent, controller):
        self.controller = controller
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Create a Graph", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        parampacker = tk.Frame(self)

        phi = tk.Frame(parampacker, self)
        self.phivalue = tk.Entry(phi)
        self.phivalue.insert(tk.END, '10')
        defaultphi = float(self.phivalue.get())
        self.phivalue.pack(padx=5, pady=5, side=tk.RIGHT)
        philabel = tk.Label(phi, text="Phi(degrees):")
        philabel.pack(padx=5, pady=5, side=tk.RIGHT)
        phi.pack(side=tk.LEFT)

        phidel = tk.Frame(parampacker, self)
        self.phidelvalue = tk.Entry(phidel)
        self.phidelvalue.insert(tk.END, '0')
        defaultphidel = float(self.phidelvalue.get())
        self.phidelvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        phidellabel = tk.Label(phidel, text="Derivative of Phi (degrees):")
        phidellabel.pack(padx=5, pady=5, side=tk.RIGHT)
        phidel.pack(side=tk.LEFT)

        delta = tk.Frame(parampacker, self)
        self.deltavalue = tk.Entry(delta)
        self.deltavalue.insert(tk.END, '0')
        defaultdelta = float(self.deltavalue.get())
        self.deltavalue.pack(padx=5, pady=5, side=tk.RIGHT)
        deltalabel = tk.Label(delta, text="Delta (degrees):")
        deltalabel.pack(padx=5, pady=5, side=tk.RIGHT)
        delta.pack(side=tk.LEFT)

        deltadel = tk.Frame(parampacker, self)
        self.deltadelvalue = tk.Entry(deltadel)
        self.deltadelvalue.insert(tk.END, '0')
        defaultdeltadel = float(self.deltadelvalue.get())
        self.deltadelvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        deltadellabel = tk.Label(deltadel, text="Derivative of Delta (degrees):")
        deltadellabel.pack(padx=5, pady=5, side=tk.RIGHT)
        deltadel.pack(side=tk.LEFT)

        parampacker.pack()

        parampacker1 = tk.Frame(self)

        timespan = tk.Frame(parampacker1, self)
        self.timespanvalue = tk.Entry(timespan)
        self.timespanvalue.insert(tk.END, '60')
        defaulttimespan = float(self.timespanvalue.get())
        self.timespanvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        timespanlabel = tk.Label(timespan, text="Timespan (seconds):")
        timespanlabel.pack(padx=5, pady=5, side=tk.RIGHT)
        timespan.pack(side=tk.LEFT)

        vel = tk.Frame(parampacker1, self)
        self.velvalue = tk.Entry(vel)
        self.velvalue.insert(tk.END, '5.5')
        defaultvel = float(self.velvalue.get())
        self.velvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        vellabel = tk.Label(vel, text="Velocity in m/s:")
        vellabel.pack(padx=5, pady=5, side=tk.RIGHT)
        vel.pack(side=tk.LEFT)

        parampacker1.pack()

        self.controldict = {
            1: None,
            2: PDPhi(k_p=5, k_d=8),
            3: PIDPhi(k_p=5, k_i=.0000001, k_d=8),
            4: PIDPhiInterpolated(0, 0, 0),
            5: Lyapunov(E3=.1),
            6: FuzzyLyapunov(np=5.3497, z=2.5390, npd=0.0861, zd=.4162, E1=1.5743, E3=.0064)
        }

        self.titledict = {
            1: "Uncontrolled",
            2: "PD Controlled",
            3: "PID Controlled",
            4: "PID Interpolated Controlled",
            5: "Lyapunov Controlled",
            6: "Fuzzy Lyapunov Controlled"
        }

        defaultcontrol = None
        defaultperturb = None

        self.create_plot(defaultphi, defaultdelta, defaultphidel, defaultdeltadel, defaulttimespan, defaultvel,
                         defaultcontrol, defaultperturb)

    def create_plot(self, defaultphi, defaultdelta, defaultphidel, defaultdeltadel, defaulttimespan, defaultvel,
                    defaultcontrol, defaultperturb):
        self.v = tk.IntVar(None, 1)
        self.controlbuttons = tk.Frame(self)

        self.nocontrol = tk.Radiobutton(self.controlbuttons,
                                        text="Uncontrolled",
                                        padx=20,
                                        variable=self.v,
                                        value=1
                                        ).pack(side=tk.TOP)
        self.pdcontrol = tk.Radiobutton(self.controlbuttons,
                                        text="PD Controller",
                                        padx=20,
                                        variable=self.v,
                                        value=2).pack(side=tk.TOP)
        self.pidcontrol = tk.Radiobutton(self.controlbuttons,
                                         text="PID Controller",
                                         padx=20,
                                         variable=self.v,
                                         value=3).pack(side=tk.TOP)
        self.pidIntercontrol = tk.Radiobutton(self.controlbuttons,
                                              text="PID Interpolated Controlled",
                                              padx=20,
                                              variable=self.v,
                                              value=4).pack(side=tk.TOP)
        self.lyapunov = tk.Radiobutton(self.controlbuttons,
                                       text="Lyapunov Controlled",
                                       padx=20,
                                       variable=self.v,
                                       value=5).pack(side=tk.TOP)
        self.fuzzylyapunov = tk.Radiobutton(self.controlbuttons,
                                            text="Fuzzy Lyapunov Controlled",
                                            padx=20,
                                            variable=self.v,
                                            value=6).pack(side=tk.TOP)
        self.controlbuttons.pack(side=tk.RIGHT)

        self.model = MeijaardModel()
        results = simulate(self.model, (defaultphi, defaultdelta, defaultphidel, defaultdeltadel), defaulttimespan,
                           defaultvel, control_method=defaultcontrol, perturbation=defaultperturb)
        st = float(metrics.settling_time(results['t'], results['phi'], 0))
        sth = float(metrics.settling_threshold(results['t'], results['phi'], 0))
        osv, ost = metrics.overshoot(results['t'], results['phi'], 0)
        osv, ost = float(osv), float(ost)

        self.f = generate_figure('Uncontrolled Bicycle at %.2f m/s' % 5.5, (results['t'], 'Time (seconds)'),
                                 (results['phi'], 'Phi (degrees)'), (results['delta'], 'Delta (degrees)'),
                                 (st, "Settling time: %.2f" % st, 'v'), (sth, "Settling threshold: %.2f" % sth, 'ph'),
                                 (ost, "Maximum Overshoot: %.2f" % osv, 'v', osv))

        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.button3 = tk.Button(self, text="Update Graph", command=lambda:
        self.update_plot(float(self.phivalue.get()), float(self.deltavalue.get()), \
                         float(self.phidelvalue.get()), float(self.deltadelvalue.get()), \
                         float(self.timespanvalue.get()), float(self.velvalue.get()), self.controldict[self.v.get()],
                         defaultperturb))
        self.button1 = tk.Button(self, text="Back to Home",
                                 command=lambda: self.controller.show_frame(StartPage))
        self.button1.pack(side=tk.BOTTOM)
        self.button3.pack(side=tk.BOTTOM)

        self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        self.toolbar.update()

    def update_plot(self, newphi, newdelta, newphidel, newdeltadel, newtimespan, newvelvalue, newcontrol,
                    defaultperturb):
        self.controlbuttons.pack_forget()
        self.controlbuttons = tk.Frame(self)

        self.nocontrol = tk.Radiobutton(self.controlbuttons,
                                        text="Uncontrolled",
                                        padx=20,
                                        variable=self.v,
                                        value=1
                                        ).pack(side=tk.TOP)
        self.pdcontrol = tk.Radiobutton(self.controlbuttons,
                                        text="PD Controller",
                                        padx=20,
                                        variable=self.v,
                                        value=2).pack(side=tk.TOP)
        self.pidcontrol = tk.Radiobutton(self.controlbuttons,
                                         text="PID Controller",
                                         padx=20,
                                         variable=self.v,
                                         value=3).pack(side=tk.TOP)
        self.pidIntercontrol = tk.Radiobutton(self.controlbuttons,
                                              text="PID Interpolated Controlled",
                                              padx=20,
                                              variable=self.v,
                                              value=4).pack(side=tk.TOP)
        self.lyapunov = tk.Radiobutton(self.controlbuttons,
                                       text="Lyapunov Controlled",
                                       padx=20,
                                       variable=self.v,
                                       value=5).pack(side=tk.TOP)
        self.fuzzylyapunov = tk.Radiobutton(self.controlbuttons,
                                            text="Fuzzy Lyapunov Controlled",
                                            padx=20,
                                            variable=self.v,
                                            value=6).pack(side=tk.TOP)
        self.controlbuttons.pack(side=tk.RIGHT)

        self.canvas.get_tk_widget().pack_forget()
        self.canvas._tkcanvas.pack_forget()
        self.button3.pack_forget()
        self.button1.pack_forget()
        self.toolbar.pack_forget()
        self.button1 = tk.Button(self, text="Back to Home",
                                 command=lambda: self.controller.show_frame(StartPage))
        self.button1.pack(side=tk.BOTTOM)
        self.button3 = tk.Button(self, text="Update Graph", command=lambda:
        self.update_plot(float(self.phivalue.get()), float(self.deltavalue.get()), \
                         float(self.phidelvalue.get()), float(self.deltadelvalue.get()), \
                         float(self.timespanvalue.get()), float(self.velvalue.get()), self.controldict[self.v.get()],
                         defaultperturb))
        self.button3.pack(side=tk.BOTTOM)

        results = simulate(self.model, [newphi, newdelta, newphidel, newdeltadel], newtimespan, newvelvalue,
                           control_method=newcontrol, perturbation=defaultperturb)
        st = float(metrics.settling_time(results['t'], results['phi'], 0))
        sth = float(metrics.settling_threshold(results['t'], results['phi'], 0))
        osv, ost = metrics.overshoot(results['t'], results['phi'], 0)
        osv, ost = float(osv), float(ost)

        self.f.clear()
        self.f = generate_figure('%s Bicycle at %.2f m/s' % (self.titledict[self.v.get()], newvelvalue),
                                 (results['t'], 'Time (seconds)'),
                                 (results['phi'], 'Phi (degrees)'), (results['delta'], 'Delta (degrees)'),
                                 (st, "Settling time: %.2f" % st, 'v'), (sth, "Settling threshold: %.2f" % sth, 'ph'),
                                 (ost, "Maximum Overshoot: %.2f" % osv, 'v', osv))
        self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        self.toolbar.update()
        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.canvas._tkcanvas.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)


class InfoPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        infotext = "This application visualizes the motion of a bicycle over time.\n Utilizing a variety of starting" + \
                   " parameters and control schemes, the roll, yaw, and other parameters of a bicycle may be visualized over" + \
                   " a span of time.\n Currently, these results may be seen in graph form only.\n In the next update, you will" + \
                   " be able to watch an animation of the bike in motion.\n For more information, please visit www.autocycle.io."

        label = tk.Label(self, text=infotext, font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = tk.Button(self, text="Back to Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()


if __name__ == '__main__':
    app = AutocycleGUI()
    app.mainloop()
