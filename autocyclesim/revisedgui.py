import tkinter as tk
from tkinter import ttk

LARGE_FONT = ("Verdana", 12)
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from graphs import generate_figure
from simulation import simulate
from bikemodel import MeijaardModel
from graphs import plot_params


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
        self.phivalue.insert(tk.END, '1')
        defaultphi = float(self.phivalue.get())
        self.phivalue.pack(padx=5, pady=5, side=tk.RIGHT)
        philabel = tk.Label(phi, text="Phi in Radians:")
        philabel.pack(padx=5, pady=5, side=tk.RIGHT)
        phi.pack(side=tk.LEFT)

        phidel = tk.Frame(parampacker, self)
        self.phidelvalue = tk.Entry(phidel)
        self.phidelvalue.insert(tk.END, '0')
        defaultphidel = float(self.phidelvalue.get())
        self.phidelvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        phidellabel = tk.Label(phidel, text="Derivative of Phi in Radians:")
        phidellabel.pack(padx=5, pady=5, side=tk.RIGHT)
        phidel.pack(side=tk.LEFT)

        delta = tk.Frame(parampacker, self)
        self.deltavalue = tk.Entry(delta)
        self.deltavalue.insert(tk.END, '0')
        defaultdelta = float(self.deltavalue.get())
        self.deltavalue.pack(padx=5, pady=5, side=tk.RIGHT)
        deltalabel = tk.Label(delta, text="Delta in Radians:")
        deltalabel.pack(padx=5, pady=5, side=tk.RIGHT)
        delta.pack(side=tk.LEFT)

        deltadel = tk.Frame(parampacker, self)
        self.deltadelvalue = tk.Entry(deltadel)
        self.deltadelvalue.insert(tk.END, '0')
        defaultdeltadel = float(self.deltadelvalue.get())
        self.deltadelvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        deltadellabel = tk.Label(deltadel, text="Derivative of Delta in Radians:")
        deltadellabel.pack(padx=5, pady=5, side=tk.RIGHT)
        deltadel.pack(side=tk.LEFT)

        parampacker.pack()

        parampacker1 = tk.Frame(self)

        timespan = tk.Frame(parampacker1, self)
        self.timespanvalue = tk.Entry(timespan)
        self.timespanvalue.insert(tk.END, '60')
        defaulttimespan = float(self.timespanvalue.get())
        self.timespanvalue.pack(padx=5, pady=5, side=tk.RIGHT)
        timespanlabel = tk.Label(timespan, text="Timespan in seconds:")
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

        self.create_plot(defaultphi, defaultdelta, defaultphidel, defaultdeltadel, defaulttimespan, defaultvel)

    def create_plot(self, defaultphi, defaultdelta, defaultphidel, defaultdeltadel, defaulttimespan, defaultvel):
        self.model = MeijaardModel()
        results = simulate(self.model, (defaultphi, defaultdelta, defaultphidel, defaultdeltadel), defaulttimespan,
                           defaultvel)

        self.f = generate_figure('Uncontrolled Bicycle at %.2f m/s' % 5.5, (results['t'], 'Time (seconds)'),
                                 (results['phi'], 'Phi (radians)'), (results['delta'], 'Delta (radians)'))

        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2TkAgg(self.canvas, self)
        toolbar.update()
        self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.button3 = tk.Button(self, text="Update Graph", command=lambda:
        self.update_plot(float(self.phivalue.get()), float(self.deltavalue.get()), \
                         float(self.phidelvalue.get()), float(self.deltadelvalue.get()), \
                         float(self.timespanvalue.get()), float(self.velvalue.get())))
        self.button3.pack()

        self.button1 = tk.Button(self, text="Back to Home",
                                 command=lambda: self.controller.show_frame(StartPage))
        self.button1.pack()

    def update_plot(self, newphi, newdelta, newphidel, newdeltadel, newtimespan, newvelvalue):
        self.canvas.get_tk_widget().pack_forget()
        self.canvas._tkcanvas.pack_forget()
        self.button3.pack_forget()
        self.button1.pack_forget()
        self.button1 = tk.Button(self, text="Back to Home",
                                 command=lambda: self.controller.show_frame(StartPage))
        self.button1.pack(side=tk.BOTTOM)
        self.button3 = tk.Button(self, text="Update Graph", command=lambda:
        self.update_plot(float(self.phivalue.get()), float(self.deltavalue.get()), \
                         float(self.phidelvalue.get()), float(self.deltadelvalue.get()), \
                         float(self.timespanvalue.get()), float(self.velvalue.get())))
        self.button3.pack(side=tk.BOTTOM)

        print('%.2f %.2f %.2f %.2f' % (newphi, newdelta, newphidel, newdeltadel))
        results = simulate(self.model, [newphi, newdelta, newphidel, newdeltadel], newtimespan, newvelvalue)
        self.f = generate_figure('Uncontrolled Bicycle at %.2f m/s' % newvelvalue, (results['t'], 'Time (seconds)'),
                                 (results['phi'], 'Phi (radians)'), (results['delta'], 'Delta (radians)'))
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


app = AutocycleGUI()
app.mainloop()
