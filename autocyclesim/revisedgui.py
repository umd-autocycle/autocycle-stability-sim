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

        for F in (StartPage, PageOne, PageTwo, PageThree):
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
                           command=lambda: controller.show_frame(PageOne))
        button.pack()

        button2 = ttk.Button(self, text="More Info",
                            command=lambda: controller.show_frame(PageTwo))
        button2.pack()

class PageOne(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Create a Graph", font=LARGE_FONT)
        label.pack(pady=10,padx=10)

        button3 = tk.Button(self, text = "See your graph",
                            command = lambda: controller.show_frame(PageThree))
        button3.pack()

        button1 = tk.Button(self, text="Back to Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()

        button2 = tk.Button(self, text="More Info",
                            command=lambda: controller.show_frame(PageTwo))
        button2.pack()


class PageTwo(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page Two!!!", font=LARGE_FONT)
        label.pack(pady=10,padx=10)

        button1 = tk.Button(self, text="Back to Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()

        button2 = tk.Button(self, text="Page One",
                            command=lambda: controller.show_frame(PageOne))
        button2.pack()

class PageThree(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        button1 = ttk.Button(self, text="Back to Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()

        model = MeijaardModel()
        results = simulate(model, (.25,0,0,0),60,5.5)

        f = generate_figure('Uncontrolled Bicycle at %.2f m/s' % 5.5, (results['t'], 'Time (seconds)'),
                    (results['phi'], 'Phi (radians)'), (results['delta'], 'Delta (radians)'))



        canvas = FigureCanvasTkAgg(f, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2TkAgg(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

def qf(quickPrint):
    print(quickPrint)


app = AutocycleGUI()
app.mainloop()

