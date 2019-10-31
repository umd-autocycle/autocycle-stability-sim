import tkinter as tk

def makeagraph():
   pass

def seethebike():
    pass
def moreinfo():
    pass

root = tk.Tk()
menubar = tk.Menu(root)
visualmenu = tk.Menu(menubar, tearoff=0)
visualmenu.add_command(label='Make a Graph', command=makeagraph)
visualmenu.add_command(label='See the Bike', command=seethebike)
menubar.add_cascade(label="Simulate", menu=visualmenu)
menubar.add_command(label="More Info", command=moreinfo)
menubar.add_command(label="Exit", command=root.quit)

welcometext = tk.Text(root, bg='white', bd=0, font="Arial", height=10,width=80)
welcometext.tag_configure("center",justify=tk.CENTER)
welcometext.insert("1.0", "Select the Simulation menu above or buttons below to create a graph" +
                   " or to see the bike in action.")
welcometext.tag_add("center","1.0")
welcometext.insert("1.0", "Welcome to the Autocycle bicycle simulator!\n")
welcometext.tag_add("center","1.0")
welcometext.insert("1.0","\n\n\n\n")
welcometext.config(state=tk.DISABLED)
welcometext.grid(row=0,column=0)

buttons = tk.Frame(height=10, width=60)
buttons.grid(row=1,column=0)

graphbutton = tk.Button(buttons, text="Make a Graph", command=makeagraph,height = 10,width = 20)
graphbutton.grid(row=0, column=0)
bikebutton = tk.Button(buttons, text="See the Bike", command=seethebike,height = 10,width = 20)
bikebutton.grid(row=0, column=1)
infobutton = tk.Button(buttons, text="Learn More", command=moreinfo,height = 10,width = 20)
infobutton.grid(row=0, column=2)



# display the menu
root.config(menu=menubar)
root.resizable(width=False, height=False)

root.mainloop()

