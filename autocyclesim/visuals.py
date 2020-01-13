from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.interval.IntervalGlobal import *

# import revisedgui
import simulation
import bikemodel


class Visuals(ShowBase):
    def __init__(self):
        # generate results based on current state of gui
        # results = revisedgui.GraphPage().results
        results = simulation.simulate(bikemodel.MeijaardModel(), (10, 0, 0, 0), 60, 5.5, control_method=None,
                                      perturbation=None)

        ShowBase.__init__(self)

        # load and transform environment
        self.scene = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/grid floor.egg")
        self.scene.setHpr(0, 180, 0)
        self.scene.reparentTo(self.render)
        self.scene.setScale(100, 100, 100)

        # load and transform front_fork
        self.front_fork = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/front_fork.egg")
        self.front_fork.reparentTo(self.render)
        self.front_fork.setPos(0, 0, 1)
        self.front_fork.setColor(1, 0, 0)

        # load and transform back_frame
        self.back_frame = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/back_frame.egg")
        self.back_frame.reparentTo(self.front_fork)
        self.back_frame.setColor(0, 0, 1)
        # continuously move back_frame in relation to rotation of front_fork
        self.taskMgr.add(self.back_frame_task)

        # move bike and follow with camera
        self.taskMgr.add(self.edge_task)
        self.taskMgr.add(self.camera_task)

        # interval loop to move bike
        ff_sequence = Sequence()
        for (i, j) in zip(results['phi'], results['delta']):
            ff_pos = self.front_fork.posInterval(0.01, (0, -0.5, 0), other=self.front_fork)
            ff_hpr = self.front_fork.hprInterval(0.01, (j, 0, i))
            ff_sequence.append(Parallel(ff_pos, ff_hpr))
        ff_sequence.start()

    # define task to move front fork in relation to rotation
    def back_frame_task(self, task):
        self.back_frame.setPos(-2.2 * sin(pi * self.back_frame.getH() / 180),
                               -2.2 + 2.2 * cos(pi * self.back_frame.getH() / 180), 0)

    # define procedure to move bike if it reaches an edge
    def edge_task(self, task):
        # move bike based on position and angle
        if self.front_fork.getY() >= 110 and 90 < self.front_fork.getHpr()[0] % 360 < 270:
            self.front_fork.setPos(self.front_fork.getX(), -200, 1)
        if self.front_fork.getY() <= -110 and -90 < self.front_fork.getHpr()[0] % 360 < 90:
            self.front_fork.setPos(self.front_fork.getX(), 200, 1)
        if self.front_fork.getX() >= 110 and 0 < self.front_fork.getHpr()[0] % 360 < 180:
            self.front_fork.setPos(-200, self.front_fork.getY(), 1)
        if self.front_fork.getX() <= -110 and 180 < self.front_fork.getHpr()[0] % 360 < 360:
            self.front_fork.setPos(200, self.front_fork.getY(), 1)

        return Task.cont

    # define task to move camera
    def camera_task(self, task):
        self.camera.setPos(self.front_fork.getPos() + (
            -30 * sin(pi * self.front_fork.getH() / 180), 30 * cos(pi * self.front_fork.getH() / 180), 10))
        self.camera.lookAt(self.back_frame)

        return Task.cont


app = Visuals()
app.run()
