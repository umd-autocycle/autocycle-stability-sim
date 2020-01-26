from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.interval.IntervalGlobal import *

import revisedgui
import simulation
import bikemodel


class Visuals(ShowBase):
    # generate results based on current state of gui
    # results = revisedgui.GraphPage().results
    results = simulation.simulate(bikemodel.MeijaardModel(), (10, 0, 0, 0), 60, 5.5, control_method=None,
                                  perturbation=None)
    v = 5.5 / 10

    def __init__(self):
        ShowBase.__init__(self)

        # load and transform environment
        self.scene = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/grid floor.egg")
        self.scene.setHpr(0, 180, 0)
        self.scene.reparentTo(self.render)
        self.scene.setScale(100, 100, 100)

        # load and transform back_frame
        self.back_frame = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/back_frame.egg")
        self.back_frame.reparentTo(self.render)
        self.back_frame.setPos(0, 0, 1)
        self.back_frame.setColor(0, 0, 1)

        # load and transform front_fork
        self.front_fork = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/front_fork.egg")
        self.front_fork.reparentTo(self.back_frame)
        self.front_fork.setColor(1, 0, 0)

        self.disableMouse()
        self.camera.reparentTo(self.back_frame)
        self.camera.setPos(0, 30, 10)
        self.camera.setHpr(180, -18, 0)
        self.camLens.setFar(175)

        # move bike and follow with camera
        self.taskMgr.add(self.edge_task)
        # self.taskMgr.add(self.camera_task)

        # initial conditions
        self.front_fork.setHpr(self.results['delta'][0], 0, self.results['phi'][0])

        # interval loop to move bike
        prev = self.results['phi'][0]
        ff_sequence = Sequence()
        for (i, j) in zip(self.results['delta'][1:], self.results['phi'][1:]):
            bf_pos = self.back_frame.posInterval(0.01, (0, -self.v, 0), other=self.front_fork)
            ff_yaw = self.front_fork.hprInterval(0.01, (i, 0, 0))
            bf_hpr = self.back_frame.hprInterval(0.01, (self.v * sin(i) / 5, 0, j-prev), other=self.back_frame)
            ff_sequence.append(Parallel(bf_pos, ff_yaw, bf_hpr))
            prev = j
        ff_sequence.start()

    # define task to move bike if it reaches an edge
    def edge_task(self, task):
        if self.back_frame.getZ() != 1:
            self.back_frame.setPos(self.back_frame.getX(), self.back_frame.getY(), 1)
        if self.back_frame.getP() != 0:
            self.back_frame.setHpr(self.back_frame.getH(), 0, self.back_frame.getR())
        if self.back_frame.getY() >= 110:
            self.back_frame.setPos(self.back_frame.getX(), -109, 1)
        if self.back_frame.getY() <= -110:
            self.back_frame.setPos(self.back_frame.getX(), 109, 1)
        if self.back_frame.getX() >= 110:
            self.back_frame.setPos(-109, self.back_frame.getY(), 1)
        if self.back_frame.getX() <= -110:
            self.back_frame.setPos(109, self.back_frame.getY(), 1)

        return Task.cont

    # define task to move camera
    """ def camera_task(self, task):
        self.camera.setPos(self.front_fork.getPos() + (
            -30 * sin(pi * self.back_frame.getH() / 180), 30 * cos(pi * self.back_frame.getH() / 180), 10))
        self.camera.lookAt(self.back_frame)

        return Task.cont """


app = Visuals()
app.run()

