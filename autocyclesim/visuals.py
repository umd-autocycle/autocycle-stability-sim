from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor

from simulation import simulate
from bikemodel import MeijaardModel


class MyApp(ShowBase):
    simulate = []
    counter = 0

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
        self.back_frame.setColor(0, 0, 1)
        self.back_frame.setHpr(180, 0, 0)
        self.back_frame.setPos(0, 0, 1)

        # load and transform front_fork
        self.front_fork = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/front_fork.egg")
        self.front_fork.reparentTo(self.back_frame)
        self.front_fork.setColor(1, 0, 0)
        # continuously move front fork in relation to rotation
        self.taskMgr.add(self.front_fork_task, "front_forkTask")

        # move bike and follow with camera
        self.taskMgr.add(self.bike_task, "bikeTask")
        self.taskMgr.add(self.camera_task, "cameraTask")

    # define task to move front fork in relation to rotation
    def front_fork_task(self, task):
        self.front_fork.setPos(-2.2 * sin(pi * self.front_fork.getH() / 180),
                               -2.2 + 2.2 * cos(pi * self.front_fork.getH() / 180), 0)

    # define procedure to move bike
    def bike_task(self, task):
        # set bike's velocity vector
        self.back_frame.setPos(self.back_frame.getPos() + (0, 1, 0))

        # move bike based on position and angle
        if self.back_frame.getY() == 110 and 90 < self.back_frame.getHpr()[0] % 360 < 270:
            self.back_frame.setPos(0, -200, 1)
        if self.back_frame.getY() == -110 and -90 < self.back_frame.getHpr()[0] % 360 < 90:
            self.back_frame.setPos(0, 200, 1)
        if self.back_frame.getX() == 110 and 0 < self.back_frame.getHpr()[0] % 360 < 180:
            self.back_frame.setPos(-200, 0, 1)
        if self.back_frame.getX() == -110 and 180 < self.back_frame.getHpr()[0] % 360 < 360:
            self.back_frame.setPos(200, 0, 1)

        return Task.cont

    # define task to move camera
    def camera_task(self, task):
        self.camera.setPos(self.back_frame.getPos() + (
            -30 * sin(pi * self.back_frame.getH() / 180), 30 * cos(pi * self.back_frame.getH() / 180), 10))
        self.camera.lookAt(self.back_frame)

        return Task.cont


app = MyApp()
app.run()
