from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor


class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # load environment
        self.scene = self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/grid floor.egg")
        self.scene.setHpr(0, 180, 0)
        # reparent model to render
        self.scene.reparentTo(self.render)
        # apply scale and position
        self.scene.setScale(100, 100, 100)

        # load and transform bike actor
        self.bike = Actor("../Users/Cooper Grill/Documents/Autocycle/bike.egg")
        self.bike.setHpr(180, 0, 0)
        self.bike.setPos(0, 0, 1)
        self.bike.reparentTo(self.render)

        # move bike and follow with camera
        self.taskMgr.add(self.bike_task, "bikeTask")

    # define procedure to move panda
    def bike_task(self, task):
        # set bike's velocity vector
        self.bike.setPos(self.bike.getPos() + (0, 0.05, 0))

        # set camera
        self.camera.setPos(self.bike.getPos() - (0, 30, -10))
        self.camera.lookAt(self.bike)

        # teleport bike based on position and angle
        if self.bike.getY() == 110 and 90 < self.bike.getHpr()[0] % 360 < 270: self.bike.setPos(0, -200, 1)
        if self.bike.getY() == -110 and -90 < self.bike.getHpr()[0] % 360 < 90: self.bike.setPos(0, 200, 1)
        if self.bike.getX() == 110 and 0 < self.bike.getHpr()[0] % 360 < 180: self.bike.setPos(-200, 0, 1)
        if self.bike.getX() == -110 and 180 < self.bike.getHpr()[0] % 360 < 360: self.bike.setPos(200, 0, 1)

        return Task.cont


app = MyApp()
app.run()
