from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Point3
from panda3d.core import GeoMipTerrain

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        #load environment
        self.scene=self.loader.loadModel("../Users/Cooper Grill/Documents/Autocycle/untitled.egg")
        self.scene.setHpr(0, 180, 0)
        #reparent model to render
        self.scene.reparentTo(self.render)
        #apply scale and position
        self.scene.setScale(100, 100, 100)
        self.scene.setPos(-8, 42, 0)

        # load and transform panda actor
        self.bike = Actor("models/panda-model", {"walk": "models/panda-walk4"})
        self.bike.setScale(0.001, 0.01, 0.01)
        self.bike.reparentTo(self.render)
        # loop animation
        self.bike.loop("walk")

        # create intervals needed for panda to walk
        self.bike.setHpr(180, 0, 1)
        self.bike.setPos(0, -200, 1)
        self.taskMgr.add(self.bikeTask, "bikeTask")


    #define procedure to move panda
    def bikeTask(self, task):
        self.bike.setPos(self.bike.getPos()+(0, .5, 0))
        self.camera.setPos(self.bike.getPos() - (0, 30, -10))
        self.camera.lookAt(self.bike)
        if self.bike.getPos()==(0, 155, 1): self.bike.setPos(0, -200, 1)

        return Task.cont

app=MyApp()
app.run()