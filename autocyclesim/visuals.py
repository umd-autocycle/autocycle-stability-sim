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
        self.pandaActor = Actor("models/panda-model", {"walk": "models/panda-walk4"})
        self.pandaActor.setScale(0.001, 0.01, 0.01)
        self.pandaActor.reparentTo(self.render)
        # loop animation
        self.pandaActor.loop("walk")

        # create intervals needed for panda to walk
        self.pandaActor.setHpr(180, 0, 1)
        self.pandaActor.setPos(0, 0, 0)
        self.taskMgr.add(self.pandaTask, "PandaTask")


    #define procedure to move panda
    def pandaTask(self, task):
        angleDegrees=task.time*6
        angleRadians=angleDegrees*pi/180
        self.pandaActor.setPos(10*sin(angleRadians), -10*cos(angleRadians), 1)
        self.camera.setPos(self.pandaActor.getPos() - (0, 30, -15))
        self.camera.lookAt(self.pandaActor)

        return Task.cont

app=MyApp()
app.run()