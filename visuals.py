from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Point3

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        #load environment
        self.scene=self.loader.loadModel("models/environment")
        #reparent model to render
        self.scene.reparentTo(self.render)
        #apply scale and position
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)

        #load and transform panda actor
        self.pandaActor=Actor("models/panda-model", {"walk": "models/panda-walk4"})
        self.pandaActor.setScale(0.005, 0.005, 0.005)
        self.pandaActor.reparentTo(self.render)
        #loop animation
        self.pandaActor.loop("walk")

        #create four lerp intervals needed for panda to walk back and forth
        pandaPosInterval1=self.pandaActor.posInterval(13, Point3(0, -10, 0), startPos=Point3(0, 10, 0))
        pandaPosInterval2=self.pandaActor.posInterval(13, Point3(0, 10, 0), startPos=Point3(0, -10, 0))
        pandaHprInterval1=self.pandaActor.hprInterval(3, Point3(180, 0, 0), startHpr=Point3(0, 0, 0))
        pandaHprInterval2=self.pandaActor.hprInterval(3, Point3(0, 0, 0), startHpr=Point3(180, 0, 0))

        #create and play sequence that coordinates intervals
        self.pandaPace=Sequence(pandaPosInterval1, pandaHprInterval1, pandaPosInterval2, pandaHprInterval2, name="pandaPace")
        self.pandaPace.loop()

        # add camera procedure to task manager
        self.taskMgr.add(self.cameraTask, "CameraTask")

    #define procedure to move camera
    def cameraTask(self, task):
        self.camera.setPos(self.pandaActor.getPos()-(0, 10, -5))
        self.camera.lookAt(self.pandaActor)
        return Task.cont

app=MyApp()
app.run()