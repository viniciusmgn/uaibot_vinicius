import uaibot as ub
import numpy as np


robot = ub.Robot.create_kuka_kr5()

box1 = ub.Box(name="box1", width=0.5, height=0.3, depth=0.2, color="blue")
box2 = ub.Box(name="box2", width=0.25, height=0.4, depth=0.6, color="red")
cil1 = ub.Cylinder(name="cyl1", radius=0.3, height=0.4, color="purple")
ball1 = ub.Ball(name="ball1", radius=0.4, color="brown")
smoothBox1 = ub.SmoothBox(name="box1", width=0.5, height=0.3, depth=0.2, color="blue")

obj1=smoothBox1
obj2=cil1

pointA = ub.Ball(name="pointA", radius=0.035, color="yellow")
pointB = ub.Ball(name="pointB", radius=0.035, color="green")

sim = ub.Simulation([obj1, obj2, pointA, pointB])

dt=0.01

for i in range(3000):

    if i%300==0:
        t1 = np.random.uniform(0 ,6)
        t2 = np.random.uniform(0, 6)
        t3 = np.random.uniform(0, 6)
        t4 = np.random.uniform(-2, 2)
        t5 = np.random.uniform(-2, 2)
        t6 = np.random.uniform(-2, 2)
        htm1 = ub.Utils.rotx(dt*t1)*ub.Utils.roty(dt*t2)*ub.Utils.rotz(dt*t3)*ub.Utils.trn([dt*t4,dt*t5,dt*t6])

        t1 = np.random.uniform(0 ,6)
        t2 = np.random.uniform(0, 6)
        t3 = np.random.uniform(0, 6)
        t4 = np.random.uniform(-2, 2)
        t5 = np.random.uniform(-2, 2)
        t6 = np.random.uniform(-2, 2)
        htm2 = ub.Utils.rotx(dt*t1)*ub.Utils.roty(dt*t2)*ub.Utils.rotz(dt*t3)*ub.Utils.trn([dt*t4,dt*t5,dt*t6])


    obj1.add_ani_frame(dt * i, htm=obj1.htm*htm1)
    obj2.add_ani_frame(dt * i, htm=obj2.htm*htm2)

    [pa,pb,d] = ub.Utils.compute_dist(obj1,obj2)

    pointA.add_ani_frame(dt * i, htm=ub.Utils.trn(pa))
    pointB.add_ani_frame(dt * i, htm=ub.Utils.trn(pb))

sim.save("D://","test_obj_col")
