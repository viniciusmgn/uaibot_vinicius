import uaibot as ub
import numpy as np



box = ub.Box(width=0.1, depth=0.2, height=0.3, color = 'brown')
cyl = ub.Cylinder(radius=0.1, height=0.3, color = 'brown')
ball = ub.Ball(radius=0.1, color = 'brown')

obj = box


sim=ub.Simulation([obj])

for i in range(1):
    obj.add_ani_frame(i, htm=ub.Utils.htm_rand())
    #obj.add_ani_frame(i, htm=np.identity(4))
    w, d, h = obj.aabb()
    col_box = ub.Box(htm = ub.Utils.trn(obj.htm[0:3,-1]), width=w, depth=d, height=h, color='blue', opacity=0.3)


    col_box.add_ani_frame(i, col_box.htm)

    sim.add(col_box)

sim.save("D://","test_aabb")

