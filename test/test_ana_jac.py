import uaibot as ub
import numpy as np


robot = ub.Robot.create_kuka_lbr_iiwa()

robot.add_ani_frame(0,q=[0,0,0,2,0,0,0])

box1 = ub.Box(width=0.5, height=0.3, depth=0.2, color="blue")



sim = ub.Simulation.create_sim_factory([box1])

dt=0.001

box1.add_ani_frame(0, htm=ub.Utils.trn([0.5,0,0.8]))



for i in range(len(robot.links)):
    for j in range(len(robot.links[i].col_objects)):
        sim.add(robot.links[i].col_objects[j][0])

robot.update_col_object(0)

dist_struct_box = robot.compute_dist(box1)
dist_struct_auto = robot.compute_dist_auto()

for k in range(0,dist_struct_auto.no_items):
    i = dist_struct_auto[k].link_number_1
    isub = dist_struct_auto[k].link_col_obj_number_1
    j = dist_struct_auto[k].link_number_2
    jsub = dist_struct_auto[k].link_col_obj_number_2

    pointA = ub.Ball(radius=0.01, color=robot.links[i].col_objects[isub][0].color)
    pointB = ub.Ball(radius=0.01, color=robot.links[j].col_objects[jsub][0].color)
    pointA.add_ani_frame(0, ub.Utils.trn(dist_struct_auto[k].point_link_1))
    pointB.add_ani_frame(0, ub.Utils.trn(dist_struct_auto[k].point_link_2))
    sim.add([pointA, pointB])

    print("AAA")


robot.check_free_configuration()

sim.save("D://","test_obj_col")
