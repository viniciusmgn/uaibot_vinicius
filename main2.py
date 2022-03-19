from uaibot import *
import numpy as np

robot1 = Robot.create_kuka_kr5()
robot2 = Robot.create_kuka_lbr_iiwa(Utils.trn([1,0,0]))
robot3 = Robot.create_abb_crb(Utils.trn([-1,0,0]))
robot4 = Robot.create_epson_t6(Utils.trn([0,1,0]),color="black")
robot5 = Robot.create_staubli_tx60(Utils.trn([0,-1,0]))

robots = [robot1, robot2, robot3, robot4, robot5]

dt=0.01
k = 0
i = 0



for i in range(7):
    for j in range(5):
        if len(robots[j].links) > i:
            for t in range(500):
                alpha = 1 - t / 499
                q = robots[j].q0
                q[i] = alpha * robots[j].joint_limit[i, 0] + (1 - alpha) * robots[j].joint_limit[i, 1]
                robots[j].add_ani_frame((k+t) * dt, q=q)

    k += 500




# Create simulation
sim = Simulation(robots, camera_type="orthographic")

sim.save("D://","robots")