import uaibot as ub
import numpy as np

def isfree(obstacles, point):

    i=0
    free = True
    while free and i < len(obstacles):
        _,d = ub.obstacles[i].projection(point)
        free = d>0.01
        i += 1

    return free


def ispathfree(obstacles, pointA, pointB):

    N = 20
    point_c = pointA
    i=0
    free = True

    while free and i < N:
        free = isfree(obstacles, point_c)
        point_c += (pointB-pointA)/N
        i+=1

    return free




