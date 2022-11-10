import uaibot as ub
import numpy as np

from utils import *
import robot as rb

import plotly.express as px
import plotly.graph_objects as go

def freefun(_robot, _q, _obstacles, hint):

    _robot.add_ani_frame(0, q=_q)
    _robot.update_col_object(0)

    #Try the hint:
    collision = False
    if not (hint is None):
        link_num = hint[0]
        col_obj_num = hint[1]
        obs_num = hint[2]

        _, _, d = ub.Utils.compute_dist(_robot.links[link_num].col_objects[col_obj_num][0], _obstacles[obs_num])

        collision = (d < 0.005)

    if not collision:
        list_try = []

        for link_num in range(np.shape(_robot.q)[0]):
            for col_obj_num in range(len(_robot.links[link_num].col_objects)):
                for obs_num in range(len(_obstacles)):
                    list_try.append([link_num, col_obj_num, obs_num])

        hint = None
        k = 0
        while not collision and k < len(list_try):
            link_num = list_try[k][0]
            col_obj_num = list_try[k][1]
            obs_num = list_try[k][2]
            _, _, d = ub.Utils.compute_dist(_robot.links[link_num].col_objects[col_obj_num][0], _obstacles[obs_num])
            collision = (d < 0.005)

            if collision:
                hint = list_try[k]

            k += 1

    return collision, hint

def generatefree_uniform(_robot, _obstacles, q_base, indA, indB, N = 50):

    hint = None

    mat = np.matrix(np.zeros((N,N)))
    v = []

    for a in range(N):
        v.append( round(-180 + 360*float(a)/float(N-1)) )

    for a in range(N):
        print(a)
        for b in range(N):
            _q = q_base.copy()
            _q[indA, 0] = -np.pi + 2*np.pi*float(a)/float(N-1)
            _q[indB, 0] = -np.pi + 2*np.pi*float(b)/float(N-1)
            notfree, hint = freefun(_robot, _q, _obstacles, hint)
            mat[a,b] = 1 if notfree else np.nan

    return v, mat

def show_slice(_robot, _obstacles, q_base, indA, indB, htm_target, N = 50):

    v, mat_obs = generatefree_uniform(_robot, _obstacles, q_base, indA, indB, N)

    mat = np.matrix(np.zeros((N,N)))

    val_max=0
    val_min=1000
    for a in range(N):
        for b in range(N):
            _q = q_base.copy()
            _q[indA, 0] = -np.pi + 2*np.pi*float(a)/float(N-1)
            _q[indB, 0] = -np.pi + 2*np.pi*float(b)/float(N-1)
            r, _ = _robot.task_function(htm_target, _q)


            mat[a,b] = np.linalg.norm(r)
            val_max = max(val_max, mat[a, b])
            val_min = min(val_min, mat[a, b])


    #fig = go.Figure(data=go.Heatmap(z=mat, x=v, y=v, hoverongaps=False, colorscale='Blues'))
    #fig.add_trace(go.Heatmap(z=mat_obs, x=v, y=v, hoverongaps=False, colorscale='Reds'))

    mat_mod = np.sqrt((mat-val_min)/(val_max-val_min))
    fig = go.Figure(data=go.Heatmap(z=mat_mod, x=v, y=v, hoverongaps=False, colorscale='Blues'))
    fig.add_trace(go.Heatmap(z=mat_obs, x=v, y=v, hoverongaps=False, colorscale='Reds'))

    fig.update_layout(
        xaxis_title="q"+str(indB+1),
        yaxis_title="q"+str(indA+1),
    )

    fig.show()

    return mat, mat_obs, fig

