import uaibot as ub
import numpy as np

from utils import *
import robot as rb

import plotly.express as px
import plotly.graph_objects as go
from cvxopt import matrix, solvers

def gentan():
    vn = np.random.uniform(-1,1,(3,1))
    vn = vn/np.linalg.norm(vn)
    #vn = np.matrix([[0],[0],[1]])
    return vn

def moveball(p_init, p_target, obstacles, radius, k0, mode_approach_0, vn_0, N):

    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    eta=0.5
    kp=0.5
    p=p_init
    dt=0.5 #0.2



    ball = ub.Ball(radius=radius)

    hist_p=[]



    mode_approach=mode_approach_0
    k=k0
    vn = vn_0

    for i in range(N):

        A = np.matrix(np.zeros((0,3)))
        b = np.matrix(np.zeros((0,1)))
        ball.add_ani_frame(i, htm=Utils.trn(p))

        #Bounding sphere
        R=1
        p_center = np.matrix([[0],[0],[0.5]])
        dir = (p-p_center)/(0.0001 + np.linalg.norm(p-p_center))
        p_obs = p_center + R * dir
        p_ball = p + radius * dir
        d = np.linalg.norm(p_obs-p_ball)
        gradD = (p_ball - p_obs).T
        gradD = gradD / (0.0001 + np.linalg.norm(gradD))
        A = np.block([[A], [-gradD]])
        b = np.block([[b], [eta * (d - 0.01)]])


        dmin=d
        gradDmin=gradD

        #Obstacles
        for obs in obstacles:
            if ub.Utils.compute_aabbdist(obs, ball) < 0.3:
                p_obs, p_ball, d = ub.Utils.compute_dist(obs, ball)
                gradD = (p_ball-p_obs).T
                gradD = gradD/(0.0001 + np.linalg.norm(gradD))
                A = np.block([[A], [-gradD]  ])
                b = np.block([[b], [eta*(d-0.01)]])

                if d<dmin:
                    dmin=d
                    gradDmin = gradD

        if mode_approach and dmin <=0.02 and k >= 40:
            mode_approach = False
            vn = gentan()
            k = 0

        if np.linalg.norm(p - p_target) < 0.01:
            p = p_center

        if mode_approach:
            v_target = -kp * (p-p_target)
        else:
            N = gradDmin.T
            T = ub.Utils.S(vn) * N.copy()
            #T = np.matrix(np.zeros((3,1)))
            #T[0,0] =  -N[1,0]
            #T[1,0] =   N[0,0]
            T = T/np.linalg.norm(T)

            g_f = -0.99 * (2/pi) * np.arctan(100*(dmin))
            h_f = np.sqrt(1-g_f**2)
            v_target = 0.1 * (g_f * N + h_f * T)

            if k == 250:
                k=0
                mode_approach = True



        H = 2 * np.identity(3)
        f = - 2 * v_target

        try:
            v = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
            p += v * dt
        except:
            p = p

        hist_p.append(p.copy())
        k += 1

    return hist_p, k, mode_approach, vn

def moveballaccel(p_init, p_target, obstacles, radius, N):

    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)


    etad = 0.5
    etap = (etad**2)/4

    kd=0.5
    kp=(kd**2)/4

    p=p_init
    v=np.random.uniform(-0.5,0.5,(3,1))

    dt=0.1

    ball = ub.Ball(radius=radius)

    hist_p=[]


    for i in range(N):

        A = np.matrix(np.zeros((0,3)))
        b = np.matrix(np.zeros((0,1)))
        ball.add_ani_frame(i, htm=Utils.trn(p))

        for obs in obstacles:
            if ub.Utils.compute_aabbdist(obs, ball) < 0.3:
                p_obs, p_ball, d = ub.Utils.compute_dist(obs, ball)
                gradD = (p_ball - p_obs).T
                gradD = gradD/(0.0001 + np.linalg.norm(gradD))
                A = np.block([[A], [-gradD]  ])
                b = np.block([[b], [etad * gradD * v + etap*(d-0.02)]])

        a_target = -kp * (p-p_target) - kd * v
        H = 2 * np.identity(3)
        f = - 2 * a_target

        try:
            a = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
            p += v * dt
            v += a * dt
        except:
            a = 0



        hist_p.append(p.copy())

    return hist_p


