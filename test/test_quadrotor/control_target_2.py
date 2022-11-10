import uaibot as ub
import numpy as np
from cvxopt import matrix, solvers
import scipy.linalg

def func_phi(xi,beta):
    if xi>0:
        return xi + np.log(1+np.exp(-beta*xi))/beta
    else:
        return np.log(np.exp(beta*xi)+1)/beta

def der_func_phi(xi,beta):
    return 1/(1 + np.exp(-beta*xi))

def func_zd(p_target, p, beta):
    z0 = np.matrix([[0],[0],[1]])
    xi = z0.T * (p - p_target)
    xi = xi[0,0]
    z_d_nn = p_target - p + func_phi(xi, beta) * z0
    return z_d_nn/np.linalg.norm(z_d_nn)

def der_func_zd(p_target, p, beta):

    z0 = np.matrix([[0],[0],[1]])
    xi = z0.T * (p - p_target)
    xi = xi[0, 0]
    z_d_nn = p_target - p + func_phi(xi, beta) * z0
    norm_zd_nn = np.linalg.norm(z_d_nn)
    z_d = z_d_nn/norm_zd_nn

    M = (np.eye(3) - z_d * z_d.T)/norm_zd_nn
    N = -np.eye(3) + der_func_phi(xi, beta) * z0 * z0.T

    return M * N




def control_target_2(vc, wc, robot, robot_next, radius_robot, obstacle, radius_obstacle, p_target):
    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    #Parâmetros
    dt=0.005
    eta_p = 0.1
    eta_v = 2*np.sqrt(eta_p)
    Kr = 0.2
    Kdotr = 2*np.sqrt(Kr)
    beta = 2
    alpha= 0.2
    kv = 0.2


    p = robot.htm[0:3, -1]
    z = robot.htm[0:3,2]

    p_next = robot_next.htm[0:3, -1]
    z_next = robot_next.htm[0:3, 2]

    z0 = np.matrix([[0],[0],[1]])
    g = -9.8 * z0






    dist_min = 1000
    A = np.matrix(np.zeros((0, 4)))
    b = np.matrix(np.zeros((0, 1)))
    list_dist=[]


    #Compute the distances
    for i in range(len(obstacle)):
        if np.linalg.norm(p - obstacle[i].htm[0:3, -1]) - (radius_robot + radius_obstacle[i]) < 0.4:

            p_robot, p_obs, D = ub.Utils.compute_dist(robot, obstacle[i])
            p_robot_next, p_obs_next, D_next = ub.Utils.compute_dist(robot_next, obstacle[i])

            a = np.block([(p_robot - p_obs).T, (p_robot - p_obs).T * ub.Utils.S(p - p_robot)])
            a_next = np.block([(p_robot_next - p_obs_next).T, (p_robot_next - p_obs_next).T * ub.Utils.S(p_next - p_robot_next)])
            a_mod = np.block([(p_robot - p_obs).T * z, (p_robot - p_obs).T * ub.Utils.S(p - p_robot)])
            h = (p_robot - p_obs).T * g


            dadt = (a_next-a)/dt
            dadt_uc = dadt.T * np.block([[vc], [wc]])

            dDdt = (D_next-D)/dt

            b_mod = -eta_v * dDdt -eta_p * (D - 0.03) - h - dadt_uc


            A = np.block([[A], [a_mod]])
            b = np.block([[b], [b_mod]])

            list_dist.append(D)
            dist_min = min(dist_min, D)

    #Create orientation constraint
    #z = robot.htm[0:3,2]
    #z_d = np.matrix([[0],[0],[1]])
    #a = np.block([ np.matrix([0,0,0])  , -z_d.T * ub.Utils.S( z )  ])
    #A = np.block([[A], [a]])
    #b = np.block([[b], [-eta * (z_d.T * z - costheta)]])

    #Create the task

    zd = func_zd(p_target, p, beta)
    zd_next = func_zd(p_target, p_next, beta)
    pzdp = der_func_zd(p_target, p, beta)
    pzdp_next = der_func_zd(p_target, p_next, beta)

    r = 1 - zd.T * z
    dotr = -z.T * pzdp * vc + zd.T * ub.Utils.S(z) * wc
    rem_r = z.T * pzdp * 9.8 * z0 - ( z_next.T * pzdp_next - z.T * pzdp) * vc/dt + \
            (zd_next.T * ub.Utils.S(z_next) - zd.T * ub.Utils.S(z)) * wc/dt

    f_w = np.block([ - z.T * pzdp * z  ,  zd.T * ub.Utils.S(z) ])
    ftg_w = -rem_r - Kdotr* dotr - Kr * r


    #New
    r = 1 - zd.T * z
    dotr = -zd.T * (z_next - z)/dt
    rem_r =  zd.T * ub.Utils.S( (z_next - z)/dt ) * wc
    f_w = np.block([ 0  ,  zd.T * ub.Utils.S(z) ])
    ftg_w = -rem_r - Kdotr * dotr - Kr * r



    f_T = np.block([  vc.T * zd  ,  np.matrix([0,0,0]) ])
    ftg_T = vc.T * (- kv * vc + 9.8 * z0 - alpha * (p - p_target))

    #f = np.block([ [f_w], [f_T]])
    #ftg = np.block([ [ftg_w] , [ftg_T] ])

    f_temp = f_w
    ftg = ftg_w

    #Create and solve the optimization problem
    H = 2 * f_temp.T * f_temp + 0.01 * np.eye(4) + np.diag([100,0,0,0])
    f = -2 * f_temp.T * ftg

    try:
        u = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']
        u = np.reshape(u, (4, 1))
        T = u[0, 0]
        dotw = np.matrix(u[1:4, 0]).reshape((3, 1))
        error = False


        print("Value of r:" +str(r))
        print("Value of dotr: " + str(dotr))
        print("Value of dotr estimated : " + str(zd.T*(z-z_next)/dt))
        print("Value of ddotr: "+str(f_w*u + rem_r))
        print("Value of dotz: " + str( (ub.Utils.S(wc)*z).T))
        print("Value of dotz estimated: " + str( ((z_next-z)/dt).T ))
        print("Value of wc:" +str(wc.T))
        print("Value of target: "+str(-rem_r - Kdotr * dotr - Kr * r))
        print(" ")

    except:
        T = 0
        dotw = np.matrix(np.zeros((3, 1)))
        error = True

    return T, dotw, error, dist_min