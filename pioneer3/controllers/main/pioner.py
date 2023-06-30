
from controller import Robot, GPS, InertialUnit
import numpy as np
import osqp
import scipy as sp
from scipy import sparse

def get_motor(robot, name, time_step):
    motor = robot.getDevice(name)
    return motor

def set_motor(motor):
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    return motor

def get_motor_velocity(motor):
    velocity = motor.getVelocity()
    return velocity

def set_gps(name, time_step):
    gps = GPS(name)
    gps.enable(time_step)
    return gps

def set_imu(name, time_step):
    imu = InertialUnit(name)
    imu.enable(time_step)
    return imu

def set_motors_velocity(motor_r, motor_l, w):
    w_r = w[0, 0]
    w_l = w[1, 0]
    motor_r.setVelocity(w_r)
    motor_l.setVelocity(w_l)
    return None

def get_system_velocity_initial(motor_r, motor_l, L, robot, time_step):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    # Transformation matrix 
    T=np.array([[r/2,r/2],[r/l,-r/l]])
    if robot.step(time_step) != -1:
        wr = get_motor_velocity(motor_r)
        wl = get_motor_velocity(motor_l)
        W = np.array([[wr], [wl]], dtype=np.double)
        V = T@W
    return V[0:2,0]

def get_system_velocity(motor_r, motor_l, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    # Transformation matrix 
    T=np.array([[r/2,r/2],[r/l,-r/l]])
    wr = get_motor_velocity(motor_r)
    wl = get_motor_velocity(motor_l)
    W = np.array([[wr], [wl]], dtype=np.double)
    V = T@W
    return V[0:2,0]


def velocities_max(L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    # Transformation matrix 
    T=np.matrix([[r/2,r/2],[r/l,-r/l]])
    wr =12.3
    wl =-12.3
    W = np.array([[wr], [wl]], dtype=np.double)
    V = T@W
    print(V)
    return None

def get_initial(robot, gps, imu, time_step, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]

    if robot.step(time_step) != -1:
        position = gps.getValues()
        orientation = imu.getRollPitchYaw()
        data = [position[0] + a*np.cos(orientation[2]), position[1] + a*np.sin(orientation[2]), orientation[2]]
    x = np.array(data)
    return x

def get_states(robot, gps, imu, time_step, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    if robot.step(time_step) != -1:
        position = gps.getValues()
        orientation = imu.getRollPitchYaw()
        data = [position[0] + a*np.cos(orientation[2]), position[1] + a*np.sin(orientation[2]), orientation[2]]
    x = np.array(data)
    return x

def controlador(h, hd, hdp, k1, k2, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]

    # Gain Matrices
    K1=k1*np.eye(2,2)
    K2=k2*np.eye(2,2)

    # Control error defintion
    herr=hd[0:2]-h[0:2]
    e = herr.reshape(2, 1)
    hdp = hdp[0:2]
    hdp = hdp.reshape(2, 1)

    # Horientation of the system
    q = h[2]

    # Jacobian Matrix Definition
    J = Jacobian_system(L, h)

    control=np.linalg.inv(J)@(hdp+1*np.tanh(e))
    return control[:, 0]
    
def conversion(v, L):
    # reshape control values
    v = v.reshape(2, 1)
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    # Transformation matrix 
    T=np.matrix([[r/2,r/2],[r/l,-r/l]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas

def Jacobian_system(L, h):
    # Split Values of the system
    q = h[2]
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]

    # Jacobian Matrix control of the system
    J11 = np.cos(q)
    J12 = -a*np.sin(q)
    J21 = np.sin(q)
    J22 = a*np.cos(q)
    J = np.array([[J11, J12],[J21, J22]], dtype=np.double)
    return J

def Jacobian_system_complete(L, h):
    # Split Values of the system
    q = h[2]
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]

    # Jacobian Matrix control of the system
    J11 = np.cos(q)
    J12 = -a*np.sin(q)
    J21 = np.sin(q)
    J22 = a*np.cos(q)
    J31 = 0
    J32 = 1
    J = np.array([[J11, J12],[J21, J22], [J31, J32]], dtype=np.double)
    return J

def QP_controller(hd, h, L):
    n = 2
    m = 2

    q = np.zeros(n+m)

    Pn = sparse.csc_matrix([[1, 0], [0, 1]])
    Pm = sparse.csc_matrix([[10, 0], [0, 10]])
    P = sparse.block_diag([Pn, Pm], format='csc')
    J = Jacobian_system(L, h)
    Ad = sparse.csc_matrix([[J[0, 0], J[0, 1]], [J[1, 0], J[1, 1]]])
    A = sparse.vstack([
        sparse.hstack([Ad, -sparse.eye(m)]),
        sparse.hstack([sparse.eye(n), sparse.csc_matrix((n, m))])], format='csc')
    e = hd - h[0:2]
    he = 1*np.tanh((e))

    l = np.hstack([he, -1*np.ones(n)])
    u = np.hstack([he, 1*np.ones(n)])
    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l, u)

    return prob

def QP_controller_all(hd, h, L):
    n = 2
    m = 3

    q = np.zeros(n+m)

    # Control error definition
    e = hd - h
    he = 1*np.tanh((e))
    norm_error = np.linalg.norm(he[0:2])

    Pn = sparse.csc_matrix([[1, 0], [0, 1]])
    Pm = sparse.csc_matrix([[20, 0, 0], [0, 20, 0], [0, 0, 0/(1+norm_error)]])

    P = sparse.block_diag([Pn, Pm], format='csc')
    J = Jacobian_system_complete(L, h)
    Ad = sparse.csc_matrix([[J[0, 0], J[0, 1]], [J[1, 0], J[1, 1]], [J[2, 0], J[2, 1]]])
    A = sparse.vstack([
        sparse.hstack([Ad, -sparse.eye(m)]),
        sparse.hstack([sparse.eye(n), sparse.csc_matrix((n, m))])], format='csc')

    l_new = np.hstack([he, -1.1, -6])
    u_new = np.hstack([he, 1.1, 6])
    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l_new, u_new)

    return prob

def QP_solver(prob, hd, h, L):
    n = 2
    m = 2
    # Jacobian System
    J = Jacobian_system(L, h)
    Ad = sparse.csc_matrix([[J[0, 0], J[0, 1]], [J[1, 0], J[1, 1]]])
    A = sparse.vstack([sparse.hstack([Ad, -sparse.eye(m)]),
        sparse.hstack([sparse.eye(n), sparse.csc_matrix((n, m))])], format='csc')
    # Update problem
    e = hd - h[0:2]
    he = 1*np.tanh((e))

    # Limits Control
    l_new = np.hstack([he, -1*np.ones(n)])
    u_new = np.hstack([he, 1*np.ones(n)])
    prob.update(Ax= A.data, l = l_new, u = u_new)

    # Solve Control Problem
    res = prob.solve()
    return res.x[0:2]

def QP_solver_all(prob, hd, h, L):
    n = 2
    m = 3
    # Control error
    e = hd - h
    he = 2*np.tanh((e))
    norm_error = np.linalg.norm(he[0:2])

    Pn = sparse.csc_matrix([[1, 0], [0, 1]])
    Pm = sparse.csc_matrix([[40, 0, 0], [0, 40, 0], [0, 0, 0/(1+norm_error)]])
    P = sparse.block_diag([Pn, Pm], format='csc')
    # Jacobian System
    J = Jacobian_system_complete(L, h)
    Ad = sparse.csc_matrix([[J[0, 0], J[0, 1]], [J[1, 0], J[1, 1]], [J[2, 0], J[2, 1]]])
    A = sparse.vstack([sparse.hstack([Ad, -sparse.eye(m)]),
        sparse.hstack([sparse.eye(n), sparse.csc_matrix((n, m))])], format='csc')

    # Limits Control
    l_new = np.hstack([he, -1.1, -6])
    u_new = np.hstack([he, 1.1, 6])
    prob.update(Px =P.data ,Ax= A.data, l = l_new, u = u_new)

    # Solve Control Problem
    res = prob.solve()
    return res.x[0:2]