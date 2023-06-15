#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS, InertialUnit
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from fancy_plots import fancy_plots_2, fancy_plots_1
from pioner import get_motor, set_motor, set_gps, set_imu, get_initial, controlador, set_motors_velocity, get_states, conversion, QP_controller, QP_solver
from pioner import QP_controller_all, QP_solver_all
from pioner import velocities_max
from pioner import get_motor_velocity, get_system_velocity, get_system_velocity_initial

def main(robot):
    # Get the time step of the current world.
    time_step = int(robot.getBasicTimeStep())
    # Time Definition
    t_final = 10
    # Sample time
    t_s = 0.03
    # Time simulation
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    # System states
    x = np.zeros((3, t.shape[0]+1), dtype = np.double)

    # Reference Trajectory
    x_d = np.zeros((3, t.shape[0]), dtype = np.double)
    #x_d[0, :] = 1*np.cos(0.6*t)
    #x_d[1, :] = 1*np.sin(0.6*t)

    x_d[0, :] = 2
    x_d[1, :] = 3
    x_d[2, :] =0*(np.pi/180)

    x_dp = np.zeros((3, t.shape[0]), dtype = np.double)
    #x_dp[0, :] = -1*0.6*np.sin(0.6*t)
    #x_dp[1, :] = 1*0.6*np.cos(0.6*t)
    x_dp[0, :] = 0
    x_dp[1, :] = 0
    x_dp[2, :] = 0
    
    # Definicion de las ganancias del controlador
    k1 = 1
    k2 = 0.5

    # System control values
    u = np.zeros((2, t.shape[0]), dtype = np.double)

    # Real velocities robot
    u_r = np.zeros((2, t.shape[0]+1), dtype = np.double)

    
    # Robot Acuators
    motor_left = get_motor(robot, "left wheel", time_step)
    motor_right = get_motor(robot, "right wheel", time_step)

    # Set Robot actuators
    motor_left = set_motor(motor_left)
    motor_right = set_motor(motor_right)

    # Get system sensors
    gps = set_gps("gps", time_step)
    imu = set_imu("inertial unit", time_step)

    # Parameters of the robot
    r = 0.190/2
    l = 0.381
    a = 0.1
    L = [r, l ,a]

    # Get maximun velocities
    velocities_max(L)

    # Get initial Conditions of the system
    x[:, 0] = get_initial(robot, gps, imu, time_step, L)
    u_r[:, 0] = get_system_velocity_initial(motor_right, motor_left, L, robot, time_step)
    #QP = QP_controller(x_d[:, 0], x[:, 0], L)
    QP = QP_controller_all(x_d[:, 0], x[:, 0], L)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    for k in range(0, t.shape[0]):
        if robot.step(time_step) != -1:
            tic = time.time()
            # Control values generation
            #opti = QP_solver(QP, x_d[:, k], x[:, k], L)
            opti = QP_solver_all(QP, x_d[:, k], x[:, k], L)
            #u[:, k] = controlador(x[:, k], x_d[:, k], x_dp[:, k], k1, k2, L)
            u[:, k] = opti
            w_wheels = conversion(u[:, k], L)

            # Send control values to the robot
            set_motors_velocity(motor_right, motor_left, w_wheels)

            # Get system states
            x[: , k+1] = get_states(robot, gps, imu, time_step, L)
            u_r[:, k+1] = get_system_velocity(motor_right, motor_left, L)
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
            print(toc)

    # Set zero values to the robot 
    set_motors_velocity(motor_right, motor_left, np.array([[0], [0]]))

    # Fancy plots
    fig1, ax11 = fancy_plots_1()
    states_x, = ax11.plot(t[:], x[0,0:t.shape[0]],
                    color='#BB5651', lw=2, ls="-")
    states_y, = ax11.plot(t[:], x[1,0:t.shape[0]],
                    color='#69BB51', lw=2, ls="-")
    states_yaw, = ax11.plot(t[:], x[2,0:t.shape[0]],
                    color='#5189BB', lw=2, ls="-")

    states_xd, = ax11.plot(t[:], x_d[0,0:t.shape[0]],
                    color='#BB5651', lw=2, ls="-.")
    states_yd, = ax11.plot(t[:], x_d[1,0:t.shape[0]],
                    color='#69BB51', lw=2, ls="-.")

    ax11.set_ylabel(r"$[states]$", rotation='vertical')
    ax11.set_xlabel(r"$[t]$", labelpad=5)
    ax11.legend([states_x, states_y, states_yaw, states_xd, states_yd],
            [r'$x$', r'$y$', r'$\psi$', r'$x_d$', r'$y_d$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)

    fig1.savefig("states_xyz.eps")
    fig1.savefig("states_xyz.png")
    fig1
    plt.show()

    fig2, ax21 = fancy_plots_1()
    control_u, = ax21.plot(t[:], u[0,0:t.shape[0]],
                    color='#BB5651', lw=2, ls="-")
    control_w, = ax21.plot(t[:], u[1,0:t.shape[0]],
                    color='#69BB51', lw=2, ls="-")

    real_u, = ax21.plot(t[:], u_r[0,0:t.shape[0]],
                    color='#BB5651', lw=2, ls="-.")
    real_w, = ax21.plot(t[:], u_r[1,0:t.shape[0]],
                    color='#69BB51', lw=2, ls="-.")

    ax21.set_ylabel(r"$[Velocities]$", rotation='vertical')
    ax21.set_xlabel(r"$[t]$", labelpad=5)
    ax21.legend([control_u, control_w, real_u, real_w],
            [r'$x\mu$', r'$\omega$', r'$\mu_r$', r'$\omega_r$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax21.grid(color='#949494', linestyle='-.', linewidth=0.5)

    fig2.savefig("control_signals.eps")
    fig2.savefig("control_signals.png")
    fig2
    plt.show()
    return None

   
   
if __name__ == '__main__':
    try:
        robot = Robot()
        main(robot)
        pass
    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
