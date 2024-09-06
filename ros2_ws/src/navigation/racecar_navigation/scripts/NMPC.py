# Vanila MPC with Linearized Kinematics bicycle model around the centerline of the track. 
# It's effective for slow speed path following, 
# At high speed, not so good. 
# Chala
# import cvxpy as cp
import numpy as np
import numpy as np
import math
import sys
from time import sleep
import time as tm 
import pathlib
import argparse
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from scripts import cubic_spline_planner
import scipy as spy
from scipy import sparse
import osqp
from numba import njit 


# working but suboptimal parameter set for the map addis 
NX = 4 # x, y, v, yaw
NU = 2 # acceleration, steering angle
T = 12 #horizon length
DT = 0.02 # time step
Q = sparse.diags([1, 1., .1525, 0.81]) # State cost matrix
R = sparse.diags([0.1, 10.501]) # Control cost matrix
Qf = sparse.diags([10., 10., 0.85, 0.85]) # Final state cost matrix
Rd = sparse.diags([1, 1]) # Control difference cost matrix

ds = 0.5 # [m] distance of each interpolated points
TARGET_SPEED = 8 # [m/s] target speed
N_IND_SEARCH = 15  # Search index number
WB = 0.325 # [m]
MAX_STEER = 0.325 # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(100.0)  # maximum steering speed [rad/s]
MAX_SPEED = 15 # maximum speed [m/s]
MIN_SPEED = -2  # minimum speed [m/s]
MAX_ACCEL = 5.0  # maximum accel [m/ss]
MAX_ITER = 1 # Max iteration
DU_TH = 0.001  # iteration finish parameter

# slow but working
# NX = 4 # x, y, v, yaw
# NU = 2 # acceleration, steering angle
# T = 5 # horizon length
# DT = 0.05 # time step
# Q = sparse.diags([1.0, 1.0, .5, .5]) # State cost matrix
# R = sparse.diags([.1, .1]) # Control cost matrix
# Qf = Q # Final state cost matrix
# Rd = sparse.diags([.1, .1]) # Control difference cost matrix
# ds = 1.0 # [m] distance of each interpolated points
# TARGET_SPEED = 3 # [m/s] target speed
# N_IND_SEARCH = 10  # Search index number
# WB = 0.325 # [m]
# MAX_STEER = 0.325 # maximum steering angle [rad]
# MAX_DSTEER = np.deg2rad(100.0)  # maximum steering speed [rad/s]
# MAX_SPEED = 10 # maximum speed [m/s]
# MIN_SPEED = -10  # minimum speed [m/s]
# MAX_ACCEL = 5.0  # maximum accel [m/ss]


# NX = 4 # x, y, v, yaw
# NU = 2 # velocity, steering angle
# T = 10 # horizon length
# DT = 0.081 # time step
# Q = sparse.diags([1.0, 1.0, .05, .5]) # State cost matrix
# R = sparse.diags([.01, .1]) # Control cost matrix
# Qf = Q # Final state cost matrix
# Rd = sparse.diags([0.01, 0.01]) # Control difference cost matrix
# ds = 0.3 # [m] distance of each interpolated points
# TARGET_SPEED = 3 # [m/s] target speed
# N_IND_SEARCH = 15  # Search index number
# WB = 0.325 # [m]
# MAX_STEER = 0.5 # maximum steering angle [rad]
# MAX_DSTEER = np.deg2rad(100.0)  # maximum steering speed [rad/s]
# MAX_SPEED = 5 # maximum speed [m/s]
# MIN_SPEED = -2  # minimum speed [m/s]
# MAX_ACCEL = 6 # maximum accel [m/ss]

# NX = 4 # x, y, v, yaw
# NU = 2 # acceleration, steering angle
# T = 10 # horizon length
# DT = 0.05 # time step
# Q = sparse.diags([1 , 1 , 1 ,1]) # State cost matrixf
# R = sparse.diags([.1, .1]) # Control cost matrix
# Qf = Q # Final state cost matrix
# Rd = sparse.diags([.1, 1]) # Control difference cost matrix
# ds = 1 # [m] distance of each interpolated points
# TARGET_SPEED = 5 # [m/s] target speed
# N_IND_SEARCH = 10  # Search index number
# WB = 0.325 # [m]
# MAX_STEER = 0.645 # maximum steering angle [rad]
# MAX_DSTEER = np.deg2rad(90.0)  # maximum steering speed [rad/s]
# MAX_SPEED = 10 # maximum speed [m/s]
# MIN_SPEED = -2  # minimum speed [m/s]
# MAX_ACCEL = 5  # maximum accel [m/ss]
# # Iterative paramter
# MAX_ITER                =       3                               # Max iteration
# DU_TH                   =       0.1                             # iteration finish param
# TARGET_SPEED            =       8.0                             # [m/s] target speed
# N_IND_SEARCH            =       10                              # Search index number
# DT                      =       0.3                           # [s] time tick

# # Vehicle parameters
# LENGTH                  =       0.48                            # [m] 
# WIDTH                   =       0.268                           # [m] 
# BACKTOWHEEL             =       0.07                            # [m] 0.1
# WHEEL_LEN               =       0.07                            # [m] 0.1
# WHEEL_WIDTH             =       0.07                            # [m]  0.05
# TREAD                   =       0.5                             # [m]
# WB                      =       0.325                            # [m]
# MAX_STEER               =       np.deg2rad(50.0)                # maximum steering angle [rad]
# MAX_DSTEER              =       np.deg2rad(120.0)                 # maximum steering speed [rad/s]
# MAX_SPEED               =       10.0                            # maximum speed [m/s]
# MIN_SPEED               =       0                               # minimum speed [m/s]
# MAX_ACCEL               =       3.5                             # maximum accel [m/ss]

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
@njit(fastmath=False, cache=True)
def nearest_point_on_trajectory(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.

    Args:
        point: A size 2 numpy array representing the coordinates of the point.
        trajectory: A Nx2 matrix of (x,y) trajectory waypoints.
            - These waypoints must be unique. If they are not unique, a divide by 0 error will occur.

    Returns:
        A tuple containing the following:
        - The nearest point on the trajectory.
        - The distance between the input point and the nearest point on the trajectory.
        - The parameter t, which represents the position of the nearest point on the trajectory relative to the segment it belongs to.
        - The index of the segment in the trajectory that contains the nearest point.

    Notes:
        This method is quite fast, and time constraints should not be an issue as long as the trajectories are not excessively long.
        The order of magnitude for trajectory length: 1000 is approximately 0.0002 seconds computation time (5000fps).
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    projections = trajectory[:-1,:] + (t*diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

def get_linear_model_matrix(v, phi, delta):      # since the linearization is dependent on the accuracy of the model prediction, need to change methods 
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C

def update_state(state, a, delta):
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    while state.yaw >= 2* math.pi:
        state.yaw -= 2*np.pi
    while state.yaw <= -2* math.pi:
        state.yaw += 2*np.pi
    return state

def angle_mod(x, zero_2_2pi=False, degree=False):
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle

def pi_2_pi(angle):
    return angle_mod(angle)

def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

################################## OSQP ##################################


############################# #######################################



# Linear MPC using osqp 
def Linear_MPC(xref , xbar, x0 , dref):
    # # print('xref[3]:', xref[3], 'x0[3]:', x0[3])
    # for i in range(1, T+1):
    #     yaw_diff = xref[3, i] - x0[3]
    #     while yaw_diff > np.pi:
    #         xref[3, i] -= 2*np.pi
    #         yaw_diff = xref[3, i] - x0[3]
    #     while yaw_diff < -np.pi:
    #         xref[3, i] += 2*np.pi
    #         yaw_diff = xref[3, i] - x0[3]
    #     else:
    #         pass
    for i in range(1, T+1):
        yaw_diff = xref[3, i] - x0[3]
        while yaw_diff > np.pi:
            xref[3, i] -= 2*np.pi
            yaw_diff = xref[3, i] - x0[3]
        while yaw_diff < -np.pi:
            xref[3, i] += 2*np.pi
            yaw_diff = xref[3, i] - x0[3]
        else:
            pass
    

    # cast MPC to a QP x = [x u]
    P = sparse.block_diag([sparse.kron(sparse.eye(T),Q ), Qf, 
                           sparse.kron(sparse.eye(T), R)], format= 'csc')
    # linear objective 
    # QQ = sparse.block_diag([Q,Qf]).toarray()
    # print(QQ.shape, Q.shape, xref.shape)
    q_ = ( -Q@xref[:,:T]).flatten(order='F')
    qq_ = ( -Qf@xref[:,T]).flatten(order='F')
    q = np.hstack([q_,qq_,np.zeros(T*NU)])
    # Linear Dynamics
    # Get the first LTV ABCs 
    AA, B, C = get_linear_model_matrix(
            xbar[2, 0], xbar[3, 0], dref[0, 0]) 
    Ax = sparse.kron(sparse.eye(T+1),-sparse.eye(NX)) + sparse.kron(sparse.eye(T+1, k=-1), AA)
    Bu = sparse.kron(sparse.vstack([sparse.lil_matrix((1, T)), sparse.eye(T)]), B)
    Bu = sparse.lil_matrix(Bu)
    Cu = sparse.lil_matrix(np.kron(np.ones((T,1)),C))
    # print(C, Cu.toarray().flatten().shape)
    for t in range(1, T+1):
            AA, B, C = get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            Ax[NX*t:NX*(t+1),NX*(t-1):NX*t] = sparse.lil_matrix(AA) 
            Bu[NX*t:NX*(t+1),NU*(t-1):NU*t] = sparse.lil_matrix(B)
            Cu[NX*t:NX*(t+1),:] = C 
    # print(C, Cu.shape)
    Aeq = sparse.hstack([Ax, Bu])
    # Cu[:NX*1,:] = np.array(x0)
    leq = np.hstack([-np.array(x0), -Cu.toarray().flatten()])
    # print(leq)
    ueq = leq

    Aineq = sparse.eye((T+1)*NX + T*NU)
    lineq = np.hstack([np.kron(np.ones(T+1), np.array([-200,-200, MIN_SPEED, -2*3.14])), np.kron(np.ones(T), np.array([-MAX_ACCEL, -MAX_STEER]))])
    uineq = np.hstack([np.kron(np.ones(T+1), np.array([200 ,200, MAX_SPEED, 2*3.14])), np.kron(np.ones(T),  np.array([MAX_ACCEL, MAX_STEER]))])
    A = sparse.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l, u, warm_start=True, verbose = False)
    sol = prob.solve()
    sol.x
    if sol.info.status != 'solved':
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None
    else:
        x = sol.x[:NX * (T + 1)].reshape((T + 1, NX))
        u = sol.x[NX * (T + 1):NX * (T + 1) + NU * T].reshape((T, NU))
        # print('solution: ', sol.x)
        # print('u:', u[:,0])
        ox = x[:, 0]
        oy = x[:,1]
        ov = x[ :,2]
        oyaw = x[ :,3]
        oa = u[:, 0]
        odelta = u[:, 1]
        # print(NX * (T + 1) + NU * T , sol.x.shape)

    return oa, odelta, ox, oy, oyaw, ov


# def linear_mpc_control(xref, xbar, x0, dref):
#     x = cp.Variable((NX, T + 1))
#     u = cp.Variable((NU, T))

#     cost = 0.0
#     constraints = []

#     for t in range(T):
#         cost += cp.quad_form(u[:, t], R)

#         if t != 0:
            
#             yaw_diff = xref[3, t] - x[3, t]
#             if yaw_diff > np.pi:
#                 yaw_diff -= 2*np.pi
#             elif yaw_diff < -np.pi:
#                 yaw_diff += 2*np.pi
#             else:
#                 pass
            
#             cost += cp.quad_form(xref[:3, t] - x[:3, t], Q[:3,:3]) 
#             cost += Q[3,3] * yaw_diff**2
#              #+ cp.quad_form(u[:, t], R)


#         A, B, C = get_linear_model_matrix(
#             xbar[2, t], xbar[3, t], dref[0, t])
#         # A = sparse.lil_matrix(A)
#         # B = sparse.lil_matrix(B)
#         # C = sparse.lil_matrix(C)
#         constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C.flatten()]
#         # convert A, B and C to space.lil_matrix 

#         if t < (T - 1):
#             cost += cp.quad_form(u[:, t + 1] - u[:, t], Rd)
#             constraints += [cp.abs(u[1, t + 1] - u[1, t]) <=
#                             MAX_DSTEER * DT]
#     cost += cp.quad_form(xref[:, T] - x[:, T], Qf)
#     constraints += [x[:, 0] == x0]
#     constraints += [x[2, :] <= MAX_SPEED]
#     constraints += [x[2, :] >= MIN_SPEED]
#     constraints += [u[0, :] <= MAX_ACCEL, u[0, :] >= -MAX_ACCEL]
#     constraints += [u[1, :] <= MAX_STEER, u[1, :] >= -MAX_STEER]
    
#     # constraints = sparse.lil_matrix(constraints)
#     prob = cp.Problem(cp.Minimize(cost), constraints)
#     prob.solve(solver=cp.OSQP, warm_start = True , verbose=False)

#     if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
#         ox = get_nparray_from_matrix(x.value[0, :])
#         oy = get_nparray_from_matrix(x.value[1, :])
#         ov = get_nparray_from_matrix(x.value[2, :])
#         oyaw = get_nparray_from_matrix(x.value[3, :])
#         oa = get_nparray_from_matrix(u.value[0, :])
#         odelta = get_nparray_from_matrix(u.value[1, :])
#     else:
#         print("Error: Cannot solve mpc..")
#         oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

#     return oa, odelta, ox, oy, oyaw, ov

# MAX_ITER = 1 # Max iteration
# DU_TH = 0.01  # iteration finish parameter

def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    ox, oy, oyaw, ov = None, None, None, None
    since = tm.time()
    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = Linear_MPC(xref, xbar, x0, dref)
        if oa is not None: 
            du = sum(abs(oa - poa)) + sum(abs(od - pod))
            if du <= DU_TH:
                break
        if  oa is None and i == MAX_ITER - 1:
            print("Error: Cannot solve mpc. will use prevous solution.")
            break
    print('Time taken by MPC', tm.time()- since)
    return oa, od, ox, oy, oyaw, ov

def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)
    wpts = np.vstack((cx, cy)).T
    nearest_point , nearest_dist , t , i = nearest_point_on_trajectory(np.array([state.x, state.y]), wpts)
    # if pind >= ind:
    #     ind = pind

    xref[0, 0] = cx[i]
    xref[1, 0] = cy[i]
    xref[2, 0] = sp[i]
    xref[3, 0] = cyaw[i]
    dref[0, 0] = 0.0

    if i + 1 < ncourse: 
        dl = ((cx[i+1]- cx[i])**2 +(cy[i+1]- cy[i])**2 )**0.5
    else:
        dl = ((cx[i]- cx[i-1])**2 +(cy[i]- cy[i-1])**2 )**0.5
    travel = state.v * DT
    dind = int(round(travel/dl))
    if dind>2: 
        for k in range(1,T + 1):
            travel += state.v * DT
            dind = int(round(travel / dl))
            n = (i+dind) % ncourse
            # current_wpt = np.array([cx[n], cy[n]])
            # dind_ = int(round((travel + state.v *DT )/ dl)) 
            # n_ = (i+dind_) % ncourse
            # next_wpt = np.array([cx[n_], cy[n_]])
            # diff = next_wpt - current_wpt
            # yaw = np.arctan2(diff[1], diff[0])
            xref[0, k] = cx[n]
            xref[1, k] = cy[n]
            xref[2, k] = sp[n]
            xref[3, k] = cyaw[n]
            dref[0, k] = 0.0
    else: 
        for k in range(1, T+1):
            n= (i+k+1) % ncourse
            xref[0, k] = cx[n]
            xref[1, k] = cy[n]
            xref[2, k] = sp[n]
            xref[3, k] = cyaw[n]
            dref[0, k] = 0.0
    
    xref[3,:] = smooth_yaw(xref[3,:])
    return xref, i, dref

def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0
    return speed_profile

def smooth_yaw(yaw):
    for i in range(len(yaw)):
        while yaw[i] >= 2* math.pi:
            yaw[i] -= math.pi * 2.0
        while yaw[i] <= -2* math.pi:
            yaw[i] += math.pi * 2.0
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]
        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]
        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]
    return yaw

def smooth_yaw_without(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]
        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]
        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]
    return yaw

