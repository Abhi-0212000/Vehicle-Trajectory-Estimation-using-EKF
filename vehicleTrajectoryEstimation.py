
import pickle
import numpy as np
import matplotlib.pyplot as plt

with open('data/data.pickle', 'rb') as f:
    data = pickle.load(f)

t = data['t']  # timestamps [s]

# Initial data

x_init  = data['x_init'] # initial x position [m]
y_init  = data['y_init'] # initial y position [m]
th_init = data['th_init'] # initial theta position [rad]

# input signal
v  = data['v']  # translational velocity input [m/s]
om = data['om']  # rotational velocity input [rad/s]

# bearing and range measurements, LIDAR constants
b = data['b']  # bearing to each landmarks center in the frame attached to the laser [rad]
r = data['r']  # range measurements [m]
l = data['l']  # x,y positions of landmarks [m]
d = data['d']  # distance between robot center and laser rangefinder [m]


# Remember:that it is neccessary to tune the measurement noise variances `r_var`, `b_var`, `om_var`, `v_var` in order for the filter to perform well. It's a trail and error process

# Variances in control commands and LiDAR measurements
v_var = 0.004  # translation velocity variance
om_var = 0.008  # rotational velocity variance
r_var = 0.001   # range measurements variance
b_var = 0.0005  # bearing measurement variance

# Noise Covariance Matrices
Q = np.diag([v_var, om_var]) # input noise covariance 
R = np.diag([r_var, b_var])  # measurement noise covariance 


# To store the histogram of data. we are storing all the estimated state, covar matrices in below multi-dim array's
x_est = np.zeros([len(v), 3])  # estimated states, x, y, and theta i.e 501 rows and 3 cols. each row is for time k.
P_est = np.zeros([len(v), 3, 3])  # state covariance matrices. 3D array. i.e 501 matrices of size 3x3. each matrix is a covariance matrix containing covar of x, y, theta.

# Storing initial state and covar matrices
x_est[0] = np.array([x_init, y_init, th_init]) # initial state
P_est[0] = np.diag([1, 1, 0.1]) # initial state covariance

# State and Covariance matrices for timestep k-1
P_check = P_est[0]
x_check = x_est[0].reshape(3,1)


# In order for the orientation estimates to coincide with the bearing measurements, it is also neccessary to wrap all estimated $\theta$ values to the $(-\pi , \pi]$ range.

# Wraps angle to (-pi,pi] range

def wraptopi(x):
    if x > np.pi:
        x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
        #print((np.floor(x / (2 * np.pi))))
    elif x < -np.pi:
        x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
    #print(x)
    return x
'''
def wraptopi(x):
    x %= (2*np.pi)
    if x > np.pi:
        x -= 2 * np.pi
    elif x < -np.pi:
        x += 2 * np.pi
    print(x)
    return x
'''


def measurement_update(lk, rk, bk, P_check, x_check):
    x_k = x_check[0]
    y_k = x_check[1]
    theta_k = wraptopi(x_check[2])
    x_l = lk[0]
    y_l = lk[1]
    
    # 1. Compute measurement Jacobian
    d_x = x_l - x_k - d*np.cos(theta_k)
    d_y = y_l - y_k - d*np.sin(theta_k)
    lidar_range = np.sqrt(d_x**2 + d_y**2)
    bearing_meas = np.arctan2(d_y, d_x) - theta_k

    H = np.zeros((2, 3))
    H[0, 0] = -d_x/lidar_range
    H[0, 1] = -d_y/lidar_range
    H[0, 2] = d * (d_x * np.sin(theta_k) - d_y * np.cos(theta_k)) / lidar_range
    H[1, 0] = d_y / lidar_range**2
    H[1, 1] = -d_x / lidar_range**2
    H[1, 2] = -1 - d * (d_y * np.sin(theta_k) + d_x * np.cos(theta_k)) / lidar_range ** 2
    
    #print('H: \n', H)
    
    M = np.diag([1, 1])

    #print('M: \n',M)
    
    # 2. Compute Kalman Gain
    K_k = P_check.dot(H.T).dot(np.linalg.inv(H.dot(P_check).dot(H.T) + M.dot(R).dot(M.T)))
    
    #print('K_k: \n', K_k)

    # 3. Correct predicted state (remember to wrap the angles to [-pi,pi])
    pred_meas = np.array([lidar_range, wraptopi(bearing_meas)])
    x_check = x_check + K_k.dot(np.array([[rk], [wraptopi(bk)]]) - pred_meas)
    x_check[2] = wraptopi(x_check[2])
    #print('x_check : \n', x_check)
    
    # 4. Correct covariance
    P_check = (np.diag([1, 1, 1]) - K_k.dot(H)).dot(P_check)
    #print('P_check : \n', P_check)
    return x_check, P_check

#### 5. Main Filter Loop #######################################################################
for k in range(1, len(t)):  # start at 1 because we've set the initial prediciton
    delta_t = t[k] - t[k - 1]  # time step (difference between timestamps)

    # 1. Update state with odometry readings (remember to wrap the angles to [-pi,pi])
    
    theta = wraptopi(x_check[2])
    sin_theta = np.sin(theta)[0]
    cos_theta = np.cos(theta)[0]
    
    x_check = x_check + np.array([[cos_theta, 0], [sin_theta, 0], [0, 1.0]]).dot(np.array([[v[k-1]], [om[k-1]]])).dot(delta_t)
    
    # 2. Motion model jacobian with respect to last state
    F = np.array([[1, 0, -delta_t*v[k-1]*sin_theta], 
                     [0, 1, delta_t*v[k-1]*cos_theta], 
                     [0, 0, 1.0]])

    # 3. Motion model jacobian with respect to noise
    L = np.array([[delta_t * cos_theta, 0],
                    [delta_t * sin_theta, 0],
                    [0, delta_t]])

    # 4. Propagate uncertainty
    P_check = F.dot(P_check.dot(F.T)) + L.dot(Q.dot(L.T))
    
    # 5. Update state estimate using available landmark measurements
    for i in range(len(r[k])):
        x_check, P_check = measurement_update(l[i], r[k, i], b[k, i], P_check, x_check)

    # Set final state predictions for timestep
    x_est[k, 0] = x_check[0]
    x_est[k, 1] = x_check[1]
    x_est[k, 2] = x_check[2]
    P_est[k, :, :] = P_check


# plot the resulting state estimates:


e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(x_est[:, 0], x_est[:, 1])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Estimated trajectory')
plt.show()

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(t[:], x_est[:, 2])
ax.set_xlabel('Time [s]')
ax.set_ylabel('theta [rad]')
ax.set_title('Estimated trajectory')
plt.show()
