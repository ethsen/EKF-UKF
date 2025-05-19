import numpy as np
from scipy import io
from quaternion import Quaternion

#data files are numbered on the server.
#for exmaple imuRaw1.mat, imuRaw2.mat and so on.
#write a function that takes in an input number (1 through 6)
#reads in the corresponding imu Data, and estimates
#roll pitch and yaw using an unscented kalman filter
def get_readings(i,accel,gyro):
    """
    get_readings returns the calibrated readings from the IMU
    """

    acc_sens_x = 32.959
    acc_sens_y = 32.932
    acc_sens_z = 33.568
    acc_bias_x = 512.867
    acc_bias_y = 502.805
    acc_bias_z = 502.43
    
    gyro_sens_x = 182.831
    gyro_sens_y = 182.324
    gyro_sens_z = 182.891
    gyro_bias_x = 373.077
    gyro_bias_y = 373.144
    gyro_bias_z = 369.7


    ax = (accel[i][0] -acc_bias_x)*(3300 /(1023*acc_sens_x))
    ay = (accel[i][1] -acc_bias_y)*(3300 /(1023*acc_sens_y))
    az = (accel[i][2] -acc_bias_z)*(3300 /(1023*acc_sens_z))
    wx = (gyro[i][0] - gyro_bias_x)*(3300 /(1023*gyro_sens_x))
    wy = (gyro[i][1] - gyro_bias_y)*(3300 /(1023*gyro_sens_y)) 
    wz = (gyro[i][2] - gyro_bias_z)*(3300 /(1023*gyro_sens_z))

    return -ax,-ay,az,wx,wy,wz

def initialize_state(accel,gyro):
    """
    initialize_state speaks for itself
    """
    state = np.array([1,0,0,0,0,0,0])

    cov = np.eye(6)
    Q = np.eye(6)*0.1 # Measurement Noise 
    R = np.eye(6) * 0.01 # Process Noise

    return state, cov, Q, R

def gen_sigma_points(state,cov, R):
    """
    Using the approach in the EK paper, gen_sigma_points
    generates sigma points around a given mean (state)
    """
    q_k_prev = Quaternion(state[0],state[1:4])
    w_k_prev = state[4:]

    S = np.linalg.cholesky((cov + R)* np.sqrt(6))
    W = np.concatenate([S,-S]).T

    X = np.zeros((7,12))
    q_w = Quaternion()
    for i in range(X.shape[-1]):
        q_w.from_axis_angle(W[0:3,i])
        q = q_k_prev * q_w
        X[:4,i] = q.q
        X[4:,i] = w_k_prev + W[3:,i]

    return X

def transform_points(X,dt):
    """
    Using the approach in the EK paper, transform_points
    transforms the sigma points into quaternion/ang velo state
    """
    Y = np.zeros((7,12))
    q_delta = Quaternion()
    for i in range(Y.shape[-1]):
        q_delta.from_axis_angle(X[4:,i]*dt)
        q_k = Quaternion(X[0,i],X[1:4,i])
        q = q_k * q_delta

        Y[:4,i] = (q).q
        Y[4:,i] = X[4:,i]

    return Y

def quat_GD(state,sigma_pts):
    """
    Gradient Descent algorithm to get q_mean based off of
    EK paper and handout
    """
    q_mean = Quaternion(state[0],state[1:4])
    e_mean = np.array([10,10,10])
    e_quat = Quaternion(1,[0,0,0])

    while np.linalg.norm(e_mean) > 0.001:
        E = np.zeros((3,12))
        for i in range(E.shape[-1]):
            q_i = Quaternion(sigma_pts[0,i],sigma_pts[1:4,i])
            err = q_i * q_mean.inv()
            rot_vec = err.axis_angle()
            #print(rot_vec)
            E[:,i] = rot_vec

        e_mean = np.mean(E,axis =1)
        #print(e_mean)
        e_quat.from_axis_angle(e_mean)
        q_mean = e_quat * q_mean
        
    return q_mean

def get_omega_mean(Y):
    """
    get_omega_mean speaks for itself
    """
    omega = Y[4:,:]
    omega_mean = np.mean(omega,axis=1)
    return omega_mean

def get_cov_mean(quat_mean,omega_mean,Y):
    """
    Using the approach in the EK paper, I first find W'
    which I then use to calculate the mean covariance
    """
    W_prime = np.zeros((6,12))

    for i in range(W_prime.shape[-1]):
        q_i = Quaternion(Y[0,i],Y[1:4,i])
        quat_diff = q_i * quat_mean.inv()
        r_w = quat_diff.axis_angle()
        w_w = Y[4:7,i] - omega_mean
        W_prime[:,i] = np.hstack((r_w,w_w))
    
    cov = (W_prime @ W_prime.T)/12

    return cov, W_prime

def gen_meas_set(sigma_pts,g_quat):
    """
    Using the approach in the EK paper, I find the 
    measurement set Z using the transformed sigma points
    and the gravity quaternion
    """
    meas_set = np.zeros((6,12))
    for i in range(sigma_pts.shape[-1]):
        q_z = Quaternion(sigma_pts[0,i],sigma_pts[1:4,i])
        g_prime = (q_z.inv() * g_quat * q_z)
        meas_set[:,i] = np.concatenate((g_prime.vec(),sigma_pts[4:,i]))
    return meas_set

def calc_meas_cov(Z, Q,w_prime):
    """
    Using the approach in the EK paper and info in the
    handout/notes I calculate Z_k_mean and P_vv and P_xz
    """
    Z_k_mean = np.mean(Z,axis= 1).reshape((6,1))
    d = Z - Z_k_mean
    cov_zz = (d @ d.T)/12
    cov_vv = cov_zz + Q

    cov_xz = (w_prime @ d.T)/12

    return Z_k_mean,cov_vv,cov_xz

def update(x_mean,cov_mean, K_k,cov_vv,innovation):
    """
    Using the approach in the EK paper and info in the
    handout/notes I update the state and the covariance
    """

    q_v = Quaternion()
    state_q = Quaternion(x_mean[0],x_mean[1:4])
    
    v_k = K_k @ innovation.T
    q_v.from_axis_angle(v_k[0:3])

    state_q = q_v * state_q
    state_q.normalize()

    w_k = x_mean[4:] + v_k[3:]

    state = np.concatenate((state_q.q,w_k))
    cov = cov_mean - (K_k @ cov_vv @ K_k.T)
    return state,cov,state_q

def estimate_rot(data_num=1):
    imu = io.loadmat('imu/imuRaw'+str(data_num)+'.mat')
    accel = imu['vals'][0:3,:].T
    gyro = imu['vals'][3:6,:].T
    gyro[:,[2,0]] = gyro[:,[0,2]]
    gyro[:,[1,0]] = gyro[:,[0,1]]
    T = np.shape(imu['ts'])[1]
    time =imu['ts'][0]
    roll_col = []
    pitch_col = []
    yaw_col = []
    g_quat = Quaternion(0,[0,0,9.8])

    state,cov,Q,R = initialize_state(accel,gyro) # This is the initial start

    for i in range(T):
        #Read in readings
        ax,ay,az,wx,wy,wz =  get_readings(i,accel,gyro) 

        if i == T-1:
            dt = time[-1] - time[T-2]
        else:
            dt = time[i+1] - time[i]

        #Generate sigma points
        X = gen_sigma_points(state,cov,R) 

        #Transform sigma points
        Y = transform_points(X,dt) 

        #Get the quat and ang velo means and errors
        q_mean = quat_GD(X[0:4,0],Y)
        omega_mean = get_omega_mean(Y)
        x_mean = np.concatenate([q_mean.q,omega_mean])
        #print(x_mean)

        # Compute covariance mean
        P_k_mean,W_prime = get_cov_mean(q_mean,omega_mean,Y)
        #print(P_k_mean)

        # Compute the measurement set
        Z = gen_meas_set(Y,g_quat)
        #print(Z)

        #Compute the covariances and get the mean observation
        Z_k_mean,cov_vv,cov_xz = calc_meas_cov(Z,Q,W_prime)
        #print(Z_k_mean)
        #print(cov_xz)

        innovation = np.array([ax,ay,az,wx,wy,wz]) - Z_k_mean.flatten()

        #Calculate Kalman gain
        K_k = cov_xz @ np.linalg.inv(cov_vv)
        #print(K_k)
        
        #Update
        #print(x_mean)
        state,cov,state_q = update(x_mean,P_k_mean,K_k,cov_vv,innovation)
        #print(state)
        roll,pitch,yaw = state_q.euler_angles()
        roll_col.append(roll)
        pitch_col.append(pitch)
        yaw_col.append(yaw)

    return roll_col,pitch_col,yaw_col

if __name__ == '__main__':
    roll, pitch, yaw = estimate_rot(3)
