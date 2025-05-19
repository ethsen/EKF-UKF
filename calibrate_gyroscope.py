from scipy import io
import numpy as np
import matplotlib.pyplot as plt
from quaternion import Quaternion

def calibrate(data_num):
    imu = io. loadmat ('imu/imuRaw'+str( data_num )+'.mat ')
    accel = (imu['vals'][0:3,:]).T
    gyro = imu['vals'][3:6,:].T
    T = np.shape(imu['ts'])[1]
    g = np.array([0,0,-9.81])
    vicon = io.loadmat('vicon/viconRot' + str(data_num) + '.mat')

    gyro[:,[2,0]] = gyro[:,[0,2]]
    gyro[:,[1,0]] = gyro[:,[0,1]]
    rot_matrices =vicon['rots']
    time =imu['ts'][0]

    if rot_matrices.shape[-1] > gyro.shape[0]:
        rot = rot_matrices[:,:,:gyro.shape[0]]
    else:
        gyro = gyro[:rot_matrices.shape[-1]] 
        rot = rot_matrices

    q = Quaternion()
    count = 0
    true_w = np.zeros_like(gyro,dtype= float)
    for i in range(0,rot.shape[-1]-1,1):
        r_old = rot[:,:,i]
        r_new = rot[:,:,i+1]
        t_old = time[i]
        t_new = time[i+1]

        rot_diff = r_new @ r_old.T
        q.from_rotm(rot_diff)
        angle = 2* np.arccos(q.scalar())
        axis = q.vec()


        true_w[i] = (axis *angle)/(t_new-t_old)
        #if np.linalg.norm(true_w[i]) <0.1:
        #    count+=1



    thresh_x = .018
    thresh_y = .015
    thresh_z = .005



    norms = np.abs(true_w[:,0]) < thresh_x

    true_w_adj = true_w[norms]
    gyro_adj = gyro[norms]

    # X-AXIS
    b = (gyro_adj)[:,0] 
    a_true = true_w_adj[:,0] * (180/np.pi)
    a = np.column_stack((b,np.ones_like(b)))

    x = np.linalg.lstsq(a,a_true,rcond=None)[0]

    sens_x = (3300/(1023*x[0]))*(np.pi/180)
    bias_x = -x[1]/x[0]


    norms = np.abs(true_w[:,1]) < thresh_y 

    true_w_adj = true_w[norms]
    gyro_adj = gyro[norms]
    # Y-AXIS
    b = (gyro_adj)[:,1] 
    a_true = true_w_adj[:,1]* (180/np.pi)
    a = np.column_stack((b,(1)*np.ones_like(b)))

    x = np.linalg.lstsq(a,a_true,rcond=None)[0]

    sens_y = (3300/(1023*x[0]))*(np.pi/180)
    bias_y = -x[1]/x[0]

    norms = np.abs(true_w[:,2]) < thresh_z
    true_w_adj = true_w[norms]
    gyro_adj = gyro[norms]

    # Z-AXIS
    b = (gyro_adj)[:,2]
    a_true = true_w_adj[:,2]* (180/np.pi)
    a = np.column_stack((b,(1)*np.ones_like(b)))

    x = np.linalg.lstsq(a,a_true,rcond=None)[0]

    sens_z = (3300/(1023*x[0]))*(np.pi/180)
    bias_z = -x[1]/x[0]

    return np.array([bias_x,bias_y,bias_z,sens_x,sens_y,sens_z])

def getWeights(configs):
    weights  = []
    for config in configs:
        err = 0
        bias_x,bias_y,bias_z,sens_x,sens_y,sens_z = config
        for d in range(3):
            data_num = d+1
            imu = io. loadmat ('imu/imuRaw'+str( data_num )+'.mat ')
            accel = (imu['vals'][0:3,:]).T
            gyro = imu['vals'][3:6,:].T
            T = np.shape(imu['ts'])[1]
            g = np.array([0,0,-9.81])
            vicon = io.loadmat('vicon/viconRot' + str(data_num) + '.mat')


            gyro[:,[2,0]] = gyro[:,[0,2]]
            gyro[:,[1,0]] = gyro[:,[0,1]]
            rot_matrices =vicon['rots']
            time =imu['ts'][0]

            if rot_matrices.shape[-1] > gyro.shape[0]:
                rot = rot_matrices[:,:,:gyro.shape[0]]
            else:
                gyro = gyro[:rot_matrices.shape[-1]] 
                rot = rot_matrices

            q = Quaternion()
            true_w = np.zeros_like(gyro,dtype= float)
            for i in range(0,rot.shape[-1]-1,1):
                r_old = rot[:,:,i]
                r_new = rot[:,:,i+1]
                t_old = time[i]
                t_new = time[i+1]

                rot_diff = r_new @ r_old.T
                q.from_rotm(rot_diff)
                angle = 2* np.arccos(q.scalar())
                axis = q.vec()


                true_w[i] = (axis *angle)/(t_new-t_old)

            for i in range(len(true_w)-1):
                #alpha = np.arctan2(rot[1,0],rot[0,0]) #YAW
                wx_est = (gyro[i][0] - bias_x)*(3300/(1023*sens_x)) #* (np.pi/180)
                wy_est = (gyro[i][1] - bias_y)*(3300/(1023*sens_y)) #* (np.pi/180)
                wz_est = (gyro[i][2] - bias_z)*(3300/(1023*sens_z)) #* (np.pi/180)
                dt = time[i+1] -time[i]
                w_est = np.array([wx_est,wy_est,wz_est])*dt
                err +=  np.linalg.norm(true_w[i]-w_est) 

        weights.append(1/err)

    return np.array(weights)/np.sum(weights)


data_1_config = calibrate(1)
data_2_config = calibrate(2)

config_col = np.array([data_1_config,data_2_config])
w1,w2 = getWeights(config_col)
best = w1*data_1_config + w2*data_2_config 
<<<<<<< HEAD
sens_x = 182.831
sens_y = 182.324
sens_z = 182.891
bias_x = 373.077
bias_y = 373.144
bias_z = 369.7
=======
bias_x,bias_y,bias_z,sens_x,sens_y,sens_z = best
>>>>>>> 03fdaf0b1eea731625190615acbb7b605eda2a2f

print('sens_x: ',sens_x)
print('sens_y: ',sens_y)
print('sens_z: ',sens_z)

print('bias_x: ',bias_x)
print('bias_y: ',bias_y)
print('bias_z: ',bias_z)
(fig, axes) = plt.subplots(nrows=3, ncols=3, sharex=True, num='$\omega$ vs Time')
<<<<<<< HEAD
fig.supxlabel('Time (s)')
fig.supylabel('rad/s')
=======

>>>>>>> 03fdaf0b1eea731625190615acbb7b605eda2a2f
for j in range(3):
    
    data_num = j+1
    imu = io. loadmat ('imu/imuRaw'+str( data_num )+'.mat ')
    accel = (imu['vals'][0:3,:]).T
    gyro = imu['vals'][3:6,:].T
    T = np.shape(imu['ts'])[1]
    g = np.array([0,0,-9.81])
    vicon = io.loadmat('vicon/viconRot' + str(data_num) + '.mat')
    time =vicon['ts'][0]
    gyro[:,[2,0]] = gyro[:,[0,2]]
    gyro[:,[1,0]] = gyro[:,[0,1]]
    rot_matrices =vicon['rots']


    if rot_matrices.shape[-1] > gyro.shape[0]:
        rot = rot_matrices[:,:,:gyro.shape[0]]
    else:
        gyro = gyro[:rot_matrices.shape[-1]] 
        rot = rot_matrices

    w_est = []

    q = Quaternion()
    true_w = np.zeros_like(gyro,dtype= float)
    for i in range(0,rot.shape[-1]-1):
        r_old = rot[:,:,i]
        r_new = rot[:,:,i+1]
        t_old = time[i]
        t_new = time[i+1]

        rot_diff = r_new @ r_old.T
        q.from_rotm(rot_diff)
        angle = 2* np.arccos(q.scalar())
        axis = q.vec()
        wx_est = (gyro[i][0] - bias_x)*(3300/(1023*sens_x)) 
        wy_est = (gyro[i][1] - bias_y)*(3300/(1023*sens_y)) 
        wz_est = (gyro[i][2] - bias_z)*(3300/(1023*sens_z))

        w_est.append([wx_est,wy_est,wz_est])
        true_w[i] = (axis *angle)/(t_new-t_old)

    w_est = np.array(w_est)
    # Position and Velocity vs. Time

    ax = axes[0][j]
    ax.plot(w_est[:,0],'r',linestyle='dashed')
    ax.plot(true_w[:,0],'g')
    ax.set_ylabel('rad')
    ax.set_xlabel('time,s ')
    ax.grid('major')


    ax = axes[1][j]
    ax.plot(w_est[:,1],'r',linestyle='dashed')
    ax.plot(true_w[:,1],'b')
    ax.set_ylabel('rad')
    ax.set_xlabel('time,s ')
    ax.grid('major')

    ax = axes[2][j]
    ax.plot(w_est[:,2],'r',linestyle='dashed')
    ax.plot(true_w[:,2],'k')
    ax.set_ylabel('rad')
    ax.set_xlabel('time,s ')
    ax.grid('major')

plt.show()