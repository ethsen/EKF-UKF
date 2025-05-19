from scipy import io
import numpy as np
import matplotlib.pyplot as plt

def calibrate(data_num,x_thresh,y_thresh,z_thresh):
    imu = io. loadmat ('imu/imuRaw'+str( data_num )+'.mat ')
    accel = (imu['vals'][0:3,:]).T
    gyro = imu['vals'][3:6,:]
    T = np.shape(imu['ts'])[1]
    g = np.array([0,0,-9.81])
    vicon = io.loadmat('vicon/viconRot' + str(data_num) + '.mat')
    rot_matrices =vicon['rots']

    if rot_matrices.shape[-1] > accel.shape[0]:
        rot = rot_matrices[:,:,:accel.shape[0]]
        acc = accel
    else:
        acc = accel[:rot_matrices.shape[-1]] #* np.array([-1,-1,1])
        rot = rot_matrices

    acc_true = (rot[:,2,:]*9.81).T

    norms = np.abs(acc_true[:,0]) > x_thresh#np.linalg.norm(true_w,axis = 1) < 0.1
    acc_adj = acc[norms]
    acc_true_adj = acc_true[norms]

    b = (acc_adj)[:,0] * (3300/1023)
    a_true = acc_true_adj[:,0]
    a = np.column_stack((a_true,(1)*np.ones_like(a_true)))

    x = np.linalg.lstsq(a,b,rcond=None)[0]

    sens_x = x[0]
    bias_x = x[1]*(1023/3300)
    norms = np.abs(acc_true[:,1]) > y_thresh#np.linalg.norm(true_w,axis = 1) < 0.1
    acc_adj = acc[norms]
    acc_true_adj = acc_true[norms]

    b = (acc_adj)[:,1] * (3300/1023)
    a_true = acc_true_adj[:,1]
    a = np.column_stack((a_true,(1)*np.ones_like(a_true)))

    x = np.linalg.lstsq(a,b,rcond=None)[0]

    sens_y = x[0]
    bias_y = x[1]*(1023/3300)

    norms = np.abs(acc_true[:,2]) > z_thresh#np.linalg.norm(true_w,axis = 1) < 0.1
    acc_adj = acc[norms]
    acc_true_adj = acc_true[norms]

    b = (acc_adj)[:,2]* (3300/1023)
    a_true = acc_true_adj[:,2]
    a = np.column_stack((a_true,(1)*np.ones_like(a_true)))

    x = np.linalg.lstsq(a,b,rcond=None)[0]

    sens_z = x[0]
    bias_z = x[1]*(1023/3300)

    return np.array([bias_x,bias_y,bias_z,sens_x,sens_y,sens_z])

#bias = 500
#sensitivity = 35

def getWeights(configs):
    weights  = []
    for config in configs:
        err = 0
        bias_x,bias_y,bias_z,sens_x,sens_y,sens_z = config
        for d in range(3):
            data_num = d+1
            imu = io. loadmat ('imu/imuRaw'+str( data_num )+'.mat ')
            accel = (imu['vals'][0:3,:]).T
            gyro = imu['vals'][3:6,:]
            T = np.shape(imu['ts'])[1]
            g = np.array([0,0,-9.81])
            vicon = io.loadmat('vicon/viconRot' + str(data_num) + '.mat')


            rot_matrices =vicon['rots']


            if rot_matrices.shape[-1] > accel.shape[0]:
                rot = rot_matrices[:,:,:accel.shape[0]]
                acc = accel
            else:
                acc = accel[:rot_matrices.shape[-1]] #* np.array([-1,-1,1])
                rot = rot_matrices

            for i in range(len(acc)):

                gamma = np.arctan2(rot[2,1,i],rot[2,2,i])  #ROLL
                beta = np.arctan2(-rot[2,0,i], np.sqrt(rot[2,1,i]**2 + rot[2,2,i]**2)) #PITCH
               
                #alpha = np.arctan2(rot[1,0],rot[0,0]) #YAW
                ax = (acc[i][0] -bias_x)*(-3300/(1023*sens_x))
                ay = (acc[i][1] -bias_y)*(-3300/(1023*sens_y))
                az = (acc[i][2] -bias_z)*(3300/(1023*sens_z))

                gamma_est = np.arctan2(ay,az)
                beta_est = np.arctan2(-ax,np.sqrt(ay**2 +az**2))

                err += np.linalg.norm([gamma- gamma_est,beta-beta_est])
        
        weights.append(1/err)

    return np.array(weights)/np.sum(weights)


data_1_config = calibrate(1,1,1,1)
data_2_config = calibrate(2,1,1,4.5)
data_3_config = calibrate(3,5.9,6.3, 6.7)

config_col = np.array([data_1_config,data_2_config,data_3_config])
w1,w2,w3 = getWeights(config_col)
best = w1*data_1_config + w2*data_2_config + w3*data_3_config

bias_x,bias_y,bias_z,sens_x,sens_y,sens_z = best

print('sens_x: ',sens_x)
print('sens_y: ',sens_y)
print('sens_z: ',sens_z)

print('bias_x: ',bias_x)
print('bias_y: ',bias_y)
print('bias_z: ',bias_z)

(fig, axes) = plt.subplots(nrows=2, ncols=3, sharex=True, num='Acc vs Time')
<<<<<<< HEAD
fig.supxlabel('Time (s)')
fig.supylabel('rad/s')
=======

>>>>>>> 03fdaf0b1eea731625190615acbb7b605eda2a2f
for j in range(3):
    
    data_num = j+1
    imu = io. loadmat ('imu/imuRaw'+str( data_num )+'.mat ')
    accel = (imu['vals'][0:3,:]).T
    gyro = imu['vals'][3:6,:]
    T = np.shape(imu['ts'])[1]
    g = np.array([0,0,-9.81])
    vicon = io.loadmat('vicon/viconRot' + str(data_num) + '.mat')


    rot_matrices =vicon['rots']


    if rot_matrices.shape[-1] > accel.shape[0]:
        rot = rot_matrices[:,:,:accel.shape[0]]
        acc = accel
    else:
        acc = accel[:rot_matrices.shape[-1]] #* np.array([-1,-1,1])
        rot = rot_matrices



    true_roll = []
    true_pitch = []
<<<<<<< HEAD

    est_roll = []
    est_pitch = []
    err = 0
    for i in range(len(acc)):

        gamma = np.arctan2(rot[2,1,i],rot[2,2,i])  #ROLL
        beta = np.arctan2(-rot[2,0,i], np.sqrt(rot[2,1,i]**2 + rot[2,2,i]**2)) #PITCH
        
        true_roll.append(gamma)
        true_pitch.append(beta)
        #alpha = np.arctan2(rot[1,0],rot[0,0]) #YAW
        ax = (acc[i][0] -bias_x)*(-3300/(1023*sens_x))
        ay = (acc[i][1] -bias_y)*(-3300/(1023*sens_y))
        az = (acc[i][2] -bias_z)*(3300/(1023*sens_z))

        est_roll.append(np.arctan2(ay,az))
        est_pitch.append(np.arctan2(-ax,np.sqrt(ay**2 +az**2)))

        err += np.linalg.norm([gamma- est_roll,beta-est_pitch])


    true_roll = np.array(true_roll)
    true_pitch = np.array(true_pitch)
    est_roll = np.array(est_roll)
    est_pitch = np.array(est_pitch)
    # Position and Velocity vs. Time
    

    ax = axes[0][j]
    ax.plot(true_pitch,'g', label = 'True Pitch')
    ax.plot(est_pitch,'r',linestyle='dashed',label = 'Est Pitch')
    ax.grid('major')
    ax.legend()


    ax = axes[1][j]
    ax.plot(true_roll , 'b', label = 'True Roll')
    ax.plot(est_roll,'r',linestyle='dashed',label = 'Est Roll')
    ax.grid('major')
    ax.legend()


=======

    est_roll = []
    est_pitch = []
    err = 0
    for i in range(len(acc)):

        gamma = np.arctan2(rot[2,1,i],rot[2,2,i])  #ROLL
        beta = np.arctan2(-rot[2,0,i], np.sqrt(rot[2,1,i]**2 + rot[2,2,i]**2)) #PITCH
        
        true_roll.append(gamma)
        true_pitch.append(beta)
        #alpha = np.arctan2(rot[1,0],rot[0,0]) #YAW
        ax = (acc[i][0] -bias_x)*(-3300/(1023*sens_x))
        ay = (acc[i][1] -bias_y)*(-3300/(1023*sens_y))
        az = (acc[i][2] -bias_z)*(3300/(1023*sens_z))

        est_roll.append(np.arctan2(ay,az))
        est_pitch.append(np.arctan2(-ax,np.sqrt(ay**2 +az**2)))

        err += np.linalg.norm([gamma- est_roll,beta-est_pitch])


    true_roll = np.array(true_roll)
    true_pitch = np.array(true_pitch)
    est_roll = np.array(est_roll)
    est_pitch = np.array(est_pitch)
    # Position and Velocity vs. Time

    ax = axes[0][j]
    ax.plot(true_pitch,'g')
    ax.plot(est_pitch,'g',linestyle='dashed')
    ax.set_ylabel('rad')
    ax.set_xlabel('time,s ')
    ax.grid('major')


    ax = axes[1][j]
    ax.plot(true_roll , 'r', )
    ax.plot(est_roll,'r',linestyle='dashed')
    ax.set_ylabel('rad')
    ax.set_xlabel('time,s ')
    ax.grid('major')
>>>>>>> 03fdaf0b1eea731625190615acbb7b605eda2a2f

plt.show()