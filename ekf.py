import numpy as np
import matplotlib.pyplot as plt
def genDataset():
    a = -1
    x_k = []
    y_k = []
    x_0  = np.random.normal(1,2)
    x_k.append(x_0)

    for i in range(100):
        nu = np.random.normal(0,0.5)
        y_new = np.sqrt(x_k[i]**2 +1) + nu
        y_k.append(y_new)

        eps_k = np.random.normal(0,1)
        x_new = a * x_k[i] + eps_k
        x_k.append(x_new)

    return y_k


if __name__ == '__main__':
    a_init = -.1
    a_sd_init = 10
    x_init = 1
    x_sd_init = 2
    mu = np.array([a_init * x_init, a_init]).reshape((2,1))

    A =np.array([[a_init,x_init],
                    [0,1]])

    cov = np.array([[x_sd_init,0],
                    [0, a_sd_init]])
    R = np.array([[1,0],
                  [0,0]])
    a_sd = []
    a_sd.append(cov[1,1])

    cov = A @ cov @ A.T + R

    C = np.array([(x_init * a_init) / np.sqrt((x_init * a_init)**2 + 1),0])
    # Q = 1/2
    I = np.eye(2)
    y_k = genDataset()

    a_est = []
    a_est.append(mu[1][0])
    for i in range(len(y_k)):
        K = ((cov @ C.T) / (C @ cov @ C.T + 0.5)).reshape((2,1))
        mu += K*(y_k[i] - np.sqrt((mu[0][0])**2 +1))
        cov = (I - (K @ C.reshape((1,2)))) @ cov
        a_est.append(mu[1][0])
        a_sd.append(cov[1,1])
        
        # PROPAGATE DYNAMICS
        A[0,0] = mu[1][0] 
        A[0,1] = mu[0][0] 
        mu[0][0] *= mu[1][0]
        cov = A @ cov @ A.T + R
        C[0] = (mu[0][0]) / np.sqrt((mu[0][0])**2 + 1)

    a_sd = np.sqrt(np.array(a_sd))
    a_est = np.array(a_est)
    a_true =  a_est * 0 + -1
    lower = a_est - a_sd
    upper = a_est + a_sd
    
    print('a_est', a_est[-1])
    plt.plot(a_est, color = 'k', label = 'Estimated a')
    plt.plot(upper, color = 'r', label = '$\mu + \sigma$',linestyle = 'dashed')
    plt.plot(lower, color = 'b', label = '$\mu - \sigma$',linestyle = 'dashed')
    plt.plot(a_true, color = 'g',label = 'True a')
    plt.xlabel("Time (k)")
    plt.ylabel("a-value")
    
    plt.legend()
    plt.show()