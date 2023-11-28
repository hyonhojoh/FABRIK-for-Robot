import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-whitegrid')

def distance(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(1/2)

def isReachable(d_target, d_total):
    if d_target > d_total:
        return False
    else:
        return True
    
def cal_theta_forward(p, i):
    theta_axis = np.arctan2(p[i-1][1]-p[i][1], p[i-1][0]-p[i][0])
    theta_p = np.arctan2(p[i-2][1]-p[i-1][1], p[i-2][0]-p[i-1][0])
    return theta_axis-theta_p # radian
    

def cal_theta_backward(p, i):
    theta_axis = np.arctan2(p[i+1][1]-p[i][1], p[i+1][0]-p[i][0])
    theta_p = np.arctan2(p[i+2][1]-p[i+1][1], p[i+2][0]-p[i+1][0])
    return theta_axis-theta_p # radian
    
    
def iterate(p, d, target, tol, history, theta_cons):
    theta_cons = np.radians(theta_cons)
    base = np.copy(p[0])
    dif_A = distance(p[-1], target)
    history[0] = np.copy(p)
    n = 0

    while dif_A > tol:
        # Stage 1: Forward reaching(target -> base)
        p[-1] = target
        for i in range(len(p)-1, 0, -1): # i=len(p)-1,len(p)-2...,1
            if i == len(p) - 1: # initial process
                p[i-1] = p[i] + ((p[i-1]-p[i])/distance(p[i-1],p[i]))*d[i-1]
                theta = cal_theta_forward(p, i) # radian
            elif i == 1: # final process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    p[i-1] = p[i] + ((p[i-1]-p[i])/distance(p[i-1],p[i]))*d[i-1]
                else: # 허용 범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(-theta_cons[i]), -np.sin(-theta_cons[i])],
                                             [np.sin(-theta_cons[i]), np.cos(-theta_cons[i])]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
                    else:
                        rev_mat = np.array([[np.cos(theta_cons[i]), -np.sin(theta_cons[i])],
                                             [np.sin(theta_cons[i]), np.cos(theta_cons[i])]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
            else: # 나머지 process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    p[i-1] = p[i] + ((p[i-1]-p[i])/distance(p[i-1],p[i]))*d[i-1]                    
                else: # 허용범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(-theta_cons[i]), -np.sin(-theta_cons[i])],
                                             [np.sin(-theta_cons[i]), np.cos(-theta_cons[i])]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
                    else:
                        rev_mat = np.array([[np.cos(theta_cons[i]), -np.sin(theta_cons[i])],
                                             [np.sin(theta_cons[i]), np.cos(theta_cons[i])]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
                theta = cal_theta_forward(p, i) # radian    
                        
        # Stage 2: Backward reaching(base -> target)
        p[0] = base
        for i in range(len(p)-1):
            if i == 0: # initial process
                p[i+1] = p[i] + ((p[i+1]-p[i])/distance(p[i+1],p[i]))*d[i]
                theta = cal_theta_backward(p, i) # radian                
            elif i == len(p)-2: # final process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    p[i+1] = p[i] + ((p[i+1]-p[i])/distance(p[i+1],p[i]))*d[i]
                else: # 허용 범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(-theta_cons[i]), -np.sin(-theta_cons[i])],
                                             [np.sin(-theta_cons[i]), np.cos(-theta_cons[i])]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
                    else:
                        rev_mat = np.array([[np.cos(theta_cons[i]), -np.sin(theta_cons[i])],
                                             [np.sin(theta_cons[i]), np.cos(theta_cons[i])]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat               
            else: # 나머지 process
                if abs(theta) <= theta_cons[i]: # 허용 범위 안에 있는 경우
                    p[i+1] = p[i] + ((p[i+1]-p[i])/distance(p[i+1],p[i]))*d[i]                    
                else: # 허용범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(-theta_cons[i]), -np.sin(-theta_cons[i])],
                                             [np.sin(-theta_cons[i]), np.cos(-theta_cons[i])]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
                    else:
                        rev_mat = np.array([[np.cos(theta_cons[i]), -np.sin(theta_cons[i])],
                                             [np.sin(theta_cons[i]), np.cos(theta_cons[i])]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
                theta = cal_theta_backward(p, i) # radian        
                
               
        dif_A = distance(p[-1], target)
        
        n = n + 1
        print(n)

    history[1] = np.copy(p)
    print("\n-------------------------------------")
    print("<target is reachable>\n")
    print("number of iteration: {}".format(n))
    
def non_reachable(p, target, totallinks, history):
    r_i = np.zeros(totallinks-1)
    for i in range(totallinks-1):
        r_i[i] = distance(p[i], target)
    lambda_i = np.zeros(totallinks-1)
    for i in range(len(r_i)):
        lambda_i[i] = distance(p[i+1], p[i])/r_i[i]

    # history
    history[0] = np.copy(p)

    # line
    for i in range(totallinks-1):
        p[i+1] = (1-lambda_i[i])*p[i] + lambda_i[i]*target

    history[1] = np.copy(p)
    print("\n-------------------------------------")
    print("<target is not reachable>\n")
    
def plotIterations(history, p, save=False): 
    # point print
    print("initial state:")
    for i in range(len(p)):
        print("joint {}: {}".format(i+1, history[0][i]))
    print("final state:")
    for i in range(len(p)):
        print("joint {}: {}".format(i+1, history[1][i]))
    
    # graph print
    for i in range(len(history)):
        x = []
        y = []
        for point in range(len(p)):
            x.append(history[i][point][0])
            y.append(history[i][point][1])
        plt.plot(x, y, label=("line " + str(i + 1)))
        plt.plot(x, y, 'ro')
        plt.plot(Target[0], Target[1], 'bo')
    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    plt.title('Iterations history')
    plt.legend()

    if save == True:
            plt.savefig('{}.png'.format("my_fabrik_2D"))
    plt.show()
    

try:
    TotalLinks = int(input("Total links: "))
    Theta_cons = np.zeros(TotalLinks-1)

    Tol = 0.01


    p = np.zeros((TotalLinks, 2))
    d = np.zeros(TotalLinks-1)
    History = np.zeros((2, TotalLinks, 2))
    theta_initial = np.zeros(TotalLinks-1)

    for i in range(TotalLinks):
        a = tuple(float(x) for x in input("Enter coord[" + str(i+1) + "]: ").split())
        p[i] = a[0], a[1]
        

    for i in range(len(p)-1):
        d[i] = distance(p[i], p[i+1])

    for i in range(TotalLinks-2):
        Theta_cons[i+1] = float(input("Enter rotational constraints[" + str(i+2) + "]: "))


    Target_ = tuple(float(x) for x in input("Enter end effector final position: ").split())
    Target = np.array([Target_[0], Target_[1]])
    total_distance = np.sum(d)

    for i in range(TotalLinks-2):
        theta_initial[i+1] = cal_theta_backward(p,i)

    # 예외처리
    if (np.abs(theta_initial[1:]) > np.radians(Theta_cons[1:])).any():
        raise Exception('주어진 각 joint에 대한 각도가 주어진 제한각도를 초과합니다. 불가능!')

    Dist = distance(p[0], Target)
    if isReachable(Dist, total_distance):
        iterate(p, d, Target, Tol, History, Theta_cons)
        plotIterations(History, p, save=False)
    else:
        non_reachable(p, Target, TotalLinks, History)
        plotIterations(History, p, save=False)

except Exception as e:
    print("error", e)


