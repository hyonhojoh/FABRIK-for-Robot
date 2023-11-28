import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-whitegrid')

def distance(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(0.5)

def isReachable(d_target, d_total):
    if d_target > d_total:
        return False
    else:
        return True
    
    
def iterate(p, d, target, tol, history):
    b = np.copy(p[0])
    r_i = np.zeros(len(p)-1)
    lambda_i = np.zeros(len(p)-1)
    dif_A = distance(p[-1], target)
    history[0] = np.copy(p)
    n = 0

    while dif_A > tol:
        # Stage 1: Forward reaching(target -> base)
        p[-1] = target
        for i in range(len(p)-1, 0, -1):
            r_i[i-1] = distance(p[i-1], p[i])
            lambda_i[i-1] = d[i-1]/r_i[i-1]
            p[i-1] = (1-lambda_i[i-1])*p[i] + lambda_i[i-1]*p[i-1]

        # Stage 2: Backward reaching(base -> target)
        p[0] = b
        for i in range(len(p)-1):
            r_i[i] = distance(p[i], p[i+1])
            lambda_i[i] = d[i]/r_i[i]
            p[i+1] = (1-lambda_i[i])*p[i] + lambda_i[i]*p[i+1]
        
        dif_A = distance(p[-1], target)

        n = n + 1

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

def plotIterations(history, p, save=False): # zeros()의 shape를 결정지을 수 없으므로 모든 경로를 그리지 말고 initial state와 final state만 그리기
                                # 아마도 isReachable == True 일때 shape 문제가 생길 것이므로 새로운 plot을 만드는 것도 고려
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
    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    plt.title('Iterations history')
    plt.legend()

    if save == True:
            plt.savefig('{}.png'.format("my_fabrik_2D"))
    plt.show()
    

totalLinks = int(input("Total links: "))
Tol = 0.001

p = np.zeros((totalLinks, 2))
d = np.zeros(totalLinks-1)
History = np.zeros((2, totalLinks, 2))

for i in range(totalLinks):
    m = tuple(float(x) for x in input("Enter coord[" + str(i+1) + "]: ").split())
    p[i] = m[0], m[1]
    

for i in range(len(p)-1):
    d[i] = distance(p[i], p[i+1])
    
Target_ = tuple(float(x) for x in input("Enter end effector final position: ").split())
Target = np.array([Target_[0], Target_[1]])
total_distance = np.sum(d)

Dist = distance(p[0], Target)
if isReachable(Dist, total_distance):
    iterate(p, d, Target, Tol, History)
    plotIterations(History, p, save=False)
else:
    non_reachable(p, Target, totalLinks, History)
    plotIterations(History, p, save=False)
