import numpy as np
import matplotlib.pyplot as plt
import random
random.seed(42)


history_ = []
iterate_ = 0
plt.style.use('seaborn-whitegrid')
np.seterr(divide='ignore', invalid='ignore')


def distance(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(1/2)


class Fabrik_cons(object):
    def __init__(self, totallinks, points, theta_cons, target, tol=0.01):
        self.totallinks = totallinks
        self.points = np.copy(points)
        self.init_points = np.copy(points)
        self.theta_cons = theta_cons
        self.target = target
        self.tol = tol
        dist = np.zeros(totallinks-1)
        for i in range(totallinks-1):
            dist[i] = distance(points[i], points[i+1])
        self.dist = dist

    def isReachable(self):
        d_target = distance(self.points[0], self.target)
        d_total = np.sum(self.dist)
        if d_target > d_total:
            return False
        else:
            return True
        

    def cal_theta_forward(self, i):
        points = self.points
        theta_axis = np.arctan2(points[i-1][1]-points[i][1],
                                points[i-1][0]-points[i][0])
        theta_p = np.arctan2(points[i-2][1]-points[i-1][1],
                             points[i-2][0]-points[i-1][0])
        return theta_axis-theta_p # radian

    def cal_theta_backward(self, i):
        points = self.points
        theta_axis = np.arctan2(points[i+1][1]-points[i][1],
                                points[i+1][0]-points[i][0])
        theta_p = np.arctan2(points[i+2][1]-points[i+1][1],
                             points[i+2][0]-points[i+1][0])
        return theta_axis-theta_p # radian
    
    
    def iterate(self):
        theta_cons = np.radians(self.theta_cons)
        p = self.points
        d = self.dist
        target = self.target
        base = np.copy(p[0])
        dif_A = distance(p[-1], target)
        theta = 0
        n = 0

        while dif_A > self.tol and n < 50:
            # Stage 1: Forward reaching(target -> base)
            p[-1] = target
            for i in range(len(p)-1, 0, -1): # i=len(p)-1,len(p)-2...,1
                if i == len(p) - 1: # initial process
                    p[i-1] = p[i] + ((p[i-1]-p[i])/distance(p[i-1],p[i]))*d[i-1]
                    theta = self.cal_theta_forward(i) # radian
                elif i == 1: # final process
                    p[i-1] = p[i] + ((p[i-1]-p[i])/distance(p[i-1],p[i]))*d[i-1]
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
                    theta = self.cal_theta_forward(i) # radian            
            
    # Stage 2: Backward reaching(base -> target)
            p[0] = base
            for i in range(len(p)-1):
                if i == 0: # initial process
                    p[i+1] = p[i] + ((p[i+1]-p[i])/distance(p[i+1],p[i]))*d[i]
                    theta = self.cal_theta_backward(i) # radian                
                elif i == len(p)-2: # final process
                    
                    p[i+1] = p[i] + ((p[i+1]-p[i])/distance(p[i+1],p[i]))*d[i]
                         
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
                    theta = self.cal_theta_backward(i) # radian        
                    
                
            dif_A = distance(p[-1], target)
            # print(n)
            n = n + 1

        if (dif_A > self.tol):
            history_.append(target)
            

        # print("\n-------------------------------------")
        # print("<target is reachable>\n")
        # print("number of iteration: {}".format(n))


    def non_reachable(self):
        p = self.points
        r_i = np.zeros(self.totallinks-1)

        for i in range(self.totallinks-1):
            r_i[i] = distance(p[i], self.target)
        lambda_i = np.zeros(self.totallinks-1)
        for i in range(len(r_i)):
            lambda_i[i] = distance(p[i+1], p[i])/r_i[i]

        # line
        for i in range(self.totallinks-1):
            p[i+1] = (1-lambda_i[i])*p[i] + lambda_i[i]*self.target

        # history
        history_.append(self.target)

        # print("\n-------------------------------------")
        # print("<target is not reachable>\n")


    def plotIterations(self, save=False): 
        p = self.points
        # point print
        # print("initial state:")
        # for i in range(len(p)):
        #     print("joint {}: {}".format(i+1, self.history[0][i]))
        # print("final state:")
        # for i in range(len(p)):
        #     print("joint {}: {}".format(i+1, self.history[1][i]))
        
        x = [a[0] for a in history_]
        y = [a[1] for a in history_]
        x_init = [a[0] for a in self.init_points]
        y_init = [a[1] for a in self.init_points]
        print(len(history_))

        # graph print
        plt.plot(x, y, '.')
        plt.plot(x_init, y_init, linestyle='-', color='red', linewidth='2') # draw initial line for checking
        plt.plot(x_init, y_init, '.', color='red', markersize='15') # draw initial line for checking
        plt.xlabel('x - axis')
        plt.ylabel('y - axis')
        plt.title('Iterations history')
        # plt.legend()

        if save == True:
                plt.savefig('{}.png'.format("my_fabrik_2D"))
        plt.show()

def main():
    try:
        # input
        TotalLinks = int(input("Total links: "))
        Theta_cons = np.zeros(TotalLinks-1)

        Points = np.zeros((TotalLinks, 2))
        Distance = np.zeros(TotalLinks-1)
        theta_initial = np.zeros(TotalLinks-1)

        for i in range(TotalLinks):
            a = tuple(float(x) for x in input("Enter coord[" + str(i+1) + "]: ").split())
            Points[i] = a[0], a[1]
        Init_Points = np.copy(Points)

        for i in range(len(Points)-1):
            Distance[i] = distance(Points[i], Points[i+1])

        for i in range(TotalLinks-2):
            Theta_cons[i+1] = float(input("Enter rotational constraints[" + str(i+2) + "]: "))

        number = 0
        while number <10000:
            Target = np.array([random.uniform(-4, 4), random.uniform(-4, 4)])
            # print(Init_Points)
            fabrik_cons = Fabrik_cons(TotalLinks, Init_Points, Theta_cons, Target, tol=0.01)
            # print(Points)
            # exception
            for i in range(fabrik_cons.totallinks-2):
                theta_initial[i+1] = fabrik_cons.cal_theta_backward(i)

            
            # if (np.abs(theta_initial[1:]) > np.radians(Theta_cons[1:])).any():
            #     raise Exception('주어진 각 joint에 대한 각도가 주어진 제한각도를 초과합니다. 불가능!')
            

            # iterate
            if fabrik_cons.isReachable():
                fabrik_cons.iterate()            
            else:
                fabrik_cons.non_reachable()
            # print(number)
            number = number + 1
            print(number)
        # show
        fabrik_cons.plotIterations(save=False)
            
    except Exception as e:
        print("error", e)

        
if __name__ == "__main__":
    main()