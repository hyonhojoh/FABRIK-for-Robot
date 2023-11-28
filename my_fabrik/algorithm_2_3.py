import numpy as np
import math as m

def distance(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(1/2)

def cal_theta_forward(p, i):
    theta_axis = m.atan2(p[i-1][1]-p[i][1], p[i-1][0]-p[i][0])
    theta_p = m.atan2(p[i-2][1]-p[i-1][1], p[i-2][0]-p[i-1][0])
    return theta_axis-theta_p # radian
    

def cal_theta_backward(p, i):
    theta_axis = m.atan2(p[i][1]-p[i-1][1], p[i][0]-p[i-1][0])
    theta_p = m.atan2(p[i+1][1]-p[i][1], p[i+1][0]-p[i][0])
    return theta_axis-theta_p # radian
    

def iterate(p, d, target, tol, history, theta_cons):
    b = np.copy(p[0])
    r_i = np.zeros(len(p)-1)
    lambda_i = np.zeros(len(p)-1)
    dif_A = distance(p[-1], target)
    history[0] = np.copy(p)
    n = 0

    while dif_A > tol:
        # Stage 1: Forward reaching(target -> base)
        
        
        for i in range(len(p)-1, 0, -1):
            p[-1] = target
            if i == len(p) - 1: # initial process
                r_i[i-1] = distance(p[i-1], p[i])
                lambda_i[i-1] = d[i-1]/r_i[i-1]
                p[i-1] = (1-lambda_i[i-1])*p[i] + lambda_i[i-1]*p[i-1]
                theta = cal_theta_forward(p, i) # radian
            elif i == 1: # final process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    r_i[i-1] = distance(p[i-1], p[i])
                    lambda_i[i-1] = d[i-1]/r_i[i-1]
                    p[i-1] = (1-lambda_i[i-1])*p[i] + lambda_i[i-1]*p[i-1]
                else: # 허용 범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(m.radians(theta_cons[i])), -np.sin(m.radians(theta_cons[i]))],
                                             [np.sin(m.radians(theta_cons[i])), np.cos(m.radians(theta_cons[i]))]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
                    else:
                        rev_mat = np.array([[np.cos(m.radians(-theta_cons[i])), -np.sin(m.radians(-theta_cons[i]))],
                                             [np.sin(m.radians(-theta_cons[i])), np.cos(m.radians(-theta_cons[i]))]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
            else: # 나머지 process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    r_i[i-1] = distance(p[i-1], p[i])
                    lambda_i[i-1] = d[i-1]/r_i[i-1]
                    p[i-1] = (1-lambda_i[i-1])*p[i] + lambda_i[i-1]*p[i-1]
                    theta = cal_theta_forward(p, i) # radian
                else: # 허용범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(m.radians(theta_cons[i])), -np.sin(m.radians(theta_cons[i]))],
                                             [np.sin(m.radians(theta_cons[i])), np.cos(m.radians(theta_cons[i]))]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
                        theta = cal_theta_forward(p, i) # radian

                    else:
                        rev_mat = np.array([[np.cos(m.radians(-theta_cons[i])), -np.sin(m.radians(-theta_cons[i]))],
                                             [np.sin(m.radians(-theta_cons[i])), np.cos(m.radians(-theta_cons[i]))]])
                        p[i-1] = p[i] + (((p[i]-p[i+1])/d[i])*d[i-1])@rev_mat
                        theta = cal_theta_forward(p, i)


        # Stage 2: Backward reaching(base -> target)
        p[0] = b
        for i in range(len(p)-1):
            if i == 0: # initial process
                r_i[i] = distance(p[i], p[i+1])
                lambda_i[i] = d[i]/r_i[i]
                p[i+1] = (1-lambda_i[i])*p[i] + lambda_i[i]*p[i+1]
                theta = cal_theta_backward(p, i) # radian
            elif i == len(p)-2: # final process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    r_i[i] = distance(p[i], p[i+1])
                    lambda_i[i] = d[i]/r_i[i]
                    p[i+1] = (1-lambda_i[i])*p[i] + lambda_i[i]*p[i+1]
                else: # 허용 범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(m.radians(theta_cons[i])), -np.sin(m.radians(theta_cons[i]))],
                                             [np.sin(m.radians(theta_cons[i])), np.cos(m.radians(theta_cons[i]))]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
                    else:
                        rev_mat = np.array([[np.cos(m.radians(-theta_cons[i])), -np.sin(m.radians(-theta_cons[i]))],
                                             [np.sin(m.radians(-theta_cons[i])), np.cos(m.radians(-theta_cons[i]))]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
            else: # 나머지 process
                if abs(theta) <= theta_cons[i]: # 허용 범위에 있는 경우
                    r_i[i-1] = distance(p[i-1], p[i])
                    lambda_i[i-1] = d[i-1]/r_i[i-1]
                    p[i-1] = (1-lambda_i[i-1])*p[i] + lambda_i[i-1]*p[i-1]
                    theta = cal_theta_backward(p, i) # radian
                else: # 허용범위에 있지 않은 경우
                    if theta >= 0:
                        rev_mat = np.array([[np.cos(m.radians(theta_cons[i])), -np.sin(m.radians(theta_cons[i]))],
                                             [np.sin(m.radians(theta_cons[i])), np.cos(m.radians(theta_cons[i]))]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
                        theta = cal_theta_backward(p, i) # radian

                    else:
                        rev_mat = np.array([[np.cos(m.radians(-theta_cons[i])), -np.sin(m.radians(-theta_cons[i]))],
                                             [np.sin(m.radians(-theta_cons[i])), np.cos(m.radians(-theta_cons[i]))]])
                        p[i+1] = p[i] + (((p[i]-p[i-1])/d[i-1])*d[i])@rev_mat
                        theta = cal_theta_backward(p, i)
            dif_A = distance(p[-1], target)

        n = n + 1

    history[1] = np.copy(p)
    print("\n-------------------------------------")
    print("<target is reachable>\n")
    print("number of iteration: {}".format(n))

# totalLinks = int(input("Total links: "))
# theta_cons = np.array(totalLinks-1)
# theta_cons[0] = 0
# p = np.zeros((totalLinks, 2))
# d = np.zeros(totalLinks-1)
# History = np.zeros((2, totalLinks, 2))

# for i in range(totalLinks):
#     m = tuple(float(x) for x in input("Enter coord[" + str(i+1) + "]: ").split())
#     p[i] = m[0], m[1]

# for i in range(len(p)-1):
#     d[i] = distance(p[i], p[i+1])

# for i in range(totalLinks-2):
#     theta_cons[i+1] = float(input("Enter constraints theta [" + str(i+1) + "]: "))

# Target_ = tuple(float(x) for x in input("Enter end effector final position: ").split())
# Target = np.array([Target_[0], Target_[1]])


p4 = np.array([1, 3**(1/2)])
p3 = np.array([0, 0])
p2 = np.array([-1, -1])

theta_axis = m.atan2(p3[1]-p4[1], p3[0]-p4[0])
print(m.degrees(theta_axis))
theta_p = m.atan2(p2[1]-p3[1], p2[0]-p3[0])
print(m.degrees(theta_p))
theta = theta_axis - theta_p
print(m.degrees(theta))

constraint_degree = 10


rev_mat = np.array([[np.cos(m.radians(10)), -np.sin(m.radians(10))], [np.sin(m.radians(10)), np.cos(m.radians(10))]])
p_new = p3 + ((p3-p4)*2)@rev_mat
p_new_angle = m.atan2(p_new[1]-p3[1], p_new[0]-p3[0])
print(m.degrees(p_new_angle))



