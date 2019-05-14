import numpy as np
from geom import Line
import math

def distance(a,b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_between(a,b,c):
    
    return distance(a,c) + distance(c,b) == distance(a,b)



def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def seg_intersect(l1, l2, inside = False):
    a1 = np.array([l1["x"][0], l1["y"][0]])
    a2 = np.array( [l1["x"][1], l1["y"][1]])
    b1 = np.array( [l2["x"][0], l2["y"][0]])
    b2 = np.array( [l2["x"][1], l2["y"][1]])
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    # print denom
    num = np.dot(dap, dp)
    res = (num / denom.astype(float))*db + b1
    if inside:
        if is_between(a1,a2,res) and  is_between(b1,b2,res):
            return (num / denom.astype(float))*db + b1
    else:
        return (num / denom.astype(float))*db + b1
    


l1 = Line(-1,1,0,0)
l2 = Line(2,2,-1,1)

print (seg_intersect(l1,l2,1))


Q = np.array([0., .0])                  # Centre of circle
r = 1.0                  # Radius of circle
P1 = np.array([-1.1,0.])      # Start of line segment
V = np.array([1.,0.3]) - P1  # Vector along line segment

a = V.dot(V)
b = 2 * V.dot(P1 - Q)
c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2

disc = b**2 - 4 * a * c
if disc < 0:
    print(False, None)

sqrt_disc = math.sqrt(disc)
t1 = (-b + sqrt_disc) / (2 * a)
t2 = (-b - sqrt_disc) / (2 * a)

if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
    print(False, None)

t = max(0, min(1, - b / (2 * a)))
print( True, P1 + t * V)