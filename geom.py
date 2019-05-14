from math import sqrt
import numpy as np
import math
import matplotlib.pyplot as plt

def distance(a,b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_between(a,b,c):
    
    return abs(distance(a,c) + distance(c,b) - distance(a,b)) < 0.05

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
    

def LineIntersectCircle(p,lsp,lep):
# p is the circle parameter, lsp and lep is the two end of the line
  x0,y0,r0 = p
  x1,y1 = lsp
  x2,y2 = lep
  if x1 == x2:
    if abs(r0) >= abs(x1 - x0):
        p1 = x1, y0 - sqrt(r0**2 - (x1-x0)**2)
        p2 = x1, y0 + sqrt(r0**2 - (x1-x0)**2)
        inp = [p1,p2]
        # select the points lie on the line segment
        inp = [p for p in inp if p[1]>=min(y1,y2) and p[1]<=max(y1,y2)]
    else:
        inp = []
  else:
    k = (y1 - y2)/(x1 - x2)
    b0 = y1 - k*x1
    a = k**2 + 1
    b = 2*k*(b0 - y0) - 2*x0
    c = (b0 - y0)**2 + x0**2 - r0**2
    delta = b**2 - 4*a*c
    if delta >= 0:
        p1x = (-b - sqrt(delta))/(2*a)
        p2x = (-b + sqrt(delta))/(2*a)
        p1y = k*x1 + b0
        p2y = k*x2 + b0
        inp = [[p1x,p1y],[p2x,p2y]]
        # select the points lie on the line segment
        inp = [p for p in inp if p[0]>=min(x1,x2) and p[0]<=max(x1,x2)]
    else:
        inp = []
  return inp


class Line():
    def __init__(self, x0, x1, y0, y1):
        self.x0 = x0
        self.x1 = x1
        self.y0 = y0
        self.y1 = y1

    def getLine(self):
        return {"x": [self.x0, self.x1], "y": [self.y0, self.y1]}

    def getNorm(self):
        return {"y": [-self.x0 + self.x1], "x": [self.y0 - self.y1]}

    def __getitem__(self, index):

        if index == "x":
            return [self.x0, self.x1]
        if index == "y":
            return [self.y0, self.y1]
    def  __repr__(self):
        res = "x0: " + str(self.x0) + "  x1: " + str(self.x1) + "  y0: " + str(self.y0) + "   y1: " + str(self.y1)
        return  res

    def get_dist_from_point(self, x,y):
        
        p1 = np.array([self.__getitem__(["x"])[0], self.__getitem__(["y"])[0]])
        p2 = np.array([self.__getitem__(["x"])[1], self.__getitem__(["y"])[1]])
        p3 = np.array([x, y])
        d = np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1)
        return np.abs(d)

    def dot(self, line):
        self_line = np.array([self.x1 - self.x0, self.y1 - self.y0])
        self_line /=  np.linalg.norm(self_line)
        line = np.array([ line["x"][1] - line["x"][0], line["y"][1] - line["y"][0] ])
        line /=  np.linalg.norm(line)
        
        return self_line.dot(line)

def criteria_to_remove(p1,p2,p_intersection,  center = (0,0)):
    pass
    directon_p1_p2 = ((p2[0] - p1[0])/10., (p2[1] - p1[1])/10.)
    p_int_moved = (p_intersection[0] + directon_p1_p2[0], p_intersection[1] + directon_p1_p2[1])
    if distance(p_int_moved, center) > distance(p_intersection, center):
        return (p1,p_intersection)
    else:
        return (p2,p_intersection)


def modify_lines(l1,l2,p_intersection, center = (0,0)):
    # print (l1,l2)
    p1 = (l1["x"][0], l1["y"][0])
    p2 = (l1["x"][1], l1["y"][1])
    
    p1,p2 = criteria_to_remove (p1,p2,p_intersection, center)
    l1 = Line(p1[0],p2[0], p1[1],p2[1])
    # if distance(p1,p_intersection) < distance(p2,p_intersection):
    #     l1 = Line(p_intersection[0],l1["x"][1], p_intersection[1], l1["y"][1])
    # else:
    #     l1 = Line(l1["x"][0], p_intersection[0], l1["y"][0], p_intersection[1])
    p1 = (l2["x"][0], l2["y"][0])
    p2 = (l2["x"][1], l2["y"][1])
    p1,p2 = criteria_to_remove (p1,p2,p_intersection, center)
    l2 = Line(p1[0],p2[0], p1[1],p2[1])
    # if distance(p1,p_intersection) < distance(p2,p_intersection):
    #     l2 = Line(p_intersection[0],l2["x"][1], p_intersection[1], l2["y"][1])
    # else:
    #     l2 = Line(l2["x"][0], p_intersection[0], l2["y"][0], p_intersection[1])
    # print (l1,l2)
    return (l1,l2)

def plot_line(l,ax, center = [0,0], c="lightblue", le =3):
    x = [center[0]+ l["x"][0], center[0]+ l["x"][1]]
    y = [center[1]+ l["y"][0], center[1]+ l["y"][1]]
    ax.plot(x, y, color=c, linewidth=le)

if __name__ == "__main__":
    orig_lines = [Line(-0.5,-0.5,-1, 1), Line(-1.2,1.2, 0.2, 0.2), Line(-0.8,1,-0.8,0.5)]
    orig_lines = [Line(1.1, 0.7, 1.04, -1.17), Line(-0.5,-0.6,-1, 1), Line(-1.2,1.2, 0.2, 0.2), Line(-0.8,1,-0.8,0.5) ]

    modif_lines = []
    fig = plt.figure()
    ax = fig.add_subplot(111)
    p  = (0,0,1)
    res = LineIntersectCircle(p,(-2,0), (1,0))
    
    for line in orig_lines:
        
        plot_line(line, ax, c="red", le =1)


    for line in orig_lines:
        intersect_with_c = LineIntersectCircle(p, [line["x"][0], line["y"][0]], [line["x"][1], line["y"][1]])

        if len(intersect_with_c) != 2:
            continue
        intersect_with_c = Line(intersect_with_c[0][0],intersect_with_c[1][0], intersect_with_c[0][1], intersect_with_c[1][1])
        plot_line(intersect_with_c, ax, c="black", le =1)
        print ("intersect_with_c ", intersect_with_c)
        for i in range(0,len(modif_lines)):
            intersection = seg_intersect(modif_lines[i], intersect_with_c, inside=True)
            # print (intersection)

            if (intersection is not None):
                modif_lines[i], intersect_with_c = modify_lines(modif_lines[i], intersect_with_c, intersection)
            else:
                print ("No Inters")
        modif_lines.append(intersect_with_c)
    

    for line in modif_lines:
        # print (line)
        plot_line(line, ax)
    circ = plt.Circle((0,0), 1, alpha=0.1)
    ax.add_patch(circ)
    ax.set_xlim(-2., 2)
    ax.set_ylim(-2., 2)
    ax.scatter(0, 0, color='darkgreen', marker='*')
    plt.axis('off')
    # plt.ion()
    plt.show()
        


        