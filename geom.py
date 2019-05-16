from math import sqrt
import numpy as np
import math
import matplotlib.pyplot as plt


def generate_line_from_sensor(sensor):

    angle = sensor[0]
    distance = sensor[1]
    p = [distance * math.cos(angle), distance *  math.sin(angle)]
    # !!!!!!!!!!! p[1] == 0
    if p[1] == 0:
        return Line(-2, 2, 3, 3)
    k = -p[0]/p[1]
    b = p[1] - p[0] * k 
    y = []
    y.append(k*(-2) + b)
    y.append(k*(2) + b)
    l = Line(-2, 2, y[0], y[1])
    return l



def distance(a,b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_between(a,b,c):
    return abs(distance(a,c) + distance(c,b) - distance(a,b)) < 0.01

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
        p1y = k*p1x + b0
        p2y = k*p2x + b0
        inp = [[p1x,p1y],[p2x,p2y]]
        # select the points lie on the line segment
        inp = [p for p in inp if p[0]>=min(x1,x2) and p[0]<=max(x1,x2)]
    else:
        inp = []
  return inp


def set_speed():
    try:
        v_x = float(input("v_x: "))
    except:
        v_x = 0.0
    try:
        v_y = float(input("v_y: "))
    except:
        v_y = 0.0

    if v_x == v_y == 0:
        v_y = 1
    return [v_x, v_y]

def plot_speed(ax, V_x, V_y, center = (0,0)):
    len_c = math.sqrt(V_x**2+V_y**2)
    V_x /= len_c
    V_y /= len_c

    ax.arrow(center[0], center[1], V_x, V_y, length_includes_head=True,
             head_width=0.08, head_length=0.2)



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

    def __eq__(self, other):
        try:
            if other.x0 == self.x0 and other.x1 == self.x1 and  other.y0 == self.y0 and other.y1 == self.y1 :
                return True
            return False
        except:
            return False

def criteria_to_remove(p1,p2,p_intersection,  center = (0,0)):
    pass
    directon_p1_p2 = ((p2[0] - p1[0])/100., (p2[1] - p1[1])/100.)
    p_int_moved = (p_intersection[0] + directon_p1_p2[0], p_intersection[1] + directon_p1_p2[1])
    d1 = distance(p_int_moved, center)
    d2 = distance(p_intersection, center)
    if d1 > d2:
        return (p1,p_intersection)
    else:
        return (p2,p_intersection)


def plot_line(l,ax, center = [0,0], c="lightblue", le =3):
    x = [center[0]+ l["x"][0], center[0]+ l["x"][1]]
    y = [center[1]+ l["y"][0], center[1]+ l["y"][1]]
    ax.plot(x, y, color=c, linewidth=le)



def is_in_same_side_as_center(l1,p_int, center):
    
    x1 = l1["x"][0]
    y1 = l1["y"][0]
    x2 = l1["x"][1]
    y2 = l1["y"][1]
    k = (y1 - y2)/(x1 - x2)
    b0 = y1 - k*x1
    p_int_side = p_int[1] - p_int[0]*k - b0
    center_side = center[1] - center[0]*k - b0
    return p_int_side * center_side > 0


def line_circle_clothest_point(p1, p2, r, center=(0, 0)):
    Q = np.array([center[0], center[1]])        # Centre of circle
    point1 = np.array([p1[0], p1[1]])
    point2 = np.array([p2[0], p2[1]])
    r = r                  # Radius of circle
    P1 = point1      # Start of line segment
    V = point2 - P1  # Vector along line segment

    a = V.dot(V)
    b = 2 * V.dot(P1 - Q)
    c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2

    disc = b**2 - 4 * a * c
    if disc < 0:
        return [False, None]

    sqrt_disc = math.sqrt(disc)
    t1 = (-b + sqrt_disc) / (2 * a)
    t2 = (-b - sqrt_disc) / (2 * a)

    # if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
    #     print ("this sheet")
    #     return [False, None]

    t = max(0, min(1, - b / (2 * a)))
    return [True, P1 + t * V]


def modify_lines(l1,l2,p_intersection, center = (0,0)):
    # print (l1,l2)
    p1 = (l1["x"][0], l1["y"][0])
    p2 = (l1["x"][1], l1["y"][1])
    
    directon_p1_p2 = ((p2[0] - p1[0])/100., (p2[1] - p1[1])/100.)
    p_int_moved = (p_intersection[0] + directon_p1_p2[0], p_intersection[1] + directon_p1_p2[1])
    same_side_as_center = is_in_same_side_as_center(l2,p_int_moved, center)
    if same_side_as_center:
        l1 = Line(p_intersection[0], p2[0], p_intersection[1], p2[1])

    else:
        l1 = Line(p1[0], p_intersection[0], p1[1], p_intersection[1])


    p1 = (l2["x"][0], l2["y"][0])
    p2 = (l2["x"][1], l2["y"][1])
    
    directon_p1_p2 = ((p2[0] - p1[0])/100., (p2[1] - p1[1])/100.)
    p_int_moved = (p_intersection[0] + directon_p1_p2[0], p_intersection[1] + directon_p1_p2[1])
    same_side_as_center = is_in_same_side_as_center(l1,p_int_moved, center)
    if same_side_as_center:
        l2 = Line(p_intersection[0], p2[0], p_intersection[1], p2[1])
    else:
        l2 = Line(p1[0], p_intersection[0], p1[1], p_intersection[1])
        
    return (l1,l2)

def check_lines_no_intersection(l1,l2, center = (0,0)):
    p1_1 = (l2["x"][0], l2["y"][0])
    p1_2 = (l2["x"][1], l2["y"][1])
    p2_1 = (l1["x"][0], l1["y"][0])
    p2_2 = (l1["x"][1], l1["y"][1])
    same_side_as_center_p1 = is_in_same_side_as_center(l1, p1_1, center)
    same_side_as_center_p2 = is_in_same_side_as_center(l2, p2_1, center)
    if same_side_as_center_p1 and same_side_as_center_p2:
        return (False, [])
    else:
        l1_circle_distance = line_circle_clothest_point(p1_1, p1_2, 1)
        if l1_circle_distance[0] == True:
            l1_circle_distance[1] = np.linalg.norm(l1_circle_distance[1])
        else: 
            print ("AHTUNG")
            return (False, [])
        l2_circle_distance = line_circle_clothest_point(p2_1, p2_2, 1)
        if l2_circle_distance[0] == True:
            l2_circle_distance[1] = np.linalg.norm(l2_circle_distance[1])
        else: 
            print ("AHTUNG")
            return (False , [])
        if l1_circle_distance < l2_circle_distance:
            return (True, l1)
        else:
            return (True, l2)



def ClosestPointOnLine(a, b, p):
    ap = p-a
    ab = b-a
    result = a + ap.dot(ab)/ab.dot(ab) * ab
    return result

def distance(a,b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_point_inside_line(a,b,p):
    if distance (a,p)+ distance(p,b) -  distance(a,b) < 0.0000001:
        return True
    else:
        return False

def pick_clothest_point_of_line_by_point(a,b,p):

    if distance(a,p) > distance(p,b):
        return b
    else:
        return a


if __name__ == "__main__":
    
    
    # orig_lines = [Line(-1,1.01,-1.,1.0), Line(-1,0.999,-1.,1.01), Line(-1,1,0.65,-0.2), Line(-1,1,0.65,0.65), Line(-0.5,-0.6,-1, 1),  Line(0.2,1,1.2,-0.2), Line(-1.2,1.2, 0.2, 0.2), Line(-0.8,1,-0.8,0.5), Line(1.1, -0.7, 1.04, -1.17), Line(-1.1, 0.7, -1.04, 1.17)]
    fig = plt.figure()
    ax = fig.add_subplot(111)
    p = (0,0,1)
    plt.ion()
    d = 0.01
    sensors = { 
                "4":[math.pi/180.0*30.,    2], 
                "5":[math.pi/180.0*90.,    2], 
                "6":[math.pi/180.0*150,    2], 
                "7":[math.pi/180.0*210,    d], 
                "8":[math.pi/180.0*270,    d], 
                "9":[math.pi/180.0*-30,    d]}
    
    orig_lines = []
    for sensor in sensors:
        
        l = generate_line_from_sensor(sensors[sensor])
        orig_lines.append(l)
        # plot_line(l, plt)

    for angle in np.arange(-2, 10*math.pi*2, 0.05):
        speed = np.array([math.cos(angle), math.sin(angle)])
        plot_speed(plt, speed[0], speed[1])
        
        for line in orig_lines:
            plot_line(line, ax, c="red", le =1)

        modif_lines = []
        for line in orig_lines:
            # intersect_with_c = LineIntersectCircle(p, [line["x"][0], line["y"][0]], [line["x"][1], line["y"][1]])

            # if len(intersect_with_c) != 2:
            #     continue
            intersect_with_c= line
            # intersect_with_c = Line(intersect_with_c[0][0],intersect_with_c[1][0], intersect_with_c[0][1], intersect_with_c[1][1])
            plot_line(intersect_with_c, plt, c="black", le =1)

            for i, line in enumerate(modif_lines):
                intersection = seg_intersect(modif_lines[i], intersect_with_c, inside=1)


                if (intersection is not None):
                    modif_lines[i], intersect_with_c = modify_lines(modif_lines[i], intersect_with_c, intersection)
                else:
                    do_remove = check_lines_no_intersection(intersect_with_c, modif_lines[i])
                    if do_remove[0] == True:
                        if do_remove[1] == intersect_with_c:
                            intersect_with_c = False
                            break
                        else:
                            modif_lines.remove(modif_lines[i])
            if intersect_with_c != False:
                modif_lines.append(intersect_with_c)

        speed_line = Line(0,speed[0],0,speed[1])

        flag = True
        for line in modif_lines:
            is_interacted = seg_intersect(line, speed_line, inside= True)
            print ("is_interacted ",is_interacted, " angle= ", angle)
            if is_interacted is not None:
                project_point = ClosestPointOnLine(np.array([line["x"][0], line["y"][0]]), np.array([line["x"][1], line["y"][1]]), np.array([speed[0],speed[1]]))
                if is_point_inside_line ( np.array([line["x"][0], line["y"][0]]), np.array([line["x"][1], line["y"][1]])  , project_point) == True:
                    plt.plot([0,project_point[0] ], [0, project_point[1]], color="red", linewidth="5")
                    print ("prjection inside")
                    flag = False
                    break
                else:
                    project_point = pick_clothest_point_of_line_by_point (np.array([line["x"][0], line["y"][0]]), np.array([line["x"][1], line["y"][1]])  , project_point)
                    print ("prjection outside")
                    plt.plot([0,project_point[0] ], [0, project_point[1]], color="red", linewidth=5)
                    flag = False
                    break
        if flag:
            plt.plot([0,speed[0]], [0, speed[1]], color="red", linewidth=5)


        print ()
        for line in modif_lines:
            # print (line)
            plot_line(line, plt)

        circ = plt.Circle((0,0), 1, alpha=0.1)
        ax.add_patch(circ)
        ax.set_xlim(-3., 3)
        ax.set_ylim(-3., 3)

        plt.scatter(0, 0, color='darkgreen', marker='*')
        plt.axis('off')
        plt.ylim(-3., 3)
        plt.xlim(-3., 3)
        plt.draw()

        plt.show()
        plt.pause(0.001)
        plt.clf()
