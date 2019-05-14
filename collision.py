import time
import matplotlib.pyplot as plt
import numpy as np
import math
import operator



def is_angle_between(target, angle1, angle2):
    if angle1 < 0:
        angle1 += math.pi*2
    
    if angle2 < 0:
        angle2 += math.pi*2
    # print (angle1, angle2)
    rAngle = ((angle2 - angle1) % (math.pi*2) + math.pi*2) % (math.pi*2)
    if (rAngle >= math.pi):
        angle1, angle2 = angle2, angle1

#   // check if it passes through zero
    if (angle1 <= angle2):
        return (target >= angle1) and  (target <= angle2)
    else:
        return (target >= angle1) or (target <= angle2)


def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def seg_intersect(l1, l2):
    a1 = np.array([l1["x"][0], l1["y"][0]])
    a2 = np.array( [l1["x"][1], l1["y"][1]])
    b1 = np.array( [l2["x"][0], l2["y"][0]])
    b2 = np.array( [l2["x"][1], l2["y"][1]])
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    return (num / denom.astype(float))*db + b1

class line():
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


min_dist = 0.6
crit_dist = 0.01
init_velocity_center = [-1.0, -1.0]
V_x = float(input('set speed X:'))
V_y = float(input('set speed Y:'))
norm_koeff = math.sqrt(V_x**2 + V_y**2)
V_x /= norm_koeff
V_y /= norm_koeff

fig = plt.figure()
ax = fig.add_subplot(111)
c_x = 0.1
c_y = 0.3
robot = {"side_one": line(c_x + 1, c_x+2, c_y + 1.0, c_y+2.0),
         "side_two": line(c_x + 2.5, c_x + 3.5, c_y + 2.0, c_y + 1.0),
         "side_three": line(c_x + 3.5, c_x + 1, c_y + 1.0, c_y + 1.0),
         "side_one_two": line(c_x+2, c_x + 2.5, c_y+2.0 , c_y+2.0)
         }


# border_1 = {'x': [0.4, 1.9], 'y': [1.0, 2.5]}
# border_2 = {'x': [2.6, 4.1], 'y': [2.5, 1.0]}

border_1 = line(0.4, 1.9, 1.0, 2.5)
border_2 = line(2.6, 4.1, 2.5, 1.0)
border_3 = line(2, 2.5, 2.5, 2.5)

side_one_sensor_1 = {"x": c_x + 1.25, "y": c_y + 1.225}
side_one_sensor_2 = {"x": c_x + 1.75, "y": c_y + 1.75}
side_one_two_sensor = {"x": c_x + 2.25, "y": c_y+2.0}
side_two_sensor_1 = {"x": c_x + 2.7, "y": c_y + 1.8}
side_two_sensor_2 = {"x": c_x + 3.35, "y": c_y + 1.2}


def get_norm_to_side(side):
    # print("side ", side)
    norm = {"x": side["y"][0] - side["y"][1], "y": side["x"][1] - side["x"][0]}
    return norm


def plot_side_norm_and_point(side_one_norm, side_one_norm_point, ax):
    x = [side_one_norm_point["x"], side_one_norm_point["x"] + side_one_norm["x"]]
    y = [side_one_norm_point["y"], side_one_norm_point["y"] + side_one_norm["y"]]
    ax.plot(x, y, color='black', linewidth=1, linestyle='--')


def plot_side_norm_point(side, point, ax):
    norm = get_norm_to_side(side)
    plot_side_norm_and_point(norm, point, ax)


def plot_init_speed(Vx, Vy, ax):
    center = init_velocity_center
    x = [center[0], center[0] + Vx]
    y = [center[1], center[1] + Vy]
    # ax.plot(x, y, color='black', linewidth=3)
    ax.arrow(x[0], y[0], V_x, V_y, length_includes_head=True,
             head_width=0.08, head_length=0.2)
    axis_x = angle = np.array([1, 0])
    angle = math.acos(np.array([Vx, Vy]).dot(axis_x))
    # print("speed angle  = ", angle)
    limits = (-math.pi/4, math.pi/4)
    limit_speeds = []
    for limit in limits:
        x = [center[0], center[0] + math.cos(angle+limit)]
        y = [center[1], center[1] + math.sin(angle+limit)]
        ax.plot(x, y, color='black', linewidth=1, linestyle='-.')
        limit_speeds.append(line (x[0], x[1], y[0],y[1]))
    circ = plt.Circle(center, 1, alpha=0.1)
    ax.add_patch(circ)
    return limit_speeds




def plot_robot(robot, ax):
    x = []
    y = []
    x.extend(robot["side_one"]["x"])
    x.extend(robot["side_two"]["x"])
    x.extend(robot["side_three"]["x"])

    y.extend(robot["side_one"]["y"])
    y.extend(robot["side_two"]["y"])
    y.extend(robot["side_three"]["y"])

    ax.plot(x, y, color='lightblue', linewidth=3)
    return 0


def plot_border(border, ax):
    x = []
    y = []
    x.extend(border["x"])
    y.extend(border["y"])

    ax.plot(x, y, color='pink', linewidth=3)
    return 0


def find_angle_speed_and_sensor(V_x, V_y, normal_sensor_x, normal_sensor_y):
    v_vec = np.array([V_x, V_y])
    sensor_vet = np.array([normal_sensor_x, normal_sensor_y])
    # print (sensor_vet)
    v_vec /= np.linalg.norm(v_vec)
    # print (np.linalg.norm(sensor_vet))
    sensor_vet /= np.linalg.norm(sensor_vet)
    # print (v_vec, sensor_vet)
    return v_vec.dot(sensor_vet)


def calc_speed(V_x, V_y, robot, borders, distance=[1, 1, 0, 0, 0, 0]):
    first_side_lim = 0
    second_side_lim = 0
    third_side = 0
    if (limits[0] or limits[1]) == 1:
        first_side_lim = 1
    if (limits[2] or limits[3]) == 1:
        second_side_lim = 1
    if (limits[4] or limits[5]) == 1:
        third_side = 1

    if first_side_lim+second_side_lim+third_side == 0:
        pass
        # go
        return (V_x, V_y)

    if first_side_lim+second_side_lim+third_side == 1:
        # find perp speed:
        pass

    if first_side_lim+second_side_lim+third_side >= 2:
        # find perp speed:
        pass


def plot_sensors(sensors, ax, center=[0, 0]):
    for sensor in sensors:
        x = sensor["x"]
        y = sensor["y"]
        ax.scatter(x, y, color='darkgreen', marker='^')



def find_sensor_distance(sensor, border):
    p3 = np.array([sensor["x"], sensor["y"]])
    p1 = np.array([border["x"][0], border["y"][0]])
    p2 = np.array([border["x"][1], border["y"][1]])
    d = np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1)
    return np.abs(d)


def plot_sensor_limit_to_vec_circle(border, dist, limits, ax, center=init_velocity_center):
    # border {"x": [1,2],  "y":[1,3]}

    mapped_dist = (dist - limits[0]) / (limits[1] - limits[0])
    if mapped_dist > 1:
        return
    norm_border = [- border["y"][1] + border["y"]
                   [0], border["x"][1] - border["x"][0]]
    # normilize  norm_border
    norm_border_koeff = 1 / \
        math.sqrt(norm_border[0]**2 + norm_border[1]**2) * mapped_dist
    norm_border[0] *= norm_border_koeff
    norm_border[1] *= norm_border_koeff
    # print("norm_border[0], norm_border[1] ", norm_border[0], norm_border[1])
    # print("mapped_dist ", mapped_dist)
    # ax.scatter(center[0]+norm_border[0], center[1] + norm_border[1], color='darkgreen', marker='^')
    x = [center[0]+norm_border[0] - border["x"][1] + border["x"][0],
         center[0]+norm_border[0] + border["x"][1] - border["x"][0]]
    y = [center[1]+norm_border[1] - border["y"][1] + border["y"][0],
         center[1]+norm_border[1] + border["y"][1] - border["y"][0]]
    ax.plot(x, y, color='lightblue', linewidth=3)
    # ax.plot( [center[0]+norm_border[0], ] ,[center[1]+norm_border[1], ], color='lightblue', linewidth=3)

    return (line (x[0], x[1], y[0],y[1]))


def get_distances_btw_speed_and_limits (line_limits, vec_speed):
    intersections = {}
    for limit in line_limits:
        intersection_point = seg_intersect(vec_speed, limit)
        intersection_point[0] -= init_velocity_center[0]
        intersection_point[1] -= init_velocity_center[1]
        distance = np.linalg.norm(intersection_point)
        if distance > 0 and distance < 1:
            intersections[limit] = distance
            # print distance
    # print (intersections)
    sorted_x = sorted(intersections.items(), key=operator.itemgetter(1))
    return (sorted_x)


def analyze_line_limits(line_limits, V_x, V_y):
    # print(init_velocity_center)
    vec_speed = line(init_velocity_center[0], init_velocity_center[0] + V_x, init_velocity_center[1], init_velocity_center[1] + V_y)
    init_angle = math.acos(line(0.,1.,0.,0.).dot(vec_speed) )
    sorted_x = get_distances_btw_speed_and_limits(line_limits, vec_speed)

    try:
        clothest =  sorted_x[0][0]
    except:
        return
    
    counter = 0
    while (clothest == sorted_x[0][0]):
        counter += 1
        #angle btw speed and clothest border
        angle = math.acos (vec_speed.dot(sorted_x[0][0]))

        print ("angle speed & border" , angle)
        #angle btw speed and X axis
        vel_angle = math.acos(line(0.,1.,0.,0.).dot(vec_speed) )
        
        if vel_angle > math.pi * 2:
            vel_angle = vel_angle % (math.pi * 2)
            print ("vel_angle > math.pi * 2", vel_angle)
        if vel_angle < 0:
            print ("WAS vel_angle < 0 ", vel_angle)
            vel_angle += math.pi * 2
            print ("vel_angle < 0 ", vel_angle)

        if (angle < math.pi/2.): # clockwise
            print("clockwise")
            vel_angle -=  0.03
            
        
        else:
            vel_angle += 0.03

        

        print ("vel_angle ", vel_angle)
        print ()
        r = [sorted_x[0][1] * math.cos(vel_angle), sorted_x[0][1] * math.sin(vel_angle)]
        x = [init_velocity_center[0], init_velocity_center[0] + r[0]]
        y = [init_velocity_center[1], init_velocity_center[1] + r[1]]
        vec_speed = line(x[0], x[1], y[0], y[1])
        sorted_x = get_distances_btw_speed_and_limits(line_limits, vec_speed)

        if len(sorted_x) == 0:
            print ("sorted_x == 0")
            return
        
        if counter > 1000:
            print ("overloaded counter" )
            ax.plot(x, y, color='pink', linewidth=2)
            return
        if is_angle_between(vel_angle, init_angle - math.pi/4.0, init_angle + math.pi/4.0) == 0:
            print(" angle limit! ", vel_angle)
            print ("init_angle ", init_angle)
            ax.plot(x, y, color='pink', linewidth=2)
            return

    print (counter)
    print ("fine!")
    ax.plot(x, y, color='pink', linewidth=2)


    # if angle 



# ax.plot([1, 2, 2.5, 3.5, 1], [10, 20, 20, 10, 10], color='lightblue', linewidth=3)
line_limit = []
plot_robot(robot, ax)
plot_border(border_1, ax)
plot_border(border_2, ax)
plot_border(border_3, ax)
line_limit.extend(plot_init_speed(V_x, V_y, ax))
plot_side_norm_point(robot["side_one"], side_one_sensor_1, ax)
plot_side_norm_point(robot["side_one"], side_one_sensor_2, ax)
plot_side_norm_point(robot["side_one_two"], side_one_two_sensor, ax)
plot_side_norm_point(robot["side_two"], side_two_sensor_1, ax)
plot_side_norm_point(robot["side_two"], side_two_sensor_2, ax)


plot_sensors([side_one_sensor_1, side_one_sensor_2,
              side_one_two_sensor, side_two_sensor_1, side_two_sensor_2], ax)

# sensors
# ax.scatter([1.74, 1.25, 2.7, 3.35], [1.8, 1.225, 1.81, 1.2], color='darkgreen', marker='^')


dist_side_one_sensor_1_border_1 = find_sensor_distance(
    side_one_sensor_1, border_1)
dist_side_two_sensor_1_border_2 = find_sensor_distance(
    side_two_sensor_1, border_2)
dist_side_one_two_sensor_border_3 = find_sensor_distance(
    side_one_two_sensor, border_3)



line_limit.append(plot_sensor_limit_to_vec_circle(
    border_1, dist_side_one_sensor_1_border_1, [crit_dist, min_dist], ax))
line_limit.append(plot_sensor_limit_to_vec_circle(
    border_2, dist_side_two_sensor_1_border_2, [crit_dist, min_dist], ax))
line_limit.append(plot_sensor_limit_to_vec_circle(
    border_3, dist_side_one_two_sensor_border_3, [crit_dist, min_dist], ax))

# for the_line in line_limit:
#     print("limit: ", the_line)


analyze_line_limits(line_limit, V_x, V_y)

ax.set_xlim(-2., 4.5)
ax.set_ylim(-2., 4.5)
plt.axis('off')
# plt.ion()
plt.show()
plt.pause(1.001)
plt.clf()
