import time
import matplotlib.pyplot as plt
import numpy as np
import math


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

	


min_dist = 0.6
crit_dist = 0.1
init_velocity_center = [-1, -1]
V_x = float(input('set speed X:'))
V_y = float(input('set speed Y:'))
norm_koeff = math.sqrt(V_x**2 + V_y**2)
V_x /= norm_koeff
V_y /= norm_koeff

fig = plt.figure()
ax = fig.add_subplot(111)
c_x = 0.0
c_y = 0.32
robot = {"side_one": {"x": [c_x + 1, c_x+2], "y": [c_y + 1.0, c_y+2.0]},
         "side_two": {"x": [c_x + 2.5, c_x + 3.5], "y": [c_y + 2.0, c_y + 1.0]},
         "side_three": {"x": [c_x + 3.5, c_x + 1], "y": [c_y + 1.0, c_y + 1.0]},
         "side_one_two": {"x": [c_x+2, c_x + 2.5], "y": [c_y+2.0, c_y+2.0]}

         }
border_1 = {'x': [0.4, 1.9], 'y': [1.0, 2.5]}
border_2 = {'x': [2.6, 4.1], 'y': [2.5, 1.0]}
border_3 = {'x': [2, 2.5], 'y': [2.5, 2.5]}

side_one_norm = {"x": -1.0, "y": 1.0}


side_one_sensor_1 = {"x": c_x + 1.25, "y": c_y + 1.225}
side_one_sensor_2 = {"x": c_x + 1.75, "y": c_y + 1.75}
side_one_two_sensor = {"x": c_x + 2.25, "y": c_y+2.0}
side_two_sensor_1 = {"x": c_x + 2.7, "y": c_y + 1.8}
side_two_sensor_2 = {"x": c_x + 3.35, "y": c_y + 1.2}


def get_norm_to_side(side):
    print("side ", side)
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
    print("speed angle  = ", angle)
    limits = (-math.pi/4, math.pi/4)
    limit_speeds = 0
    for limit in limits:
        x = [center[0], center[0] + math.cos(angle+limit)]
        y = [center[1], center[1] + math.sin(angle+limit)]
        ax.plot(x, y, color='black', linewidth=1, linestyle='-.')
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


def find_line_point_distance(point, border):
    p3 = np.array([sensor["x"], sensor["y"]])
    p1 = np.array([border["x"][0], border["y"][0]])
    p2 = np.array([border["x"][1], border["y"][1]])
    d = np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1)
    return np.abs(d)


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
    print("norm_border[0], norm_border[1] ", norm_border[0], norm_border[1])
    print("mapped_dist ", mapped_dist)
    # ax.scatter(center[0]+norm_border[0], center[1] + norm_border[1], color='darkgreen', marker='^')
    x = [center[0]+norm_border[0] - border["x"][1] + border["x"][0],
         center[0]+norm_border[0] + border["x"][1] - border["x"][0]]
    y = [center[1]+norm_border[1] - border["y"][1] + border["y"][0],
         center[1]+norm_border[1] + border["y"][1] - border["y"][0]]
    ax.plot(x, y, color='lightblue', linewidth=3)
    # ax.plot( [center[0]+norm_border[0], ] ,[center[1]+norm_border[1], ], color='lightblue', linewidth=3)
    return (x, y)


def analyze_line_limits(line_limits, V_x, V_y):
    pass


# ax.plot([1, 2, 2.5, 3.5, 1], [10, 20, 20, 10, 10], color='lightblue', linewidth=3)
plot_robot(robot, ax)
plot_border(border_1, ax)
plot_border(border_2, ax)
plot_border(border_3, ax)
plot_init_speed(V_x, V_y, ax)
plot_side_norm_point(robot["side_one"], side_one_sensor_1, ax)
plot_side_norm_point(robot["side_one"], side_one_sensor_2, ax)
plot_side_norm_point(robot["side_one_two"], side_one_two_sensor, ax)
plot_side_norm_point(robot["side_two"], side_two_sensor_1, ax)
plot_side_norm_point(robot["side_two"], side_two_sensor_2, ax)


plot_sensors([side_one_sensor_1, side_one_sensor_2,
              side_one_two_sensor, side_two_sensor_1, side_two_sensor_2], ax)

# sensors
# ax.scatter([1.74, 1.25, 2.7, 3.35], [1.8, 1.225, 1.81, 1.2], color='darkgreen', marker='^')
res = find_angle_speed_and_sensor(
    V_x, V_y, side_one_norm["x"], side_one_norm["y"])

dist_side_one_sensor_1_border_1 = find_sensor_distance(
    side_one_sensor_1, border_1)
dist_side_two_sensor_1_border_2 = find_sensor_distance(
    side_two_sensor_1, border_2)
dist_side_one_two_sensor_border_3 = find_sensor_distance(
    side_one_two_sensor, border_3)

line_limit = []

line_limit.append(plot_sensor_limit_to_vec_circle(
    border_1, dist_side_one_sensor_1_border_1, [crit_dist, min_dist], ax))
line_limit.append(plot_sensor_limit_to_vec_circle(
    border_2, dist_side_two_sensor_1_border_2, [crit_dist, min_dist], ax))
line_limit.append(plot_sensor_limit_to_vec_circle(
    border_3, dist_side_one_two_sensor_border_3, [crit_dist, min_dist], ax))

for line in line_limit:
    print("limit: ", line)


ax.set_xlim(-2., 4.5)
ax.set_ylim(-2., 4.5)
plt.axis('off')
# plt.ion()
plt.show()
plt.pause(1.001)
plt.clf()
