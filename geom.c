
#include <stdio.h>
#include <math.h>
#include "geom.h"
// const double PI = 3.14;

#define PI 3.14159265359

#define NO_INTERSECTION -999.0
#define MAX_DISTANCE 0.6
#define MIN_DISTANCE 0.1
#define STOP_SPEED 0.01
// Point SPEED_p = {0.4,-0.7};


#define d 0.4
double sensors[NUMB_SENSORS][2] = {
    {PI / 180.0 * 30.0,  d},  //0
    {PI / 180.0 * 90.0,  d},  //1
    {PI / 180.0 * 90.0,  0},  //2
    {PI / 180.0 * 150.0, d}, //3
    {PI / 180.0 * 210.0, d}, //4
    {PI / 180.0 * 210.0, d}, //5
    {PI / 180.0 * 270.0, d}, //6
    {PI / 180.0 * -32.0, d}, //7
    {PI / 180.0 * -28.0, d}  //8
};


Line SPEED_l = {0, 0, 0.9, 0.9};
Line limit_square[4] = {
    {4, -4, 4, 4},
    {4, 4, -4, 4},
    {-4, 4, -4, -4},
    {-4, -4, 4, -4}};



Line generate_line_from_sensor(double angle, double distance)
{
    // p = [, distance *  math.sin(angle)]
    Point p1 = {0, 0};
    Point p2 = {1, 1};

    Line result = {p1, p2};
    Point p = {distance * cos(angle), distance * sin(angle)};
    if (p.y == 0)
    {

        p1.x = cos(angle) * distance;
        p1.y = -20.0;
        p2.x = cos(angle) * distance;
        p2.y = 20.0;
        result.p1 = p1;
        result.p2 = p2;
        return result;
    }
    double k = -p.x / p.y;
    double b = p.y - p.x * k;
    p1.y = k * (-20.0) + b;
    p1.x = -20.0;
    // y.append(k*(-2) + b)
    p2.y = k * (20.0) + b;
    p2.x = 20.0;
    result.p1 = p1;
    result.p2 = p2;
    return result;
}

double distance(Point a, Point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}




int is_point_inside_line(Point a, Point b, Point p)
{
    if (distance(a, p) + distance(p, b) - distance(a, b) < 0.0000001)
        return 1;
    else
        return 0;
}

int is_between(Point a, Point b, Point c)
{
    return fabs(distance(a, c) + distance(c, b) - distance(a, b)) < 0.0000001;
}

double dot(Point v1, Point v2)
{
    double p = v1.x * v2.x + v1.y * v2.y;
    return p;
}

Point perp(Point a)
{
    Point b = {-a.y, a.x};
    return b;
}

Point seg_intersect(Line l1, Line l2, int is_inside)
{
    Point l1_p1 = l1.p1;
    Point l1_p2 = l1.p2;
    Point l2_p1 = l2.p1;
    Point l2_p2 = l2.p2;

    // a1 = np.array([l1["x"][0], l1["y"][0]])
    // a2 = np.array( [l1["x"][1], l1["y"][1]])
    // b1 = np.array( [l2["x"][0], l2["y"][0]])
    // b2 = np.array( [l2["x"][1], l2["y"][1]])

    Point da = {l1_p2.x - l1_p1.x, l1_p2.y - l1_p1.y};
    Point db = {l2_p2.x - l2_p1.x, l2_p2.y - l2_p1.y};
    Point dp = {l1_p1.x - l2_p1.x, l1_p1.y - l2_p1.y};
    Point dap = perp(da);

    // da = a2-a1
    // db = b2-b1
    // dp = a1-b1
    // dap = perp(da)

    double denom = dot(dap, db);
    // denom = np.dot(dap, db)
    if (fabs(denom) < 0.001)
    {
        // KOSTIL`
        Point res = {NO_INTERSECTION, NO_INTERSECTION};
        return res;
    }
    double num = dot(dap, dp);
    // num = np.dot(dap, dp)
    Point res = {0, 0};
    res.x = num / denom * db.x + l2_p1.x;
    res.y = num / denom * db.y + l2_p1.y;
    // res = (num / denom.astype(float))*db + b1

    if (is_inside > 0)
    {
        if (is_between(l1_p1, l1_p2, res) & is_between(l2_p1, l2_p2, res))
        {
            return res;
        }
        else
        {
            res.x = NO_INTERSECTION;
            res.y = NO_INTERSECTION;
            return res;
        }
    }

    else
    {
        return res;
    }

    // if inside:
    //     if is_between(a1, a2, res) and is_between(b1, b2, res) :
    //          return (num / denom.astype(float)) * db + b1
    //     else : return (num / denom.astype(float)) * db + b1
}

int is_in_same_side_as_center(Line l1, Point p_int, Point center)
{
    // x1 = l1["x"][0]
    // y1 = l1["y"][0]
    // x2 = l1["x"][1]
    // y2 = l1["y"][1]
    // if abs(x1 - x2) < 0.00001:
    //     return ((p_int[0]-x1) * (center[0]-x1) > 0)
    if (fabs(l1.p1.x - l1.p2.x) < 0.00001)
    {
        return ((p_int.x - l1.p1.x) * (center.x - l1.p1.x) > 0);
    }

    double k = (l1.p1.y - l1.p2.y) / (l1.p1.x - l1.p2.x);
    double b0 = l1.p1.y - k * l1.p1.x;
    double p_int_side = p_int.y - p_int.x * k - b0;
    double center_side = center.y - center.x * k - b0;
    return p_int_side * center_side > 0;
    // k = (y1 - y2)/(x1 - x2)
    // b0 = y1 - k*x1
    // p_int_side = p_int[1] - p_int[0]*k - b0
    // center_side = center[1] - center[0]*k - b0
    // return p_int_side * center_side > 0
}

void modify_lines(Line *l1, Line *l2, Point p_intersection)
{
    double dif_coef = 10.0;
    Point directon_p1_p2 = {(l1->p2.x - l1->p1.x) / dif_coef, (l1->p2.y - l1->p1.y) / dif_coef};
    // p1 = (l1["x"][0], l1["y"][0])
    // p2 = (l1["x"][1], l1["y"][1])
    // dif_coef = 10.
    // directon_p1_p2 = ((p2[0] - p1[0])/dif_coef, (p2[1] - p1[1])/dif_coef)

    Point p_int_moved = {p_intersection.x + directon_p1_p2.x, p_intersection.y + directon_p1_p2.y};
    Point center = {0, 0};
    int same_side_as_center = is_in_same_side_as_center(*l2, p_int_moved, center);
    if (same_side_as_center)
    {
        l1->p1.x = p_intersection.x;
        l1->p1.y = p_intersection.y;
    }
    else
    {
        l1->p2.x = p_intersection.x;
        l1->p2.y = p_intersection.y;
    }

    Point directon_l2 = {(l2->p2.x - l2->p1.x) / dif_coef, (l2->p2.y - l2->p1.y) / dif_coef};
    Point p_int_moved_l2 = {p_intersection.x + directon_l2.x, p_intersection.y + directon_l2.y};
    same_side_as_center = is_in_same_side_as_center(*l1, p_int_moved_l2, center);
    if (same_side_as_center)
    {
        l2->p1.x = p_intersection.x;
        l2->p1.y = p_intersection.y;
    }
    else
    {
        l2->p2.x = p_intersection.x;
        l2->p2.y = p_intersection.y;
    }
    // p1 = (l2["x"][0], l2["y"][0])
    // p2 = (l2["x"][1], l2["y"][1])

    // directon_p1_p2 = ((p2[0] - p1[0])/dif_coef, (p2[1] - p1[1])/dif_coef)
    // p_int_moved = (p_intersection[0] + directon_p1_p2[0], p_intersection[1] + directon_p1_p2[1])
    // same_side_as_center = is_in_same_side_as_center(l1,p_int_moved, center)
    // if  same_side_as_center:
    //     l2 = Line(p_intersection[0], p2[0], p_intersection[1], p2[1])
    // else:
    //     l2 = Line(p1[0], p_intersection[0], p1[1], p_intersection[1])

    // return (l1,l2)
}

Point ClosestPointOnLine(Point a, Point b, Point p)
{
    // ap = p-a
    // ab = b-a
    // result = a + ap.dot(ab)/ab.dot(ab) * ab
    // return result

    Point ap = {p.x - a.x, p.y - a.y};
    Point ab = {b.x - a.x, b.y - a.y};
    double koef = dot(ap, ab) / dot(ab, ab);
    Point result = {a.x + koef * ab.x, a.y + koef * ab.y};
    return result;
}


Point pick_clothest_point_of_line_by_point(Point a, Point b, Point p)
{
    if (distance(a, p) > distance(p, b))
        return b;
    else
        return a;
}


Point process_collision_sensors(Line SPEED_l, double sensors[NUMB_SENSORS][2])
{
       Line limit_square[4] = {
            {4, -4, 4, 4},
            {4, 4, -4, 4},
            {-4, 4, -4, -4},
            {-4, -4, 4, -4}};
        
        // Line sp = {0, 0, cos(angle), sin(angle)};
        // SPEED_l = sp;
        // SPEED_l.p2.x = -0.112153;
        // SPEED_l.p2.y = -0.993691;
        Point out_speed = {SPEED_l.p2.x, SPEED_l.p2.y};
        printf("init  speed : %lf, %lf \n", SPEED_l.p2.x, SPEED_l.p2.y);
        Line limiting_lines[9];

        for (size_t i = 0; i < NUMB_SENSORS; i++)
        {
            limiting_lines[i] = generate_line_from_sensor(sensors[i][0], sensors[i][1]);
        }

        for (size_t i = 0; i < NUMB_SENSORS; i++)
        {
            // int flag_inter = 0;
            for (size_t j = 0; j < 4; j++)
            {
                Point intersection = seg_intersect(limit_square[j], limiting_lines[i], 1);
                if (intersection.x != NO_INTERSECTION)
                {
                    // printf( "HERE! \n");
                    modify_lines(&limit_square[j], &limiting_lines[i], intersection);
                    // flag_inter = 1;
                }
            }

            for (size_t j = 0; j < i; j++)
            {
                Point intersection = seg_intersect(limiting_lines[j], limiting_lines[i], 1);
                if (intersection.x != NO_INTERSECTION)
                {
                    // printf( "HERE! \n");
                    modify_lines(&limiting_lines[j], &limiting_lines[i], intersection);
                    // flag_inter = 1;
                }
            }
        }
        
        for (size_t i = 0; i < NUMB_SENSORS; i++)
        {
            Point is_interacted = seg_intersect(limiting_lines[i], SPEED_l, 1);
            if (is_interacted.x != NO_INTERSECTION)
            {
                Point project_point = ClosestPointOnLine(limiting_lines[i].p1, limiting_lines[i].p2, SPEED_l.p2);
                if (is_point_inside_line(limiting_lines[i].p1, limiting_lines[i].p2, project_point) == 1)
                {
                    Point center = {0,0};
                    if (distance (center, project_point) < distance (center, out_speed))
                    {
                        // printf("inside\n");
                        out_speed = project_point;
                    }
                        
                    // printf("intersected\n");
                }
                else
                {
                    project_point = pick_clothest_point_of_line_by_point(limiting_lines[i].p1, limiting_lines[i].p2, project_point);
                    Point center = {0,0};
                    if (distance (center, project_point) < distance (center, out_speed))
                    {
                        // printf("inside\n");
                        out_speed = project_point;
                    }
                        
                    // printf("intersected\n");
                    // break;
                }
            }
        }
        if (sqrt( pow(out_speed.x, 2) + pow(out_speed.y, 2) ) < STOP_SPEED)
        {
            out_speed.x = 0.0;
            out_speed.y = 0.0;
        }
        return out_speed;
}


void map_sensors(double sensors[NUMB_SENSORS][2], double max_dist, double min_dist, double mapped_sensors[NUMB_SENSORS][2])
{
    // double mapped_sensors[NUMB_SENSORS][2];
    double slope = 1.0 / (max_dist - min_dist);
    double b = 1 - (slope * max_dist);
    for (int i =0; i < NUMB_SENSORS; i++)
    {
        mapped_sensors[i][0] = sensors[i][0];
        if (sensors[i][1] <= min_dist+0.01)
        {
            mapped_sensors[i][1] = 0.002;
        }
        else 
        {
            mapped_sensors[i][1] = slope * sensors[i][1] + b;
        }
    }
    // return mapped_sensors;
}

int main()
{
    Point p1 = {0, 1}; // The variable p1 is declared like a normal variable
    Point p2 = {1, 0};
    Line l1 = {p1, p2};
    double c = 0;

    for (double angle = 0.0; angle < PI*2; angle += 0.1)
    {
        Point out_speed;
        Line sp = {0, 0, 0.3*cos(angle), 0.3*sin(angle)};
        // Line sp = {0, 0, -0.666276, 0.745705};
        // -0.666276, 0.745705
        double mapped_sensors[NUMB_SENSORS][2];

        map_sensors(sensors, MAX_DISTANCE, MIN_DISTANCE, mapped_sensors);
        out_speed = process_collision_sensors(sp, mapped_sensors);
        printf("res speed   : %lf, %lf \n", out_speed.x, out_speed.y);
        printf(" \n");
        continue;      
    }
}