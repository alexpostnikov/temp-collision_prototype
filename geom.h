
#define NUMB_SENSORS 9
typedef struct Point
{
    double x;
    double y;

} Point;

typedef struct Line
{
    Point p1;
    Point p2;
} Line;


void map_sensors(double sensors[NUMB_SENSORS][2], double max_dist, double min_dist, double mapped_sensors[NUMB_SENSORS][2]);
Point process_collision_sensors(Line SPEED_l, double sensors[NUMB_SENSORS][2]);
