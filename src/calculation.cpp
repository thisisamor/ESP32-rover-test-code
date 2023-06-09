#include <Arduino.h>
#include <math.h>
#include <utility>

int length = 3600; // mm
int width = 2400; 

int intersect_TH = 0; // TODO: slightly larger than 0? 

int wheel_diameter = 65; 
int steps_per_revo = 200;
int distance_per_step = M_PI * wheel_diameter / steps_per_revo;  
double radian_per_step = 2 * M_PI / steps_per_revo; 

double cot(double angle)
{   
    if (fmod(angle, PI) == 0.0){      // tan(angle) is 0 if the angle is a multiple of PI. This results in an infinite (Undefined) cot() result.
        return NAN;
    }
    else if (fmod(angle, PI/2) == 0.0){   // tan(angle) is infinite if the angle is PI/2, 3PI/2, etc. This results in a zero cot() result.
        return 0;
    }
    return 1.0 / tan(angle);
}

// change angle into range (-Pi, Pi)
double angle_range(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2*M_PI; 
    }
    while (angle < -M_PI)
    {
        angle += 2*M_PI; 
    }
    return angle; 
}

// input = motor revolution count
// output = angle turned by the rover 
std::pair<double, double> angle_calculation(int motor_R, int motor_B, int motor_Y)
{
    double a, b; 
    if (motor_B > motor_Y)
    {
        a = 2*M_PI*(motor_B - motor_Y)/steps_per_revo; 
    }
    else 
    {
        a = 2*M_PI*(motor_B - motor_Y + steps_per_revo)/steps_per_revo; 
    }
    if (motor_R > motor_B)
    {
        b = 2*M_PI*(motor_R - motor_B)/steps_per_revo; 
    }
    else 
    {
        b = 2*M_PI*(motor_R - motor_B + steps_per_revo)/steps_per_revo; 
    }
    return std::make_pair(a, b); 
}

std::pair<int, int> position_calculation(double angle_a, double angle_b)
{
    int a = (length * width / 2 ) * ( length/2 * cot(angle_a) - width/2) * (cot(angle_a)*cot(angle_b)-1) /( pow(length/2 * cot(angle_a)-width/2, 2) + pow(length/2 - width/2 *cot(angle_b), 2) ); 
    int b = (length * width / 2 ) * ( width/2 * cot(angle_b)- length/2) * (cot(angle_a)*cot(angle_b)-1) /( pow(length/2 * cot(angle_a)-width/2, 2) + pow(length/2 - width/2 *cot(angle_b), 2) ) - width;
    return std::make_pair(a, b); 
}

double angle_estimation(double current_angle, int direction, int steps)
{
    double tmp = current_angle+direction*(steps*2*M_PI/steps_per_revo); 
    return angle_range(tmp); 
}

std::pair<int, int> position_estimation(std::pair<int, int> current_position, double current_angle, int steps)
{
    current_position.first = current_position.first + cos(current_angle) * distance_per_step * steps; 
    current_position.second = current_position.second + sin(current_angle) * distance_per_step * steps; 
    return current_position; 
}

bool check_intersection(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4)
{
    int x1 = node1.first; 
    int x2 = node2.first; 
    int x3 = node3.first; 
    int x4 = node4.first; 
    int y1 = node1.second; 
    int y2 = node2.second; 
    int y3 = node3.second; 
    int y4 = node4.second; 
    // avoid overflow
    double A = ( (y3-y2)*(x3-x1) - (x3-x2)*(y3-y1) )/1000.0 * ( (y4-y2)*(x4-x1) - (x4-x2)*(y4-y1) )/1000.0; 
    double B = ( (y1-y4)*(x1-x3) - (x1-x4)*(y1-y3) )/1000.0 * ( (y2-y4)*(x2-x3) - (x2-x4)*(y2-y3) )/1000.0; 

    if ( std::min(A, B) < intersect_TH && std::max(A, B) <= intersect_TH )
    {
        return true; 
    }
    else
    {
        return false; 
    }  
}

std::pair<int, int> intersection_calculation(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4)
{
    double x1 = node1.first/100.0;  // avoid overflow
    double x2 = node2.first/100.0; 
    double x3 = node3.first/100.0; 
    double x4 = node4.first/100.0; 
    double y1 = node1.second/100.0; 
    double y2 = node2.second/100.0; 
    double y3 = node3.second/100.0; 
    double y4 = node4.second/100.0; 

    double A = 100.0 * ( (x2-x1)*(y4*x3 - y3*x4) - (x4-x3)*(y2*x1 - y1*x2) ) / ( (x2-x1)*(y4-y3) - (x4-x3)*(y2-y1) ); 
    double B = 100.0 * ( (y2-y1)*(y4*x3 - y3*x4) - (y4-y3)*(y2*x1 - y1*x2) ) / ( (x2-x1)*(y4-y3) - (x4-x3)*(y2-y1) ); 

    return (std::make_pair(int(A), int(B))); 
}

bool check_wall(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4, bool wall_side, bool direction)
{
    // known: node1 to node2
    // wall_side:  true = left wall; false = right wall; 

    
}

double get_angle(std::pair<int, int> node1, std::pair<int, int> node2, int direction)
{
    // angle of vector: from node1 to node2
    // left = 1; forward = 0; right = -1; backward = +-2;
    double tmp = atan2(node2.second - node1.second, node1.first - node2.first);
    tmp += M_PI_4 * direction; 
    return angle_range(tmp);
}

int get_angle_num(double angle1, double angle2)
{
    // from angle1 to angle2
    // +ve = left; -ve = right; 
    double tmp = angle_range(angle2 - angle1); 
    return int( tmp / radian_per_step ); 
}

int get_step_num(std::pair<int, int> node1, std::pair<int, int> node2)
{    
    double distance = sqrt(pow(node2.first - node1.first, 2) + pow(node2.second - node1.second, 2));
    return int(distance / distance_per_step); 
}