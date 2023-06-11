#ifndef calculation_h
#define calculation_h

#include <Arduino.h>
#include <math.h>
#include <utility>

std::pair<double, double> angle_calculation(int motor_R, int motor_B, int motor_Y); 
double angle_estimation(double current_angle, int direction, int steps); 
std::pair<int, int> position_calculation(double angle_a, double angle_b); 
std::pair<int, int> position_estimation(std::pair<int, int> current_position, double current_angle, int steps); 

bool same_position(std::pair<int, int> position1, std::pair<int, int> position2); 
int check_intersection(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4); 
std::pair<int, int> intersection_calculation(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4); 

bool check_wall(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4, bool wall_side, bool direction); 

double get_angle(std::pair<int, int> node1, std::pair<int, int> node2, int direction); 
int get_angle_num(double angle1, double angle2); 
int get_step_num(std::pair<int, int> node1, std::pair<int, int> node2); 

#endif