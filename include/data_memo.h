#ifndef data_memo_h
#define data_memo_h

#include <Arduino.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <queue>
#include <limits>
#include "calculation.h"

void next_state(int next); 
int get_state(); 

void init_stepper_count(); 
void add_stepper_count(int n); 

void init_turn_count(); 
void track_turn_count(int n); 
std::vector<int> track_beacon_angle(String beacon_colour); 

void update_current_angle(double angle); 
void estimate_current_angle(int direction, int steps); 
double get_current_angle(); 
void calculate_current_position(); 
void estimate_current_position(int steps); 
std::pair<int, int> get_current_position(); 

void graph_init(); 
String get_graph(); 

void add_process(); 

void add_node_at_current_position(); 
void check_node_at_new_node(); 
void check_node_at_new_path(); 

std::vector<std::vector<int>> find_new_path(); 
std::vector<int> dijkstra(); 

#endif