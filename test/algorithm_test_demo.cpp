#include <string>
#include <vector>
#include <stdio.h>
#include <queue>
#include <limits>
#include <math.h>
#include <utility>
#include <iostream>
#include <algorithm>

/*
A C++ demo showing how the maze exploring and solving algorithm work, 
motor commands and node/process stack operations are made visible as terminal output, 
different scenarios, camera detection results can be tested as terminal inputs. 
*/

std::string camera_detection_wall; 
std::string camera_detection_beacon; 
double route_angle; 

int length = 3600; // mm
int width = 2400; 

int intersect_TH = 1; // TODO: slightly larger than 0? 
int same_position_TH = 20; 

int wheel_diameter = 65; 
int steps_per_revo = 200;
double distance_per_step = M_PI * wheel_diameter / steps_per_revo;  
double radian_per_step = 2 * M_PI / steps_per_revo; 

void printVector(const std::vector<int>& vec)
{
    for (const auto& element : vec) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}

// -------------------------- Calculation ----------------------------

double cot(double angle)
{   
    if (fmod(angle, M_PI) == 0.0){      // tan(angle) is 0 if the angle is a multiple of PI. This results in an infinite (Undefined) cot() result.
        return NAN;
    }
    else if (fmod(angle, M_PI/2) == 0.0){   // tan(angle) is infinite if the angle is PI/2, 3PI/2, etc. This results in a zero cot() result.
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
std::pair<double, double> angle_calculation(int motor_Y, int motor_B, int motor_R)
{
    std::cout << motor_Y << " " << motor_B << " " << motor_R << std::endl; 
    double a, b; 
    if (motor_B > motor_Y)
    {
        b = 2*M_PI*(motor_B - motor_Y)/steps_per_revo; 
    }
    else 
    {
        b = 2*M_PI*(motor_B - motor_Y + steps_per_revo)/steps_per_revo; 
    }
    if (motor_R > motor_B)
    {
        a = 2*M_PI*(motor_R - motor_B)/steps_per_revo; 
    }
    else 
    {
        a = 2*M_PI*(motor_R - motor_B + steps_per_revo)/steps_per_revo; 
    }
    std::cout << "calculated angle a = " << a*180/M_PI << ", angle b = " << b*180/M_PI << std::endl; 
    return std::make_pair(a, b); 
}

std::pair<int, int> position_calculation(double angle_a, double angle_b)
{
    double a = (length * width / 2 ) * ( length/2 * cot(angle_a) - width/2) * (cot(angle_a)*cot(angle_b)-1) /( pow(length/2 * cot(angle_a)-width/2, 2) + pow(length/2 - width/2 *cot(angle_b), 2) ); 
    double b = (length * width / 2 ) * ( width/2 * cot(angle_b)- length/2) * (cot(angle_a)*cot(angle_b)-1) /( pow(length/2  * cot(angle_a)-width/2 , 2) + pow(length/2  - width/2  *cot(angle_b), 2) );
    return std::make_pair(int(3600-a), int(b-2400)); 
}

double angle_estimation(double current_angle, int direction, int steps)
{
    double tmp = current_angle+direction*(steps*2*M_PI/steps_per_revo); 
    return angle_range(tmp); 
}

std::pair<int, int> position_estimation(std::pair<int, int> current_position, double current_angle, int steps)
{
    //std::cout << "moved: " << sin(current_angle) * distance_per_step << std::endl; 
    double tmp = double(steps); 
    current_position.first = current_position.first + cos(current_angle) * distance_per_step * tmp; 
    current_position.second = current_position.second + sin(current_angle) * distance_per_step * tmp; 
    return current_position; 
}

bool same_position(std::pair<int, int> position1, std::pair<int, int> position2)
{
    // std::cout << abs(position1.first-position2.first) << " " << abs(position1.second-position2.second) << std::endl; 
    if ( abs(position1.first-position2.first)<=same_position_TH && abs(position1.second-position2.second)<=same_position_TH )
    {
        return true; 
    }
    else 
    {
        return false; 
    }
}

int check_intersection(std::pair<int, int> node1, std::pair<int, int> node2, std::pair<int, int> node3, std::pair<int, int> node4)
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
    double A = ( (y3-y2)*(x3-x1) - (x3-x2)*(y3-y1) )/1000.0 * ( (y4-y2)*(x4-x1) - (x4-x2)*(y4-y1) )/1000.0; // 3 4 different sides of 1 2
    double B = ( (y1-y4)*(x1-x3) - (x1-x4)*(y1-y3) )/1000.0 * ( (y2-y4)*(x2-x3) - (x2-x4)*(y2-y3) )/1000.0; // 1 2 different sides of 3 4

    if ( A <= -intersect_TH && B <= -intersect_TH ) 
    {
        return 11; 
    }
    else if ( A <= -intersect_TH && B <= intersect_TH && B >= -intersect_TH ) 
    {
        return 10; 
    }
    else if ( B <= -intersect_TH && A <= intersect_TH && A >= -intersect_TH )
    {
        return 01; 
    }
    else
    {
        return 00; 
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

    return false; 
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

// ------------------------- state memory ---------------------------------
typedef enum 
{
    no_wall,
    turn_left,
    turn_right,
    // may have: try
    new_node,
    find_next_path, 
    start, 
    finish
} State;

State currentState = start;
State lastState = no_wall; 

void next_state(int next)
{
    switch (currentState) 
    {
        case no_wall:  // 0
            lastState = no_wall; 
            if (next == 010) 
            {
                currentState = turn_right;
            } 
            else if(next == 100)
            {
                currentState = turn_left; 
            }
            else if(next == 111)
            {
                currentState = new_node; 
            }
            break;

        case turn_left:  // 1
            lastState = turn_left; 
            currentState = new_node;
            break;

        case turn_right:  // 2
            lastState = turn_right; 
            currentState = new_node;
            break;

        case new_node:  // 3
            currentState = find_next_path;
            break;

        case find_next_path:  // 4
            currentState = no_wall;
            break;

        case start: 
            if ( next == 0 )
            {
                currentState = no_wall; 
                break; 
            }
            else if ( next == 1 )
            {
                currentState = finish; 
                break; 
            }

        default:
            currentState = no_wall;

    }
}

int get_state()
{
    return currentState; 
}


// ------------------------- track position -----------------------------
double current_angle = -M_PI_2; 
std::pair<int, int> current_position = {175, -175}; 
// TODO: may change for init position

int stepper_count; // forward
int turn_count; 
int motor_Y, motor_B, motor_R; 
std::vector<int> beacon_detected; 

void init_stepper_count()
{
    stepper_count = 0; 
}

void add_stepper_count(int n)
{
    stepper_count += n; 
}

void init_turn_count()
{
    turn_count = 0; 
    motor_Y = motor_B = motor_R = 0; 
    beacon_detected = {0, 0, 0, 0}; 
}

void track_turn_count(int n)
{
    turn_count += n; 
}

std::vector<int> track_beacon_angle(std::string beacon_colour)
{
    if (beacon_colour=="01")
    {
        motor_Y = turn_count; 
        beacon_detected[1] = 1; 
    }
    else if (beacon_colour=="10")
    {
        motor_B = turn_count; 
        beacon_detected[2] = 1; 
    }
    else if (beacon_colour=="11")
    {
        motor_R = turn_count; 
        beacon_detected[3] = 1; 
    }
    return beacon_detected; 
}

void update_current_angle(double angle)
{
    current_angle = angle; 
}

void estimate_current_angle(int direction, int steps)
{
    // direction:  1 = left; -1 = right; 
    current_angle = angle_estimation(current_angle, direction, steps); 
    std::cout << "Current angle (estimated): " <<  current_angle*180/M_PI << std::endl; 
}

double get_current_angle() 
{
    return current_angle; 
}

void calculate_current_position() 
{
    std::pair<double, double> angle_a_b = angle_calculation(motor_Y, motor_B, motor_R);
    current_position = position_calculation(angle_a_b.first, angle_a_b.second); 
    std::cout << "Current position (calculated): " << (current_position.first) << ", " << (current_position.second) << std::endl; 
}

void calculate_current_position_gyro(double angle_Y, double angle_B, double angle_R)
{
    double a, b; 
    b = angle_range((angle_B-angle_Y)*M_PI/180); 
    a = angle_range((angle_R-angle_B)*M_PI/180); 
    std::cout << "calculated angle a = " << a*180/M_PI << ", angle b = " << b*180/M_PI << std::endl; 
    current_position = position_calculation(a, b); 
    std::cout << "Current position (calculated): " << (current_position.first) << ", " << (current_position.second) << std::endl; 
}

void estimate_current_position(int steps) 
{
    
    current_position = position_estimation(current_position, current_angle, steps); 
    // std::cout << "distance per step: " << distance_per_step << std::endl; 
    std::cout << "Current position (estimated): " << (current_position.first) << ", " << (current_position.second) << std::endl; 
}

std::pair<int, int> get_current_position()
{
    std::cout << "Current position: " << (current_position.first) << ", " << (current_position.second)<< std::endl; 
    return current_position; 
}


// ---------------------------- graph data ------------------------------
struct Node
{
    int node_id; 
    std::pair<int, int> position; 
    std::vector<std::vector<int>> connected_node; 
    // [0] = id; [1] = left_wall; [2] = right_wall; 
}; 
Node null_node; 

struct HashEntry 
{
    int key;
    Node value;
    struct HashEntry* next;
};

#define TABLE_SIZE 30
struct HashTable 
{
    struct HashEntry* table[TABLE_SIZE];
    HashTable()
    {
        for (int i = 0; i < TABLE_SIZE; i++)
        {
            table[i] = NULL; 
        }
    }
};

int hashFunction(int key) 
{
    // TODO: more efficient hash function? 
    return key % TABLE_SIZE;
}

HashTable* node_graph = new HashTable; 
int last_id; 

void insert(struct HashTable* ht, int key, Node value) 
{
    int index = hashFunction(key);
    struct HashEntry* newEntry = new HashEntry; 
    newEntry->key = key; 
    newEntry->value = value;
    newEntry->next = NULL;
    if (ht->table[index] == NULL)
    {
        ht->table[index] = newEntry;
        if ( index != 0 )
        {
            ht->table[index-1]->next = newEntry; 
        }
        std::cout << "insert: ok" << std::endl;
    }
    else
    {
        ht->table[index]->value = value;
        std::cout << "insert: BUG" << std::endl;
    }
}

void graph_init()
{
    Node node0; 
    node0.node_id = 0; 
    node0.position = std::make_pair(175, -175); 
    insert(node_graph, 0, node0); 
    last_id = 0; 
}

Node get_node(int key)
{
    int index = hashFunction(key);
    struct HashEntry* current = node_graph->table[index];
    while (current != NULL) 
    {
        if (current->key == key) 
        {
            return current->value;
        }
        current = current->next;
    }
    // should not happen
    std::printf("Wrong: cannot find node!"); 
    return null_node; // (if not found)
}

std::string get_graph()
{
    std::string graph_string = "/"; 
    std::vector<std::vector<int>> tmp; 
    int index = hashFunction(0);
    struct HashEntry* current = node_graph->table[index];
    while (current != NULL) 
    {
        // std::cout <<"make graph string loop: " << current->key << std::endl; 
        graph_string = graph_string + '/' + std::to_string(current->value.node_id) + '/' + std::to_string(current->value.position.first) + '/' + std::to_string(current->value.position.second) + '/'; 
        tmp = current->value.connected_node; 
        for (int i =0; i<tmp.size(); i++)
        {
            graph_string = graph_string + std::to_string((tmp[i])[0]); 
            if (i!= tmp.size()-1)
            {
                graph_string = graph_string + ','; 
            }
        }
        graph_string = graph_string + '/'; 
        current = current->next;
    }
    return graph_string; 
}

void add_node( std::pair<int, int> new_node_position, std::vector<std::vector<int>> connect )
{
    Node tmp; 
    last_id ++; 
    tmp.node_id = last_id; 
    tmp.position = new_node_position; 
    tmp.connected_node = connect; 
    insert(node_graph, last_id, tmp); 
    std::cout << "New node id: " << last_id << std::endl; 
}

void update_node( int key, std::vector<std::vector<int>> new_connect )
{
    int hashValue = hashFunction(key); 

    struct HashEntry* current = node_graph->table[hashValue];
    while (current != NULL) 
    {
        if (current->key == key) 
        {
            (current->value).connected_node = new_connect; 
            std::printf("Node updated!"); 
            break; 
        }
        current = current->next;
    }
}

std::vector<int> delete_connect( std::vector<std::vector<int>> &connect, int delete_id )
{
    std::vector<int> tmp; 
    for ( int i=0; i<connect.size(); i++ )
    {
        if ( (connect[i])[0] == delete_id)
        {
            tmp = (connect[i]); 
            for ( int j=i; j<connect.size()-1; j++ )
            {
                connect[j] = connect[j+1]; 
            }
            connect.pop_back(); 
            return tmp; 
        }
    }
    // should not happen
    return tmp; 
    std::printf("Wrong: delete connection failed!"); 
}

void insert_new_node( int node1_id, int node2_id, int new_node_id )
{
    std::vector<int> node1_migrate; 
    std::vector<int> node2_migrate; 

    // modify node1 connect
    int hashValue = hashFunction(node1_id); 
    struct HashEntry* current = node_graph->table[hashValue];
    while (current != NULL) 
    {
        if (current->key == hashValue) 
        {
            node1_migrate = delete_connect((current->value).connected_node, node2_id); 
            break; 
        }
        current = current->next;
    }

    // modify node2 connect
    hashValue = hashFunction(node2_id); 
    current = node_graph->table[hashValue];
    while (current != NULL) 
    {
        if (current->key == hashValue) 
        {
            node2_migrate = delete_connect((current->value).connected_node, node1_id); 
            break; 
        }
        current = current->next;
    }

    // add node 1 and node 2 connection into new_node
    hashValue = hashFunction(new_node_id); 
    current = node_graph->table[hashValue];
    while (current != NULL) 
    {
        if (current->key == hashValue) 
        {
            ((current->value).connected_node).push_back(node1_migrate); 
            ((current->value).connected_node).push_back(node2_migrate); 
            break; 
        }
        current = current->next;
    }
    
    std::cout << "Node inserted between " << node1_id << " and " << node2_id << std::endl; 
}

// ----------------------------- process stack ---------------------------
struct Process
{
    std::pair<int, int> node_id_pair;   // node1 -> node2 (older to newer)
    std::pair<bool, bool> path_left_right_checked; 
    std::pair<bool, bool> node_left_right_checked; 
    Process* last_process; 
}; 

Process* stack = NULL; 

void data_memo_init(bool left_check)
{
    Node node0; 
    node0.node_id = 0; 
    node0.position = std::make_pair(175, -175); 
    insert(node_graph, 0, node0); 
    last_id = 0; 

    stack = new Process; 
    stack->last_process = NULL; 
    stack->node_id_pair = std::make_pair(0, 0); 
    stack->node_left_right_checked = {true, left_check}; // for 0 only: front_left_checked
    stack->path_left_right_checked = {true, true}; // no path exist for node 0

    std::cout << "Data memo init succeeded!" << std::endl; 
}

void add_process()
{
    Process* new_process = new Process; 
    std::pair<int, int> new_node_id_pair; 
    new_node_id_pair = std::make_pair(stack->node_id_pair.second, last_id); 
    new_process->node_id_pair = new_node_id_pair; 
    new_process->node_left_right_checked = {false, false}; 
    new_process->path_left_right_checked = {false, false};
    if (lastState == turn_left)   // TODO: warning?
    {
        new_process->node_left_right_checked.second = true; 
    }
    else if (lastState == turn_right)
    {
        new_process->node_left_right_checked.first = true; 
    }
    new_process->last_process = stack; 
    stack = new_process; 
    std::cout << "New process: " << stack->node_id_pair.first << " -> " << stack->node_id_pair.second << std::endl; 
}

std::string view_process_stack()
{
    std::string process_string = "/"; 
    Process* tmp_process = stack; 
    while ( tmp_process != NULL )
    {
        process_string = "/" + std::to_string(tmp_process->node_id_pair.first) + "->" + std::to_string(tmp_process->node_id_pair.second) + process_string; 
        tmp_process = tmp_process->last_process; 
    }
    return process_string; 
}

void finish_process()
{
    Process* previous_process = stack;
    stack = stack->last_process;
    delete previous_process;
    if (stack == NULL) // TODO: review
    {
        currentState = finish; 
    }
} 

void split_process(std::pair<int, int> pair_to_split, int new_id)
{
    Process* tmp_process = stack; 
    while(tmp_process->last_process->node_id_pair != pair_to_split)
    {
        tmp_process = tmp_process->last_process; 
    }
    tmp_process->last_process->node_id_pair = std::make_pair(pair_to_split.first, new_id); 
    Process* new_process = new Process; 
    new_process->node_id_pair = std::make_pair(new_id, pair_to_split.second); 
    new_process->node_left_right_checked = {true, true}; // TODO: test this
    new_process->path_left_right_checked = tmp_process->last_process->path_left_right_checked; 
    new_process->last_process = tmp_process->last_process; 
    tmp_process->last_process = new_process; 
}


// ================================ Algorithm ==============================

void add_node_at_current_position()
{
    std::vector<std::vector<int>> tmp; 
    std::vector<int> tmp2 = {stack->node_id_pair.second, 0, 0}; 
    tmp.push_back(tmp2); 
    add_node(current_position, tmp); 
    tmp = get_node(stack->node_id_pair.second).connected_node; 
    tmp2 = {last_id, 0, 0}; 
    tmp.push_back(tmp2); 
    update_node(stack->node_id_pair.second, tmp); 
    // add_process();   // duplication 
    std::cout << "New node added at: " << current_position.first << ", " << current_position.second << std::endl; 
}

bool check_node_connection(Node node1, Node node2)
{
    for (int i=0; i<node1.connected_node.size(); i++)
    {
        if ( node1.connected_node[i][0] == node2.node_id )
        {
            return true; 
        }
    }
    return false; 
}

void check_node_at_new_node()
{
    struct HashEntry* first = node_graph->table[0];
    while (first != NULL)    // TODO: (not necessary?) optimise this algorithm 
    {
        struct HashEntry* second = node_graph->table[first->key+1];
        while (second != NULL) 
        {
            // TODO: double check the parameters! 
            if (check_node_connection(first->value, second->value))
            {
                int check = check_intersection((first->value).position, (second->value).position, (get_node(stack->node_id_pair.first)).position, (get_node(stack->node_id_pair.second)).position); 
                if ( check == 11)
                {
                    std::pair<int, int> new_node_position = intersection_calculation((first->value).position, (second->value).position, (get_node(stack->node_id_pair.first)).position, (get_node(stack->node_id_pair.second)).position); 
                    std::vector<std::vector<int>> tmp; 
                    add_node(new_node_position, tmp);  // TODO: check node at new node? 
                    insert_new_node((first->value).node_id, (second->value).node_id, last_id); 
                    insert_new_node(stack->node_id_pair.first, stack->node_id_pair.second, last_id); 
                    split_process(std::make_pair((first->value).node_id, (second->value).node_id), last_id); 
                    split_process(stack->node_id_pair, last_id); 
                    // std::printf("New node added at (" + String(new_node_position.first) + ", " + String(new_node_position.second) + " )."); 
                }
                // insert 1 or 2 between 3 4
                else if ( check == 10 )
                {
                    std::pair<int, int> new_node_position = intersection_calculation((first->value).position, (second->value).position, (get_node(stack->node_id_pair.first)).position, (get_node(stack->node_id_pair.second)).position); 
                    if ( same_position((first->value).position, new_node_position) )
                    {
                        insert_new_node(stack->node_id_pair.first, stack->node_id_pair.second, first->value.node_id); 
                        split_process(stack->node_id_pair, first->value.node_id); 
                        // std::printf("Inserted node" + String(first->value.node_id) + " between node " + String(get_node(stack->node_id_pair.first).node_id) + " and node " + String(get_node(stack->node_id_pair.second).node_id)); 
                    }
                    else if ( same_position((second->value).position, new_node_position) )
                    {
                        insert_new_node(stack->node_id_pair.first, stack->node_id_pair.second, second->value.node_id); 
                        split_process(stack->node_id_pair, second->value.node_id); 
                        // std::printf("Inserted node" + String(second->value.node_id) + " between node " + String(get_node(stack->node_id_pair.first).node_id) + " and node " + String(get_node(stack->node_id_pair.second).node_id)); 
                    }
                }
                // insert 3 or 4 between 1 2
                else if ( check == 01 )
                {
                    std::pair<int, int> new_node_position = intersection_calculation((first->value).position, (second->value).position, (get_node(stack->node_id_pair.first)).position, (get_node(stack->node_id_pair.second)).position); 
                    if ( same_position( (get_node(stack->node_id_pair.first)).position, new_node_position) )
                    {
                        insert_new_node((first->value).node_id, (second->value).node_id, (get_node(stack->node_id_pair.first)).node_id);  
                        split_process(std::make_pair((first->value).node_id, (second->value).node_id), (get_node(stack->node_id_pair.first)).node_id); 
                        // std::printf("Inserted node" + String((get_node(stack->node_id_pair.first)).node_id) + " between node " + String(first->value.node_id) + " and node " + String(second->value.node_id)); 
                    }
                    else if ( same_position( (get_node(last_id)).position, new_node_position) )
                    {
                        insert_new_node((first->value).node_id, (second->value).node_id, last_id);  
                        split_process(std::make_pair((first->value).node_id, (second->value).node_id), last_id); 
                        // std::printf("Inserted node" + String((get_node(last_id)).node_id) + " between node " + String(first->value.node_id) + " and node " + String(second->value.node_id)); 
                    }
                }
            }
            second = second->next; 
        }
        first = first->next;
    }
}

// from node1 to node2
// direction: 1 = left; -1 = right; 
// return: ture = checked; false = not_checked; 
bool check_wall_from_memo(int node3, int node4, bool direction)
{
    struct HashEntry* first = node_graph->table[0];
    while (first != NULL)    
    {
        int i = 0; 
        while ( i < first->value.connected_node.size() ) 
        {
            std::vector<int> tmp = first->value.connected_node[i]; // TODO: optimise the order of checking 
            if ( tmp[1] == 1 ) // left wall -> bool = true
            {
                if ( check_wall((first->value).position, get_node(tmp[0]).position, get_node(node3).position, get_node(node4).position, true, direction))
                {
                    return true; 
                }
            }
            if ( tmp[2] == 1 ) // right wall -> bool = false
            {
                if ( check_wall((first->value).position, get_node(tmp[0]).position, get_node(node3).position, get_node(node4).position, false, direction))
                {
                    return true; 
                }
            }
            i++; 
        }
        first = first->next;
    }
    return false; 
}


// TODO: make sure the vectors are correct
std::vector<int> forward = {1, 0, 1, 1}; 
std::vector<int> left = {1, 0, 0, 1}; 
std::vector<int> right = {1, 0, 1, 0}; 
std::vector<int> fake = {11, 11, 11, 11, 111};

std::vector<std::vector<int>> motor_command_generator(std::vector<int> node_route)
{
    std::vector<std::vector<int>> motor_command; 
    int i = 1; 
    while ( i < node_route.size() )
    {
        // forward command
        std::vector<int> tmp = forward; 
        tmp.push_back( get_step_num( get_node(node_route[i-1]).position, get_node(node_route[i]).position ) ); 
        motor_command.push_back(tmp); 
        i++; 
        // turn direction
        if (i != node_route.size() )
        {
            int tmp_turn = get_angle_num(get_angle(get_node(node_route[i-1]).position, get_node(node_route[i]).position, 0), get_angle(get_node(node_route[i-1]).position, get_node(node_route[i]).position, 0) ); 
            if ( tmp_turn > 0 )
            {
                tmp = left; 
                tmp.push_back(tmp_turn); 
                motor_command.push_back(tmp); 
            }
            else if( tmp_turn < 0 )
            {
                tmp = right; 
                tmp.push_back(-tmp_turn); 
                motor_command.push_back(tmp); 
            }
        }
    }
    return motor_command; 
}

typedef enum 
{
    moving, 
    node_left, 
    node_right, 
    path_left, 
    path_right
} Checking;

Checking current_check; 

void update_checking()
{
    switch (current_check) 
    {
        case node_left:  
            stack->node_left_right_checked.first = true;
            break; 
        case node_right:  
            stack->node_left_right_checked.second = true;
            break; 
        case path_left:  
            stack->path_left_right_checked.first = true;
            break; 
        case path_right: 
            stack->path_left_right_checked.second = true;
            break; 
        default: 
            current_check = moving; 
    }
}

std::vector<std::vector<int>> find_new_path()
{
    std::vector<std::vector<int>> motor_command; 
    while (currentState != finish)
    {
        // TODO: check if at stack_second
        if ( same_position(current_position, get_node(stack->node_id_pair.second).position) == false )
        {
            std::cout << "Moving to stack position: check node " << stack->node_id_pair.second << std::endl; 
            // if not, move to that position
            for (int i=0; i<=last_id; i++)
            {
                if ( same_position(get_node(i).position, current_position) )
                {
                    motor_command = motor_command_generator({i, stack->node_id_pair.second}); 
                }
            }
        }
        

        // 1. top process: node_left_right_checked
            // not {true, true} -> motor command generate, [return]
            // {true, true} -> continue
        if ( stack->node_left_right_checked.first == false ) // node left not checked
        {
            current_check = node_left; 
            motor_command.push_back(left); // turn left
            (motor_command[0]).push_back(get_angle_num(0, M_PI_2)); 
            // std::cout << get_angle_num(0, M_PI_2) <<std::endl; 
            std::cout << "Node left not checked, motor command: "; 
            printVector(motor_command[0]); 
            return motor_command; 
            // TODO: modify node.left_wall
        }
        else if ( stack->node_left_right_checked.second == false ) // node right not checked
        {
            current_check = node_right; 
            motor_command.push_back(right); // turn right
            (motor_command[0]).push_back(get_angle_num(0, M_PI_2)); 
            std::cout << "Node right not checked, motor command: "; 
            printVector(motor_command[0]); 
            return motor_command; 
            // TODO: modify node.right_wall
        }

        // 2. top process: first -> second (backwards)
            // 2.1 check_wall_from_memo
                // wall not checked yet -> motor command generate, [return]
                // wall exist -> update process stack, continue
        if ( stack->path_left_right_checked.first == false ) // path left not checked
        {
            stack->path_left_right_checked.first = true; 
            if (check_wall_from_memo( stack->node_id_pair.first, stack->node_id_pair.second, true ) == false)
            {
                // generate motor command 
                // TODO: test camera detection path
                    // if accurate: one step
                    // else: many steps (one at a time)
                current_check = path_left; 
                motor_command.push_back(fake); 
                std::cout << "Path left not checked, logic not implemented, here is a fake command: "; 
                printVector(motor_command[0]); 
                return motor_command; 
            }
        }
        if ( stack->path_left_right_checked.second == false ) // path left not checked
        {
            stack->path_left_right_checked.second = true; 
            if (check_wall_from_memo( stack->node_id_pair.first, stack->node_id_pair.second, false ) == false)
            {
                // generate motor command 
                // TODO: test camera detection path
                    // if accurate: one step
                    // else: many steps (one at a time)
                current_check = path_right; 
                motor_command.push_back(fake); 
                std::cout << "Path right not checked, logic not implemented, here is a fake command: "; 
                printVector(motor_command[0]); 
                return motor_command; 
            }
        }

        // 3. if everything have been checked, delete top process
        std::cout << "Top stack all checked, delete stack: " << stack->node_id_pair.first << "->" << stack->node_id_pair.second << std::endl; 
        finish_process(); 

        // NOTE: currently using recursive call until new motor command is found
        // not sure if this is the best way
        return find_new_path(); 
    }
    
    // if stack is empty, return {{0, 0, 0, 0, 0}} to stop exploring
    return {{0, 0, 0, 0, 0}}; 
}


// when a new path is found
// check if current_position is a new node
// if so, add the new one
void check_node_at_new_path() 
{
    bool indicator = false; 
    struct HashEntry* current = node_graph->table[0];
    while (current != NULL)    
    {
        if (same_position((current->value).position, current_position) == 0)
        {
            indicator = true; 
        }
        current = current->next;
    }
    if (indicator == false)
    {
        std::cout << "Checking current position: node not added yet" << std::endl; 
        std::pair<int, int> new_node_position = current_position; 
        std::vector<std::vector<int>> tmp; 
        add_node(new_node_position, tmp); 
        // 1. split top process
            // 1.1  new_node -> stack_second  : checking side = true; other side = migrate; 
            // 1.2  stack_first -> new_node   : checking side = false; other side = migrate; 
            // TODO: checking side memo
        Process* new_process = new Process; 
        new_process->node_id_pair = std::make_pair(last_id, stack->node_id_pair.second); 
        
        new_process->path_left_right_checked = 
        stack->node_id_pair = std::make_pair(stack->node_id_pair.first, last_id); 

        // 2. add stack_first and stack_second to tmp
        // (connection info)


        // std::printf("New node added at (" + String(new_node_position.first) + ", " + String(new_node_position.second) + " )."); 

    }
}


// ======================== shortest route =================================

std::vector<int> dijkstra()
{
    int start_id = 0; 
    int target_id = last_id; 
    std::vector<int> dist(last_id + 1, std::numeric_limits<int>::max());
    std::vector<int> prev(last_id + 1, -1);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[start_id] = 0;
    pq.push(std::make_pair(0, start_id));

    while (!pq.empty())
    {
        int current_id = pq.top().second;
        pq.pop();

        struct HashEntry* current_entry = node_graph->table[hashFunction(current_id)];
        while (current_entry != NULL && current_entry->key != current_id)
        {
            current_entry = current_entry->next;
        }

        if (current_entry == NULL)
            continue;

        Node current_node = current_entry->value;

        for (const std::vector<int>& connected : current_node.connected_node)
        {
            int neighbor_id = connected[0];
            int edge_weight = connected[1];

            int new_dist = dist[current_id] + edge_weight;

            if (new_dist < dist[neighbor_id])
            {
                dist[neighbor_id] = new_dist;
                prev[neighbor_id] = current_id;
                pq.push(std::make_pair(new_dist, neighbor_id));
            }
        }
    }
    // reconstruction
    std::vector<int> shortest_path;
    int current_id = target_id;
    while (current_id != -1)
    {
        shortest_path.push_back(current_id);
        current_id = prev[current_id];
    }
    std::reverse(shortest_path.begin(), shortest_path.end());

    return shortest_path;
}


// ======================== main ==================================

int TEST_NUM = 1; 
std::string INPUT; 

void loop() 
{
    if ( TEST_NUM == 0 )
    {
        // data_memo_init(); 
        TEST_NUM = 1; 
    }
    else
    {    
        // camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
        // camera_detection_beacon = read_FPGA_beacon(beacon_0, beacon_1); 


        std::cout << "Explored nodes: " << get_graph() << std::endl; 
        std::cout << "Current process stack: " << view_process_stack() << std::endl; 
        // ------- check "state machine" --------------------------------
        int currentState = get_state(); 
        std::cout << "Current state: " << currentState << std::endl; 

        // ------ state = no_wall ---------------------------------------
        if (currentState == 0)
        {
            std::printf("No wall: Moving forward\n"); 
            // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 1, 1}, 25); // 2 revo ~= 350 mm
            std::printf("Motor command: forward 50 steps\n"); 
            add_stepper_count(50); 

            std::cout << "Type ok when finished" << std::endl; 
            std::cin >> INPUT; 
            while ( INPUT!="ok" ){std::cin >> INPUT; } // delay

            std::printf("Tracking current position\n"); 
            estimate_current_position(50); 
            // std::cout<< "Current position: " << (get_current_position().first) << ", " << (get_current_position().second) <<std::endl; 
            
            // wall detection
            // camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
            std::cout << "Waiting for input camera_detection_wall (111/010/100)" << std::endl; 
            std::cin >> camera_detection_wall; 
            if (camera_detection_wall == "111")
            {
                next_state(111); 
            }
            else if(camera_detection_wall == "010")
            {
                init_turn_count(); 
                next_state(010); 
            }
            else if(camera_detection_wall == "100")
            {
                init_turn_count(); 
                next_state(100); 
            }
        }

        // ------ state = turn_left ---------------------------------------
        else if (currentState == 1)
        {
            std::printf("Yes wall: Turning left\n"); 
            // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 0, 1}, 5); 
            std::printf("Motor command: turning left 5 steps (9 degree)\n"); 

            std::cout << "Type ok when finished" << std::endl; 
            std::cin >> INPUT; 
            while ( INPUT!="ok" ){std::cin >> INPUT; } // delay

            estimate_current_angle(1, 5); 
            // std::cout << "Current angle: " << (get_current_angle()) << std::endl; 
            track_turn_count(5); // not used
            
            // wall detection
            // camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
            std::cout << "Waiting for input camera_detection_wall (111/other)" << std::endl; 
            std::cin >> camera_detection_wall; 
            if (camera_detection_wall == "111")
            {
                route_angle = get_current_angle(); 
                next_state(1); 
            }
        }

        // ------ state = turn_right ---------------------------------------
        else if (currentState == 2)
        {
            std::printf("Yes wall: Turning right\n"); 
            // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 1, 0}, 5); 
            std::printf("Motor command: turning right 5 steps (9 degree)\n"); 
            
            std::cout << "Type ok when finished" << std::endl; 
            std::cin >> INPUT; 
            while ( INPUT!="ok" ){std::cin >> INPUT; } // delay

            estimate_current_angle(-1, 5); 
            // std::cout << "Current angle: " << (get_current_angle()) << std::endl; 
            track_turn_count(5); // not used
            
            // wall detection
            // camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
            std::cout << "Waiting for input camera_detection_wall (111/other)" << std::endl; 
            std::cin >> camera_detection_wall; 
            if (camera_detection_wall == "111")
            {
                route_angle = get_current_angle(); 
                next_state(1); 
            }
        }

        // ------ state = new_node ---------------------------------------
        else if (currentState == 3)
        {
            std::printf("Facing the wall: Adding new node\n"); 

            std::printf("Locating the beacons\n"); 
            init_turn_count(); 
            std::string demo_mood; 
            std::cout << "Choose a demo mood (real/fast/old):"; 
            std::cin >> demo_mood; 
            if (demo_mood=="real")
            {
                do {  
                    // camera_detection_beacon = read_FPGA_beacon(beacon_0, beacon_1); 
                    std::cout << "Waiting for input camera_detection_beacon (00/01/10/11): "; 
                    std::cin >> camera_detection_beacon; 
                    track_beacon_angle(camera_detection_beacon); 
                    // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 0, 1}, 5); 
                    std::printf("Motor command: turning left 5 steps (9 degree)\n"); 

                    std::cout << "Type ok when finished: "; 
                    std::cin >> INPUT; 
                    while ( INPUT!="ok" ){} // delay

                    track_turn_count(5); // TODO: double check
                } while ( track_beacon_angle(camera_detection_beacon)[1] != 1 || track_beacon_angle(camera_detection_beacon)[2] != 1 || track_beacon_angle(camera_detection_beacon)[3] != 1 ); 
            }
            else if(demo_mood == "old")
            {
                int tmp1, tmp2, tmp3; 
                std::cout << "Waiting for input: motor_Y, motor_B, motor_R: "; 
                std::cin >> tmp1 >> tmp2 >> tmp3; 
                track_turn_count(tmp1);
                track_beacon_angle("01"); 
                track_turn_count(tmp2-tmp1);
                track_beacon_angle("10"); 
                track_turn_count(tmp3-tmp2);
                track_beacon_angle("11"); 
                std::cout << "All beacons detected, locating rover..." << std::endl; 
                calculate_current_position(); 
            }
            else 
            {
                double tmp1, tmp2, tmp3; 
                std::cout << "Waiting for input: angle_Y, angle_B, angle_R: " << std::endl; 
                std::cin >> tmp1 >> tmp2 >> tmp3; 
                calculate_current_position_gyro(tmp1, tmp2, tmp3); 
            }
            
            std::cout << "Adding node at current position..." << std::endl; 
            add_node_at_current_position(); 
            add_process(); 
            
            std::printf("Checking node at new node\n"); 
            check_node_at_new_node(); 
            
            next_state(1); 
        }

        // ------ state = find_next_path ---------------------------------------
        else if (currentState == 4)
        {
            std::printf("Finding next path\n"); 
            
            // if (route_angle != 100) // use memorised route angle
            // {
            //     // TODO: turn to route angle
            //     route_angle = 100; 
            // }
            // else
            // {
                bool loop_controller = true; 
                while ( loop_controller == true )
                {    
                    std::vector<std::vector<int>> motor_command = find_new_path(); 
                    std::vector<int> break_indicator = {0, 0, 0, 0, 0}; 
                    if( motor_command[0] != break_indicator)
                    {
                        // motor_control_command_list(STPL, STPR, DIRL, DIRR, motor_command); 

                        // wall detection
                        // camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
                        std::cout << "Waiting for camera detection result (000 for no wall)" ; 
                        std::cin >> camera_detection_wall; 
                        if (camera_detection_wall == "000")
                        {
                            loop_controller = false; 
                            update_checking(); 
                            check_node_at_new_path(); 
                            next_state(1); 
                        }
                        else
                        {
                            update_checking(); 
                        }
                    } 
                    // TODO: next loop would entre "finish" state
                    else
                    {
                        loop_controller = false; 
                        std::cout << "Break indicator detected: send graph to server" << std::endl; 
                    }
                }
            //}
        }

        else if(currentState == 5)   // start
        {
            // TODO: add starting position check

            std::cout << "Waiting for input camera_detection_wall (111/010/100)" << std::endl; 
            std::cin >> camera_detection_wall; 
            if (camera_detection_wall == "111")
            {
                data_memo_init(true); 
                std::cout << "Wall in front of starting point. " << std::endl; 
                std::cout << "Motor command: turn left" << std::endl; 
                std::cout << "Waiting for input camera_detection_wall (111/010/100)" << std::endl; 
                std::cin >> camera_detection_wall; 
                if (camera_detection_wall == "111")
                { // should not happen
                    next_state(1); 
                }
                else 
                {
                    next_state(0); 
                }
            }
            else // 000
            {
                data_memo_init(false); 
                next_state(0); 
            }
        }

        else if (currentState == 6)  // finish
        {
            std::cout << "Exploring finished: send data now. " << std::endl; 
        }
    }

// ------------- shortest path algorithm -------------------------------
    // else if (TEST_NUM == 3) // test dijkstra algorithm (optional)
    // {
    //     std::vector<int> shortest_route = dijkstra(); 
    // }
}

int main ()
{
    // std::cout << "Explored nodes: " << get_graph() << std::endl; 
    // data_memo_init(); 
    // bool tmp = same_position({245, 425}, {245, 425}); 
    // std::cout << tmp; 
    // tmp = same_position({-2147480048, 2147481223}, {-2147480048, 2147481223}); 
    // std::cout << tmp; 

    // std::cout << angle_calculation(25, 75, 125).first << ", " <<angle_calculation(25, 75, 125).second; 
    for ( int i=0; i<10; i++ )
    {
        std::cout << "---- loop count = " << i << " ---- " << std::endl; 
        std::cout << "Current position: " << current_position.first << ", " << current_position.second << std::endl; 
        loop(); 
    }    
    // std::cout << currentState; 
}