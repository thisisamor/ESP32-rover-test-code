#include <Arduino.h>
#include <string>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <queue>
#include <limits>
#include "calculation.h"

// ------------------------- state memory ---------------------------------
typedef enum 
{
    no_wall,
    turn_left,
    turn_right,
    // may have: try
    new_node,
    find_next_path, 
    // optional: fast
    finish
} State;

State currentState = no_wall;
State lastState = no_wall; 

void next_state(int next)
{
    switch (currentState) 
    {
        case no_wall:  // 0
            lastState = no_wall; 
            if (next == 010) 
            {
                currentState = turn_left;
            } 
            else if(next == 100)
            {
                currentState = turn_right; 
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

std::vector<int> track_beacon_angle(String beacon_colour)
{
    if (beacon_colour=="01")
    {
        motor_R = turn_count; 
        beacon_detected[1] = 1; 
    }
    else if (beacon_colour=="10")
    {
        motor_B = turn_count; 
        beacon_detected[2] = 1; 
    }
    else if (beacon_colour=="11")
    {
        motor_Y = turn_count; 
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
    Serial.println("Current angle (estimated): " + String(current_angle)); 
}

double get_current_angle() 
{
    return current_angle; 
}

void calculate_current_position() 
{
    std::pair<double, double> angle_a_b = angle_calculation(motor_R, motor_B, motor_Y);
    current_position = position_calculation(angle_a_b.first, angle_a_b.second); 
    Serial.println("Current position (calculated): " + String(current_position.first) + ", " + String(current_position.second)); 
}

void estimate_current_position(int steps) 
{
    current_position = position_estimation(current_position, current_angle, steps); 
    Serial.println("Current position (estimated): " + String(current_position.first) + ", " + String(current_position.second)); 
}

std::pair<int, int> get_current_position()
{
    Serial.println("Current position: " + String(current_position.first) + ", " + String(current_position.second)); 
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
};

int hashFunction(int key) 
{
    // TODO: more efficient hash function? 
    return key % TABLE_SIZE;
}

HashTable* node_graph = NULL; 
Node null_node; 
int last_id; 

void insert(struct HashTable* ht, int key, Node value) 
{
    int index = hashFunction(key);
    struct HashEntry* newEntry = (struct HashEntry*)malloc(sizeof(struct HashEntry));
    newEntry->key = key;
    newEntry->value = value;
    newEntry->next = NULL;
    if (ht->table[index] == NULL) 
    {
        ht->table[index] = newEntry;
    } 
    else 
    {
        struct HashEntry* current = ht->table[index];
        while (current->next != NULL) 
        {
            current = current->next;
        }
        current->next = newEntry;
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
    Serial.println("Wrong: cannot find node!"); 
    return null_node; // (if not found)
}

String get_graph()
{
    String graph_string; 
    std::vector<std::vector<int>> tmp; 
    int index = hashFunction(0);
    struct HashEntry* current = node_graph->table[index];
    while (current != NULL) 
    {
        graph_string = graph_string + '/' + String(current->value.node_id) + '/' + String(current->value.position.first) + '/' + String(current->value.position.second) + '/'; 
        tmp = current->value.connected_node; 
        for (int i =0; i<tmp.size(); i++)
        {
            graph_string = graph_string + String((tmp[i])[0]); 
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
    Serial.println("New node added!"); 
}

void update_node( struct HashTable* ht, int key, std::vector<std::vector<int>> new_connect )
{
    int hashValue = hashFunction(key); 

    struct HashEntry* current = ht->table[hashValue];
    while (current != NULL) 
    {
        if (current->key == key) 
        {
            (current->value).connected_node = new_connect; 
            Serial.println("Node updated!"); 
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
    Serial.println("Wrong: delete connection failed!"); 
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
    
    Serial.println("Node inserted between " + String(node1_id) + " and " + String(node2_id) + "!"); 
}

// ----------------------------- process stack ---------------------------
struct Process
{
    std::pair<int, int> node_id_pair;   // node1 -> node2 (older to newer)
    std::pair<bool, bool> path_left_right_checked; 
    std::pair<bool, bool> node_left_right_checked; 
    Process* last_process; 
}; 

Process* stack = new Process; 

void data_memo_init()
{
    Node node0; 
    node0.node_id = 0; 
    node0.position = std::make_pair(175, -175); 
    insert(node_graph, 0, node0); 
    last_id = 0; 

    stack->last_process = NULL; 
    stack->node_id_pair = std::make_pair(0, 0); 
    stack->node_left_right_checked = {false, false}; // for 0 only: front_left_checked
    stack->path_left_right_checked = {true, true}; // no path exist for node 0
}

void add_process()
{
    Process* new_process = new Process; 
    std::pair<int, int> new_node_id_pair; 
    new_node_id_pair = std::make_pair(stack->node_id_pair.second, last_id); 
    new_process->node_id_pair = new_node_id_pair; 
    new_process->node_left_right_checked = {false, false}; 
    stack->path_left_right_checked = {false, false};
    if (lastState == turn_left)   // TODO: warning?
    {
        stack->node_left_right_checked.second = true; 
    }
    else if (lastState == turn_right)
    {
        stack->node_left_right_checked.first = true; 
    }
    new_process->last_process = stack; 
    stack = new_process; 
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
    add_node(current_position, tmp); 
    // add_process();   // duplication 
    Serial.println("New node added at current position."); 
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
                    Serial.println("New node added at (" + String(new_node_position.first) + ", " + String(new_node_position.second) + " )."); 
                }
                // insert 1 or 2 between 3 4
                else if ( check == 10 )
                {
                    std::pair<int, int> new_node_position = intersection_calculation((first->value).position, (second->value).position, (get_node(stack->node_id_pair.first)).position, (get_node(stack->node_id_pair.second)).position); 
                    if ( same_position((first->value).position, new_node_position) )
                    {
                        insert_new_node(stack->node_id_pair.first, stack->node_id_pair.second, first->value.node_id); 
                        split_process(stack->node_id_pair, first->value.node_id); 
                        Serial.println("Inserted node" + String(first->value.node_id) + " between node " + String(get_node(stack->node_id_pair.first).node_id) + " and node " + String(get_node(stack->node_id_pair.second).node_id)); 
                    }
                    else if ( same_position((second->value).position, new_node_position) )
                    {
                        insert_new_node(stack->node_id_pair.first, stack->node_id_pair.second, second->value.node_id); 
                        split_process(stack->node_id_pair, second->value.node_id); 
                        Serial.println("Inserted node" + String(second->value.node_id) + " between node " + String(get_node(stack->node_id_pair.first).node_id) + " and node " + String(get_node(stack->node_id_pair.second).node_id)); 
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
                        Serial.println("Inserted node" + String((get_node(stack->node_id_pair.first)).node_id) + " between node " + String(first->value.node_id) + " and node " + String(second->value.node_id)); 
                    }
                    else if ( same_position( (get_node(last_id)).position, new_node_position) )
                    {
                        insert_new_node((first->value).node_id, (second->value).node_id, last_id);  
                        split_process(std::make_pair((first->value).node_id, (second->value).node_id), last_id); 
                        Serial.println("Inserted node" + String((get_node(last_id)).node_id) + " between node " + String(first->value.node_id) + " and node " + String(second->value.node_id)); 
                    }
                }
            }
            second = second->next; 
        }
        first = first->next;
    }
}

// when a new path is found
// check if current_position is a new node
// if so, add the new one
void check_node_at_new_path() 
{
    struct HashEntry* current = node_graph->table[0];
    while (current != NULL)    
    {
        if (same_position((current->value).position, current_position) == false)
        {
            std::pair<int, int> new_node_position = current_position; 
            std::vector<std::vector<int>> tmp; 
            add_node(new_node_position, tmp); 
            // 1. split top process
                // 1.1  new_node -> stack_second  : checking side = true; other side = migrate; 
                // 1.2  stack_first -> new_node   : checking side = false; other side = migrate; 
                // TODO: checking side memo

            // 2. add stack_first and stack_second to tmp
            // (connection info)


            Serial.println("New node added at (" + String(new_node_position.first) + ", " + String(new_node_position.second) + " )."); 
        }
        current = current->next;
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

std::vector<std::vector<int>> find_new_path()
{
    std::vector<std::vector<int>> motor_command; 
    while (currentState != finish)
    {
        // TODO: check if at stack_second
        if ( same_position(current_position, get_node(stack->node_id_pair.second).position) == false )
        {
            // if not, move to that position
        }
        

        // 1. top process: node_left_right_checked
            // not {true, true} -> motor command generate, [return]
            // {true, true} -> continue
        if ( stack->node_left_right_checked.first == false ) // node left not checked
        {
            motor_command.push_back(left); // turn left
            motor_command[motor_command.size()-1][4] = get_angle_num(0, M_PI_2); 
            return motor_command; 
            // TODO: modify node.left_wall
        }
        else if ( stack->node_left_right_checked.second == false ) // node right not checked
        {
            motor_command.push_back(right); // turn right
            motor_command[motor_command.size()-1][4] = get_angle_num(0, M_PI_2); 
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
                return motor_command; 
            }
        }

        // 3. if everything have been checked, delete top process
        finish_process(); 

        // NOTE: currently using recursive call until new motor command is found
        // not sure if this is the best way
        return find_new_path(); 
    }
    
    // if stack is empty, return {{0, 0, 0, 0, 0}} to stop exploring
    return {{0, 0, 0, 0, 0}}; 
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