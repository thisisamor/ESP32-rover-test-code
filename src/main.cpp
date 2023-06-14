#include <Arduino.h>
#include <string>
#include <vector>
#include "balance_control.h"
#include "wifi_connection.h"
#include "FPGA_communication.h"
#include "data_memo.h"
#include "calculation.h"

// TESTING VARIABLES
const int TEST_NUM = 2;
int loopCounter = 0;

// MPU 6050 Variables
MPU_6050 mpu6050 = MPU_6050(0.98);    // Creating an object for the MPU 6050 and specifying the Complementary Filter parameter.

// pins reserved for FPGA
#define wall_0 15   // pin 12  (horizontal_flag)
#define wall_1 14   // pin 10  
#define wall_2 4    // pin 11
#define beacon_0 18 // pin 6
#define beacon_1 5  // pin 7
#define key_1 2     // pin 13  (bin level)
#define UART_RXD 16 // pin 9
#define UART_TXD 17 // pin 8

// pins reserved for motor
#define STPR 32     // A4
#define DIRR 33     // A3
#define STPL 25     // A2
#define DIRL 26     // A1

String camera_detection_wall; 
String camera_detection_beacon; 
// int camera_command; 
double route_angle; 

String web_addr_graph_data = "http://44.211.152.110:3001/nodes/"; 
String web_addr_current_position = "http://44.211.152.110:3001/rover"; 

void setup() {
  Serial.begin(9600);
  // Serial.begin(115200);
  // Serial2.begin(115200, SERIAL_8N1, UART_RXD, UART_TXD); 

  // Pin initializations.
  pinMode(wall_0, INPUT);
  pinMode(wall_1, INPUT);
  pinMode(wall_2, INPUT); 
  pinMode(beacon_0, INPUT); 
  pinMode(beacon_1, INPUT); 
  pinMode(key_1, OUTPUT); 
  pinMode(UART_RXD, INPUT); 
  pinMode(UART_TXD, OUTPUT); 

  pinMode(STPR, OUTPUT);
  pinMode(STPL, OUTPUT);
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);

  // Initializing the MPU 6050
  // mpu6050.activate();
  // mpu6050.gyro_calibrate();

  // graph_init(); 
  // data_memo_init();  // TODO: check & correct start position

  wifi_connection(); 
}

void loop() {

  // Test MPU 6050
  if (TEST_NUM == 0)
  {
    mpu6050.readSensor();   // Sensor readings should be taken continously for accurate angle estimations using the gyroscope.

    if (loopCounter >= 200){
      Serial.println("HERE");
      Serial.printf("%f,%f,%f\n", mpu6050.accAngles.x, mpu6050.accAngles.y, mpu6050.accAngles.z);
      Serial.printf("%f,%f,%f\n", mpu6050.gyroAngles.x, mpu6050.gyroAngles.y, mpu6050.gyroAngles.z);
      Serial.printf("%f,%f,%f\n", mpu6050.filteredAngles.x, mpu6050.filteredAngles.y, mpu6050.filteredAngles.z);
      Serial.printf("%f,%f,%f\n", mpu6050.gyroAngVelCal.x, mpu6050.gyroAngVelCal.y, mpu6050.gyroAngVelCal.z);
      loopCounter = 0;
    }
    loopCounter++;
  }
  else if (TEST_NUM == 1) // Test receive camera data from FPGA and main state machine.
  {    
    // camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
    // camera_detection_beacon = read_FPGA_beacon(beacon_0, beacon_1); 


    Serial.println("Explored nodes: " + get_graph()); 
    // ------- check "state machine" --------------------------------
    int currentState = get_state(); 
    Serial.println("Current state: " + String(currentState)); 

    // ------ state = no_wall ---------------------------------------
    if (currentState == 0)
    {
      Serial.println("No wall: Moving forward"); 
      // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 1, 1}, 25); // 2 revo ~= 350 mm
      Serial.println("Motor command: forward 25 steps"); 
      add_stepper_count(25); 

      while ( readFromSerial()!="ok" ){} // delay

      Serial.println("Tracking current position"); 
      estimate_current_position(25); 
      Serial.println("Current position: " + String(get_current_position().first) + ", " + String(get_current_position().second)); 
    
      // wall detection
      camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
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
      Serial.println("Yes wall: Turning left"); 
      // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 0, 1}, 5); 
      Serial.println("Motor command: turning left 5 steps (9 degree)"); 

      while ( readFromSerial()!="ok" ){} // delay

      estimate_current_angle(1, 5); 
      Serial.println("Current angle: " + String(get_current_angle())); 
      track_turn_count(5); // not used
    
      // wall detection
      camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
      if (camera_detection_wall == "111")
      {
        route_angle = get_current_angle(); 
        next_state(1); 
      }
    }

    // ------ state = turn_right ---------------------------------------
    else if (currentState == 2)
    {
      Serial.println("Yes wall: Turning right"); 
      // motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 1, 0}, 5); 
      Serial.println("Motor command: turning left 5 steps (9 degree)"); 
      
      while ( readFromSerial()!="ok" ){} // delay

      estimate_current_angle(-1, 5); 
      Serial.println("Current angle: " + String(get_current_angle())); 
      track_turn_count(5); // not used
    
      // wall detection
      camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
      if (camera_detection_wall == "111")
      {
        route_angle = get_current_angle(); 
        next_state(1); 
      }
    }

    // ------ state = new_node ---------------------------------------
    else if (currentState == 3)
    {
      Serial.println("Facing the wall: Adding new node"); 

      Serial.println("Locating the beacons"); 
      init_turn_count(); 
      do {  
        camera_detection_beacon = read_FPGA_beacon(beacon_0, beacon_1); 
        track_beacon_angle(camera_detection_beacon); 
        motor_control_custom(STPL, STPR, DIRL, DIRR, {1, 0, 0, 1}, 5); 
        track_turn_count(5); // TODO: double check
      } while ( track_beacon_angle(camera_detection_beacon)[1] != 1 || track_beacon_angle(camera_detection_beacon)[2] != 1 || track_beacon_angle(camera_detection_beacon)[3] != 1 ); 

      calculate_current_position(); 

      add_node_at_current_position(); 
      add_process(); 
      
      Serial.println("Checking node connection"); 
      check_node_at_new_node(); 
      
      next_state(1); 
    }

    // ------ state = find_next_path ---------------------------------------
    else if (currentState == 4)
    {
      Serial.println("Finding next path"); 
      
      if (route_angle != 100) // use memorised route angle
      {
        // TODO: turn to route angle
        route_angle = 100; 
      }
      else
      {
        std::vector<std::vector<int>> motor_command = find_new_path(); 
        std::vector<int> break_indicator = {0, 0, 0, 0, 0}; 
        if( motor_command[0] != break_indicator)
        {
          motor_control_command_list(STPL, STPR, DIRL, DIRR, motor_command); 

          // wall detection
          camera_detection_wall = read_FPGA_wall(wall_0, wall_1, wall_2); 
          if (camera_detection_wall == "000")
          {
            check_node_at_new_path(); // TODO: possible to optimise
            // TODO: review process stack
            next_state(1); 
          }
        } // else: next loop would entre "finish" state
      }
    }
  }
    
  else if (TEST_NUM == 2)  // Test server connection.
  {    
    //------- POST data (explored from maze) to the server ---------- 
    if (!check_wifi)
    {
      wifi_connection(); 
    }
    else
    {
      String message; 
      if (get_state()==5)
      {
        message = "/20/175/-175/21//21/100/-580/20,22//22/2000/-1870/21/"; 
        Serial.println("Sending POST request to server: " + message); 
        client_post(web_addr_graph_data, message); 
      }
      else 
      {
        message = "175,-175"; 
        Serial.println("Sending POST request to server: " + message); 
        client_post(web_addr_current_position, message); 
        // message = String(get_current_position().first) + ',' + String (get_current_position().second); 
      }
    }
  }
  else if (TEST_NUM == 3) // test dijkstra algorithm (optional)
  {
    std::vector<int> shortest_route = dijkstra(); 
  }
}