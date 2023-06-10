#include <AFMotor.h>
#include <NewPing.h>
#include <stdio.h>
#include <Arduino.h>
#include <StackArray.h>
#include <float.h>
#include <LinkedList.h>



// For Ultrasonic Sensors
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define STACK_SIZE 100


// Defining a constant 10x10 Grid
#define ROW 10
#define COL 10


AF_DCMotor motor_1(1);
AF_DCMotor motor_2(2);
AF_DCMotor motor_3(3);
AF_DCMotor motor_4(4);

// The following line of code is for moving the robot:
void speed_Increase(int speed) {
  int maxSpeed = speed;  // Maximum speed
  for (int i = 0; i <= maxSpeed; i++) {
    delay(10);  // Delay for smooth ramp-up
    return i;  // Return the current speed value
  }
  return maxSpeed;  // Return the maximum speed if the loop completes
}

void set_Speed(int Speed) {
  motor_1.setSpeed(Speed);
  motor_2.setSpeed(Speed);
  motor_3.setSpeed(Speed);
  motor_4.setSpeed(Speed);
}

void stop_motors() {
  motor_1.run(RELEASE);
  motor_2.run(RELEASE);
  motor_3.run(RELEASE);
  motor_4.run(RELEASE);
}

void move_direction(int Speed, const String& Direction) {
  set_Speed(Speed);
  uint8_t direction_Value_Left;
  uint8_t direction_Value_Right;
  if (Direction == "FORWARD") {
    direction_Value_Left = FORWARD;
    direction_Value_Right = FORWARD;
  } else if (Direction == "BACKWARD") {
    direction_Value_Left = BACKWARD;
    direction_Value_Right = BACKWARD;
  } else if (Direction == "RIGHT" || Direction == "LEFT") {
    set_Speed(150);
    if (Direction == "RIGHT") {
      direction_Value_Left = FORWARD;
      direction_Value_Right = BACKWARD;
      delay(1000);
      stop_motors();
    } else if (Direction == "LEFT") {
      direction_Value_Left = BACKWARD;
      direction_Value_Right = FORWARD;
      delay(1000);
      stop_motors();
    }
  } else{
    stop_motors();
  }
  stop_motors();
}

// The following line of code is for the A* algorithm:

template <typename T>
class Stack {
private:
  T stackArray[STACK_SIZE];
  int top;

public:
  Stack() {
    top = -1;
  }

  bool isEmpty() {
    return (top == -1);
  }

  bool isFull() {
    return (top == STACK_SIZE - 1);
  }

  void push(T item) {
    if (isFull()) {
      Serial.println
      ("Stack overflow!");
      return;
    }
    stackArray[++top] = item;
  }

  T pop() {
    if (isEmpty()) {
      Serial.println("Stack underflow!");
      return T();
    }
    return stackArray[top--];
  }

  T topElement() {
    if (isEmpty()) {
      Serial.println("Stack is empty!");
      return T();
    }
    return stackArray[top];
  }
};

struct Pair {
  int first;
  int second;
};

struct pPair {
  double first;
  Pair second;
};

struct cell {
  int parent_i;   // Row index of the parent cell
  int parent_j;   // Column index of the parent cell
  double f_cost;       // Total cost of the cell (f = g + h)
  double g_cost;       // Cost from the start cell to this cell
  double h_cost;       // Heuristic cost from this cell to the goal cell
};

bool is_Valid (int row, int col){
  return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

bool is_Blocked (int grid [ROW][COL], int row, int col) {
  return (grid[row][col] == 1);
}
bool is_Goal (Pair goal, int row, int col);
bool is_Goal (Pair goal, int row, int col) {
  return (goal.first == row && goal.second == col);
}

double calculate_Heuristic_Value(int row, int col, Pair goal);
double calculate_Heuristic_Value(int row, int col, Pair goal)
{
    // Return using the Euclidean's distance formula
    return ((double)sqrt(
        (row - goal.first) * (row - goal.first)
        + (col - goal.second) * (col - goal.second)));
}
void insert_Sorted(LinkedList<pPair>& list, const pPair& element);
void insert_Sorted(LinkedList<pPair>& list, const pPair& element) {
  for (int i = 0; i < list.size(); i++) {
    if (element.first < list.get(i).first) {
      list.add(i, element);
      return;
    }
  }
  list.add(element);
}

// Check if the current cell that is being proccessed has a parent cell
// If the current cell does not have a parent cell it mean that the starting cell has been reached
// The parent cell is the previous cell that leads to the current cell in the optimal path.
// The parent cells are added to a stack call Path which store the optimal path 
// Thereafter the stack is printed from the top
void tracePath(cell cell_Details[ROW][COL], Pair goal);
void tracePath(cell cell_Details[ROW][COL], Pair goal) {
  Serial.println("\nThe Path is ");
  int row = goal.first;
  int col = goal.second;

  Stack<Pair> Path;

  while (!(cell_Details[row][col].parent_i == row &&
           cell_Details[row][col].parent_j == col)) {
    Path.push({row, col});
    int temp_row = cell_Details[row][col].parent_i;
    int temp_col = cell_Details[row][col].parent_j;
    row = temp_row;
    col = temp_col;
  }

  Path.push({row, col});
  while (!Path.isEmpty()) {
    Pair path_coords = Path.topElement();
    Path.pop();
    Serial.print(path_coords.first);
    Serial.print(",");
    Serial.print(path_coords.second);
  }

  return;
}


void a_Star_Search(int grid[ROW][COL], Pair src, Pair goal);
void a_Star_Search(int grid[ROW][COL], Pair src, Pair goal)
{
  // If the source is out of range
  if (is_Valid(src.first, src.second) == false) {
      Serial.print("Source is invalid\n");
      return;
  }

  // If the destination is out of range
  if (is_Valid(goal.first, goal.second) == false) {
      Serial.print("Destination is invalid\n");
      return;
  }

  // Either the source or the destination is blocked
  if (is_Blocked(grid, src.first, src.second) == true
      || is_Blocked(grid, goal.first, goal.second)
              == true) {
      Serial.print("Source or the Goal Cell is blocked\n");
      return;
  }

  // If the destination cell is the same as source cell
  if (is_Goal(goal, src.first, src.second) == true) {
      Serial.print("We are already at the destination\n");
      return;
  }


  // Create a closed list and initialise it to false which
  // means that no cell has been included yet This closed
  // list is implemented as a boolean 2D array
  bool closed_List[ROW][COL];
  memset(closed_List, false, sizeof(closed_List));

  // Declare a 2D array of structure to hold the details
  // of that cell
  cell cell_Details[ROW][COL];

  int i, j;

  for (i = 0; i < ROW; i++) {
      for (j = 0; j < COL; j++) {
          cell_Details[i][j].f_cost = FLT_MAX;
          cell_Details[i][j].g_cost = FLT_MAX;
          cell_Details[i][j].h_cost = FLT_MAX;
          cell_Details[i][j].parent_i = -1;
          cell_Details[i][j].parent_j = -1;
      }
  }

  // Initialising the parameters of the starting node
  i = src.first, j = src.second;
  cell_Details[i][j].f_cost = 0.0;
  cell_Details[i][j].g_cost = 0.0;
  cell_Details[i][j].h_cost = 0.0;
  cell_Details[i][j].parent_i = i;
  cell_Details[i][j].parent_j = j;

  /*Create an open list having information as-
  <f, <i, j>>
  where f = g + h,
  and i, j are the row and column index of that cell
  Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
  This open list is implemented as a set of pair of
  pair.*/
  LinkedList<pPair> open_List;
  // Put the starting cell on the open list and set its
  // 'f' as 0
  insert_Sorted(open_List, {0.0, {i, j}});

  bool found_Goal = false;
 
  while (open_List.size() > 0) {
    pPair p = open_List.get(0); // Get the element with the lowest 'f' value
    open_List.remove(0); // Remove the element from the open list

    // Add this vertex to the closed list
    i = p.second.first;
    j = p.second.second;
    closed_List[i][j] = true;

    /*Generating all the 4 successor of this cell

          N.W   N   N.E
            \   |   /
            \  |  /
          W----Cell----E
              / | \
            /   |  \
          S.W    S   S.E

      Cell-->Popped Cell (i, j)
      N -->  North       (i-1, j)
      S -->  South       (i+1, j)
      E -->  East        (i, j+1)
      W -->  West        (i, j-1)*/

    // To store the 'g', 'h' and 'f' of the 8 successors
    double gNew, hNew, fNew;

    //----------- 1st Successor (North) ------------

    // Only process this cell if this is a valid one
    if (is_Valid(i - 1, j) == true) {
      // If the destination cell is the same as the
      // current successor
      if (is_Goal(goal, i - 1, j) == true) {
          // Set the Parent of the destination cell
          cell_Details[i - 1][j].parent_i = i;
          cell_Details[i - 1][j].parent_j = j;
          Serial.print("The destination cell is found\n");
          tracePath(cell_Details, goal);
          found_Goal = true;
          return;
      }
      // If the successor is already on the closed
      // list or if it is blocked, then ignore it.
      // Else do the following
      else if (closed_List[i - 1][j] == false && is_Blocked(grid, i - 1, j) == true) {
        gNew = cell_Details[i][j].g_cost + 1.0;
        hNew = calculate_Heuristic_Value(i - 1, j, goal);
        fNew = gNew + hNew;

        // If it isn’t on the open list, add it to
        // the open list. Make the current square
        // the parent of this square. Record the
        // f, g, and h costs of the square cell
        //                OR
        // If it is on the open list already, check
        // to see if this path to that square is
        // better, using 'f' cost as the measure.
        if (cell_Details[i - 1][j].f_cost == FLT_MAX || cell_Details[i - 1][j].f_cost > fNew) {
          insert_Sorted(open_List, {fNew, {i-1, j}});
          // Update the details of this cell

          cell_Details[i - 1][j].f_cost = fNew;
          cell_Details[i - 1][j].g_cost = gNew;
          cell_Details[i - 1][j].h_cost = hNew;
          cell_Details[i - 1][j].parent_i = i;
          cell_Details[i - 1][j].parent_j = j;
        }
      } 
    }

    //----------- 2nd Successor (South) ------------

    // Only process this cell if this is a valid one
    if (is_Valid(i + 1, j) == true) {
      // If the destination cell is the same as the current successor
      if (is_Goal(goal, i + 1, j) == true) {
          // Set the Parent of the destination cell
          cell_Details[i + 1][j].parent_i = i;
          cell_Details[i + 1][j].parent_j = j;
          Serial.print("The destination cell is found\n");
          tracePath(cell_Details, goal);
          found_Goal = true;
          return;
      }
      // If the successor is already on the closed list or if it is blocked, then ignore it.
      // Else do the following
      else if (closed_List[i + 1][j] == false && is_Blocked(grid, i + 1, j) == true) {
        gNew = cell_Details[i][j].g_cost + 1.0;
        hNew = calculate_Heuristic_Value(i + 1, j, goal);
        fNew = gNew + hNew;

        // If it isn’t on the open list, add it to the open list. Make the current square
        // the parent of this square. Record the f, g, and h costs of the square cell
        //                OR
        // If it is on the open list already, check to see if this path to that square is
        // better, using 'f' cost as the measure.
        if (cell_Details[i + 1][j].f_cost == FLT_MAX || cell_Details[i + 1][j].f_cost > fNew) {
          insert_Sorted(open_List, {fNew, {i + 1, j}});
          // Update the details of this cell

          cell_Details[i + 1][j].f_cost = fNew;
          cell_Details[i + 1][j].g_cost = gNew;
          cell_Details[i + 1][j].h_cost = hNew;
          cell_Details[i + 1][j].parent_i = i;
          cell_Details[i + 1][j].parent_j = j;
        }
      } 
    }

    //----------- 3rd Successor (East) ------------

    // Only process this cell if this is a valid one
    if (is_Valid(i, j + 1) == true) {
      // If the destination cell is the same as the current successor
      if (is_Goal(goal, i, j + 1) == true) {
          // Set the Parent of the destination cell
          cell_Details[i][j + 1].parent_i = i;
          cell_Details[i][j + 1].parent_j = j;
          Serial.print("The destination cell is found\n");
          tracePath(cell_Details, goal);
          found_Goal = true;
          return;
      }
      // If the successor is already on the closed list or if it is blocked, then ignore it.
      // Else do the following
      else if (closed_List[i][j + 1] == false && is_Blocked(grid, i, j + 1) == true) {
        gNew = cell_Details[i][j].g_cost + 1.0;
        hNew = calculate_Heuristic_Value(i, j + 1, goal);
        fNew = gNew + hNew;

        // If it isn’t on the open list, add it to the open list. Make the current square
        // the parent of this square. Record the f, g, and h costs of the square cell
        //                OR
        // If it is on the open list already, check to see if this path to that square is
        // better, using 'f' cost as the measure.
        if (cell_Details[i][j + 1].f_cost == FLT_MAX || cell_Details[i][j + 1].f_cost > fNew) {
          insert_Sorted(open_List, {fNew, {i, j + 1}});
          // Update the details of this cell

          cell_Details[i][j + 1].f_cost = fNew;
          cell_Details[i][j + 1].g_cost = gNew;
          cell_Details[i][j + 1].h_cost = hNew;
          cell_Details[i][j + 1].parent_i = i;
          cell_Details[i][j + 1].parent_j = j;
        }
      } 
    }

    //----------- 4th Successor (West) ------------
    // Only process this cell if this is a valid one
    if (is_Valid(i, j - 1) == true) {
      // If the destination cell is the same as the current successor
      if (is_Goal(goal, i, j - 1) == true) {
          // Set the Parent of the destination cell
          cell_Details[i][j - 1].parent_i = i;
          cell_Details[i][j - 1].parent_j = j;
          Serial.print("The destination cell is found\n");
          tracePath(cell_Details, goal);
          found_Goal = true;
          return;
      }
      // If the successor is already on the closed list or if it is blocked, then ignore it.
      // Else do the following
      else if (closed_List[i][j - 1] == false && is_Blocked(grid, i, j - 1) == true) {
        gNew = cell_Details[i][j].g_cost + 1.0;
        hNew = calculate_Heuristic_Value(i, j - 1, goal);
        fNew = gNew + hNew;

        // If it isn’t on the open list, add it to the open list. Make the current square
        // the parent of this square. Record the f, g, and h costs of the square cell
        //                OR
        // If it is on the open list already, check to see if this path to that square is
        // better, using 'f' cost as the measure.
        if (cell_Details[i][j - 1].f_cost == FLT_MAX || cell_Details[i][j - 1].f_cost > fNew) {
          insert_Sorted(open_List, {fNew, {i, j - 1}});
          // Update the details of this cell

          cell_Details[i][j - 1].f_cost = fNew;
          cell_Details[i][j - 1].g_cost = gNew;
          cell_Details[i][j - 1].h_cost = hNew;
          cell_Details[i][j - 1].parent_i = i;
          cell_Details[i][j - 1].parent_j = j;
        }
      } 
    }
  } 
  // When the destination cell is not found and the open list is empty, then we conclude that we failed to
  // reach the destination cell. This may happen when the there is no way to destination cell (due to blockages)
  if (found_Goal == false)
      Serial.print("Failed to find the Destination Cell\n");

  return;
}


// Driver program to test above function
void setup()
{
  Serial.begin(2400); 
  // Description of the Grid -
  // 0 --> The cell is not blocked
  // 1 --> The cell is blocked
  int grid[ROW][COL]
      = { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

  // Source is the left-most bottom-most corner
  Pair start_Cell;
  start_Cell.first = 8;
  start_Cell.second = 0;

  // Destination is the left-most top-most corner
  Pair goal_Cell;
  goal_Cell.first = 0;
  goal_Cell.second = 0;

  a_Star_Search(grid, start_Cell, goal_Cell);
  //Serial.print(is_Valid(2,2));
  //Serial.print("hi");
    
}
 
void loop()
{
    // Your code here
}
