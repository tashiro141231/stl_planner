
#ifndef CORE_INCLUDED
#define CORE_INCLUDED
enum RobotStatus { 
  NO_DESTINATION,
  MOVING_FOLLOW_GP,
  MOVING_OBJ_AVOID,
  STUCKING,
  STOP
};

typedef struct Position {
  double x;
  double y;
  double z;
  double theta;
} Position;

typedef struct Point {
  double x;
  double y;
  double theta;
} Point;

typedef struct Node{
    unsigned char cost;
    bool searched;
} Node;

#endif

