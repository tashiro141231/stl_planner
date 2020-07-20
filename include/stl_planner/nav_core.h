
#ifndef CORE_INCLUDED
#define CORE_INCLUDED
enum RobotStatus { 
  NO_DESTINATION,
  SET_DESTINATION,
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
    int x;
    int y;
} Node;

#endif

