
#include "stl_planner/gp.h"

GPlanner::GPlanner(){
  ;
}

void GPlanner::Init_robotmodel() {
  std::vector<Point> p;
  Point buff;
  buff = createpoint_cost(1, 1, 1);
  p.push_back(buff);
  buff = createpoint_cost(1, 0, 1);
  p.push_back(buff);
  buff = createpoint_cost(0, 1, 1);
  p.push_back(buff);
  buff = createpoint_cost(-1, 0, 1);
  p.push_back(buff);
  buff = createpoint_cost(0, -1, 1);
  p.push_back(buff);
  buff = createpoint_cost(-1, -1, 1);
  p.push_back(buff);
  buff = createpoint_cost(-1, 1, 1);
  p.push_back(buff);
  buff = createpoint_cost(1, -1, 1);
  p.push_back(buff);
 
  robot_model_ = p;
}

Point GPlanner::createpoint(int x, int y) {
  Point p;
  p.x = x;
  p.y = y;
  return p;
}

Point GPlanner::createpoint_cost(int x, int y, double cost) {
  Point p;
  p.x = x;
  p.y = y;
  p.theta = cost;   //Use theta for cost
}

//mapのmin,maxを取得gridのindexのmaxを計算
//todo 引数のmapx.mapy,mapcostからgridのコストを更新
void GPlanner::calc_map(std::vector<double> map_x,std::vector<double> map_y, std::vector<int> cost) {
/*
  std::vector<double>::iterator iter = max_element(map_x.begin(), map_x.end());
  size_t index = distance(map_x.begin(), iter);
  this->max_x = map_x[index];
  iter = max_element(map_y.begin(), map_y.end());
  index = distance(map_y.begin(), iter);
  this->max_y = map_y[index];
  iter = min_element(map_x.begin(), map_x.end());
  index = distance(map_x.begin(), iter);
  this->min_x = map_x[index];
  iter = min_element(map_y.begin(), map_y.end());
  index = distance(map_y.begin(), iter);
  this->min_y = map_y[index];
  this->max_xind = (this->max_x-this->min_x)/this->resolution;
  this->max_yind = (this->max_y-this->min_y)/this->resolution;
*/
}


//path生成　とりあえずresolution間隔でまっすぐつなぐだけ
//todo gridの情報使ってdijkstraとかで換装したい
void GPlanner::calc_path(){
  double dx=this->gx-this->sx;
  double dy=this->gy-this->sy;
  double d = hypot(dx,dy);
  double path_n=d/this->resolution;
  /*
  for(int i=0;i<path_n;i++){
      Point p = {this->sx+i*dx/path_n,this->sy+i*dy/path_n};
      this->path.emplace_back();
  }
  */
}

void GPlanner::calc_path_astar() {
  bool findFlag=false;
  std::vector<std::pair<Point, Point> > open;
  std::vector<std::pair<Point, Point> > close;     // vector of a node and its parent. cost is recorded to first point's thetta
  std::pair<Point, Point> buff;
  double parent_cost = calc_heuristic(start_, goal_);    //start cost
  start_.theta = parent_cost; //using theta namespace for cost
  buff.first = start_;
  buff.second = start_;
  open.push_back(buff);
  Init_robotmodel();

  while(!findFlag) {
    sortPointCost(open);

    if(isSamePoint(open[0].first, goal_)) {      //Goal judge.
      std::cout << "Found goal!!" << std::endl;
      findFlag = true;
      break;
    }
    if(open.empty()){
      std::cout << "Could not find a path." << std::endl;
      break;
    }

    std::pair<Point, Point> c_node;
    buff = open[0];
    open.erase(open.begin());
    close.push_back(c_node);      //move the searched node to the close list.
    
    for(int itr = 0; itr < robot_model_.size(); itr++) {
      c_node = buff;
      c_node.second = c_node.first;   //set the parent.
      c_node.first.x = c_node.first.x + robot_model_[itr].x;    //calcurating grid x
      c_node.first.y = c_node.first.y + robot_model_[itr].y;    //calcurating grid y
      c_node.first.theta = calc_heuristic(c_node.first, goal_) - c_node.first.theta + robot_model_[itr].theta;   //heuristic cost + moving cost
      std:: cout << "cost: " << c_node.first.theta << std::endl;
      if(isObstacle(c_node.first)) {         //
        continue;
      }
      checkLists(c_node, open, close);
    }
  }
  std::vector<Point> path = lookup_closednode(close);
  visualize_result(path, close);
}

void GPlanner::checkLists(std::pair<Point, Point> node, std::vector<std::pair<Point, Point> >& open, std::vector<std::pair<Point, Point> >& close) {
  std::pair<Point, Point> b;
  bool hit = false;

  for(int itr = 0; itr < open.size(); itr++) {
    //The case the node exist in the open list.
    //if(node.first.x == open[itr].first.x && node.first.y == open[itr].first.y) {
    if(isSamePoint(node.first, open[itr].first)) {
      if(node.first.theta < open[itr].first.theta) {
        open[itr].second = node.second;    //change parent node to more close node.
        open[itr].first.theta = node.first.theta;    //change parent node to more close node.
      }
      hit = true;
      break;
    }
  }
  for(int itr = 0; itr < close.size(); itr++) {
    //The case the node exist in the close list.
    //if(node.first.x == close[itr].first.x && node.first.y == close[itr].first.y) {
    if(isSamePoint(node.first, close[itr].first)) {
      if(node.first.theta < close[itr].first.theta) {
        close[itr].first.theta = node.first.theta;
        b = close[itr];
        b.second = node.second;
        close.erase(close.begin() + itr);
        open.push_back(b);
      }
      hit = true;
      break;
    }
  }
  //The case the node doesn't exit both lists.
  if(!hit) {
    open.push_back(node);
  }
}

std::vector<Point> GPlanner::lookup_closednode(std::vector<std::pair<Point, Point> >& close) {
  std::vector<Point> p;
  Point buff;

  buff = goal_;
  p.insert(p.begin(), buff);    //goal set
  while(1) {
    if(isSamePoint(buff, start_)) {
      p.insert(p.begin(), buff);    //path creating.
      break;
    }
    for(int itr = 0; itr < close.size(); itr++) {
      //if(close[itr].second.x == buff.x && close[itr].second.y == buff.y) {
      if(isSamePoint(buff, close[itr].first)) {
        buff = close[itr].second;
        p.insert(p.begin(), buff);    //path creating.
      }
    }
  }
  return p;
}

void GPlanner::visualize_result(std::vector<Point> path, std::vector<std::pair<Point, Point> > close) {
  FILE *gid;
  if((gid = popen("gnuplot", "w")) == NULL) std::cout << "gnuplot open error" << std::endl;
  fprintf(gid, "plot '-'\n");

  //visualize search list.
  for(int itr = 0; itr < close.size(); itr++) {
    fprintf(gid, "%lf, %lf\n", (float)close[itr].first.x, (float)close[itr].first.y);
  }

  for(int itr = 0; itr < path.size(); itr++) {
    fprintf(gid, "%lf, %lf\n", (float)path[itr].x, (float)path[itr].y);
  }

  fprintf(gid, "e\n");
  fflush(gid);
  getchar();
  fprintf(gid, "pause -l");
  pclose(gid);
}

void GPlanner::sortPointCost(std::vector<std::pair<Point, Point> >& open) {
  std::pair<Point, Point> buff;
  int sort_in = 0;

  //bubble sort
  bool sorting = true;
  while(sorting) {
    sorting = false;
    for(int i = 0; i < open.size() - 1 - sort_in; ++i) {
      if(open[i].first.theta > open[i+1].first.theta) {
        buff = open[i];
        open[i] = open[i+1];
        open[i+1] = buff;
        sorting = true;
      }
    }
    sort_in++;
  }
}

int GPlanner::calc_ind(int x,int y){
    return max_xind*y + x;
}

bool GPlanner::isSamePoint(Point start, Point goal) {
  if(start.x == goal.x && start.y == goal.y) return true;
  else  return false;
}

bool GPlanner::isObstacle(Point p) {
  for(int itr = 0; itr < o_map_.size(); itr++) {
    if(p.x == o_map_[itr].x && p.y == o_map_[itr].y) {
      return true;
    }
  }
  return false;
}

double GPlanner::calc_heuristic(Point p1, Point p2) {
  return hypot(p1.x -p2.x, p1.y - p2.y);
}

std::vector<Node> GPlanner::rawmap_to_node(Point lower_left, unsigned char* map) {
  std::vector<Node> g_map;
  Node buff;

  FILE *gid;
  if((gid = popen("gnuplot", "w")) == NULL) std::cout << "gnuplot open error" << std::endl;
  fprintf(gid, "plot '-'\n");

  for(int itr = 0; itr < width_*height_; itr++) {
    if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = map[itr];
      buff.x = (int)(lower_left.x) + (int)(itr % width_);
      buff.y = (int)(lower_left.y) + (int)(itr / width_);
      g_map.push_back(buff);
      fprintf(gid, "%lf, %lf\n", (float)buff.x, (float)buff.y);
    }
  }
  fprintf(gid, "e\n");
  fflush(gid);
  getchar();
  fprintf(gid, "pause -l");
  pclose(gid);
  return g_map;
}

void GPlanner::setStartPoint(Point start) {
  start_ = start;
}

void GPlanner::setGoalPoint(Point goal) {
  goal_ = goal;
}

void GPlanner::setStartGoal(Point start, Point goal) {
  start_ = start;
  goal_ = goal;
}

void GPlanner::setMap(int width, int height, Point lower_left, unsigned char* map) {
  width_ = width;
  height_ = height;
  max_x_ = width;
  max_y_ = height;

  std::cout << "w, h: " <<  width << " " << height << std::endl;
  o_map_ = rawmap_to_node(lower_left, map);
}


Node GPlanner::getCostOrigin(int x, int y) {
  return this->o_map_[x*this->width+y];
}

std::vector<Point> GPlanner::getPath() {
  return path;
}

/*
int main(void){
    vector<double> map_x{1,2,3};
    vector<double> map_y{1,2,3};
    vector<int> map_cost{1,2,3};
    GPlanner G(0,0,2,4,map_x,map_y,map_cost); //startx,y,goalx,y,map
    G.calc_path();
    cout<<G.path.size()<<endl;
    cout<<G.grid.size()<<endl;
    
    return 0;
}
*/
