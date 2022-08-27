/**
 * @file gp.cpp
 * @brief Global Planer for STELLA.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.04.19
*/

#include "stl_planner/gp.h"

GPlanner::GPlanner(){
  ;
}
/** @fn Set Robot model for Astar.
 * @brief 
 * @param
 * @detail
*/
void GPlanner::Init_robotmodel() {
  Point buff;
  robot_model_.clear();
  buff = createpoint_cost(1.0, 1.0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(1.0, 0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(0, 1.0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(-1.0, 0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(0, -1.0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(-1.0, -1.0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(-1.0, 1.0, 1.0);
  robot_model_.push_back(buff);
  buff = createpoint_cost(1.0, -1.0, 1.0);
  robot_model_.push_back(buff);
}

/** @fn 
 * @brief
 * @param
 * @detail
*/
Point GPlanner::createpoint(double x, double y) {
  Point p;
  p.x = x;
  p.y = y;
  return p;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
Point GPlanner::createpoint_cost(double x, double y, double cost) {
  Point p;
  p.x = x;
  p.y = y;
  p.theta = cost;   //Use theta for cost
  return p;
}

//mapのmin,maxを取得gridのindexのmaxを計算
//todo 引数のmapx.mapy,mapcostからgridのコストを更新
/** @fn 
 * @brief 
 * @param
 * @detail
*/
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
/** @fn 
 * @brief 
 * @param
 * @detail
*/
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

/** @fn 
 * @brief 
 * @param
 * @detail
*/
std::vector<Point> GPlanner::calc_path_astar() {
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

  std::cout << "Start searching!" << std::endl;
  while(!findFlag) {
    sortPointCost(open);
    //std::cout << "open size: " << open.size() << " close size: " << close.size() << std::endl;

    if(isSamePoint(open[0].first, goal_)) {      //Goal judge.
      close.push_back(open[0]);
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
    close.push_back(buff);      //move the searched node to the close list.
    
    for(int itr = 0; itr < robot_model_.size(); itr++) {
      c_node = buff;
      c_node.second = c_node.first;   //set the parent.
      c_node.first.x = c_node.first.x + robot_model_[itr].x;    //calcurating grid x
      c_node.first.y = c_node.first.y + robot_model_[itr].y;    //calcurating grid y
      c_node.first.theta = c_node.first.theta + robot_model_[itr].theta + calc_heuristic(c_node.first, goal_) - calc_heuristic(buff.first, goal_);   //heuristic cost + moving cost
      //std::cout << "cost: " << c_node.first.theta << std::endl;
      if(isObstacle(c_node.first)) {         //
        continue;
      }
      //std:: cout << "Out" << std::endl;
      checkLists(c_node, open, close);
    }
  }
  std::vector<Point> path = lookup_closednode(close, open);
  
  return path;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void GPlanner::checkLists(std::pair<Point, Point> node, std::vector<std::pair<Point, Point> >& open, std::vector<std::pair<Point, Point> >& close) {
  std::pair<Point, Point> b;
  bool hit = false;

  for(int itr = 0; itr < open.size(); itr++) {
    //The case the node exist in the open list.
    if(isSamePoint(node.first, open[itr].first)) {
      if(node.first.theta < open[itr].first.theta) {
        open[itr].second = node.second;    //change parent node to more close node.
        open[itr].first.theta = node.first.theta;    //change parent node to more close node.
      }
      hit = true;
      return;
    }
  }
  for(int itr = 0; itr < close.size(); itr++) {
    //The case the node exist in the close list.
    if(isSamePoint(node.first, close[itr].first)) {
      if(node.first.theta < close[itr].first.theta) {
        close[itr].first.theta = node.first.theta;
        b = close[itr];
        b.second = node.second;
        close.erase(close.begin() + itr);
        open.push_back(b);
      }
      hit = true;
      return;
    }
  }
  //The case the node doesn't exit both lists.
  if(!hit) {
    open.push_back(node);
  }
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
std::vector<Point> GPlanner::lookup_closednode(std::vector<std::pair<Point, Point> >& close, std::vector<std::pair<Point, Point> >& open) {
  std::vector<Point> p;
  std::vector<Point> debug;
  Point buff;

  buff = open[0].first;
  p.insert(p.begin(), ConvGridScale(buff));    //goal set
  //debug.insert(p.begin(), buff);    //goal set
  buff = open[0].second;

  while(1) {
    if(isSamePoint(buff, start_)) {
      p.insert(p.begin(), ConvGridScale(buff));    //start point.
      //debag.insert(p.begin(), buff);    //start point.
      break;
    }
    for(int itr = 0; itr < close.size(); itr++) {
      if(isSamePoint(buff, close[itr].first)) {
        buff = close[itr].second;
        p.insert(p.begin(), ConvGridScale(buff));    //path creating.
        //debag.insert(p.begin(), buff);    //path creating.
        break;
      }
    }
  }
  return p;
}

Point GPlanner::ConvGridScale(Point p) {
  p.x = p.x * resolution_;
  p.y = p.y * resolution_;
  return p;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void GPlanner::visualize_result(std::vector<Point> path, std::vector<std::pair<Point, Point> > close) {
  FILE *gid;
  if((gid = popen("gnuplot", "w")) == NULL) std::cout << "gnuplot open error" << std::endl;
  fprintf(gid, "plot '-'\n");

  //visualize search list.
  //for(int itr = 0; itr < close.size(); itr++) {
  //  fprintf(gid, "%lf, %lf\n", (float)close[itr].first.x, (float)close[itr].first.y);
  //}

  for(int itr = 0; itr < path.size(); itr++) {
    fprintf(gid, "%lf, %lf\n", (float)path[itr].x, (float)path[itr].y);
  }

  for(int itr = 0; itr < o_map_.size(); itr++) {
    fprintf(gid, "%lf, %lf\n", (float)o_map_[itr].x, (float)o_map_[itr].y);
  }
  fprintf(gid, "e\n");

  fflush(gid);
  getchar();
  fprintf(gid, "pause -l");
  pclose(gid);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
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

/** @fn 
 * @brief 
 * @param
 * @detail
*/
int GPlanner::calc_ind(int x,int y){
    return max_xind*y + x;
}

/** @fn 
 * @brief 
 * @param
 * @detail
Star_ROS has been init.*/
bool GPlanner::isSamePoint(Point start, Point goal) {
  if(start.x == goal.x && start.y == goal.y) return true;
  else  return false;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
bool GPlanner::isObstacle(Point p) {
  //for(int itr = 0; itr < o_map_.size(); itr++) {
  //  if(p.x == o_map_[itr].x && p.y == o_map_[itr].y) {
  //    return true;
  //  }
  //}
  //int index = (lower_left_.x + p.x) + (lower_left_.y + p.y) * width_;
  Point lower_left = ConvGridPoint(lower_left_);
  int index = (int)((-lower_left.x + p.x) + (-lower_left.y + p.y) * width_);
  std::cout << width_ << " " << height_ << std::endl;
  if(total_cost_[index] != 0x00 && total_cost_[index] != 0xFF) {
    std::cout << "x: " << p.x << " y: " << p.y << " index: " << index << std::endl;
    std::cout << "grid_x: " << p.x/resolution_ << " y: " << p.y/resolution_ << std::endl;
    std::cout << "start_ x: " << start_.x << " y: " << start_.y << std::endl;
    std::cout << "goal_ x: " << goal_.x << " y: " << goal_.y << std::endl;
    std::cout << "lower?left_ x: " << lower_left_.x << " y: " << lower_left_.y << std::endl;
    return true;
  }
  return false;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
double GPlanner::calc_heuristic(Point p1, Point p2) {
  return hypot(p1.x -p2.x, p1.y - p2.y);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
std::vector<Node> GPlanner::rawmap_to_node(Point lower_left, unsigned char* map) {
  std::vector<Node> g_map;
  Node buff;

  lower_left = ConvGridPoint(lower_left);

  for(int itr = 0; itr < width_*height_; itr++) {
    if(total_cost_[itr] != 0xFF && total_cost_[itr] != 0x00) {
    //if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = total_cost_[itr];
      //buff.x = (int)(lower_left.x) + (int)(itr % width_);
      //buff.y = (int)(lower_left.y) + (int)(itr / width_);A
      buff.x = (int)(lower_left.x) + (int)(itr % width_);
      buff.y = (int)(lower_left.y) + (int)(itr / width_);
      g_map.push_back(buff);
    }
  }
  return g_map;
}



/** @fn 
 * @brief 
 * @param
 * @detail
*/
std::vector<Node> GPlanner::rawmap_to_node_vis(Point lower_left, unsigned char* map) {
  std::vector<Node> g_map;
  Node buff;

  FILE *gid;
  if((gid = popen("gnuplot", "w")) == NULL) std::cout << "gnuplot open error" << std::endl;
  fprintf(gid, "plot '-'\n");

  for(int itr = 0; itr < width_*height_; itr++) {
    //if(map[itr] != 0xFF && map[itr] != 0x00) {
    if(total_cost_[itr] != 0xFF && total_cost_[itr] != 0x00) {
      buff.cost = total_cost_[itr];
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

/** @fn 
 * @brief Set the cost map around origin map.
 * @param
 * @detail
*/
std::vector<unsigned char> GPlanner::costmap_calc(Point lower_left, unsigned char* map) {
  std::vector<unsigned char> cost_map;
  cost_map.resize(width_ * height_, 0);
  //std::cout << "resize : " << cost_map.size() << std::endl;
  int cost_mergin = 0.15;  //m

  for(int i = 0; i < width_*height_; i++) {
    int x = (int)(lower_left.x) + (int)(i % width_);
    int y = (int)(lower_left.y) + (int)(i / width_);

    if(map[i] != 0xFF && map[i] != 0x00) {
      cost_map[i] = map[i];
      //std::cout << "Set cost around: " << x << " , " << y << std::endl;
      //Add the cost around a node.
      for(int xi = (int)(-cost_mergin/resolution_); xi < (int)(cost_mergin/resolution_); xi++) {
        for(int yi = (int)(-cost_mergin/resolution_); yi < (int)(cost_mergin/resolution_); yi++) {
          int cost_x = x + xi;
          int cost_y = y + yi * width_;
          if(cost_x >= 0 && cost_x < width_) {
            if(cost_y >= 0 && cost_y < height_) {
              cost_map[cost_x + cost_y] = 0x64;
            }
          }
        }
      }
    }
  }
  return cost_map;
}


/** @fn 
 * @brief Set the cost map around origin map.
 * @param
 * @detail
*/
std::vector<unsigned char> GPlanner::costmap_calc_vis(Point lower_left, unsigned char* map) {
  std::vector<unsigned char> cost_map;
  cost_map.resize(width_ * height_, 0);
  //std::cout << "resize : " << cost_map.size() << std::endl;
  int cost_mergin = 0.5;  //m

  FILE *gid;
  if((gid = popen("gnuplot", "w")) == NULL) std::cout << "gnuplot open error" << std::endl;
  fprintf(gid, "plot '-'\n");

  for(int i = 0; i < width_*height_; i++) {
    int x = (int)(lower_left.x) + (int)(i % width_);
    int y = (int)(lower_left.y) + (int)(i / width_);

    if(map[i] != 0xFF && map[i] != 0x00) {
      cost_map[i] = map[i];
      //Add the cost around a node.
      for(int xi = (int)(-cost_mergin/resolution_); xi < (int)(cost_mergin/resolution_); xi++) {
        for(int yi = (int)(-cost_mergin/resolution_); yi < (int)(cost_mergin/resolution_); yi++) {
          int cost_x = x + xi;
          int cost_y = y + yi;
          if(cost_x > 0 && cost_x < width_) {
            if(cost_y > 0 && cost_y < height_) {
              cost_map[cost_x + (int)(cost_y * width_)] = 0x64;
              fprintf(gid, "%lf, %lf\n", (float)cost_x, (float)cost_y);
            }
          }
        }
      }
    }
  }
  fprintf(gid, "e\n");
  fflush(gid);
  getchar();
  fprintf(gid, "pause -l");
  pclose(gid);

  return cost_map;
}

Point GPlanner::ConvGridPoint(Point point) {
  Point grid_point;
  grid_point.x = (int)(point.x / resolution_);
  grid_point.y = (int)(point.y / resolution_);
  return grid_point;
}


/** @fn 
 * @brief 
 * @param
 * @detail
*/
void GPlanner::setStartPoint(Point start) {
  start_ = ConvGridPoint(start);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void GPlanner::setGoalPoint(Point goal) {
  goal_ = ConvGridPoint(goal);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void GPlanner::setStartGoal(Point start, Point goal) {
  start_ = ConvGridPoint(start);
  goal_ = ConvGridPoint(goal);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void GPlanner::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  width_ = width;
  height_ = height;
  max_x_ = width;
  max_y_ = height;
  resolution_ = resolution;
  lower_left_ = lower_left;

  total_cost_ = costmap_calc(lower_left, map);
  o_map_ = rawmap_to_node(lower_left, map);
  //total_cost_ = costmap_calc_vis(lower_left, map);
  std::cout << "Map received." << std::endl;
  //o_map_ = rawmap_to_nodei_vis(lower_left, map);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
Node GPlanner::getCostOrigin(int x, int y) {
  return this->o_map_[x*this->width+y];
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
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

