
#include "stl_planner/gp.h"

GPlanner::GPlanner(){
  ;
}

//mapのmin,maxを取得gridのindexのmaxを計算
//todo 引数のmapx.mapy,mapcostからgridのコストを更新
void GPlanner::calc_map(std::vector<double> map_x,std::vector<double> map_y, std::vector<int> cost){
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

int GPlanner::calc_ind(int x,int y){
    return this->max_xind*y+x;
}

std::vector<Node> GPlanner::rawmap_to_node(unsigned char* map) {
  std::vector<Node> g_map;
  Node buff;
  buff.searched = false;
  for(int itr = 0; itr < this->width*this->height; itr++) {
    buff.cost = map[itr];
    g_map.push_back(buff);
  }
  return g_map;
}

void GPlanner::setStartPoint(Point start) {
  sx = start.x;
  sy = start.y;
}

void GPlanner::setGoalPoint(Point goal) {
  gx = goal.x;
  gy = goal.y;
}

void GPlanner::setStartGoal(Point start, Point goal) {
  sx = start.x;
  sy = start.y;
  gx = goal.x;
  gy = goal.y;
}

void GPlanner::setMap(int width, int height, unsigned char* map) {
  this->width = width;
  this->height = height;
  this->origin_map = rawmap_to_node(map);
}


Node GPlanner::getCostOrigin(int x, int y) {
  return this->origin_map[x*this->width+y];
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
