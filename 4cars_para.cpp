#include <iostream>
#include <string>
#include <random>
#include <map>
#include <set>
#include <cmath>

using namespace std;

int max_num_iteration = 60;
int population_size = 60;
int vwNum = 5;

class GeneticalAlgorithm{
    
    int init(){
        set<int> gene;
        set<int> next_gene;
        random_device rd;
        for(int i=0;i<population_size;i++){
            gene.insert(rd());
        }
    }
    
    int evaluation(){

    }

    int crossover(){
        
    }

    int selection(){

    }
    
    //一様交叉のルーレット選択
    
};

class CarAgent{
    public:
    int speed = 3;
    int x_speed, y_speed;
    double dx, dy, dig;
    map<string,int> start_pos;
    map<string,int> car_pos;
    map<string,int> goal_pos;

    void set_caragent(int start_x_pos,int start_y_pos,int goal_x_pos,int goal_y_pos);
    void update();
    void collision_avoidance();
};

void CarAgent::set_caragent(int start_x_pos,int start_y_pos,int goal_x_pos,int goal_y_pos){

    start_pos["x"] = start_x_pos;
    start_pos["y"] = start_y_pos;
    goal_pos["x"] = goal_x_pos;
    goal_pos["y"] = goal_y_pos;
    car_pos["x"] = start_x_pos;
    car_pos["y"] = start_y_pos;
}

void CarAgent::update(){
    
    dx = abs(goal_pos["x"] - car_pos["x"]);
    dy = abs(goal_pos["y"] - car_pos["y"]);
    dig = atan2(dy, dx);
    x_speed = speed * cos(dig);
    y_speed = speed * sin(dig);

    if(car_pos["x"]<goal_pos["x"] && car_pos["y"]<goal_pos["y"]){
        car_pos["x"] += x_speed;
        car_pos["y"] += y_speed;
    }
    else if(car_pos["x"]>goal_pos["x"] && car_pos["y"]<goal_pos["y"]){
        car_pos["x"] -= x_speed;
        car_pos["y"] += y_speed;
    }
    else if(car_pos["x"]<goal_pos["x"] && car_pos["y"]>goal_pos["y"]){
        car_pos["x"] += x_speed;
        car_pos["y"] -= y_speed;
    }
    else if(car_pos["x"]>goal_pos["x"] && car_pos["y"]>goal_pos["y"]){
        car_pos["x"] -= x_speed;
        car_pos["y"] -= y_speed;
    }
    
}

void CarAgent::collision_avoidance(){

}

class VW{
    public:
    

    void generate_vw(int vwNum);
};


void VW::generate_vw(int vwNum){
    random_device rd;
    std::mt19937 mt(rd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
    std::uniform_int_distribution<> rand100(0, 1);
    vector<int> vw(vwNum);

    for(int i=0;i<vwNum;i++){
        vw.at(i) = rand100(mt);
    }
}


int main(){
    CarAgent car1;
    CarAgent car2;
    CarAgent car3;
    CarAgent car4;

    car1.set_caragent(0, 500, 1000, 500);
    car2.set_caragent(500, 0, 500, 1000);
    car3.set_caragent(1000, 500, 0, 500);
    car4.set_caragent(500, 1000, 500, 0);
 
}