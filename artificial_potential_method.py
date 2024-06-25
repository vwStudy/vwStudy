import numpy as np
import math
import matplotlib.pyplot as plt


# ポテンシャル関数の計算
def cal_pot(x, y, obst_target_x, obst_target_y, goal_x, goal_y):
  tmp_pot = 0
  potential_max = 1
  potential_min = -1
  # weight_obst = 0.3
  # weight_goal = 8
  weight_obst = 0.3
  weight_goal = 8

  # 障害物がないとき(Noneがはいっている)
  if obst_target_x == None or obst_target_y == None:
    obst_pot = 0

  # 障害物の座標のpotentialはmax
  elif obst_target_x == x and obst_target_y == y:
    obst_pot = potential_max
  else:
    obst_pot =  1 / math.sqrt(pow((x - obst_target_x), 2) + pow((y - obst_target_y), 2))
    obst_pot += obst_pot * weight_obst

  tmp_pot += obst_pot

  # ゴールの座標はpotentialはmin
  if goal_x == x and goal_y == y:
    goal_pot = potential_min
  else:
    goal_pot = -1 / math.sqrt(pow((x - goal_x),  2) + pow((y - goal_y),  2))

  pot_all = tmp_pot + weight_goal * goal_pot

  return pot_all


#ルートをdfに代入
def cal_route(pos, goal, obs):
    delt=0.1
    speed=1.0
    x = pos[0]
    y = pos[1]
    goal_x = goal[0]
    goal_y = goal[1]
    obst_target_x = obs.position[0]
    obst_target_y = obs.position[1]
    #ポテンシャル場を偏微分して，xとy合成
    vx = -(cal_pot(x + delt, y, obst_target_x, obst_target_y, goal_x, goal_y) - cal_pot(x, y, obst_target_x, obst_target_y, goal_x, goal_y)) / delt
    vy = -(cal_pot(x, y+delt, obst_target_x, obst_target_y, goal_x, goal_y) - cal_pot(x, y, obst_target_x, obst_target_y, goal_x, goal_y)) / delt

    v = math.sqrt(vx * vx + vy * vy)

    # 正規化
    vx /= v / speed
    vy /= v / speed

    # # 進める
    # x += vx
    # y += vy
    
    return vx,vy

def car_cal_route(pos, goal, obs):
    delt=0.1
    speed=1.0
    x = pos[0]
    y = pos[1]
    goal_x = goal[0]
    goal_y = goal[1]
    obst_target_x = obs.position[0]
    obst_target_y = obs.position[1]
    #ポテンシャル場を偏微分して，xとy合成
    vx = -(car_cal_pot(x + delt, y, obst_target_x, obst_target_y, goal_x, goal_y) - car_cal_pot(x, y, obst_target_x, obst_target_y, goal_x, goal_y)) / delt
    vy = -(car_cal_pot(x, y+delt, obst_target_x, obst_target_y, goal_x, goal_y) - car_cal_pot(x, y, obst_target_x, obst_target_y, goal_x, goal_y)) / delt

    v = math.sqrt(vx * vx + vy * vy)

    # 正規化
    vx /= v / speed
    vy /= v / speed

    # # 進める
    # x += vx
    # y += vy
    
    return vx,vy

def car_cal_pot(x, y, obst_target_x, obst_target_y, goal_x, goal_y):
  tmp_pot = 0
  potential_max = 1
  potential_min = -1
  weight_obst = 0.4
  weight_goal = 8

  # 障害物がないとき(Noneがはいっている)
  if obst_target_x == None or obst_target_y == None:
    obst_pot = 0

  # 障害物の座標のpotentialはmax
  elif obst_target_x == x and obst_target_y == y:
    obst_pot = potential_max
  else:
    obst_pot =  1 / math.sqrt(pow((x - obst_target_x), 2) + pow((y - obst_target_y), 2))
    obst_pot += obst_pot * weight_obst

  tmp_pot += obst_pot

  # ゴールの座標はpotentialはmin
  if goal_x == x and goal_y == y:
    goal_pot = potential_min
  else:
    goal_pot = -1 / math.sqrt(pow((x - goal_x),  2) + pow((y - goal_y),  2))

  pot_all = tmp_pot + weight_goal * goal_pot

  return pot_all