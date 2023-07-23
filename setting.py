###パラメーター####

##generation世代数,popuration遺伝子数
params = {'max_num_iteration': 2, 'population_size': 10} 

##VWの設置範囲 n*n
VWfield :int = 300

##VWの設置範囲の開始位置(左上座標)
VWfield_x :int = 400
VWfield_y :int = 300

##VWの数
VWnum :int = 4

##VWの大きさ
VWsize :int = VWfield/VWnum

##車のスピード
speed :int = 3

##車の縦幅
car_length :float = 2.0
 
##車の横幅
car_width :float = 1.0

##車のスタートの座標とゴールの座標
car1_start = [0,240]
car1_goal =  [860,240]

car2_start = [440,0]
car2_goal =  [440,460]

car3_start = [860,240]
car3_goal =  [0,240]

car4_start = [440,460]
car4_goal =  [440,0]