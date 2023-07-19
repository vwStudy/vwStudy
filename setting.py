###パラメーター####

##generation世代数,popuration遺伝子数
params = {'max_num_iteration': 2, 'population_size': 10} 

##VWの設置範囲 n*n
VWfield :int = 300

##VWの設置範囲の開始位置(左上座標)
VWfield_x :int = 400
VWfield_y :int = 300

##VWの数
VWnum :int = 5 

##VWの大きさ
VWsize :int = VWfield/VWnum

##車のスピード
speed :int = 3

##車の横幅
car_width :float = 1.0