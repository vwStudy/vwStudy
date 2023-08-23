###パラメーター####

##generation世代数,popuration遺伝子数
params = {'max_num_iteration': 30, 'population_size': 30} 

##canvasの座標は1px単位
##1px=約0.2mmとし、車の全長を約5m全幅を約2mとし、車は25px,10pxとする(1/1000スケール)

##VWの設置範囲 n*n
VWfield :int = 120

##VWの設置範囲の開始位置(左上座標)
VWfield_x :int = 380
VWfield_y :int = 190

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

##車の数
car_num :int = 4

##車のスタートの座標とゴールの座標
#car1_STARTtoGOAL = [[245,240],[630,240]]##左車
# car1_STARTtoGOAL = [[245,260],[630,260]]
#car1_STARTtoGOAL = [[245,260],[680,260]]
car1_STARTtoGOAL : float = [[257.0, 250.0],[642.0, 250.0]]

car2_STARTtoGOAL : float = [[450.0, 147.0],[450.0, 363.0]]
#car2_STARTtoGOAL = [[460,115],[460,460]]##上車
#car2_STARTtoGOAL = [[440,115],[440,350]]

#car3_STARTtoGOAL = [[630,240],[245,240]]##右車
#car3_STARTtoGOAL = [[630,260],[0,240]]
car3_STARTtoGOAL : float = [[642.0, 250.0],[257.0, 250.0]]

car4_STARTtoGOAL : float = [[450.0, 363.0],[450.0, 147.0]]
#car4_STARTtoGOAL = [[460,370],[460,115]]##下車
#car4_STARTtoGOAL = [[440,350],[440,115]]

wall_edge = [[430,160],[270,160],[270,230],[470,160],[630,160],[630,230],[270,270],[270,350],[430,350],[630,270],[630,350],[470,350]]
wall_line = []
