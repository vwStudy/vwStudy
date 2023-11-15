import random

###パラメーター####

##generation世代数,popuration遺伝子数

params = {'max_num_iteration': 20, 'population_size': 20} 

##canvasの座標は1px単位
##1px=約0.2mmとし、車の全長を約5m全幅を約2mとし、車は25px,10pxとする(1/1000スケール)
##車線の幅を3.5mだと考え，幅17pxとする

##VWの設置範囲 n*n
# VWfield :int = 102
VWfield :int = 140

##VWの設置範囲の開始位置(左上座標)
#VWfield_x :int = 399
# VWfield_y :int = 199
VWfield_x :int = 380
VWfield_y :int = 180

##VWの数
VWnum :int = 5

##VWの大きさ
VWsize :int = VWfield/VWnum

##車のスピード
speed :float = 1

##車の縦幅
car_length :float = 25
 
##車の横幅
car_width :float = 20

##車の数
car_num :int = 4

##車のスタートの座標とゴールの座標
# car1_STARTtoGOAL : float = [[257.0, 250.0],[642.0, 250.0]]##左車

# car2_STARTtoGOAL : float = [[450.0, 147.0],[450.0, 363.0]]##上車 

# car3_STARTtoGOAL : float = [[642.0, 250.0],[257.0, 250.0]]##右車

# car4_STARTtoGOAL : float = [[450.0, 363.0],[450.0, 147.0]]##下車

#8台
# car1_STARTtoGOAL : float = [[270.0,260.0],[630.0,260.0]]

# car2_STARTtoGOAL : float = [[440.0,160.0],[440.0,370.0]]##上車

# car3_STARTtoGOAL : float = [[630.0,240.0],[270.0,240.0]]

# car4_STARTtoGOAL : float = [[460.0,370.0],[460.0,160.0]]##下車

# car5_STARTtoGOAL : float = [[270.0, 240.0],[630.0,240.0]]

# car6_STARTtoGOAL : float = [[460.0,160.0],[460.0,370.0]]

# car7_STARTtoGOAL : float = [[630.0,260.0],[270.0,260.0]]

# car8_STARTtoGOAL : float = [[440.0,160.0],[440,370.0]]


#car3_STARTtoGOAL = [[630,240],[245,240]]##右車
#car3_STARTtoGOAL = [[630,260],[0,240]]
car3_STARTtoGOAL = [[630,250],[245,250]]

car4_STARTtoGOAL = [[460,370],[460,115]]##下車
#car4_STARTtoGOAL = [[440,350],[440,115]]
