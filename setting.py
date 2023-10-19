###パラメーター####

##generation世代数,popuration遺伝子数
params = {'max_num_iteration': 50, 'population_size': 50} 

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
car1_STARTtoGOAL = [[245,260],[630,260]]
#car1_STARTtoGOAL = [[245,260],[680,260]]

car2_STARTtoGOAL = [[460,115],[460,460]]##上車
#car2_STARTtoGOAL = [[440,115],[440,350]]


#car3_STARTtoGOAL = [[630,240],[245,240]]##右車
#car3_STARTtoGOAL = [[630,260],[0,240]]
car3_STARTtoGOAL = [[630,250],[245,250]]

car4_STARTtoGOAL = [[460,370],[460,115]]##下車
#car4_STARTtoGOAL = [[440,350],[440,115]]

