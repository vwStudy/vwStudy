###パラメーター####

##generation世代数,popuration遺伝子数
params = {'max_num_iteration': 10, 'population_size': 10} 

##canvasの座標は1px単位
##1px=約0.2mmとし、車の全長を約5m全幅を約2mとし、車は25px,10pxとする(1/1000スケール)

##VWの設置範囲 n*n
VWfield :int = 120

##VWの設置範囲の開始位置(左上座標)
VWfield_x :int = 380
VWfield_y :int = 200

##VWの数
VWnum :int = 4

##VWの大きさ
VWsize :int = VWfield/VWnum

##車のスピード
speed :int = 3

##車の縦幅
car_length :float = 25
 
##車の横幅
car_width :float = 20

##車の数
car_num :int = 4

#車のスタートの座標とゴールの座標
# car1_STARTtoGOAL : float = [[257.0,250.0],[642.0,250.0]]

# car2_STARTtoGOAL : float = [[450.0,147.0],[460.0,363.0]]##上車

# car3_STARTtoGOAL : float = [[642.0,250.0],[257.0,250.0]]

# car4_STARTtoGOAL : float = [[450.0,363.0],[450.0,147.0]]##下車

#ラウンドアバウト用
car1_STARTtoGOAL = ([270,260],[630,260])

car2_STARTtoGOAL = ([440,160],[440,370])

car3_STARTtoGOAL = ([630,350],[245,240])

car4_STARTtoGOAL = ([460,370],[460,115])

wall_edge = [[430,160],[270,160],[270,230],[470,160],[630,160],[630,230],[270,270],[270,350],[430,350],[630,270],[630,350],[470,350]]

wall_line = [[[270,160],[270,230]],[[270,160],[430,160]],[[630,160],[630,230]],[[630,160],[470,160]],[[270,350],[270,270]],[[270,350],[430,350]],[[630,350],[630,270]],[[630,350],[470,350]]]