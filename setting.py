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
VWnum :int = 9

##VWの大きさ
VWsize :int = VWfield/VWnum

##車のスピード
speed :float = 1

##車の縦幅
car_length :float = 25
 
##車の横幅
car_width :float = 20

#車の数
car_num :int = 1
# car_num :int = 8

population_size = 80
generation_size = 80
genom_size = ((VWnum**2) * car_num)

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


#ラウンドアバウト用
# car1_STARTtoGOAL = ([270,240],[630,240])

# car2_STARTtoGOAL = ([460,160],[460,370])

# car3_STARTtoGOAL = ([630,260],[270,260])

# car4_STARTtoGOAL = ([440,370],[440,160])

# wall_edge_list = [[430,160],[270,160],[270,230],[470,160],[630,160],[630,230],[270,270],[270,350],[430,350],[630,270],[630,350],[470,350]]

# wall_line_list = [[[270,160],[270,230]],[[270,160],[430,160]],[[630,160],[630,230]],[[630,160],[470,160]],[[270,350],[270,270]],[[270,350],[430,350]],[[630,350],[630,270]],[[630,350],[470,350]]]

#実世界の道路基準(1/1000スケール)
#3.5m * 3車線の広さ(102px)

#壁のイメージ
#-------2 車 2-------
#-----1       1------
#
#車                車
#
#-----1       1------
#-------2 車 2-------

wall_edge_list = [[349,199],[399,149] #左上の壁の角1,2
                 ,[551,199],[501,149] #右上の壁の角1,2
                 ,[349,301],[399,351] #左下の壁の角1,2
                 ,[551,301],[501,351]]#右下の壁の角1,2

wall_line_list = [[[0,199],[399,199]],[[399,0],[399,199]] #左上の壁の辺
                 ,[[501,199],[900,199]],[[501,0],[501,199]] #右上の壁の辺
                 ,[[0,301],[399,301]],[[399,500],[399,301]] #左下の壁の辺
                 ,[[501,301],[900,301]],[[501,500],[501,301]]] #右下の壁の辺

#表示用壁の左上座標
wall = [[0,0],[0,0], #左上の壁
        [501,0],[551,0], #右上の壁
        [0,301],[0,351], #左下の壁
        [551,301],[501,351]] #右下の壁

#左上座標

##インターチェンジの壁設定
# wall_edge_list = [[]]

# wall_line_list = [[]]

##車両のスタート位置(ジャンクション比較用)
# car1_STARTtoGOAL : float = [[0,224.5],[424.5,0]] #左から上へ
# car2_STARTtoGOAL : float = [[450,0],[0,250]] #上から左へ
# car3_STARTtoGOAL : float = [[900,250],[450,500]] #右から下へ
# car4_STARTtoGOAL : float = [[424.5,500],[900,224.5]] #下から右へ

##車両のスタート位置
car1_STARTtoGOAL : float = [[324,250],[576,250]] #左から右へ
car2_STARTtoGOAL : float = [[450,124],[450,376]] #上から下へ
car3_STARTtoGOAL : float = [[576,250],[324,250]] #右から左へ
car4_STARTtoGOAL : float = [[450,376],[450,124]] #下から上へ



# car1_STARTtoGOAL : float = [[324,224.5],[576,224.5]] #左から右へ
# car2_STARTtoGOAL : float = [[450,124],[450,376]] #上から下へ
# car3_STARTtoGOAL : float = [[576,250],[324,250]] #右から左へ
# car4_STARTtoGOAL : float = [[424.5,376],[424.5,124]] #下から上へ

# car5_STARTtoGOAL : float = [[324,250],[576,250]] #左から右へ
# car6_STARTtoGOAL : float = [[424.5,124],[424.5,376]] #上から下へ
# car7_STARTtoGOAL : float = [[576,224.5],[324,224.5]] #右から左へ
# car8_STARTtoGOAL : float = [[450,376],[450,124]] #下から上へ