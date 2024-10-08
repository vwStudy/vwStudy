
###パラメーター####



##generation世代数,popuration遺伝子数
# params = {'max_num_iteration': 20, 'population_size': 20}
# generation_size = 30 #世代数
# population_size = 30 #遺伝子数

##canvasの座標は1px単位
##1px=約0.2mmとし、車の全長を約5m全幅を約2mとし、車は25px,10pxとする(1/1000スケール)
##車線の幅を3.5mだと考え，幅17pxとする


##車のスピード
speed :float = 8

car_angle :int = 16

#car_num :int = 4

##車の縦幅
car_length :float = 8
 
##車の横幅
car_width :float = 20

#車の数
car_num :int = 1
# car_num :int = 8

##VWの数
VWnum :int = 10

genom_size = VWnum*(VWnum)

population_size = 128
generation_size = 256

crossover_rate = 1.0 #クロスオーバー発生率
change_rate = 0.5 #ユニフォームクロスオーバーの際の確率

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

# wall_edge_list = [[349,199],[399,149] #左上の壁の角1,2
#                  ,[551,199],[501,149] #右上の壁の角1,2
#                  ,[349,301],[399,351] #左下の壁の角1,2
#                  ,[551,301],[501,351]]#右下の壁の角1,2

# wall_line_list = [[[0,199],[399,199]],[[399,0],[399,199]] #左上の壁の辺
#                  ,[[501,199],[900,199]],[[501,0],[501,199]] #右上の壁の辺
#                  ,[[0,301],[399,301]],[[399,500],[399,301]] #左下の壁の辺
#                  ,[[501,301],[900,301]],[[501,500],[501,301]]] #右下の壁の辺

#実世界の道路基準(1/1000スケール)
#3.5m * 4車線の広さ(136px)

#壁のイメージ
#------- 車 --------
#------1    1-------
#
#車                車
#
#------1     1-------
#------- 車  --------
wall_edge_list = [[399,182], #左上の壁の角1,2
                 [501,182], #右上の壁の角1,2
                 [399,284], #左下の壁の角1,2
                 [501,284]] #右下の壁の角1,2

wall_line_list = [[[0,0],[0,182]],[[0,0],[399,0]],[[399,0],[399,182]],[[399,182],[0,182]] #左上の壁の辺
                  ,[[501,0],[501,182]],[[501,0],[900,0]],[[900,0],[900,182]],[[900,182],[0,182]] #右上の壁の辺
                #   ,[[0,318],[399,318]],[[399,500],[399,318]] #左下の壁の辺
                  ,[[501,284],[501,500]],[[501,284],[900,284]]
                  ,[[900,318],[900,500]],[[900,500],[318,500]]] #右下の壁の辺

#全直進
car1_STARTtoGOAL : float = [[324,250],[576,250]] #左から右へ
car2_STARTtoGOAL : float = [[450,124],[450,376]] #上から下へ
car3_STARTtoGOAL : float = [[576,260],[324,260]] #右から左へ
car4_STARTtoGOAL : float = [[460,376],[460,124]] #下から上へ

