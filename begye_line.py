import numpy as np
import math
from matplotlib import pyplot as plt
 
# Bernstein多項式を計算する関数
def bernstein(n, t):
    B = []
    for k in range(n + 1):
        # 二項係数を計算してからBernstein多項式を計算
        nCk = math.factorial(n) / (math.factorial(k) * math.factorial(n - k))
        B.append(nCk * t ** k * (1 - t) ** (n - k))
        print(nCk, k, n-k)
    return B
 
# ベジェ曲線を描く関数
def bezie_curve(Q):
    n = len(Q) - 1
    dt = 0.01
    t = np.arange(0, 1 + dt, dt)
    B = bernstein(n, t)
    px = 0
    py = 0
    for i in range(len(Q)):
        px += np.dot(B[i], Q[i][0])
        py += np.dot(B[i], Q[i][1])
    return px, py
 
# 点座標を準備
q1 = [0, 0]
q2 = [1, 1]
q3 = [1, 0.5]
q4 = [0, 1]
Q = [q1, q2, q3, q4]
 
#q1 = [0., 0.]
#q2 = [0.5, 0.]
#q3 = [0.5, 1.]
#q4 = [1., 1.]
#Q = [q1, q2, q3, q4]
 
# ベジェ曲線を描く関数を実行
px, py = bezie_curve(Q)

print(px, py)
 
# ここからグラフ描画-------------------------------------
# フォントの種類とサイズを設定する。
plt.rcParams['font.size'] = 14
plt.rcParams['font.family'] = 'Times New Roman'
 
# 目盛を内側にする。
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
 
# グラフの上下左右に目盛線を付ける。
fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.yaxis.set_ticks_position('both')
ax1.xaxis.set_ticks_position('both')
 
# 軸のラベルを設定する。
ax1.set_xlabel('x')
ax1.set_ylabel('y')
 
# スケールの設定をする。
ax1.set_xlim(-0.1, 1.1)
ax1.set_ylim(-0.1, 1.1)
 
# ベジェ曲線をプロット
ax1.plot(px, py, color='red', label='Bezie curve')
 
# 制御点をプロット
qx = []
qy = []
for i in range(len(Q)):
    qx.append(Q[i][0])
    qy.append(Q[i][1])
ax1.plot(qx, qy, color='blue', marker='o', linestyle='--', label='Control point')
ax1.legend()
#ax1.axis('off')
 
# レイアウト設定
fig.tight_layout()
 
# グラフを表示する。
plt.show()
plt.close()