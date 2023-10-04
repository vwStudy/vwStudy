from tkinter import *
import time

def main():
    tk = Tk()
    tk.title(u"vw_result")
    tk.geometry("1000x500")
    canvas= Canvas(tk,width=900,height=500,bg="white", highlightthickness=0)
    canvas.pack()

    car=canvas.create_rectangle(0,0, 25, 10,fill='red')##左車
    canvas.moveto(car, 200, 200)
    dx = 10
    dy = 10

    #動かすためのパラメータ
    x=150#四角の初期x座標
    y=150#四角の初期y座標
    Vy=2#どのくらいの速度で動かすか
    Y_current=300-dy#どこまで動かすか（基本はcanvasの端から端）
    t=1000
    while True:
        canvas.coords(car,x,y,x+dx,y+dy) #squareを指定の座標に移動させる
        y+=Vy #縦方向に少しづつ動かす
        if y<=0: #上端に到達したら
           Vy=abs(Vy) #下向きに動かす
        elif y>=Y_current: #下端に到達したら
            Vy=-abs(Vy) #上向きに動かす
        time.sleep(0.02) #0.02秒ずつ更新
        tk.update() #ウインド画面を更新
        t-=1
        #目標終了時間に達したらループから抜ける
        if t==0:
            break
    tk.mainloop()

if __name__ == '__main__':
    main()