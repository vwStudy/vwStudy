import random
import sys
import math
import time
import setting
from roadgene_test2 import Obstacle
import csv

GEN_MAX = setting.generation_size   # 世代交代数
POP_SIZE = setting.population_size         # 個体群のサイズ
ELITE = 1               # エリート保存戦略で残す個体の数
MUTATE_PROB = 0.1      # 突然変異確率
N = 5                  # 集合の要素となる最大数の平方値
AREA = 1600
# TOURNAMENT_SIZE = 30  # トーナメントサイズ

RAND_MAX = 100000000        # 乱数の最大値
FLT_MAX = sys.float_info.max #double型のシステムの最大値

class Individual:
    def __init__(self):
        self.chrom = [0] * N    # 染色体
        self.fitness = 0.0      # 適応度

        for i in range(N):
            self.chrom[i] = random.randint(0, RAND_MAX) % AREA + 1

    def evaluate(self): # 適応度を算出する
        self.fitness = Obstacle.single_GA_function(self.chrom)

    # p1とp2から一点交叉で作った子にする
    # p1: 親個体1
    # p2: 親個体2
    def crossover(self, p1, p2): # 交叉による子にする
        point = random.randint(0, RAND_MAX) % (N - 1)
        for i in range(point+1):
            if not p1.chrom[i] in self.chrom:
                self.chrom[i] = p1.chrom[i]
        for i in range(point+1, N):
            if not p2.chrom[i] in self.chrom:
                self.chrom[i] = p2.chrom[i]

    # p1とp2から二点交叉で作った子にする
    # p1: 親個体1
    # p2: 親個体2
    # def crossover(self, p1, p2):
    #     point1 = random.randint(0, RAND_MAX) % (N - 1)
    #     point2 = (point1 + (random.randint(0, RAND_MAX) % (N - 2) + 1)) % (N - 1)
    #     if point1 > point2:
    #         tmp = point1
    #         point1 = point2
    #         point2 = tmp

    #     for i in range(point1+1):
    #         self.chrom[i] = p1.chrom[i]

    #     for i in range(point1+1, point2+1):
    #         self.chrom[i] = p2.chrom[i]

    #     for i in range(point2+1, N):
    #         self.chrom[i] = p1.chrom[i]


    def mutate(self): # 突然変異を起こす
        for i in range(N):
            if random.uniform(0, 1) < MUTATE_PROB:
                r = self.chrom[i]
                while not r in self.chrom:
                    r = random.randint(0, RAND_MAX) % AREA + 1
                self.chrom[i] = r

class Population:
    def __init__(self):
        self.ind = [None] * POP_SIZE         # 現世代の個体群のメンバ
        self.next_ind = [None] * POP_SIZE    # 次世代の個体群のメンバ
        # self.tr_fit = [None] * POP_SIZE    # 適応度を変換した値

        for i in range(POP_SIZE):
            self.ind[i] = Individual()
            self.next_ind[i] = Individual()

        # self.denom = None                       # ルーレット選択の確率を求めるときの分母
        self.evaluate()

    def evaluate(self): # 個体を評価する
        for i in range(POP_SIZE):
            self.ind[i].evaluate()

        self.sort(0, POP_SIZE - 1)
        

    # ind[lb]～ind[ub]をクイックソートで並び替える
    # lb: 並び替えの対象要素の添え字の下限
    # ub: 並び替えの対象要素の添え字の上限
    def sort(self, lb, ub): # 個体を良い順に並び替える
        if lb < ub:
            k = int((lb + ub) / 2)
            pivot = self.ind[k].fitness
            i = lb
            j = ub
            while i <= j:
                while self.ind[i].fitness < pivot:
                    i += 1

                while self.ind[j].fitness > pivot:
                    j -= 1

                if i <= j:
                    tmp = self.ind[i]
                    self.ind[i] = self.ind[j]
                    self.ind[j] = tmp
                    i += 1
                    j -= 1
            self.sort(lb, j)
            self.sort(i, ub)

    def alternate(self): # 世代交代をする

        # ルーレット選択のための処理
        # self.denom = 0.0
        # for i in range(POP_SIZE):
        #     self.tr_fit[i] = (self.ind[POP_SIZE - 1].fitness - self.ind[i].fitness)/(self.ind[POP_SIZE - 1].fitness - self.ind[0].fitness)
        #     self.denom += self.tr_fit[i]


        # エリート保存戦略で子個体を作る
        for i in range(ELITE):
            for j in range(N):
                self.next_ind[i].chrom[j] = self.ind[i].chrom[j]

        # 親を選択し交叉する
        for i in range(ELITE, POP_SIZE):
            p1 = self.select()
            p2 = self.select()
            self.next_ind[i].crossover(self.ind[p1], self.ind[p2])

        # 突然変異を起こす
        for i in range(1, POP_SIZE):
            self.next_ind[i].mutate()

        # 次世代を現世代に変更する
        tmp = self.ind
        self.ind = self.next_ind
        self.next_ind = tmp

        # 評価する
        self.evaluate();

    # 順位に基づくランキング選択で親個体を1つ選択する
    # 戻り値: 選択した親個体の添え字
    def select(self):
        denom = POP_SIZE * (POP_SIZE + 1) / 2
        r = ((random.randint(0, RAND_MAX) << 16) + (random.randint(0, RAND_MAX) << 1) + (random.randint(0, RAND_MAX) % 2)) % denom + 1

        num = POP_SIZE
        while num > 0:
            if r <= num:
                break
            r -= num
            num -= 1

        return POP_SIZE - num

    # 確率に基づくランキング選択で親個体を1つ選択する
    # 戻り値: 選択した親個体の添え字
    # def select(self):
    #     denom = POP_SIZE * (POP_SIZE + 1) / 2
    #     r = random.uniform(0, 1)
    #     for rank in range(1, POP_SIZE):
    #         prob = (POP_SIZE - rank + 1) / denom
    #         if r <= prob:
    #             break
    #         r -= prob
    #     return rank - 1

    # ルーレット選択で親個体を1つ選択する
    # 戻り値: 選択した親個体の添え字
    # def select(self):
    #     r = random.uniform(0, 1)
    #     for rank in range(1, POP_SIZE):
    #         prob = self.tr_fit[rank - 1] / self.denom
    #         if r <= prob:
    #             break
    #         r -= prob
    #     return rank - 1

    # トーナメント選択で親個体を1つ選択する
    # 戻り値: 選択した親個体の添え字
    # def select(self):
    #     tmp = [0] * N
    #     ret = -1
    #     best_fit =FLT_MAX
    #     num = 0
    #     while True:
    #         r = random.randint(0, RAND_MAX) % N
    #         if tmp[r] == 0:
    #             tmp[r] = 1
    #             if self.ind[r].fitness < best_fit:
    #                 ret = r
    #                 best_fit = self.ind[r].fitness
    #             num += 1
    #             if num == TOURNAMENT_SIZE:
    #                 break
    #     return ret

    def print_result(self): # 結果を表示する
        print(self.ind[0].chrom)
        print(f"\n差：{self.ind[0].fitness}\n")

if __name__ == "__main__":
    with open('40×40_start1gaol1_testpopu256×gene64.csv', 'w') as f:
        writer = csv.writer(f)
        start = time.time()

        pop = Population()
        for i in range(GEN_MAX):
            pop.alternate()
            print(f"第{i+1}世代：最良適応度{pop.ind[0].fitness}")

        pop.print_result()
        print(time.time() - start)
        fitness, colision, distances= Obstacle.single_GA_function(pop.ind[0].chrom)
        writer.writerow(["fitness", fitness])
        writer.writerow(["colision",colision])
        writer.writerow(["distances",sum(distances)])
        writer.writerow(["genom",pop.ind[0].chrom])
