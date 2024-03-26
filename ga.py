import numpy as np
import setting
import random
import change_ga_vw_bezier_line
import matplotlib.pyplot as plt
import csv
import time

class Individual:
    def __init__(self,genom, fitness):
        self.genom = genom
        self.fitness = fitness
        self.collision = 0
        self.all_path_length = 0
        self.set_fitness(self.fitness)
    
    def set_fitness(self, fitness):
        self.fitness, self.collision, self.all_path_length, self.total_num_obstacles, self.create_path_time_dic= fitness

    def get_fitness(self):
        return self.fitness
    
    def get_collision(self):
        return self.collision

    def get_all_path_length(self):
        return self.all_path_length
    
    def get_total_num_obstacles(self):
        return self.total_num_obstacles

    def get_cars_path(self):
        return self.cars_path
    
    def get_create_path_time_dic(self):
        return self.create_path_time_dic

def create_generation(popu_size, genoms, fitness):
    #popu_size:1世代の個体数
    #genoms:遺伝子数の長さ
    population = []
    for _ in range(popu_size):
        rnd = np.random.randint(0, 2, genoms)
        individual = Individual(rnd, fitness(rnd))
        population.append(individual)
    
    #print("population::", population[0].genom)#8個のインスタンスが入った1次元リストだけど、individualの中にvwnum*2の大きさのリストある
    return population

def create_generation_two(popu_size, genoms, two_steps_list, zeros_list, fitness):
    population = []
    for _ in range(popu_size):
        rnd = np.random.randint(0, 2, len(two_steps_list)*((setting.two_VWnum)**2))        
        individual = Individual(rnd, fitness(rnd, two_steps_list, zeros_list))
        population.append(individual)
    return population
    

# def select_roulette(populations):
#     function = [popu.get_fitness() for popu in populations]
#     probability = [popu.get_fitness() /sum(function) for popu in populations]
#     # probability = function /sum(function) 
#     selected1 = np.random.choice(populations, size=len(populations), p=probability)
#     print(selected1)
#     selected2 = np.random.choice(populations, size=len(populations), p=probability)
#     return selected1, selected2

# def select_roulette(population_list):
#     '''選択の関数(ルーレット方式)'''
#     selected_gene_list = []
    
#     weights = [ind.get_fitness() for ind in population_list]
#     norm_weights = [ind.get_fitness() / sum(weights) for ind in population_list]
#     selected_gene_list.append(np.random.choice(population_list, size=2, replace=False, p=norm_weights))
#     # print("selected_gene_list111", selected_gene_list[0][0])
#     # print("selected_gene_list222", selected_gene_list[0][1])    
#     return selected_gene_list#2次元リスト中身はインスタンス


# def select_roulette(population_list):
#     '''選択の関数(ルーレット方式)'''
#     selected_gene_list = []
#     for _ in range(2):
#         weights = [ind.get_fitness() for ind in population_list]
#         norm_weights = [ind.get_fitness() / sum(weights) for ind in population_list]
#         selected_gene_list.append(np.random.choice(population_list, size=(int(len(population_list)/2)), p=norm_weights))
#     print("selected_gene_list::",selected_gene_list)
#     return selected_gene_list

def cross_uniform(parent1_genom, parent2_genom):
    new_child1 = []
    new_child2 = []

    for par1, par2 in zip(parent1_genom, parent2_genom):
        if np.random.rand() >= setting.change_rate:
            new_child1.append(par1)
            new_child2.append(par2)

        else:
            new_child1.append(par2)
            new_child2.append(par1)


    new_child1 = np.array(new_child1)
    new_child2 = np.array(new_child2)
    children1 = Individual(new_child1, change_ga_vw_bezier_line.VW.single_GA_function(np.array(new_child1)))
    children2 = Individual(new_child2, change_ga_vw_bezier_line.VW.single_GA_function(np.array(new_child2)))
    return children1, children2

def cross_uniform_two_steps(parent1_genom, parent2_genom):
    new_child1 = []
    new_child2 = []
    for par1, par2 in zip(parent1_genom, parent2_genom):
        #print("par1::::", par1)
        #print("par2::::", par2)
        if np.random.rand() <= setting.change_rate:
            new_child2.append(par1)
            new_child1.append(par2)
        else:
            new_child1.append(par1)
            new_child2.append(par2)

    new_child1 = np.array(new_child1)
    new_child2 = np.array(new_child2) 
    children1 = Individual(new_child1, change_ga_vw_bezier_line.VW.single_GA_function(np.array([0,0,0,0,0,0,0,0,0])))
    children2 = Individual(new_child2, change_ga_vw_bezier_line.VW.single_GA_function(np.array([0,0,0,0,0,0,0,0,0])))
    return children1, children2

# def crossover(selected1, selected2, popu_size):
#     children = []
#     Cross_PB=0.5
#     child1 = selected1
#     child2 = selected2
#     if np.random.rand() < Cross_PB:
#         child1, child2 = cross_uniform(child1, child2, popu_size)
#     child1 = list(child1)
#     child2 = list(child2)
#     children.append(child1)
#     return children

# def crossover(selected_gene_list):
#     '''交叉の関数'''
#     # if setting.population_size % 2:
#     #     selected_gene_individual.append(selected_gene_individual[0])
#     children = []
#     parent1 = selected_gene_list[0][0]
#     parent2 = selected_gene_list[0][1]
#     # for parent1, parent2 in selected_gene_list[0][0], selected_gene_list[0][1]:
#     if np.random.rand() <= setting.crossover_rate:
#         children1, children2 = cross_uniform(parent1.genom, parent2.genom)
#     else:
#         children1, children2 = parent1, parent2
#     children.extend([children1,children2])
#     return children


def crossover(selected_gene_list,populations):
    '''交叉の関数'''
    # if setting.population_size % 2:
    #     selected_gene_individual.append(selected_gene_individual[0])

    children = []
    for i in range(len(populations)//2):
        cnt1=0
        cnt2=0
        parent1 = selected_gene_list[i][0]
        parent2 = selected_gene_list[i][1]
        # if np.random.rand() <= setting.crossover_rate:
        children1, children2 = cross_uniform(parent1.genom, parent2.genom)
        for i, genom in enumerate(parent1.genom):
            if children1.genom[i] == genom:
                cnt1+=1
        print("haming1", cnt1)
        for i, genom in enumerate(parent2.genom):
            if children1.genom[i] == genom:
                cnt2+=1
        print("haming2", cnt2)

        children.extend([children1,children2])
    
    #print("children::",children)
    return children

def crossover_two_steps(selected_gene_list,populations):
    '''交叉の関数'''
    # if setting.population_size % 2:
    #     selected_gene_individual.append(selected_gene_individual[0])

    children = []
    for i in range(len(populations)//2):
        cnt1=0
        cnt2=0
        parent1 = selected_gene_list[i][0]
        parent2 = selected_gene_list[i][1]
        # if np.random.rand() <= setting.crossover_rate:
        children1, children2 = cross_uniform_two_steps(parent1.genom, parent2.genom)
        for i, genom in enumerate(parent1.genom):
            if children1.genom[i] == genom:
                cnt1+=1
        print("haming1", cnt1)
        for i, genom in enumerate(parent2.genom):
            if children1.genom[i] == genom:
                cnt2+=1
        print("haming2", cnt2)

        children.extend([children1,children2])
    
    #print("children::",children)
    return children


def select_tonament(population_list):
    selected_gene_list = []
    select = []
    winner = []
    toname = len(population_list)-2
    # #[for socre in population_list.get_fitness()]
    # selected_gene_list.append(population_list[score_popu.index(min(score_popu))])
    # population_list.pop(score_popu.index(min(score_popu)))
    # score_popu = [popu_dict.update(score.get_id()=score.get_fitness()) for score in population_list]
    # popu = population_list
    # #for _ in range(len(population_list)//2):
    # while toname > 1:
    #     for _ in range(2):
    #         num = random.randint(0,len(popu))
    #         select.append(popu.pop(num))
        
    #     if select[0].get_fitness() >= select[1].get_fitness():
    #         winner.append(select[0])
    #     else:
    #         winner.append(select[1])
        
    #     select=[]
    #     toname -= 1



    # sorted_popu = sorted(population_list, key=lambda x : x.get_fitness())[:len(population_list)//2]

    # ###トーナメント
    # #上位50%のpopulationをとる
    # #とってきた中からランダムで二つとり、評価値で比較し小さい方を採用していって最終的に二つまで絞る
    for _ in range(len(population_list)//2):
        
        # sorted_popu = sorted(population_list, key=lambda x : x.get_fitness())[:len(population_list)//2]
        sorted_popu = sorted(population_list, key=lambda x : x.get_fitness())
        #print("sort",sorted_popu)

        toname_list = [[] for i in range(int(len(sorted_popu)/2))]
        random.shuffle(sorted_popu)
        # print("sort_shuffle",sorted_popu)
        for battle in toname_list:
            battle.append(sorted_popu.pop())
            battle.append(sorted_popu.pop())
        
        while len(toname_list) > 1:
            winner_list = []
            for i in range(int(len(toname_list)/2)):
                winner_list.append([])
                for j in range(2):
                    battle = toname_list.pop()
                    #print("batle",battle)
                    winner_list[i].append(min(battle, key=lambda x : x.get_fitness()))
                    
            toname_list = winner_list
            #print(toname_list)
        selected_gene_list.append(toname_list[0])
    
    #print("selectttt", selected_gene_list)
    #print(len(selected_gene_list))
    
    # '''選択の関数(ルーレット方式)'''
    # weights = [ind.get_fitness() for ind in population_list]
    # # #weights = [1/ind.ge for ind in population_list]
    # inverse_weights = [1/score for score in weights]
    # # # norm_weights = [ind / sum(inverse_weights) for ind in inverse_weights]
    # norm_weights = [ind / sum(inverse_weights) for ind in inverse_weights]
    # # # tmp_weights = sum(weights)
    # # # tmp_list = []
    # # # for ind in population_list:
    # # #     tmp_list.append(tmp_weights - ind.get_fitness())
    # # # norm_weights = [(ind / tmp_weights) for ind in tmp_list]
    # # # print("weightsss", weights)
    
    # selected_gene_list = np.random.choice(population_list, size=2, replace=False, p=norm_weights)
    # print(selected_gene_list)
    return selected_gene_list
        
def mutate(children):
    MUTATION_PB = 0.2
    for num_children in range(len(children)):
        if np.random.rand() < MUTATION_PB:# 一定の確率で突然変異させる
            
            random_number = np.random.randint(0, setting.genom_size)
            children[num_children].genom[random_number] = abs(children[num_children].genom[random_number] - 1)
    return children

def ga_solve(populations, gene_size):
    best = []
    generation_list = []
    for i in range(gene_size):
        
        best_popu = min(populations, key=Individual.get_fitness)
        # print("best_pouuu", best_popu.genom)
        best.append(best_popu)
        #print("all_path_length::",best_popu.get_all_path_length())
        #print("best::",best)
        generation_list.append(populations)
        selected = select_tonament(populations)
        
            # print("population1",selected[i][0].genom)
            # print("population2",selected[i][1].genom)
        
        children = crossover(selected, populations)
        
        # for i, genom in enumerate(selected[i][0].genom):
        #     if children1.genom[i] == genom:
        #         cnt1+=1
        # for i, genom in enumerate(parent2.genom):
        #     if children1.genom[i] == genom:
        #         cnt2+=1

        
        
        # print("population1a",selected[0].genom)
        # print("population2a",selected[1].genom)
            # print("children1::", children[0].genom)
            # print("children2::", children[1].genom)
        # print(" ")
        #print("len_children::",len(children))
        children = mutate(children)
        populations = children
        #print("new_popu", populations[0].genom)
        #print("len_populations::", len(populations))
        #print("len_populations::", len(populations))
        # populations[selected[1]] = children[1]
        #print("gengelation", i)
    # f=open("testttt.txt", "w", encoding="UTF-8")
    # f.writelines("vw4*4,gene:16,popu:16")
    # f.close()
    for i in range(len(best)):
        #print("best::",best[i].get_fitness())
        #print("best_vw::", best[i].genom)
        print("best_path::", best[i].get_all_path_length())

        # f = open("testttt.txt","a",encoding="UTF-8")
        # f.writelines('\n')
        # f.writelines(str(best[i].get_fitness()))
        
    # create_graph_best(best)

    # f.writelines('\n')
    # f.writelines('\n')
    # min(best, key=Individual.get_fitness)
    
    return best, best_popu, generation_list

def ga_solve_two_steps(populations, gene_size, two_steps_list = [], zeros_list = []):
    best = []
    generation_list = []
    for i in range(gene_size):
        
        best_popu = min(populations, key=Individual.get_fitness)
        #print("best_pouuu", best_popu.genom)
        best.append(best_popu)
        #print("all_path_length::",best_popu.get_all_path_length())
        #print("best::",best)
        generation_list.append(populations)
        selected = select_tonament(populations)
        children = crossover_two_steps(selected, populations)
        #print("len_children::",len(children))
        children = mutate(children)
        if two_steps_list and zeros_list:
            for i in range(len(children)):
                children[i].set_fitness(change_ga_vw_bezier_line.VW.two_steps_GA_function(children[i].genom, two_steps_list, zeros_list))
        populations = children
        # print("len_populations::", len(populations))
        #print("len_populations::", len(populations))
        # populations[selected[1]] = children[1]
        #print("gengelation", i)
    # f=open("testttt.txt", "w", encoding="UTF-8")
    # f.writelines("vw4*4,gene:16,popu:16")
    # f.close()
    for i in range(len(best)):
        # print("best::",best[i].get_fitness())
        # print("best_vw::", best[i].genom)
        print("best_path::", best[i].get_all_path_length())

        # f = open("testttt.txt","a",encoding="UTF-8")
        # f.writelines('\n')
        # f.writelines(str(best[i].get_fitness()))
        
    # create_graph_best(best)

    # f.writelines('\n')
    # f.writelines('\n')
    # min(best, key=Individual.get_fitness)
    return best, best_popu, generation_list

def main(popu_size, gene_size, genom_size):
    fitness = change_ga_vw_bezier_line.VW.single_GA_function
    
    populations = create_generation(popu_size, genom_size, fitness)
    # print("populations", populations)インスタンスが入ってる1次元リスト
    return ga_solve(populations, gene_size)

def main_two_steps(popu_size, gene_size, genom_size):
    fitness = change_ga_vw_bezier_line.VW.single_GA_function
    
    populations = create_generation(popu_size, genom_size, fitness)
    # print("populations", populations)インスタンスが入ってる1次元リスト
    best, best_popu, generation_list =  ga_solve(populations, gene_size//2)
    best_gene = min(best, key=Individual.get_fitness)
    writer.writerow(["best_one_steps::",best_gene.genom])
    zeros_list = [[[0] * (setting.two_VWnum)**2] * 9]

    two_steps_list = []
    for index, cell in enumerate(best_gene.genom):
        if cell == 1:
            two_steps_list.append(index)

    genom_size = len(two_steps_list) * ((setting.two_VWnum) ** 2)
        
    populations = create_generation_two(popu_size, genom_size, two_steps_list, zeros_list, change_ga_vw_bezier_line.VW.two_steps_GA_function)
    
    return ga_solve_two_steps(populations, gene_size//2, two_steps_list, zeros_list)
    

#def set_paramater_ga(popu_size,gene_size,genom_size, two_steps_list, zeros_list):
    popu_size = popu_size#1世代の遺伝子数
    gene_size = gene_size#世代数
    genom_size = genom_size#遺伝子の長さ
    two_steps_list = two_steps_list
    zeros_list = zeros_list
    populations = create_generation_two(popu_size, genom_size, two_steps_list, zeros_list)
    return ga_solve(populations, gene_size, popu_size)

def create_graph_best(best):
    l = []
    for gene in best:
        l.append(gene.get_all_path_length())

    left  = [x for x in range(len(best))]
    height = l
    plt.plot(left, height)
    plt.show()

def create_graph_best_all_path_length(best):
    l = []
    for gene in best:
        l.append(gene.get_all_path_length())

    left  = [x for x in range(len(best))]
    height = l
    plt.plot(left, height)
    
    plt.savefig("./fig/" + str(setting.VWnum) + "x" + str(setting.VWnum) + "_pop" + str(setting.population_size) + "_gen" + str(setting.generation_size) + ".png")
    
    plt.show()

def create_graph_generations(genelation_list, genelation_number):
    l = []
    for gene in genelation_list[genelation_number]:
        l.append(gene.get_all_path_length())

    left  = [x for x in range(len(genelation_list[genelation_number]))]
    height = l
    plt.plot(left, height)
    plt.show()

def create_graph(x_list):
    left  = [x for x in range(len(x_list))]
    height = x_list
    plt.plot(left, height)
    plt.show()

if __name__ == '__main__':
    # populist=setting.population_size #8
    # generation = setting.generation_size #8
    # genom_size= setting.genom_size
    # with open('./data_folder/individual_twostep2.csv', 'a', encoding='utf-8', newline='') as f:
    #     writer = csv.writer(f)
    #     for _ in range(4):
    #         for _ in range(4):
    #             for i in range(10):
    #                 writer.writerow(["popu_size", populist])
    #                 writer.writerow(["gene_size", generation])
    #                 start = time.time()
    #                 best=main_two_steps(populist, generation , genom_size)
    #                 end = time.time()
    #                 diff = end - start
    #                 writer.writerow(["time:", diff])
    #                 for i in range(len(best)):
    #                     writer.writerow(["best_length:", best[i].get_all_path_length()])
                
    #             writer.writerow([""])
    #             generation *= 2
    #         populist *= 2
    #         generation = setting.generation_size
    populist=setting.population_size #8
    generation = setting.generation_size #8
    genom_size= setting.genom_size
    with open('./data_folder/comp.csv', 'a', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        for _ in range(4):
            for _ in range(4):
                for _ in range(5):
                    writer.writerow([])
                    writer.writerow(["popu_size", populist])
                    writer.writerow(["gene_size", generation])
                    start = time.time()
                    best, best_popu, generation_list =main_two_steps(populist, generation , genom_size)
                    # best, best_popu, generation_list =main(populist, generation , genom_size)
                    end = time.time()
                    diff = end - start
                    writer.writerow(["time:", diff])
                    for i in range(len(best)):
                        writer.writerow(["best_length:", best[i].get_all_path_length()])
                        writer.writerow(["best_evo:", best[i].get_fitness()])
                        writer.writerow(["best_collision:", best[i].get_collision()])
                        writer.writerow(["best_genom:", best[i].genom])
                        # writer.writerow(["best_cars_path:", best_popu.get_cars_path()])
                        writer.writerow(["best_create_path_time:",best[i].get_create_path_time_dic()["create_path_time"]])
                generation*=2
                writer.writerow([""])

            populist*=2
            generation=setting.population_size