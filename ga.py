import numpy as np
import random
import setting
import objectVW_test

class Individual:
    def __init__(self,genom, fitness):
        self.genom = genom
        self.fitness = fitness
        self.collision = 0
        self.all_path_length = 0
        self.set_fitness(self.fitness)
    
    def set_fitness(self, fitness):
        self.fitness, self.collision, self.all_path_length = fitness

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
    children1 = Individual(new_child1, objectVW_test.Obstacle.single_GA_function(np.array(new_child1)))
    children2 = Individual(new_child2, objectVW_test.Obstacle.single_GA_function(np.array(new_child2)))
    return children1, children2


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
        #print("haming1", cnt1)
        for i, genom in enumerate(parent2.genom):
            if children1.genom[i] == genom:
                cnt2+=1
       # print("haming2", cnt2)

        children.extend([children1,children2])
    
    #print("children::",children)
    return children

# def crossover_two_steps(selected_gene_list,populations):
    '''交叉の関数'''
    # if setting.population_size 

def select_tonament(population_list):
    selected_gene_list = []
    select = []
    winner = []
    toname = len(population_list)-2
   
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
    
    return selected_gene_list
        
def mutate(children):
    MUTATION_PB = 0.2
    for num_children in range(len(children)):
        if np.random.rand() < MUTATION_PB:# 一定の確率で突然変異させる
            
            random_number = np.random.randint(0, setting.genom_size)
            # if children[num_children].genom != []:
            if children[num_children].genom.size > 0:
                children[num_children].genom[random_number] = abs(children[num_children].genom[random_number] - 1)
    return children

def ga_solve(populations, gene_size):
    best = []
    generation_list = []
    for i in range(gene_size):
        
        best_popu = min(populations, key=Individual.get_fitness)
        best.append(best_popu)
      
        generation_list.append(populations)
        selected = select_tonament(populations)
        
            
        children = crossover(selected, populations)
        
        children = mutate(children)
        populations = children
    
    
    # for i in range(len(best)):
    #     #print("best_path::", best[i].get_all_path_length())
    #     print("")
    return best, best_popu, generation_list


def create_generation(popu_size, genoms, fitness):
    #popu_size:1世代の個体数
    #genoms:遺伝子数の長さ
    population = []
    for _ in range(popu_size):
        rnd = np.random.randint(0, 2, genoms)
        #rnd = np.zeros(genoms)
        #print(type(rnd))
        individual = Individual(rnd, fitness(rnd))
        population.append(individual)
    
    #print("population::", population[0].genom)#8個のインスタンスが入った1次元リストだけど、individualの中にvwnum*2の大きさのリストある
    return population


def main(popu_size, gene_size, genom_size):
    fitness = objectVW_test.Obstacle.single_GA_function
    populations = create_generation(popu_size, genom_size, fitness)
    return ga_solve(populations, gene_size)


# if __name__ == '__main__':
#車10台,10台で遺伝子数64,世代数32同時にゴール2つ良さげな結果
populist=setting.population_size 
generation = setting.generation_size 
genom_size= setting.genom_size
best, best_popu, generation_list = main(populist, generation , genom_size)
#print("best",np.array([obstacle.position for obstacle in best]))
print("best_popu",best_popu.genom)
#objectVW_test.Obstacle.single_GA_function(best_popu.genom)
min_best = min(best, key=Individual.get_fitness)
objectVW_test.Obstacle.single_GA_function(min_best.genom)
print("min_best",min_best.genom)
#np.save('obstacles.npy', np.array([obstacle for obstacle in obs_list]))
# np.save('trajectory.npy', np.array(self.trajectory))
# np.save('obstacles.npy', np.array(best_popu.genom))
    # np.save('obstacles.npy', np.array([obstacle.position for obstacle in obs_list]))
    # np.save('end_positions.npy', np.array([car.end_position for car in self.cars]))

# for _ in range(4):
#     for _ in range(4):
        
#         best, best_popu, generation_list = main(populist, generation , genom_size)

#     generation*=2
# populist*=2
# generation=setting.population_size
print("Completed")
