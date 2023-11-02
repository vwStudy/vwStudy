import numpy as np
import setting
import change_ga_vw
import matplotlib.pyplot as plt

class Individual:
    def __init__(self, genom, fitness):
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

def create_generation(popu_size, genoms, fitness = change_ga_vw.VW.single_GA_function):
    #popu_size:1世代の個体数
    #genoms:遺伝子数の長さ
    population = []
    for _ in range(popu_size):
        individual = Individual(np.random.randint(0, 2, genoms), fitness(np.random.randint(0, 2, genoms)))
        population.append(individual)
    return population

def create_generation_two(popu_size, genoms, two_steps_list, zeros_list, fitness = change_ga_vw.VW.two_steps_ga_function):
    population = []
    for _ in range(popu_size):
        individual = Individual(np.random.randint(0, 2, genoms), fitness(np.random.randint(0, 2, genoms), two_steps_list, zeros_list))
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

def select_roulette(generation):
    '''選択の関数(ルーレット方式)'''
    selected = []
    weights = [ind.get_fitness() for ind in generation]
    norm_weights = [ind.get_fitness() / sum(weights) for ind in generation]
    selected = np.random.choice(generation, size=len(generation), p=norm_weights)
    return selected

def cross_uniform(child1, child2):
    mask = [np.random.randint(0,1) for _ in range(setting.population_size)]
    for i in range(setting.population_size):
        if mask[i]==0:
            new_child1 = child1
            new_child2 = child1
        elif mask[i]==1:
            new_child1 = child2
            new_child2 = child2
    return new_child1, new_child2

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

def crossover(selected):
    '''交叉の関数'''
    children = []
    if setting.population_size % 2:
        selected.append(selected[0])
    for child1, child2 in zip(selected[::2], selected[1::2]):
        if np.random.rand() < 0.8:
            child1, child2 = cross_uniform(child1, child2)
        children.append(child1)
        children.append(child2)
    children = children[:setting.population_size]
    return children
        
def mutate(children):
    MUTATION_PB = 0.1
    for child in children:
        # 一定の確率で突然変異させる
        if np.random.rand() < MUTATION_PB:
            child = np.random.randint(0,1)
    children.append(child)
    return children

def ga_solve(populations, gene_size, popu_size):
    best = []
    generation_list = []
    for i in range(gene_size):
        best_popu = min(populations, key=Individual.get_fitness)
        best.append(best_popu)
        generation_list.append(populations)
        selected = select_roulette(populations)
        children = crossover(selected)
        children = mutate(children)
        populations[0] = children[0]
        populations[1] = children[1]
    best_gene = min(best, key=Individual.get_fitness)
    return best, best_gene, generation_list

def main():
    popu_size = setting.population_size#1世代の遺伝子数
    gene_size = setting.generation_size#世代数
    genom_size = setting.genom_size#遺伝子の長さ
    populations = create_generation(popu_size, genom_size)
    return ga_solve(populations, gene_size, popu_size)

def set_paramater_ga(popu_size,gene_size,genom_size, two_steps_list, zeros_list):
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
        l.append(gene.get_fitness())

    left  = [x for x in range(len(best))]
    height = l
    plt.plot(left, height)
    plt.show()

def create_graph_generations(genelation_list, genelation_number):
    l = []
    for gene in genelation_list[genelation_number]:
        l.append(gene.get_fitness())

    left  = [x for x in range(len(genelation_list))]
    height = l
    plt.plot(left, height)
    plt.show()

def skip_generation():
    """
    世代を修了させる関数
    """

if __name__ == '__main__':
    main()
    