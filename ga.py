import numpy as np
import setting
import change_ga_vw

class Individual:
    def __init__(self, genom):
        self.genom = genom
        self.fitness = self.fitness = change_ga_vw.VW.GA_function(self.genom)
        self.set_fitness()
    
    def set_fitness(self):
        self.fitness = change_ga_vw.VW.GA_function(self.genom)
    
    def get_fitness(self):
        return self.fitness

def create_generation(popu_size, genoms):
    #popu_size:1世代の個体数
    #genoms:遺伝子数の長さ
    population = []
    for _ in range(popu_size):
        individual = Individual(np.random.randint(0, 2, genoms))
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
    for i in range(gene_size):
        best_popu = min(populations, key=Individual.get_fitness)
        best.append(best_popu)
        selected = select_roulette(populations)
        children = crossover(selected)
        children = mutate(children)
        populations[0] = children[0]
        populations[1] = children[1]
    return best

def main():
    popu_size = setting.population_size#1世代の遺伝子数
    gene_size = setting.generation_size#世代数
    genom_size = setting.genom_size#遺伝子の長さ
    populations = create_generation(popu_size, genom_size)
    return ga_solve(populations, gene_size, popu_size)

if __name__ == '__main__':
    main()