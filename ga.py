import numpy as np
import setting
import vw

class Individual:
    def __init__(self, genom):
        self.genom = genom
        self.fitness = 0
        self.set_fitness()
    
    def set_fitness(self):
        self.fitness = vw.VW.GA_function()
    
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

def select_roulette(populations):
    function = [popu.get_fitness for popu in populations]
    #probability = [popu.get_fitness /sum(function) for popu in populations]
    probability = [function /sum(function)] 
    selected1 = np.random.choice(populations, size=len(populations), p=probability)
    selected2 = np.random.choice(populations, size=len(populations), p=probability)
    return selected1, selected2


def cross_uniform(child1, child2, popu_size):
    mask = [np.random.randint(0,1) for _ in range(popu_size)]
    for i in range(popu_size):
        if mask[i]==0:
            new_child1 = child1
            new_child2 = child1
        elif mask[i]==1:
            new_child1 = child2
            new_child2 = child2
    return new_child1, new_child2


def crossover(selected1, selected2, popu_size):
    children = []
    Cross_PB=0.5
    child1 = selected1
    child2 = selected2
    if np.random.rand() < Cross_PB:
        new_child1, new_child2 = cross_uniform(child1, child2, popu_size)
    children.extend(new_child1,new_child2)
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
    for _ in range(gene_size):
        best_popu = min(populations, key=Individual.get_fitness)
        best.append(best_popu)
        selected1, selected2 = select_roulette(populations)
        children = crossover(selected1,selected2, popu_size)
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