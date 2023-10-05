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
    generation = []
    for _ in range(popu_size):
        individual = Individual(np.random.randint(0, 2, genoms))
        generation.append(individual)
    return generation

def select_roulette(population):
    selected = []
    function = [popu.get_fitness for popu in population]
    probability = [popu.get_fitness /sum(function) for popu in population] 
    selected = np.random.choice(population, size=len(population), p=probability)
    return selected


def cross_two_point_copy(child1, child2):
    '''交叉の関数(二点交叉)
        input: 混ぜ合わせたい個体のペア
        output: 交叉後の個体のペア'''
    size = len(child1.genom)
    tmp1 = child1.genom.copy()
    tmp2 = child2.genom.copy()
    cxpoint1 = np.random.randint(1, size)
    cxpoint2 = np.random.randint(1, size - 1)
    if cxpoint2 >= cxpoint1:
        cxpoint2 += 1
    else:
        cxpoint1, cxpoint2 = cxpoint2, cxpoint1
    tmp1[cxpoint1:cxpoint2], tmp2[cxpoint1:cxpoint2] = tmp2[cxpoint1:cxpoint2].copy(), tmp1[cxpoint1:cxpoint2].copy()
    new_child1 = Individual(tmp1)
    new_child2 = Individual(tmp2)
    return new_child1, new_child2


def uniform_crossover(selected,popu_size):
    children = []
    CROSSOVER_PB = 0.8
    if popu_size % 2:
        selected.append(selected[0])
        for child1, child2 in zip(selected[::2], selected[1::2]):
            if np.random.rand() < CROSSOVER_PB:
                child1, child2 = cross_two_point_copy(child1, child2)
                children.append(child1)
                children.append(child2)
            children = children[:popu_size]
            return children
        
def mutate(children):
    '''突然変異の関数'''
    MUTATION_PB = 0.1
    for child in children:
        # 一定の確率で突然変異させる
        if np.random.rand() < MUTATION_PB:
            child.mutate()
    return children

def ga_solve(generation, gene_size):
    best = []
    for _ in range(gene_size):
        best_popu = min(generation, key=Individual.get_fitness)
        best.append(best_popu)
        selected = select_roulette(generation)
        children = uniform_crossover(selected)
        generation = mutate(children)
    return best


def main():
    popu_size = setting.population_size
    gene_size = setting.generation_size
    genom_size = setting.genom_size

    generation = create_generation(popu_size, genom_size)
    return ga_solve(generation, gene_size)

if __name__ == '__main__':
    main()