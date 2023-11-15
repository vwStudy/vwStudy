import numpy as np
import setting
import change_ga_vw
import matplotlib.pyplot as plt

class Individual:
    def __init__(self, genom, fitness = change_ga_vw.VW.single_GA_function):
        self.genom = genom
        self.fitness = fitness
        self.collision = 0
        self.all_path_length = 0
        self.set_fitness(self.fitness)
    
    def set_fitness(self, fitness):
        self.fitness, self.collision, self.all_path_length, self.total_num_obstacles, self.cars_path = fitness

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

def create_generation(popu_size, genoms, fitness = change_ga_vw.VW.single_GA_function):
    #popu_size:1世代の個体数
    #genoms:遺伝子数の長さ
    population = []
    for _ in range(popu_size):
        individual = Individual(np.random.randint(0, 2, genoms), fitness(np.random.randint(0, 2, genoms)))
        population.append(individual)
    print("population::", population)
    return population

def create_generation_two(popu_size, genoms, two_steps_list, zeros_list, fitness = change_ga_vw.VW.single_GA_function):
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

def select_roulette(population_list):
    '''選択の関数(ルーレット方式)'''
    selected_gene_list = []
    for _ in range(2):
        weights = [ind.get_fitness() for ind in population_list]
        norm_weights = [ind.get_fitness() / sum(weights) for ind in population_list]
        selected_gene_list.append(np.random.choice(population_list, size=len(population_list), p=norm_weights))
    return selected_gene_list

def cross_uniform(parent1_genom, parent2_genom):
    new_child1 = []
    new_child2 = []
    for par1, par2 in zip(parent1_genom, parent2_genom):
        if np.random.rand() <= setting.change_rate:
            new_child2.append(par1)
            new_child1.append(par2)
        else:
            new_child1.append(par1)
            new_child2.append(par2)
    new_child1 = np.array(new_child1)
    new_child2 = np.array(new_child2) 
    children1 = Individual(new_child1, change_ga_vw.VW.single_GA_function(new_child1))
    children2 = Individual(new_child2, change_ga_vw.VW.single_GA_function(new_child2))
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

def crossover(selected_gene_list):
    '''交叉の関数'''
    # if setting.population_size % 2:
    #     selected_gene_individual.append(selected_gene_individual[0])
    children = []
    for parent1, parent2 in zip(selected_gene_list[0], selected_gene_list[1]):
        if np.random.rand() <= setting.crossover_rate:
            children1, children2 = cross_uniform(parent1.genom, parent2.genom)
        else:
            children1, children2 = parent1, parent2
        children.extend([children1,children2])
    return children
        
def mutate(children):
    MUTATION_PB = 0.1
    for child in children:
        # 一定の確率で突然変異させる
        if np.random.rand() < MUTATION_PB:
            children = []
            for i in range(setting.genom_size):
                child.genom[i] = np.random.randint(0,1)
            mutate_child = Individual(child.genom, change_ga_vw.VW.single_GA_function(child.genom))
            children.append(mutate_child)

    return children

def ga_solve(populations, gene_size, popu_size):
    best = []
    generation_list = []
    tmp = []
    for i in range(gene_size):
        best_popu = min(populations, key=Individual.get_fitness)
        best.append(best_popu.get_all_path_length())
        print("all_path_length::",best_popu.get_all_path_length())
        print("best::",best)
        generation_list.append(populations)
        selected = select_roulette(populations)
        children = crossover(selected)
        children = mutate(children)
        print("len_populations::", len(populations))
        populations = children
        # print("len_populations::", len(populations))
        # populations[selected[1]] = children[1]
    print("best::",best)

    best_gene = 0
    # min(best, key=Individual.get_fitness)
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
        l.append(gene.get_all_path_length())

    left  = [x for x in range(len(best))]
    height = l
    plt.plot(left, height)
    plt.show()

def create_graph_best_fitness(best):
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
    main()
    