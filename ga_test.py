import random
import setting
import change_ga_vw

# 適合度を評価する関数
def evaluate_fitness(solution):
    all_path_length = change_ga_vw.VW.single_GA_function(solution)
    return all_path_length
    # ここで適合度を計算する評価関数を定義する
    #     # solution: 個体（染色体）の表現



# ルーレット選択
def roulette_selection(population, fitness_scores):
    total_fitness = sum(fitness_scores)
    normalized_fitness = [score / total_fitness for score in fitness_scores]
    cumulative_prob = [sum(normalized_fitness[:i+1]) for i in range(len(normalized_fitness))]

    selected = []
    for _ in range(len(population)):
        r = random.random()
        for i, individual in enumerate(population):
            if r <= cumulative_prob[i]:
                selected.append(individual)
                break
    return selected

# 一様交叉
def uniform_crossover(parent1, parent2):
    child1, child2 = [], []
    for i in range(len(parent1)):
        if random.random() < 0.5:
            child1.append(parent1[i])
            child2.append(parent2[i])
        else:
            child1.append(parent2[i])
            child2.append(parent1[i])
    return child1, child2

# 突然変異
def mutate(solution, mutation_rate):
    mutated_solution = []
    for gene in solution:
        if random.random() < mutation_rate:
            # 突然変異を実行する処理を記述する
            # ここでは例として、ランダムに別の値に変更する例を示します
            mutated_gene = random.randint(0, 1)  # 0か1のどちらかに変更
            mutated_solution.append(mutated_gene)
        else:
            mutated_solution.append(gene)
    return mutated_solution

# 遺伝的アルゴリズムのメイン関数
def genetic_algorithm(population_size, chromosome_length, generations):
    # 初期集団の生成
    populations = [[random.randint(0, 1) for _ in range(chromosome_length)] for _ in range(population_size)]

    for gen in range(generations):
        # 適合度の評価
        fitness_scores = [evaluate_fitness(individual) for individual in populations]

        # 選択
        selected_parents = roulette_selection(populations, fitness_scores)

        # 交叉
        children = []
        for i in range(0, len(selected_parents), 2):
            parent1, parent2 = selected_parents[i], selected_parents[i + 1]
            child1, child2 = uniform_crossover(parent1, parent2)
            children.extend([child1, child2])

        # 突然変異
        for i in range(len(children)):
            children[i] = mutate(children[i], mutation_rate=0.1)

        # 次世代の形成
        populations = children

    # 最良の解などの結果を返す（必要に応じて）

# 遺伝的アルゴリズムの実行
genetic_algorithm(population_size=8, chromosome_length=setting.genom_size, generations=4)
