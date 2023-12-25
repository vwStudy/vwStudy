import numpy as np

# サンプリング対象のリスト
my_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# 各要素の選ばれる確率を設定（例: 各要素が選ばれる確率が等しい場合）
probabilities = [1/len(my_list)] * len(my_list)

# 重複なしでランダムに2つの要素を選ぶ
random_elements = np.random.choice(my_list, size=2, replace=False, p=probabilities)

print("ランダムに選ばれた要素:", random_elements)
