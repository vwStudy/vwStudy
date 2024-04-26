import csv


def read_csv():
    with open('./data_folder/3_3_4_1.csv') as f:
        contents = f.readlines()
        with open('./data_folder/3_3_4_1.csv', 'a', encoding='utf-8', newline='') as e:
            writer = csv.writer(e)
            gene_size = 8
            popu_size = 8
            path_length_list = [0] * gene_size
            sum_time = 0
            sum_path_length = 0
            sum_collision = 0
            sum_evo = 0
            sum_obstacle_num = 0
            sum_create_path_time = 0

            for content in contents:
                if 'time' in content:
                    # 分割
                    time = content.partition(',')
                    # 後ろの部分が欲しい場合は[2]
                    sum_time += float(time[2])
                    #print(sum_time)

                elif 'best_length' in content:
                    # 分割
                    path_length = content.partition(',')
                    # 後ろの部分が欲しい場合は[2]
                    #print(path_length)
                    path_length_list.append(float(path_length[2]))
                    # sum_path_length += float(path_length[2])

                elif 'best_collision' in content:
                    # 分割
                    collision = content.partition(',')
                    # 後ろの部分が欲しい場合は[2]
                    collision_list.append(float(collision[2]))
                    sum_collision += int(collision[2])
                
                elif 'best_evo' in content:
                    # 分割
                    best_evo = content.partition(',')
                    # 後ろの部分が欲しい場合は[2]
                    best_evo_list.append(float(best_evo[2]))
                    sum_evo += float(best_evo[2])
                
                elif 'best_ge' in content:
                    best_genom = content.partition(',')
                    for i in range(len(best_genom)):
                        if best_genom[2][i] == "1":
                            sum_obstacle_num += 1
                
                elif 'best_create' in content:
                    # 分割
                    best_create_path_time = content.partition(',')
                    print(best_create_path_time)
                    # 後ろの部分が欲しい場合は[2]
                    #print(path_length)
                    best_create_path_time_list.append(float(best_create_path_time[2]))
                    sum_create_path_time += float(best_create_path_time[2])
                    
                elif 'popu' in content:
                    if 1<float(path_length_list[gene_size//2-1]):
                        sum_path_length += float(path_length_list[gene_size//2-1])
                        # print(path_length_list)
                    
                    path_length_list = []
                    collision_list = []
                    total_num_obstacle = []
                    best_create_path_time_list = []
                    best_evo_list = []


                # if all(cell.strip() == ' ' for cell in content):
                #     #print(path_length_list)
                #     sum_path_length += float(path_length_list[gene_size-1])
                #     path_length_list = []

                elif '""' in content:
                    sum_path_length += float(path_length_list[gene_size//2-1])
                    avg_path_length = sum_path_length/5
                    avg_time = sum_time/5
                    avg_create_path_time = sum_create_path_time/5
                    avg_total_num_obstacle = (sum_obstacle_num)/5
                    avg_collision = (sum_collision) /5

                    writer.writerow(["gene_size:",gene_size])
                    writer.writerow(["avg_path:",avg_path_length])
                    writer.writerow(["avg_time:",avg_time])
                    writer.writerow(["avg_create_path_time:",avg_create_path_time])
                    writer.writerow(["avg_total_num_obstacle:",avg_total_num_obstacle])
                    writer.writerow(["avg_collision:",avg_collision])
                    
                    gene_size  *= 2
                    sum_path_length = 0
                    sum_time = 0
                    sum_obstacle_num = 0
                    sum_collision = 0
                    path_length_list = [0] * gene_size
                    if gene_size == 128:
                        gene_size = 8
                    #print('---------')

                # else:
                #     if content[0] == 't':
                #         # 分割
                #         time = content.partition(',')
                #         # 後ろの部分が欲しい場合は[2]
                #         sum_time += float(time[2])
                #         #print(sum_time)

                #     elif content[0] == 'b':
                #         # 分割
                #         path_length = content.partition(',')
                #         # 後ろの部分が欲しい場合は[2]
                #         print(path_length)
                #         path_length_list.append(float(path_length[2]))

def main():
    read_csv()
    # if "popu_size" == popu:
    #     print(popu)



if __name__ == '__main__':
    main()