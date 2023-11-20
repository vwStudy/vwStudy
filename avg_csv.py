import csv


def read_csv():
    with open('./data_folder/individual_twostep.csv') as f:
        contents = f.readlines()
        with open('./data_folder/individual_twostep_avg.csv', 'a', encoding='utf-8', newline='') as e:
            writer = csv.writer(e)
            gene_size = 8
            popu_size = 8
            path_length_list = [0] * gene_size
            sum_time = 0
            sum_path_length = 0

            for content in contents:
                if 'time' in content:
                    # 分割
                    time = content.partition(',')
                    # 後ろの部分が欲しい場合は[2]
                    sum_time += float(time[2])
                    #print(sum_time)

                elif 'best' in content:
                    # 分割
                    path_length = content.partition(',')
                    # 後ろの部分が欲しい場合は[2]
                    #print(path_length)
                    path_length_list.append(float(path_length[2]))

                elif 'popu' in content:
                    if 1<float(path_length_list[gene_size//2-1]):
                        sum_path_length += float(path_length_list[gene_size//2-1])
                        print(path_length_list)
                    
                    path_length_list = []

                # if all(cell.strip() == ' ' for cell in content):
                #     #print(path_length_list)
                #     sum_path_length += float(path_length_list[gene_size-1])
                #     path_length_list = []

                elif '""' in content:
                    sum_path_length += float(path_length_list[gene_size//2-1])
                    avg_path_length = sum_path_length/10
                    avg_time = sum_time/10
                    writer.writerow(["gene_size:",gene_size])
                    writer.writerow(["avg_path:",avg_path_length])
                    writer.writerow(["avg_time:",avg_time])
                    
                    gene_size  *= 2
                    sum_path_length = 0
                    sum_time = 0
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