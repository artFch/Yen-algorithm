import heapq #кучи и очереди приоритета, работа с ними
import sys #для maxsize
import time

class Graph:
    def __init__(g):
        g.vertices = {}

    def add_v(g, name, edges):
        g.vertices[name] = edges

    def shortest_path(g, startv, endv):
        # изпользуется для хранени расстояния от начала до каждой вершины
        distances = {}

        # Предыдущая вершина лучшего пути из начала к текущей вершине
        # начало->1->2->3, тогда previous[3]=2,previous[2]=1
        previous = {}

        # очередь с приоритетом изпользуется чтобы записать дилну от всех вершин до начальной
        nodes = []

        # Алгоритм Дейкстры
        for vertex in g.vertices:
            if vertex == startv:
                # Инициализируем расстояние до начальной вершины нулем
                distances[vertex] = 0
                heapq.heappush(nodes, [0, vertex])
            elif vertex in g.vertices[startv]:
                distances[vertex] = g.vertices[startv][vertex]
                heapq.heappush(nodes, [g.vertices[startv][vertex], vertex])
                previous[vertex] = startv
            else:
                #Задать расстояние между вершиной  которая напрямую не связана с начальной точкой с sys.maxsize
                distances[vertex] = sys.maxsize
                heapq.heappush(nodes, [sys.maxsize, vertex])
                previous[vertex] = None

        while nodes:
            # Достаем вершину с наименьшим расстоянием в очереди
            mini = heapq.heappop(nodes)[1]#наименьший
            if mini == endv:
                sh_path = []
                lenPath = distances[mini]
                temp = mini
                while temp != startv:
                    sh_path.append(temp)
                    temp = previous[temp]
                # Добавить начало к списку кратчайшего пути
                sh_path.append(temp)
            if distances[mini] == sys.maxsize:#
                # Если расстояние до всех вершин максимально
                break
            # Пройти к вершине соединенной с наименьшей  обновить расстояние от вершины и до предыдущей вершины
            for neighbor in g.vertices[mini]:
                dis = distances[mini] + g.vertices[mini][neighbor]
                if dis < distances[neighbor]:
                    distances[neighbor] = dis
                    # Обновить предыдущую вершину от вершины соедененной с наименьшей
                    previous[neighbor] = mini
                    for node in nodes:
                        if node[1] == neighbor:
                            # Обновить расстояние от вершины соединенной с наименьшей до начальной точки
                            node[0] = dis
                            break
                    heapq.heapify(nodes)
            #print(nodes)
            #print(distances)
        return distances, sh_path, lenPath

    def min_dist_inc(g, inputL):
        inputL.sort()
        lenList = [v[0] for v in inputL]
        #print(lenList)
        minValue = min(lenList)
        minValue_index = lenList.index(minValue)
        minPath = [v[1] for v in inputL][minValue_index]
        return minValue, minPath, minValue_index

    def yen(g, start, finish, k=2):
        distances,_, shPathLen = g.shortest_path(start, finish) #нам ненужно занчение кратчайшего пути
        #print(distances)
        k_sh_path = 0
        paths = dict()
        distIncList = [[0, finish]]

        while k_sh_path < k:
            path = []
            minValue, minPath, minIndex = g.min_dist_inc(distIncList)
            #print('m:'+minPath)
            smallest_v = minPath[-3:]
            #print(minPath[-3:])
            #print('v:'+smallest_vertex)
            distIncList.pop(minIndex)

            if smallest_v == start:
                path.append(minPath[::-1])
                k_sh_path += 1
                paths[path[0]] = minValue + shPathLen
                #print(paths)
                #Cловарь использует ключ {path:pathLen}
                continue

            for neighbor in g.vertices[smallest_v]:
                incrementValue = minPath
                inc= 0
                if neighbor == finish:
                    continue
                if distances[smallest_v] == (distances[neighbor] + g.vertices[smallest_v][neighbor]):
                    inc = minValue
                elif distances[smallest_v] < (distances[neighbor] + g.vertices[smallest_v][neighbor]):
                    inc = minValue + distances[neighbor] + g.vertices[smallest_v][neighbor] - distances[
                        smallest_v]
                elif distances[neighbor] == (distances[smallest_v] + g.vertices[smallest_v][neighbor]):
                    inc = minValue + 2 * g.vertices[smallest_v][neighbor]
                distIncList.append([inc, incrementValue + neighbor])
        return paths


if __name__ == '__main__':
    start_time = time.time()
    g = Graph()  #Граф
    g.add_v('a00', {'a01': 2, 'a02': 6, 'a03': 8, 'a06': 3})#количество нолей перед первым рязрядом числа n-1 от разряда вашей последней вершины
    g.add_v('a01', {'a00': 2, 'a02': 9, 'a03': 3, 'a05': 4, 'a06': 9})
    g.add_v('a02', {'a00': 6, 'a01': 9, 'a03': 7})
    g.add_v('a03', {'a00': 8, 'a01': 3, 'a02': 7, 'a04': 5, 'a05': 5})
    g.add_v('a04', {'a03': 5, 'a06': 8, 'a07': 9})
    g.add_v('a05', {'a01': 4, 'a03': 5, 'a07': 6, 'a08': 4})
    g.add_v('a06', {'a00': 3, 'a01': 9, 'a04': 8})
    g.add_v('a07', {'a04': 9, 'a05': 6, 'a08': 1})
    g.add_v('a08', {'a05': 4, 'a07': 1})
    '''
    k = 7
    0.000083;8;
    0.0001;11;
    0.000115;20;
    0.000123;30
    0.000133;40
    0.000145;50
    0.000159;60
    0.000173;70
    0.000185;80
    0.0002;90;
    4,10 GHz
    '''


    start = 'a00' #начало
    end = 'a07' #конец
    k = 7  #Количество нужных нам кратчайших путей
    distances, shPath, shPathLen = g.shortest_path(start, end)
    print('Наикратчайший путь из {}->{}\n{}\nдлина наикратчайшего пути: {}'.format(start, end, shPath, shPathLen))

    paths = g.yen(start, end, k)
    #print(paths)
    if (k > 1):
        print('\nПервые {} кратчайшие пути из {} в {}'.format(k,start, end))
    else :
        print('\nКратчайший путь из {} в {} :'.format(start, end, k))
    index = 1
    #print(paths)
    for path, length in paths.items():#разворот словаря path для понятного восприятия пути
        i = 0
        j = 2
        realPath = ""
        while j <= len(path):#изменить шаг i и j при 3 разряде вершины или больше на шаг = n + 1 где n разряд
            temp = path[i:j + 1]
            # print(temp[::-1])
            realPath += temp[::-1]
            i += 3
            j += 3

        print('{} Путь - {}, длина пути: {}'.format(index, realPath, length))
        index += 1
    print("--- %s seconds ---" % (time.time() - start_time))
