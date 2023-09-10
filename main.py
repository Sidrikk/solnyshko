import heapq

import networkx as nx
import matplotlib.pyplot as plt
import math

# Функция для вычисления расстояния между двумя точками на сфере по координатам
def haversine(coord1, coord2):
    lon1, lat1 = coord1
    lon2, lat2 = coord2
    # Радиус Земли в километрах
    R = 6371.0
    # Разница координат
    dlon = math.radians(lon2 - lon1)
    dlat = math.radians(lat2 - lat1)
    # Формула гаверсинуса
    a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    # Расстояние в километрах
    distance = R * c
    # Округляем до сотых долей километра
    return round(distance, 2)

# Создаем граф маршрута
G = nx.Graph()

# Добавляем вершины
for i in range(1, 12):
    G.add_node(i)



# Добавляем рёбра на основе предоставленных координат, толщины льда и даты

edges = [
    (1, 2, {"ice_thickness": {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 10: 0,
                              11: 0, 12: 20, 13: 18, 14: 0, 15: 0, 16: 1, 17: 1, 18: 1, 19: 1, 20: 1,
                              21: 1, 22: 1, 23: 1, 24: 1, 25: 1, 26: 1, 27: 1, 28: 1, 29: 1, 30: 2}}),

    (2, 3, {"ice_thickness": {1: 0, 2: 1, 3: 1, 4: 1, 5: 1, 6: 2, 7: 2, 8: 2, 9: 2, 10: 2,
                              11: 2, 12: 2, 13: 2, 14: 3, 15: 3, 16: 3, 17: 3, 18: 3, 19: 3, 20: 3,
                              21: 4, 22: 4, 23: 4, 24: 4, 25: 4, 26: 4, 27: 4, 28: 5, 29: 5, 30: 5}}),

    (3, 4, {"ice_thickness": {1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 2, 7: 2, 8: 2, 9: 2, 10: 2,
                              11: 3, 12: 3, 13: 3, 14: 3, 15: 3, 16: 3, 17: 3, 18: 4, 19: 4, 20: 4,
                              21: 4, 22: 4, 23: 5, 24: 5, 25: 5, 26: 5, 27: 6, 28: 6, 29: 6, 30: 6}}),

    (4, 5, {"ice_thickness": {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 10: 0,
                              11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 1, 17: 1, 18: 1, 19: 1, 20: 1,
                              21: 1, 22: 1, 23: 1, 24: 1, 25: 1, 26: 1, 27: 1, 28: 1, 29: 1, 30: 2}}),

    (5, 6, {"ice_thickness": {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 10: 0,
                              11: 0, 12: 0, 13: 0, 14: 0, 15: 1, 16: 1, 17: 1, 18: 1, 19: 1, 20: 1,
                              21: 1, 22: 1, 23: 1, 24: 1, 25: 1, 26: 1, 27: 1, 28: 1, 29: 2, 30: 2}}),

    (6, 1, {"ice_thickness": {1: 0, 2: 0, 3: 0, 4: 1, 5: 1, 6: 1, 7: 1, 8: 1, 9: 1, 10: 1,
                              11: 1, 12: 1, 13: 2, 14: 2, 15: 2, 16: 2, 17: 2, 18: 3, 19: 3, 20: 3,
                              21: 3, 22: 3, 23: 4, 24: 4, 25: 4, 26: 4, 27: 4, 28: 5, 29: 5, 30: 6}}),

    (4, 7, {"ice_thickness": {1: 1, 2: 1, 3: 1, 4: 2, 5: 2, 6: 2, 7: 2, 8: 2, 9: 2, 10: 3,
                              11: 3, 12: 3, 13: 3, 14: 3, 15: 3, 16: 4, 17: 4, 18: 4, 19: 4, 20: 4,
                              21: 5, 22: 5, 23: 5, 24: 5, 25: 6, 26: 6, 27: 6, 28: 6, 29: 6, 30: 7}}),

    (7, 8, {"ice_thickness": {1: 1, 2: 1, 3: 2, 4: 2, 5: 2, 6: 2, 7: 2, 8: 2, 9: 3, 10: 3,
                              11: 3, 12: 3, 13: 3, 14: 3, 15: 4, 16: 4, 17: 4, 18: 4, 19: 4, 20: 5,
                              21: 5, 22: 5, 23: 5, 24: 6, 25: 6, 26: 6, 27: 6, 28: 6, 29: 7, 30: 7}}),

    (8, 9, {"ice_thickness": {1: 2, 2: 2, 3: 2, 4: 2, 5: 3, 6: 3, 7: 3, 8: 3, 9: 4, 10: 4,
                              11: 4, 12: 4, 13: 4, 14: 5, 15: 5, 16: 5, 17: 5, 18: 5, 19: 6, 20: 6,
                              21: 6, 22: 6, 23: 7, 24: 7, 25: 7, 26: 7, 27: 7, 28: 8, 29: 8, 30: 8}}),

    (9, 10, {"ice_thickness": {1: 2, 2: 2, 3: 2, 4: 3, 5: 3, 6: 3, 7: 3, 8: 4, 9: 4, 10: 4,
                               11: 4, 12: 4, 13: 5, 14: 5, 15: 5, 16: 5, 17: 5, 18: 6, 19: 6, 20: 6,
                               21: 6, 22: 7, 23: 7, 24: 7, 25: 7, 26: 7, 27: 7, 28: 8, 29: 8, 30: 8}}),

    (10, 11, {"ice_thickness": {1: 3, 2: 3, 3: 3, 4: 3, 5: 3, 6: 3, 7: 3, 8: 3, 9: 4, 10: 4,
                                11: 4, 12: 5, 13: 5, 14: 5, 15: 5, 16: 5, 17: 6, 18: 6, 19: 6, 20: 6,
                                21: 7, 22: 7, 23: 7, 24: 7, 25: 7, 26: 7, 27: 7, 28: 7, 29: 7, 30: 7}}),

    (10, 12, {"ice_thickness": {1: 3, 2: 3, 3: 3, 4: 3, 5: 3, 6: 3, 7: 3, 8: 3, 9: 4, 10: 4,
                                11: 4, 12: 5, 13: 5, 14: 5, 15: 5, 16: 5, 17: 6, 18: 6, 19: 6, 20: 6,
                                21: 7, 22: 7, 23: 7, 24: 7, 25: 7, 26: 7, 27: 7, 28: 7, 29: 7, 30: 7}}),

    (12, 13, {"ice_thickness": {1: 3, 2: 3, 3: 3, 4: 3, 5: 3, 6: 3, 7: 3, 8: 3, 9: 4, 10: 4,
                                11: 4, 12: 5, 13: 5, 14: 5, 15: 5, 16: 5, 17: 6, 18: 6, 19: 6, 20: 6,
                                21: 7, 22: 7, 23: 7, 24: 7, 25: 7, 26: 7, 27: 7, 28: 7, 29: 7, 30: 7}}),

    (12, 14, {"ice_thickness": {1: 3, 2: 3, 3: 3, 4: 3, 5: 3, 6: 3, 7: 3, 8: 3, 9: 4, 10: 4,
                                11: 4, 12: 4, 13: 4, 14: 4, 15: 4, 16: 5, 17: 5, 18: 5, 19: 5, 20: 5,
                                21: 5, 22: 6, 23: 6, 24: 6, 25: 6, 26: 7, 27: 7, 28: 7, 29: 8, 30: 9}}),

    # Другие рёбра...

]


# Координаты вершин
coordinates = {
    1: (46.142578125, 69.93030017617484),
    2: (57.74414062500001, 70.4367988185464),
    3: (66.20361328125, 73.53462847039683),
    4: (71.54296874999999,  73.77577986189993),
    5: (68.90625, 77.44694030325893),
    6: (55.01953125, 76.2059670431415),
    7: (73.125, 72.76406472320436),
    8: (74.02587890625, 72.63337363853837),
    9: (72.92724609375, 72.0739114882038),
    10: (72.59765625, 71.30783606806223),
    11: (72.2021484375, 71.24435551310674),
    12: (73.32275390625, 70.9722375547307),
    13: (73.7127685546875, 71.03303495416577),
    14: (73.52874755859375, 68.66455067163206)
}

# Создаем словарь, в котором указаны доступные для каждого ледокола отрезки маршрутов
icebreaker_routes = {
    "Icebreaker_A": [(1, 2, 3), (4, 5, 6)],
    "Icebreaker_B": [(2, 3), (5, 6, 1)],
    # Другие ледоколы и их маршруты...
}

# Добавляем рёбра и вычисляем расстояние в км
for edge in edges:
    node1, node2, edge_attrs = edge
    ice_thickness_data = edge_attrs.get('ice_thickness', {})

    coord1, coord2 = coordinates[node1], coordinates[node2]
    distance_km = haversine(coord1, coord2)

    G.add_edge(node1, node2, ice_thickness=ice_thickness_data, distance=distance_km)

# Функция для определения возможности движения судна по ребру в определенную дату
def can_ship_move(edge, date, ice_thickness, ship_class):
    # Проверяем толщину льда в заданную дату
    ice_thickness = edge.get('ice_thickness', {}).get(date, None)

    if ice_thickness is not None:
        # В этой дате есть информация о толщине льда на ребре
        if allowed_thickness >= ice_thickness:
            return "Self-navigation"  # Судно может двигаться самостоятельно
        else:
            return "Escort"  # Судно может двигаться только под проводкой ледокола
    else:
        return "Forbidden"  # Движение судна запрещено

# Пример определения возможности движения судна по ребру в заданную дату
ship_class = "Icebreaker_A"  # Ледокол, который будет провожать судно
date = 21  # Заданная дата
start_node = 1
end_node = 2

edge_attrs = G[start_node][end_node]
allowed_thickness = 5

movement_status = can_ship_move(edge_attrs, date, allowed_thickness, ship_class)


# Простейший алгоритм поиска пути: обход в ширину (BFS)
def find_shortest_path(graph, start, end):
    queue = [[start]]
    visited = set()

    while queue:
        path = queue.pop(0)
        node = path[-1]

        if node == end:
            return path

        if node not in visited:
            for neighbor in graph[node]:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)

            visited.add(node)

    return None

# Поиск кратчайшего маршрута
start_node = 1
end_node = 4
shortest_path_nodes = find_shortest_path(G, start_node, end_node)

if shortest_path_nodes:
    # Вывод кратчайшего маршрута и расстояния
    shortest_path_edges = [(shortest_path_nodes[i], shortest_path_nodes[i+1]) for i in range(len(shortest_path_nodes) - 1)]
    total_distance = sum(G[edge[0]][edge[1]]['distance'] for edge in shortest_path_edges)
    print(f"Кратчайший маршрут: {shortest_path_nodes}")
    print(f"Общее расстояние: {total_distance} км")
else:
    print("Маршрут между указанными вершинами не найден.")


def dijkstra_with_ice(graph, start, end, date):
    distances = {vertex: float('infinity') for vertex in graph.nodes()}
    distances[start] = 0

    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor in graph.neighbors(current_vertex):
            ice_thickness = graph[current_vertex][neighbor]["ice_thickness"][date]
            weight = 1  # You can modify this if you have different weights for edges

            distance = current_distance + (weight / (1 - 0.07 * ice_thickness))

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances[end]
# Пример использования:

shortest_distance = dijkstra_with_ice(G, start_node, end_node, date)
print(f'Кратчайшее расстояние между {start_node} и {end_node} с учетом льда: {total_distance}')





# Рисуем граф
pos = coordinates
nx.draw_networkx_nodes(G, pos, node_size=20)
nx.draw_networkx_edges(G, pos, edgelist=edges)
nx.draw_networkx_edges(G, pos, edgelist=shortest_path_edges, edge_color='red', width=2.0)
nx.draw_networkx_labels(G, pos, font_size=7)
plt.show()