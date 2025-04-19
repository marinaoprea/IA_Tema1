from sokoban.map import Map
import heapq
from typing import Tuple

import random
import numpy as np
from scipy.optimize import linear_sum_assignment

k = 25

from search_methods.lrta_star import bfs


def eval(map: Map, map_box_target: dict) -> int:
    ans = 0
    targets = []
    boxes = []

    # for x, y in map.targets:
    #     if not (x, y) in map.positions_of_boxes:
    #         targets.append((x, y))
    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        ans += bfs(map, (t_x, t_y), k)
        # ans += abs(t_x - k[0]) + abs(t_y - k[1])

    # l = len(targets)
    # pair = [random.choice(range(l)) for _ in range(l)]
    # for i in range(l):
    #     ans += abs(targets[i][0] - boxes[pair[i]][0]) + abs(targets[i][1] - boxes[pair[i]][1])
    
    x_player, y_player = map.player.x, map.player.y
    if boxes:
        x, y = boxes[0]
        # ans += abs(x_player - x) + abs(y_player - y)
        ans += bfs(map, (x_player, y_player), (x, y))

    for x, y in map.positions_of_boxes:
        if x == 0 and y == 0:
            ans += 1e3
        elif x == 0 and y == map.width - 1:
            ans += 1e3
        elif x == map.length - 1 and y == 0:
            ans += 1e3
        elif x == map.length - 1 and y == map.width - 1:
            ans += 1e3
    
    return ans

class Beam_search:

    def __init__(self, map: Map) -> None:
        self.map = map
        self.no_states = 0

        box_index = {}
        i = 0
        for _, v in map.positions_of_boxes.items():
            box_index[i] = v
            i += 1

        target_index = {}
        i = 0
        for x_t, y_t in map.targets:
            target_index[i] = (x_t, y_t)
            i += 1
            
        aux2 = []
        for x_t, y_t in map.targets:
            aux = []
            for x_b, y_b in map.positions_of_boxes.keys():
                dist = bfs(map, (x_t, y_t), (x_b, y_b))
                aux.append(dist)
            aux2.append(aux)

        cost_matrix = np.array(aux2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        self.map_box_targets = {}
        for t_i, b_i in zip(row_ind, col_ind):
            self.map_box_targets[box_index[b_i]] = target_index[t_i]
            
        for k, v in self.map_box_targets.items():
            print(k)
            print(v)

    def solve(self, debug=False) -> Tuple[Map, int]:
        open_states = [(self.map, eval(self.map, self.map_box_targets))]
        visited = {str(self.map)}
        while open_states:
            new_states = []
            for state, _ in open_states:
                neighbors = state.get_neighbours()
                for neigh in neighbors:
                    if str(neigh) in visited:
                        continue
                        
                    if len(new_states) < k:
                        visited.add(str(neigh))
                        heapq.heappush(new_states, (neigh, -eval(neigh, self.map_box_targets)))
                    else:
                        score = eval(neigh, self.map_box_targets)
                        if -score > new_states[0][0]:
                            heapq.heappushpop(new_states, (neigh, -score))
                            visited.add(str(neigh))
            
            self.no_states += len(new_states)

            if debug:
                print('---------------------------------')
                for state, _ in new_states:
                    print(state)
                    print(f'with cost {eval(state, self.map_box_targets)}')

            for state, score in new_states:
                if state.is_solved():
                    return state
                
            if self.no_states > 60096:
                return open_states[0][0]
            
            open_states = new_states
        
        return None
