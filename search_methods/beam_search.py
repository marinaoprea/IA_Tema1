from sokoban.map import Map
import heapq
from typing import Tuple

import copy
import numpy as np
from scipy.optimize import linear_sum_assignment

import sokoban.gif as gif

import sokoban.moves as mv

k = 50

from search_methods.lrta_star import bfs

inf = 20

def eval(map: Map, map_box_target: dict) -> int:
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        # dist = bfs(map, (t_x, t_y), k)
        dist = bfs(map, k, (t_x, t_y), tunnel=True)
        ans += dist
        if dist == 0:
            ans -= len(map.boxes)
    
    x_player, y_player = map.player.x, map.player.y
    distances = []
    for box in boxes:
        distances.append(bfs(map, (x_player, y_player), box))
    
    if distances:
        min_dist = min(distances)
        ans += min_dist
        if min_dist == 1:
            idx = distances.index(min_dist)
            box_x, box_y = boxes[idx]
            moves = map.filter_possible_moves()
            if box_x == x_player + 1 and mv.UP in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x + 1, box_y), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    print("reduction")
            elif box_x == x_player - 1 and mv.DOWN in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x - 1, box_y), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    print("reduction")
            elif box_y == y_player + 1 and mv.RIGHT in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x, box_y + 1), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    print("reduction")
            elif box_y == y_player - 1 and mv.LEFT in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x, box_y - 1), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    print("reduction")
            # else:
            #     ans += 10
    
        # ans += min(distances)

    # for x, y in map.positions_of_boxes:
    #     if x == 0 and y == 0:
    #         ans += inf
    #     elif x == 0 and y == map.width - 1:
    #         ans += inf
    #     elif x == map.length - 1 and y == 0:
    #         ans += inf
    #     elif x == map.length - 1 and y == map.width - 1:
    #         ans += inf
    
    ans += map.undo_moves * 10

    return ans

class Beam_search:

    def __init__(self, map: Map, name: str) -> None:
        self.map = map
        self.no_states = 0
        self.path = {}
        self.name = name

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
        # open_states = [(self.map, eval(self.map, self.map_box_targets))]
        # tuple (score, state, prev)
        open_states = [(eval(self.map, self.map_box_targets), self.map)]
        self.path[str(self.map)] = None
        visited = {str(self.map)}
        while open_states:
            new_states = []
            for _, state in open_states:
                neighbors = state.get_neighbours()
                for neigh in neighbors:
                    if str(neigh) in visited:
                        continue
                        
                    new_states.append((eval(neigh, self.map_box_targets), neigh))
                    self.path[str(neigh)] = str(state)
                    visited.add(str(neigh))
                    
            if not new_states:
                state = open_states[0][1]
                if debug:
                    all_strs_path = []
                    str_curr = str(state)
                    while str_curr:
                        all_strs_path.insert(0, str_curr)
                        str_curr = self.path[str_curr]

                    gif.save_images(all_strs_path, f"images/img/beam_search/{self.name}")
                    gif.create_gif(f"images/img/beam_search/{self.name}", f"{self.name}", f"images/gif/beam_search/{self.name}")

            self.no_states += len(new_states)

            new_states.sort()
            i = 0
            # aux = []
            # while i < len(new_states) and len(aux) < k:
            #     state = new_states[i][1]
            #     if not str(state) in visited:
            #         aux.append(new_states[i])
            #     i += 1

            # new_states = aux  
            if len(new_states) > k:
                new_states = new_states[0:k]

            if debug:
                print('---------------------------------')
                for score, state in new_states:
                    print(state)
                    print(f'with cost {score}')

            for score, state in new_states:
                if state.is_solved():
                    if debug:
                        all_strs_path = []
                        str_curr = str(state)
                        while str_curr:
                            all_strs_path.insert(0, str_curr)
                            str_curr = self.path[str_curr]

                        gif.save_images(all_strs_path, f"images/img/{self.name}")
                        gif.create_gif(f"images/img/{self.name}", f"{self.name}", f"images/gif/{self.name}")
                        
                    return state
                
            if self.no_states > 60096:
                return None
            
            open_states = new_states
        
        return None
