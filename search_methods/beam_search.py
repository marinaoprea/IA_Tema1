from sokoban.map import Map
from typing import Tuple
import numpy as np
from scipy.optimize import linear_sum_assignment

import sokoban.gif as gif

import sokoban.moves as mv
from search_methods.heuristics import bfs
import time

k = 50

inf = 20

class Beam_search:

    def __init__(self, h, map: Map, name: str) -> None:
        self.map = map
        self.no_states = 0
        self.path = {}
        self.name = name
        self.h = h

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

        self.exec_time = -2

    def solve(self, debug=False, save_gif=False) -> Tuple[Map, int]:
        start_time = time.time()

        open_states = [(self.h(self.map, self.map_box_targets), self.map)]
        self.path[str(self.map)] = None
        visited = {str(self.map)}
        while open_states:
            new_states = []
            for _, state in open_states:
                neighbors = state.get_neighbours()
                for neigh in neighbors:
                    if str(neigh) in visited:
                        continue
                        
                    new_states.append((self.h(neigh, self.map_box_targets), neigh))
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

                if save_gif:
                    gif.save_images(all_strs_path, f"images/img/beam_search/{self.name}")
                    gif.create_gif(f"images/img/beam_search/{self.name}", f"{self.name}", f"images/gif/beam_search/{self.name}")

            self.no_states += len(new_states)

            new_states.sort()
            if len(new_states) > k:
                new_states = new_states[0:k]

            if debug:
                print('---------------------------------')
                for score, state in new_states:
                    print(state)
                    print(f'with cost {score}')

            for score, state in new_states:
                if state.is_solved():
                    all_strs_path = []
                    str_curr = str(state)
                    while str_curr:
                        all_strs_path.insert(0, str_curr)
                        str_curr = self.path[str_curr]

                    gif.save_images(all_strs_path, f"images/img/beam/{self.name}")
                    gif.create_gif(f"images/img/beam/{self.name}", f"{self.name}", f"images/gif/beam/{self.name}")

                    end_time = time.time()
                    self.exec_time = end_time - start_time
                        
                    return state
                
            if self.no_states > 60096:
                return None
            
            open_states = new_states
        
        return None
