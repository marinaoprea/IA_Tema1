from sokoban.map import Map
import sokoban.moves as mv

import random

from scipy.optimize import linear_sum_assignment
import numpy as np

import sokoban.gif as gif
from search_methods.heuristics import bfs

import time

def get_unplaced_box(map: Map, map_box_to_targets: dict) -> int:
    cnt = 0
    for k, v in map.positions_of_boxes.items():
        t_x, t_y = map_box_to_targets[v]
        if not (k[0] == t_x and k[1] == t_y):
            cnt += 1
    
    return cnt

class Lrta:
    def __init__(self, h, map : Map, name):
        self.box_order = []
        for k in map.boxes.keys():
            self.box_order.append(k)
        self.map = map
        self.no_states = 0
        self.H = {}
        self.res = {}
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
                dist = bfs(map, (x_t, y_t), (x_b, y_b), tunnel=True)
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
    
        self.exec_time = 0
        self.i = 0

    def cost(self, sprev : Map, a : int):
        if not (str(sprev), a) in self.res.keys():
            # if a > 4:
            #     implicit_a = a - 4
            #     future_pos = sprev.player.get_future_position(implicit_a)
            #     if not future_pos in sprev.positions_of_boxes:
            #         return self.h(sprev, self.map_box_targets, self.box_order)
            return self.h(sprev, self.map_box_targets, self.box_order)
        str_state = self.res[(str(sprev), a)]
        if a <= 4:
            return 1 + self.H[str_state]
        
        implicit_a = a - 4
        future_pos = sprev.player.get_future_position(implicit_a)
        if not future_pos in sprev.positions_of_boxes:
            return 5 + self.H[str_state]
        return 1 + self.H[str_state]

    def solve(self, debug=False, save_gif=False):
        self.i += 1
        start_time = time.time()
        random.seed(42)

        state = self.map.copy()
        sprev = None
        aprev = -1

        path = []
        actions = []

        print("again")

        while True:
            path.append(str(state))
            # if len(path) < 250:
            #     path.append(str(state))

            if debug:
                print(state)

            if not str(state) in self.H:
                self.H[str(state)] = self.h(state, self.map_box_targets, self.box_order)

            if state.is_solved():
                print("&&&&&&&&")
                aux_path = []
                aux_actions = []
                i = 1
                for curr in path:
                    if not curr in aux_path:
                        aux_path.append(curr)
                        if i - 1 < len(actions):
                            aux_actions.append(actions[i - 1])
                    else:
                        while curr != aux_path[-1]:
                            aux_path.pop()
                            aux_actions.pop()
                        aux_actions.pop()
                        aux_actions.append(actions[i - 1])
                    i += 1
                
                aux_actions = [x for x in aux_actions if x > 4]
                # aux_actions = [mv.moves_meaning[x] for x in aux_actions]
                # print(aux_actions)

                path = aux_path
                print(f"++++{len(path)}")
                
                i = self.i

                end_time = time.time()
                self.exec_time += end_time - start_time

                return state, len(aux_actions), path
            
            if sprev:
                self.res[(str(sprev), aprev)] = str(state)
                moves = sprev.filter_possible_moves()
                aux = [self.cost(sprev, b) for b in moves]
                minim = min(aux)
                self.H[str(sprev)] = minim

            moves = state.filter_possible_moves()

            # aux_moves = []
            # for m in moves:
            #     aux_state = state.copy()
            #     aux_state.apply_move(m)
            #     if get_unplaced_box(aux_state, self.map_box_targets) <= get_unplaced_box(state, self.map_box_targets):
            #         aux_moves.append(m)
            # moves = aux_moves

            aux = [self.cost(state, b) for b in moves]

            # next_moves = []
            minim = min(aux)
            for i in range(len(moves)):
                if debug:
                    print(f"move {mv.moves_meaning[moves[i]]} with cost {aux[i]}")
                # if aux[i] == minim:
                #     next_moves.append(moves[i])
            # a = random.choice(next_moves)
            idx = aux.index(minim)
            a = moves[idx]

            if a > 4:
                implicit_a = a - 4
                future_pos = state.player.get_future_position(implicit_a)
                if not future_pos in state.positions_of_boxes:
                    actions.append(a)
                else:
                    actions.append(implicit_a)
            else:
                actions.append(a)
            # actions.append(aux_a)

            sprev = state.copy()
            aprev = a
            state.apply_move(a)

            self.no_states += 1
            if self.no_states > 30000:
                end_time = time.time()
                self.exec_time += end_time - start_time
                return None, -10, []