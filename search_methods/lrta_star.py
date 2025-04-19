from sokoban.map import Map
from sokoban.dummy import Dummy 
import sokoban.moves as mv

from typing import Tuple

import random

from scipy.optimize import linear_sum_assignment
import numpy as np


def bfs(map: Map, source : Tuple[int, int], target: Tuple[int, int], tunnel=False) -> int:
    dist = [[10000000 for _ in range(map.width)] for _ in range(map.length)]
    dist[source[0]][source[1]] = 0
    q = [source]

    dirx = [0, 0, 1, -1]
    diry = [-1, 1, 0, 0]
    while q:
        curr = q[0]
        # print(curr)
        q = q[1:]

        if curr == target:
            break

        dummy = Dummy(curr[0], curr[1])
        for move in range(4):
            nextx, nexty = curr[0] + dirx[move], curr[1] + diry[move]
            if not map.object_in_bounds_move(dummy, move + 1):
                continue
            if (nextx, nexty) in map.obstacles:
                continue
            # if (nextx, nexty) in map.targets:
            #     continue
            if (nextx, nexty) in map.positions_of_boxes and (nextx, nexty) != target:
                continue

            if tunnel:
                prevx, prevy = curr[0] - dirx[move], curr[1] - diry[move]
                if (prevx, prevy) in map.obstacles:
                    continue
                if prevx < 0 or prevx >= map.length or prevy < 0 or prevy >= map.width:
                    continue

            if 1 + dist[curr[0]][curr[1]] < dist[nextx][nexty]:
                dist[nextx][nexty] = 1 + dist[curr[0]][curr[1]]
                q.append((nextx, nexty))

    # print('--------------------------------')
    return dist[target[0]][target[1]]


def h(map: Map, map_box_target: dict):
    ans = 0
    targets = []
    boxes = []

    for x, y in map.targets:
        if not (x, y) in map.positions_of_boxes:
            targets.append((x, y))
    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    # l = len(targets)
    # pair = [random.choice(range(l)) for _ in range(l)]
    # for i in range(l):
    #     # ans += abs(targets[i][0] - boxes[pair[i]][0]) + abs(targets[i][1] - boxes[pair[i]][1])
    #     ans += bfs(map, boxes[i], targets[pair[i]])

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        # ans += bfs(map, (t_x, t_y), k)
        ans += abs(t_x - k[0]) + abs(t_y - k[1])

    # x_player, y_player = map.player.x, map.player.y
    # for x, y in boxes:
    #     ans += abs(x_player - x) + abs(y_player - y)
        # ans += bfs(map, (x, y), (x_player, y_player))

    for x, y in map.positions_of_boxes:
        if x == 0 and y == 0:
            ans = 1e6
        elif x == 0 and y == map.width - 1:
            ans = 1e6
        elif x == map.length - 1 and y == 0:
            ans = 1e6
        elif x == map.length - 1 and y == map.width - 1:
            ans = 1e6
        
    return ans

class Lrta:
    def __init__(self, map : Map, name):
        self.map = map
        self.no_states = 0
        self.H = {}
        self.res = {}
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


    def cost(self, sprev : Map, a : int):
        if not (str(sprev), a) in self.res.keys():
            return h(sprev, self.map_box_targets)
        str_state = self.res[(str(sprev), a)]
        return 1 + self.H[str_state]

    def solve(self):
        random.seed(42)

        state = self.map
        sprev = None
        aprev = -1

        while True:
            print(state)

            if state.is_solved():
                return state
            
            if not str(state) in self.H:
                self.H[str(state)] = h(state, self.map_box_targets)
            
            if sprev:
                self.res[(str(sprev), aprev)] = str(state)
                aux = [self.cost(sprev, b) for b in sprev.filter_possible_moves()]
                self.H[str(sprev)] = min(aux)

            moves = state.filter_possible_moves()

            aux = [self.cost(state, b) for b in moves]

            next_moves = []
            minim = min(aux)
            for i in range(len(moves)):
                print(f"move {mv.moves_meaning[moves[i]]} with cost {aux[i]}")
                if aux[i] == minim:
                    next_moves.append(moves[i])
            a = random.choice(next_moves)

            sprev = state.copy()
            aprev = a
            state.apply_move(a)

            self.no_states += 1
            if self.no_states > 260096:
                return None