from sokoban.map import Map
from sokoban.dummy import Dummy 
import sokoban.moves as mv

from typing import Tuple

import random

from scipy.optimize import linear_sum_assignment
import numpy as np

import sokoban.gif as gif

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


def h(map: Map, map_box_target: dict, box_order: list):
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        # dist = abs(t_x - k[0]) + abs(t_y - k[1])
        dist = bfs(map, k, (t_x, t_y), tunnel=True)
        ans += dist
        # if dist == 0:
        #     ans -= len(map.boxes)

    # print(f"^^^^{box_order}")
    x_player, y_player = map.player.x, map.player.y
    # if box_order:
    #     box_name = box_order[0]
    #     found = False
    #     for x, y in boxes:
    #         if map.positions_of_boxes[(x, y)] == box_name:
    #             found = True
    #     if not found:
    #         box_order.pop()
    #     box = map.boxes[box_name]
    #     ans += bfs(map, (x_player, y_player), (box.x, box.y))
    distances = []
    for x, y in boxes:
        # distances.append(abs(x_player - x) + abs(y_player - y))
        distances.append(bfs(map, (x_player, y_player), (x, y)))

    if distances:
        ans += 2 * min(distances)
    # else:
    #     return -1000000
    
    # if distances:
    #     min_dist = min(distances)
    #     ans += min_dist
    #     if min_dist == 1:
    #         idx = distances.index(min_dist)
    #         box_x, box_y = boxes[idx]
    #         moves = map.filter_possible_moves()
    #         if box_x == x_player + 1 and mv.UP in moves:
    #             box_name = map.positions_of_boxes[(box_x, box_y)]
    #             t_x, t_y = map_box_target[box_name]
    #             dist = bfs(map, (box_x + 1, box_y), (t_x, t_y))
    #             if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
    #                 ans -= 2
    #                 print("reduction")
    #         elif box_x == x_player - 1 and mv.DOWN in moves:
    #             box_name = map.positions_of_boxes[(box_x, box_y)]
    #             t_x, t_y = map_box_target[box_name]
    #             dist = bfs(map, (box_x - 1, box_y), (t_x, t_y))
    #             if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
    #                 ans -= 2
    #                 print("reduction")
    #         elif box_y == y_player + 1 and mv.RIGHT in moves:
    #             box_name = map.positions_of_boxes[(box_x, box_y)]
    #             t_x, t_y = map_box_target[box_name]
    #             dist = bfs(map, (box_x, box_y + 1), (t_x, t_y))
    #             if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
    #                 ans -= 2
    #                 print("reduction")
    #         elif box_y == y_player - 1 and mv.LEFT in moves:
    #             box_name = map.positions_of_boxes[(box_x, box_y)]
    #             t_x, t_y = map_box_target[box_name]
    #             dist = bfs(map, (box_x, box_y - 1), (t_x, t_y))
    #             if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
    #                 ans -= 2
    #                 print("reduction")
    # else:
    #     return -10000000

    # for x, y in map.positions_of_boxes:
    #     if x == 0 and y == 0:
    #         ans = 1e6
    #     elif x == 0 and y == map.width - 1:
    #         ans = 1e6
    #     elif x == map.length - 1 and y == 0:
    #         ans = 1e6
    #     elif x == map.length - 1 and y == map.width - 1:
    #         ans = 1e6
        
    return ans

class Lrta:
    def __init__(self, map : Map, name):
        self.box_order = []
        for k in map.boxes.keys():
            self.box_order.append(k)
        self.map = map
        self.no_states = 0
        self.H = {}
        self.res = {}
        self.name = name
        self.best_action = {}

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
            return h(sprev, self.map_box_targets, self.box_order)
        str_state = self.res[(str(sprev), a)]
        if a <= 4:
            return 1 + self.H[str_state]
        return 6 + self.H[str_state]

    def solve(self, debug=False):
        random.seed(42)

        start_state = self.map.copy()

        state = self.map.copy()
        sprev = None
        aprev = -1

        path = []
        actions = []

        while True:
            path.append(str(state))
            # if len(path) < 250:
            #     path.append(str(state))

            if debug:
                print(state)

            if not str(state) in self.H:
                self.H[str(state)] = h(state, self.map_box_targets, self.box_order)

            if state.is_solved():
                # curr_state = start_state
                # while not curr_state.is_solved() or len(path) <= 250:
                # for _ in range(50):
                #     path.append(str(curr_state))
                #     if not str(curr_state) in self.best_action:
                #         break
                #     best_a = self.best_action[str(curr_state)]
                #     curr_state.apply_move(best_a)
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
                    i += 1
                
                aux_actions = [x for x in aux_actions if x > 4]

                path = aux_path
                print(f"++++{len(path)}")

                # curr_state = state.copy()
                # for _ in range(50):
                #     print(curr_state)
                #     path.append(str(curr_state))
                #     neighbors = curr_state.get_neighbours()
                #     min_state = None
                #     costmin = 10000000000
                #     for neigh in neighbors:
                #         if str(neigh) in self.H:
                #             cost = self.H[str(neigh)]
                #             if cost < costmin:
                #                 costmin = cost
                #                 min_state = neigh
                #     curr_state = min_state
                    # moves = curr_state.filter_possible_moves()
                    # print(curr_state)
                    # costmin = 100000000
                    # actmin = -1
                    # for b in moves:
                    #     if (str(curr_state), b) in self.res.keys():
                    #         cost = self.H[self.res[(str(curr_state), b)]]
                    #         if cost < costmin:
                    #             costmin = cost
                    #             actmin = b
                    # if actmin == -1:
                    #     break
                    # print(curr_state)
                    # print(mv.moves_meaning[actmin])
                    # print(costmin)
                    # curr_state.apply_move(actmin)

                # print(f"+++++ {len(path)}")
                # curr_str = str(sprev)
                # for _ in range(100):
                #     if len(path) > 100:
                #         break
                #     if not curr_str in self.res.values():
                #         break
                #     for k, v in self.res:
                #         if v == curr_str:
                #             path.insert(0, curr_str)
                #             curr_str = k[0]
                
                # gif.save_images(path, f"images/img/larta/{self.name}")
                # gif.create_gif(f"images/img/larta/{self.name}", f"{self.name}", f"images/gif/larta/{self.name}")

                return state, len(aux_actions)
            
            if sprev:
                self.res[(str(sprev), aprev)] = str(state)
                moves = sprev.filter_possible_moves()
                aux = [self.cost(sprev, b) for b in moves]
                minim = min(aux)
                self.H[str(sprev)] = minim
                self.best_action = aux.index(minim)

                # min_aux = 100000000
                # min_act = -1
                # for m in moves:
                #     if (str(sprev), m) in self.res.keys():
                #         next = self.res[(str(sprev), m)]
                #         cost = self.H[next]
                #         if cost < min_aux:
                #             min_aux = cost
                #             min_act = m
                # if min_act != -1:
                #     self.best_action[str(sprev)] = min_act

            moves = state.filter_possible_moves()

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

            actions.append(a)

            sprev = state.copy()
            aprev = a
            state.apply_move(a)

            self.no_states += 1
            if self.no_states > 100096:
                return None, -10