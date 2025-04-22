from sokoban.map import Map
from sokoban.dummy import Dummy
from typing import Tuple
import sokoban.moves as mv

def bfs(map: Map, source : Tuple[int, int], target: Tuple[int, int], tunnel=False) -> int:
    dist = [[10000000 for _ in range(map.width)] for _ in range(map.length)]
    dist[source[0]][source[1]] = 0
    q = [source]

    dirx = [0, 0, 1, -1]
    diry = [-1, 1, 0, 0]
    while q:
        curr = q[0]
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

    return dist[target[0]][target[1]]


def eval_beam_search_bfs(map: Map, map_box_target: dict) -> int:
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
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
                    # print("reduction")
            elif box_x == x_player - 1 and mv.DOWN in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x - 1, box_y), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    # print("reduction")
            elif box_y == y_player + 1 and mv.RIGHT in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x, box_y + 1), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    # print("reduction")
            elif box_y == y_player - 1 and mv.LEFT in moves:
                box_name = map.positions_of_boxes[(box_x, box_y)]
                t_x, t_y = map_box_target[box_name]
                dist = bfs(map, (box_x, box_y - 1), (t_x, t_y))
                if dist < bfs(map, (box_x, box_y), (t_x, t_y)):
                    ans -= 2
                    # print("reduction")
    
    ans += map.undo_moves * 10

    return ans

def eval_beam_search_manhattan(map: Map, map_box_target: dict) -> int:
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        # dist = bfs(map, k, (t_x, t_y), tunnel=True)
        dist = abs(t_x - k[0]) + abs(t_y - k[1])
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
    
    ans += map.undo_moves * 10

    return ans

def eval_lrta_bfs_no_box_order(map: Map, map_box_target: dict, box_order: list):
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        dist = bfs(map, k, (t_x, t_y), tunnel=True)
        ans += dist

    x_player, y_player = map.player.x, map.player.y

    distances = []
    for x, y in boxes:
        distances.append(bfs(map, (x_player, y_player), (x, y)))

    if distances:
        ans += 2 * min(distances)
   
    return ans

def eval_lrta_bfs_box_order(map: Map, map_box_target: dict, box_order: list):
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        dist = bfs(map, k, (t_x, t_y), tunnel=True)
        ans += dist

    # print(f"^^^^{box_order}")
    x_player, y_player = map.player.x, map.player.y
    if box_order:
        box_name = box_order[0]
        found = False
        for x, y in boxes:
            if map.positions_of_boxes[(x, y)] == box_name:
                found = True
        if not found:
            box_order.pop()
        box = map.boxes[box_name]
        ans += bfs(map, (x_player, y_player), (box.x, box.y))
        
    return ans

def eval_lrta_manhatten(map: Map, map_box_target: dict, box_order: list):
    ans = 0
    boxes = []

    for x, y in map.positions_of_boxes.keys():
        if not (x, y) in map.targets:
            boxes.append((x, y))

    for k, v in map.positions_of_boxes.items(): # v is box name
        t_x, t_y = map_box_target[v]
        dist = abs(t_x - k[0]) + abs(t_y - k[1])
        ans += dist

    x_player, y_player = map.player.x, map.player.y
    distances = []
    for x, y in boxes:
        distances.append(bfs(map, (x_player, y_player), (x, y)))

    if distances:
        ans += 2 * min(distances)
        
    return ans