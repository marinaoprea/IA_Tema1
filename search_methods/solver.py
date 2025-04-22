from sokoban.map import Map
from search_methods.beam_search import Beam_search
from search_methods.lrta_star import Lrta
from typing import Tuple

class Solver:

    def __init__(self, map: Map, name: str, typ: str, h) -> None:
        self.map = map
        if typ == "beam":
            self.solver = Beam_search(h, self.map, name)
        elif typ == "lrta":
            self.solver = Lrta(h, self.map, name)
        self.name = name
        self.type = typ

    def solve(self, debug=False):
        if self.type == "beam":
            print("-------beam search-----")
            final_state = self.solver.solve(debug=debug)
            if final_state:
                print(f'number of explored states is {self.solver.no_states}')
                print(f'number of undo moves is {final_state.undo_moves}')

            return final_state
        else:
            print("------lrta* search-----")
            final_state, pull_moves = self.solver.solve(debug=debug)
            print(final_state)
            if final_state:
                print(f'number of explored states is {final_state.explored_states}')
                print(f'number of undo moves is {pull_moves}')
            return final_state, pull_moves