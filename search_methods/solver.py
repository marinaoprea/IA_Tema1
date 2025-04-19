from sokoban.map import Map
from search_methods.beam_search import Beam_search
from search_methods.lrta_star import Lrta

class Solver:

    def __init__(self, map: Map, name: str) -> None:
        self.map = map
        self.beam_solver = Beam_search(self.map, name)
        self.lrta_solver = Lrta(self.map, name)
        self.name = name

    def solve_beam_search(self, debug=False) -> Map:
        print("-------beam search-----")
        final_state = self.beam_solver.solve(debug=debug)
        if final_state:
            print(f'number of explored states is {final_state.explored_states}')
            print(f'number of undo moves is {final_state.undo_moves}')
        print(f'explored {self.beam_solver.no_states}')
        return final_state


    def solve_lrta_star(self, debug=False) -> Map:
        print("------lrta* search-----")
        final_state = self.lrta_solver.solve()
        print(final_state)
        if final_state:
            print(f'number of explored states is {final_state.explored_states}')
            print(f'number of undo moves is {final_state.undo_moves}')
        return final_state