from sokoban.map import Map
from search_methods.beam_search import Beam_search
from search_methods.lrta_star import Lrta
from typing import Tuple
import sokoban.gif as gif

class Solver:

    def __init__(self, map: Map, name: str, typ: str, h) -> None:
        self.map = map
        if typ == "beam":
            self.solver = Beam_search(h, self.map, name)
        elif typ == "lrta":
            self.solver = Lrta(h, self.map, name)
        self.name = name
        self.type = typ

    def solve(self, debug=False, save_gif=False):
        if self.type == "beam":
            print("-------beam search-----")
            final_state = self.solver.solve(debug=debug, save_gif=save_gif)
            if final_state:
                print(f'number of explored states is {self.solver.no_states}')
                print(f'number of undo moves is {final_state.undo_moves}')

            return final_state
        else:
            print("------lrta* search-----")
            # final_state, pull_moves = self.solver.solve(debug=debug)
            min_state = None
            min_moves = 1000
            no_states = 0
            box_order = self.solver.box_order
            best_path = []
            for _ in range(5):
                self.solver.box_order = box_order
                final_state, pull_moves, path = self.solver.solve(debug=debug, save_gif=save_gif)
                no_states += self.solver.no_states
                print(f"pull moves {pull_moves}")

                if pull_moves > 0 and pull_moves < min_moves:
                    min_state = final_state.copy()
                    min_moves = pull_moves
                    # no_states = self.solver.no_states
                    best_path = path
                if pull_moves == 0:
                    min_moves = 0
                    # no_states = self.solver.no_states
                    best_path = path
                    break
                self.solver.no_states = 0

            if save_gif:
                gif.save_images(best_path, f"images/img/larta/{self.name}")
                gif.create_gif(f"images/img/larta/{self.name}", f"{self.name}", f"images/gif/larta/{self.name}")

            final_state = min_state
            self.solver.no_states = no_states
            if min_moves == 1000:
                min_moves = -10
            pull_moves = min_moves
            print(final_state)
            if final_state:
                print(f'number of explored states is {final_state.explored_states}')
                print(f'number of undo moves is {pull_moves}')
            return final_state, pull_moves