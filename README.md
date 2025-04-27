- Jupyter Notebook main file executes all tests with all available heuristics
- It also plots graphics with results
- It includes cell where single test may be executed
- Flag for debugging output is possible to be set when calling solve method; by default flag is False
- Flag for saving solution as gif may be as well set; by default flag is False
- Solution is saved in image folder in form of gif, separately for each heuristic
- Time for saving solution was not taken into consideration when plotting execution time because it was not part of the computational search

Additional requirements:
- `time` module
- `numpy` module
- `scipy` module