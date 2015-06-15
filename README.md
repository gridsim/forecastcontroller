# Forecast controller for gridsim simulator


### Synopsis
This project use gridsim and add some new elements to use the simulator with forecast controllers. 

### Library dependencies
* deap
* PuLP

### Solver dependencies
* Gurobi (see [here](http://www.gurobi.com/academia/for-universities "gurobi with a free academic license"))
It's also possible to use OpenSource solver such as glpk, lp_solve or cbc using:
```
# GLPK:
self.status = problem.solve(pulp.GLPK(msg=0))

# CBC:
path_to_cbc = "..."
status = problem.solve(COIN_CMD(path_to_cbc))

# GUROBI (default):
self.status = problem.solve(pulp.GUROBI(msg=0))
```

### Using this demo
clone the repository inside the gridsim/demo/ repository and run the running_script file.
