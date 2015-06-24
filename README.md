
# Forecast controller for gridsim simulator

### Version
This demo is used with gridsim 'release/0.1.1rc2' released February 4, 2015

### Synopsis
This project use gridsim and add some new elements to use the simulator with forecast controllers. 

### Library dependencies
* deap
* PuLP

### Solver dependencies
The resolution is done by an integer linear programming solver (ILP). This application support the three solver below. Be sure to have one of them installed on your computer.

* Gurobi[^gurobi] is a commercial solver but free to use for academic community. It is one of the most performing solver.
* CPLEX[^cplex] is another commercial solver developed by IBM. It support free to use academic use. 
* COIN-OR CBC[^cbc] is a free and open-source solver under Eclipse Public License (considered to be business-friendly free software license).
* GLPK[^glpk] is a free and open-source solver under GNU Public License.

According to some benchmarks[^benchmark] prefer this order: Gurobi or CPLEX, CBC and GLPK. This application was successfully tested with Gurobi, CBC and GLPK. CPLEX was not tested. 

```
test = []
```

### Using this demo
clone the repository inside the gridsim/demo/ repository and run the running_script file.

[^gurobi]: [http://www.gurobi.com/academia/for-universities](http://www.gurobi.com/academia/for-universities "gurobi with a free academic license")

[^cplex]: [https://www.ibm.com/developerworks/community/blogs/jfp/entry/cplex_studio_in_ibm_academic_initiative?lang=en](https://www.ibm.com/developerworks/community/blogs/jfp/entry/cplex_studio_in_ibm_academic_initiative?lang=en "cplex")

[^cbc]: [https://projects.coin-or.org/Cbc](https://projects.coin-or.org/Cbc "Cbc project")

[^glpk]: [http://www.gnu.org/software/glpk/](http://www.gnu.org/software/glpk/ "GNU Linear Programming Kit")

[^benchmark]: [http://www.statistik.tuwien.ac.at/forschung/CS/CS-2012-1complete.pdf](http://www.statistik.tuwien.ac.at/forschung/CS/CS-2012-1complete.pdf "statistik") and [http://www.gurobi.com/resources/switching-to-gurobi/open-source-solvers](http://www.gurobi.com/resources/switching-to-gurobi/open-source-solvers "gurobi benchmark")