# An Effective Initialization Method for Genetic Algorithm-based Robot Path Planning using a Directed Acyclic Graph

## Abstract

The goal of robot path planning is to find a feasible path that proceeds from a starting point to a destination point without intersecting any obstacles in the given environment. 
Recently, genetic algorithm-based robot path planning methods have been widely considered in the intelligent robotics community. 
Because the initialization process significantly influences the performance of the genetic algorithm, an effective initialization method is required. 
However, investigation on this subject is still lacking. In this paper, we propose an effective initialization method for genetic algorithm-based robot path planning. 
Experimental results comparing genetic algorithms with conventional initialization methods and the proposed initialization method showed that the proposed method leads to high quality paths in a significantly shorter execution time.

This program is designed to generate initial path set for genetic algorithm-based robot path planning.

### [Paper]
Jaesung Lee and Dae-Won Kim, [An Effective Initialization Method for Genetic Algorithm-based Robot Path Planning using a Directed Acyclic Graph](https://www.sciencedirect.com/science/article/pii/S0020025515007847),
Information Sciences, 332(1):1-18, 2016
Entropy, 2020

## License

This program is available for download for non-commercial use, licensed under the GNU General Public License, which is allows its use for research purposes or other free software projects but does not allow its incorporation into any type of commerical software.

## Sample Input and Output

SGMP function will return the path set according to given map. This code can be executed from MATLAB command window. Detailed information is given below.

### [Usage]:

   `>> [pathSet, execTime, dag] = sgmp( map, sp, dp, setSize, numPt );`

### [Description]
   pathSet – A set of path that contains “setSize” paths \
   execTime – A history of execution time for creating each path \
   dag – A Directed Acyclic Graph created by SGMP 

   map – A map specified in the binary square matrix [Download] \
   sp – The index of starting node \
   dp – The index of destination node \
   setSize – The number of paths to be created from DAG \
   numPt – The maximum number of allowable parents node

For more detailed information, please type “help sgmp” from MATLAB command window.

Other miscellaneous functions are also included in the ZIP package.

### [Usage]:
   `>> draw_path( map, path ); – it will draw the obtained path.` \
   `>> draw_dag( map, dag ); – it will draw the obtaind DAG.`
