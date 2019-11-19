Lincoln Schroeder
HW3 Design/Complexity

For my design, all the nodes and paths are first generated from the user input. Once complete, 
I create a NodeMap, which simply stores all the nodes. Each Node has an ID (which for our homework
this is just a number), a List of Nodes representing the shortest path to the source (this starts 
out as empty, becasue we haven't calculated it yet), the Cost of the path (initialized to max value),
and a Map of neighbor Nodes and their path costs.

Once setup is completed, we being iterating through the NodeMap. We keep track of completed Nodes
(nodes which have their complete path), and uncompleted Nodes(Nodes which do not have their complete
path). For each Node in the Set of uncompleted Nodes, check all neighbors. If the neighbor is already
in the completed Nodes, ignore it. Otherwise, find the shortest distance *1* to get to that Node, and place
it in the uncompleted Nodes Set. Once all neighbors have been checked, put the current Node into the
completed Nodes Set. Once this loop is complete, all Nodes have been checked and all shortest paths
have been found. Once all information in the NodeMap is updated, it's a simple function to output the results.

*1*: When finding the shortest distance, I compare the current Node's current path's cost (plus the cost to 
get to the neighbor Node) to the neighbor Node's current path's cost. If the checked path is shorter, update
the neighbor Node's path to use the shorter path instead.

The complexity of this program is O(NLogN), because of how it handles the completed/uncompleted Nodes.