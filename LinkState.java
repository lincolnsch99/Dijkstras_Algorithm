/**
 * File Name: LinkState.java
 * Author(s): Lincoln Schroeder
 * Purpose: LinkState takes input from the command line to structure a network topology graph, represented by Nodes
 * and a NodeMap. The program then iterates through the topology to find the shortest paths to each node (from the
 * provided source node), and how much they cost.
 */

import java.io.File;
import java.util.*;

public class LinkState
{
    private static int sourceNode;

    /**
     * Main method. Takes the input file's path as the first argument and the ID of the source node as a second
     * argument.
     * @param args the input from startup.
     */
    public static void main(String[] args)
    {
        String filePath;
        int numNodes;

        filePath = args[0];
        sourceNode = Integer.parseInt(args[1]);

        File selectedFile = new File(filePath);
        try
        {
            Scanner input = new Scanner(selectedFile);
            numNodes = input.nextInt();
            NodeMap nodeMap = new NodeMap();
            List<Node> allNodes = new ArrayList<>();

            for (int i = 0; i < numNodes; i++)
            {
                Node newNode = new Node(i);
                allNodes.add(newNode);
            }

            while (input.hasNext())
            {
                int startNode = input.nextInt();
                int endNode = input.nextInt();
                float pathCost = input.nextFloat();
                allNodes.get(startNode).addNeighbor(allNodes.get(endNode), pathCost);
                allNodes.get(endNode).addNeighbor(allNodes.get(startNode), pathCost);
            }

            for (Node node : allNodes)
                nodeMap.addNode(node);

            nodeMap = getShortestPath(nodeMap, allNodes.get(sourceNode));
            outputPaths(nodeMap, sourceNode);
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }

    /**
     * Outputs each node's shortest path and the cost.
     * @param nodeMap the map containing all nodes to be outputted.
     * @param sourceNode the source node. The path and cost would always be the same for every source node, so it is
     *                   not displayed.
     */
    private static void outputPaths(NodeMap nodeMap, int sourceNode)
    {
        int curWantedId = 0;
        while(curWantedId < nodeMap.getNodes().size())
        {
            for (Node node : nodeMap.getNodes())
            {
                if (node.getId() != sourceNode && node.getId() == curWantedId)
                {
                    StringBuilder builder = new StringBuilder();
                    builder.append("shortest path to node ");
                    builder.append(node.getId());
                    builder.append(" is ");
                    for (Node nodeInOrder : node.getShortestPath())
                    {
                        builder.append(nodeInOrder.getId());
                        builder.append("->");
                    }
                    builder.append(node.getId());
                    builder.append(" with cost ");
                    builder.append(node.getCost());
                    System.out.println(builder);
                }
            }
            curWantedId++;
        }
    }

    /**
     * Iterates through the nodeMap and finds the shortest path for every node.
     * @param map the nodeMap to be iterated through.
     * @param source the source node.
     * @return an update nodeMap with all new shortest paths and costs.
     */
    private static NodeMap getShortestPath(NodeMap map, Node source)
    {
        source.setCost(0f);
        Set<Node> completedNodes = new HashSet<>();
        Set<Node> uncompletedNodes = new HashSet<>();
        uncompletedNodes.add(source);

        while(uncompletedNodes.size() != 0)
        {
            Node curNode = findClosestNode(uncompletedNodes);
            uncompletedNodes.remove(curNode);
            for(Map.Entry<Node, Float> neighborPair : curNode.getNeighbors().entrySet())
            {
                Node neighbor = neighborPair.getKey();
                Float cost = neighborPair.getValue();
                if(!completedNodes.contains(neighbor))
                {
                    calculateShortestDistance(curNode, cost, neighbor);
                    uncompletedNodes.add(neighbor);
                }
            }
            completedNodes.add(curNode);
        }
        return map;
    }

    /**
     * Compares the destination's current shortest path to a path coming from the given source. If the given path is
     * shorter than the current existing path, change the destination's shortest path to a path coming from the given
     * source node.
     * @param source the source of this comparison, NOT the absolute source node.
     * @param cost the cost of the path between the source node and destination node.
     * @param destination the node that we are going to.
     */
    private static void calculateShortestDistance(Node source, Float cost, Node destination)
    {
        Float sourceCost = source.getCost();
        if (sourceCost + cost < destination.getCost())
        {
            destination.setCost(sourceCost + cost);
            List<Node> shortestPath = new LinkedList<>(source.getShortestPath());
            shortestPath.add(source);
            destination.setShortestPath(shortestPath);
        }
    }

    /**
     * Iterates through the given set of nodes and finds which node is the closest/least cost.
     * @param uncompletedNodes set of nodes.
     * @return
     */
    private static Node findClosestNode(Set<Node> uncompletedNodes)
    {
        Node closestNode = null;
        Float closestDistance = Float.MAX_VALUE;
        for(Node node : uncompletedNodes)
        {
            Float distance = node.getCost();
            if(distance < closestDistance)
            {
                closestDistance = distance;
                closestNode = node;
            }
        }
        return closestNode;
    }

    /**
     * The Node class is used to represent a single node of the network topology graph. Each node has an ID, a List
     * representing its shortest path to the source node, the cost of its shortest path, and a Map of all its neighbors
     * and costs to get to them.
     */
    private static class Node
    {
        private int id;
        private List<Node> shortestPath;
        private Float cost;
        Map<Node, Float> neighbors;

        /**
         * Constructor
         * @param id unique ID for this node.
         */
        public Node(int id)
        {
            this.id = id;
            shortestPath = new LinkedList<>();
            cost = Float.MAX_VALUE;
            neighbors = new HashMap<>();
        }

        /**
         * Adds a neighbor to this node.
         * @param neighbor the Node to be added.
         * @param cost the cost to get to that neighbor.
         */
        public void addNeighbor(Node neighbor, float cost)
        {
            neighbors.put(neighbor, cost);
        }

        /**
         * Getter for the ID.
         * @return id.
         */
        public int getId()
        {
            return id;
        }

        /**
         * Getter for the shortestPath.
         * @return shortestPath.
         */
        public List<Node> getShortestPath()
        {
            return shortestPath;
        }

        /**
         * Getter for the cost of the shortest path.
         * @return cost.
         */
        public Float getCost()
        {
            return cost;
        }

        /**
         * Getter for the neighbors of this node.
         * @return neighbors.
         */
        public Map<Node, Float> getNeighbors()
        {
            return neighbors;
        }

        /**
         * Setter for the unique id.
         * @param id new id.
         */
        public void setId(int id)
        {
            this.id = id;
        }

        /**
         * Setter for the shortestPath.
         * @param shortestPath new List of Nodes representing the shortest path to the source node.
         */
        public void setShortestPath(List<Node> shortestPath)
        {
            this.shortestPath = shortestPath;
        }

        /**
         * Setter for the shortest path cost.
         * @param cost new cost to traverse along the shortest path.
         */
        public void setCost(Float cost)
        {
            this.cost = cost;
        }

        /**
         * Setter for the neighbors of the node.
         * @param neighbors new neighbors.
         */
        public void setNeighbors(Map<Node, Float> neighbors)
        {
            this.neighbors = neighbors;
        }
    }

    /**
     * NodeMap is a simple class to help store all the nodes in the topology graph.
     */
    private static class NodeMap
    {
        private Set<Node> nodes;

        /**
         * Constructor.
         */
        public NodeMap()
        {
            nodes = new HashSet<>();
        }

        /**
         * Adds a node to the NodeMap.
         * @param add the node to be added.
         */
        public void addNode(Node add)
        {
            nodes.add(add);
        }

        /**
         * Getter for the nodes stored in the NodeMap.
         * @return nodes.
         */
        public Set<Node> getNodes()
        {
            return nodes;
        }
    }
}
