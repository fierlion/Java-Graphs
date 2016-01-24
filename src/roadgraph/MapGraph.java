/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private Map<RoadNode, List<RoadNode>> mapAdjList;
	private Map<GeographicPoint, RoadNode> geoNodeLookup;
	private List<RoadEdge> edges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		mapAdjList = new HashMap<>();
		geoNodeLookup = new HashMap<>();
		edges = new LinkedList<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{	
		return mapAdjList.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<RoadNode> allVertexNodes = mapAdjList.keySet();
		Set<GeographicPoint> allGeoPoints = new HashSet<>(); 
		for (RoadNode v : allVertexNodes) {
			allGeoPoints.add(v.getLocation());
		}
		return allGeoPoints;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// check for existing node at that geopoint
		Set<RoadNode> keys = mapAdjList.keySet();
		for (RoadNode k : keys) {
			if (k.isSameLocation(location)) {
				// already exists
				return false;
			}
		}
		//create and add to mapadjlist
		RoadNode newRoadNode = new RoadNode(location);
		List<RoadNode> newNeighbors = new LinkedList<>();
		mapAdjList.put(newRoadNode, newNeighbors);
		//add to lookup
		geoNodeLookup.put(location, newRoadNode);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if (from == null || to == null || roadName == null || roadType == null || (Double)length == null) {
			throw new IllegalArgumentException();
		}
		if (length < 0) {
			throw new IllegalArgumentException();
		}
		RoadNode fromIn = getNodeByGeo(from);
		RoadNode toIn = getNodeByGeo(to);
		if (fromIn.equals(null) || toIn.equals(null)) {
			throw new IllegalArgumentException();
		}
		// create RoadEdge and add to edges
		RoadEdge newEdge = new RoadEdge(from, to, roadName, roadType, length);
		edges.add(newEdge);
		// add to mapAdjList
		mapAdjList.get(fromIn).add(toIn);
		//add neighbor to from node
		fromIn.addNeighbor(toIn);
	}
	
	public RoadNode getNodeByGeo(GeographicPoint pointIn) {
		RoadNode result = null;
		Set<RoadNode> keys = mapAdjList.keySet();
		for (RoadNode k : keys) {
			if (k.isSameLocation(pointIn)) {
				result = k;
			}
		}
		return result;
	}
	
	public void printAdjList() {
		String delimiter = " : ";
		for (RoadNode r : mapAdjList.keySet()) {
			System.out.print(r.getLocation());
			System.out.print(delimiter);
			List<RoadNode> neighbors = mapAdjList.get(r);
			for (RoadNode nr : neighbors) {
				System.out.print(nr.getLocation());
				System.out.print("|");
			}
			System.out.println("");
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		//initialize queue, visited hashset and parent hashmap
		//Enqueue S onto the queue
		//while queue is not empty:
		//  dequeue node curr from front of queue
		//  if curr == G return parent map
		//  for each of curr's neighbors, n, not in visited set:
		//    add n to visited set
		//    add curr as n's parent in parent map
		//    enqueue n onto the queue
		Queue<RoadNode> q = new LinkedList<>();
		Set<RoadNode> visited = new HashSet<>();
		Map<RoadNode, RoadNode> parents = new HashMap<>();
		RoadNode startNode = getNodeByGeo(start);
		RoadNode goalNode = getNodeByGeo(goal);
		// we should be sure to have these nodes
		assert(!startNode.equals(null));
		assert(!goalNode.equals(null));
		q.add(startNode);
		while(!q.isEmpty()) {
			RoadNode curr = q.poll();
			if(curr.equals(goalNode)) {
				return(unwindParents(parents, startNode, goalNode));
			}
			for (RoadNode neigh : curr.getNeighbors()) {
				if(!containsRoadNode(neigh, parents.keySet())) {
					 nodeSearched.accept(neigh.getLocation());
					 visited.add(neigh);
					 parents.put(neigh, curr);
					 q.add(neigh);
				}
			}
		}
		return null;
	}
	
	public Boolean containsRoadNode(RoadNode nodeIn, Set<RoadNode> parents) {
		for (RoadNode n : parents) {
			if (n.isSameLocation(nodeIn.getLocation())) {
				return (Boolean.TRUE);
			}
		}
		return (Boolean.FALSE);
	}
	
	public List<GeographicPoint> unwindParents(Map<RoadNode, RoadNode> parentsIn, 
			RoadNode startIn, RoadNode goalIn) {
		LinkedList<GeographicPoint> result = new LinkedList<>();
		result.addFirst(goalIn.getLocation());
		RoadNode curr = goalIn;
		while (!curr.equals(startIn)) {
			RoadNode next = parentsIn.get(curr);
			GeographicPoint nextGeo = next.getLocation();
			result.addFirst(nextGeo);
			curr = next;
		}
		return(result);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
