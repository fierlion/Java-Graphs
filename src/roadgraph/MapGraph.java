/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
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
	private Map<GeographicPoint, ArrayList<GeographicPoint>> mapAdjList;
	// TODO arraylist might not be best choice once we need to retrieve
	// longer paths;  revisit this once it goes into use
	private ArrayList<RoadEdge> mapEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		mapAdjList = new HashMap<>();
		mapEdges = new ArrayList<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return mapAdjList.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{	
		Set<GeographicPoint> vertices = new HashSet<>();
		Set<GeographicPoint> protectedVertices = mapAdjList.keySet();
		for (GeographicPoint point : protectedVertices) {
			vertices.add(point);
		}
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return mapEdges.size();
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
		if (mapAdjList.get(location) != null) {
			return false;
		}
		else {
			ArrayList<GeographicPoint> associatedEdges = new ArrayList<>();
			mapAdjList.put(location, associatedEdges);
			return true;
		}
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
	 *   or if the length is less than 0
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if (from == null || to == null || roadName == null || roadType == null || (Double)length == null) {
			throw new IllegalArgumentException();
		}
		else if (mapAdjList.get(from) == null || mapAdjList.get(to) == null) {
			throw new IllegalArgumentException();
		}
		else {
			//add to edges array
			RoadEdge newEdge = new RoadEdge(from, to, roadName, roadType, length);
			mapEdges.add(newEdge);
			//add to mapAdjList
			mapAdjList.get(from).add(to);
		}
	}
	
	public List<GeographicPoint> getNeighbors(GeographicPoint thisVertex) 
		throws IllegalArgumentException {
		if (!mapAdjList.containsKey(thisVertex)) {
			throw new IllegalArgumentException();
		} 
		else {
			return (mapAdjList.get(thisVertex));
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
		// initialize 
		Queue<GeographicPoint> q = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint,GeographicPoint> parent = new HashMap<>();
		q.add(start);
		visited.add(start);
		// bfs using queue
		while (q.size() != 0) {
			GeographicPoint curr = q.remove();
			if (curr.equals(goal)) {
				// return path
				return (unwindParents(parent, start, goal));
			}
			List<GeographicPoint> neighbors = getNeighbors(curr);
			for (GeographicPoint neigh : neighbors) {
				if (!visited.contains(neigh)) {
					visited.add(neigh);
					nodeSearched.accept(neigh);  //for visualization
					parent.put(neigh, curr);
					q.add(neigh);
				}
			}			
		}
		return null;
	}
	
	/** Unwind the parent Map from a successful bfs 
	 * 	and from it return a path-ordered array of vertices
	 * 
	 * @param parents successful bfs result in Map form
	 * @param start The starting location from bfs method
	 * @param goal The goal location from bfs method
	 * @return
	 */
	private List<GeographicPoint> unwindParents(Map<GeographicPoint,GeographicPoint> parents, 
			GeographicPoint start, GeographicPoint goal) {
		//linked list allows simple and efficient addFirst() method
		LinkedList<GeographicPoint> unwound = new LinkedList<>();
		unwound.addFirst(goal);
		GeographicPoint curr = goal;
		//iterate backwards through map using each value as the next key
		while (!curr.equals(start)) {
			GeographicPoint next = parents.get(curr);
			unwound.addFirst(next);
			curr = next;
		}
		return unwound;
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
