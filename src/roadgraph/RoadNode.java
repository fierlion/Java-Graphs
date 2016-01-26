package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class RoadNode {
	private GeographicPoint location;
	private Double distance;
	private Double projectedDistance;
	private ArrayList<RoadNode> neighbors;
	
	// for use in astar
	RoadNode(GeographicPoint locationIn, Double distanceIn, Double projIn) {
		location = locationIn;
		distance = distanceIn;
		projectedDistance = projIn;
		neighbors = new ArrayList<>();
	}
	
	// for use in dijksra
	RoadNode(GeographicPoint locationIn, Double distanceIn) {
		location = locationIn;
		distance = distanceIn;
		projectedDistance = Double.POSITIVE_INFINITY;
		neighbors = new ArrayList<>();
	}
	
	//for use in non-weighted graphs or as a default for dijkstra/astar
	RoadNode(GeographicPoint locationIn) {
		location = locationIn;
		distance = Double.POSITIVE_INFINITY;
		projectedDistance = Double.POSITIVE_INFINITY;
		neighbors = new ArrayList<>();
	}
	
	public GeographicPoint getLocation() {
		GeographicPoint locCopy = new GeographicPoint(location.x, location.y);
		return locCopy;
	}
	
	public void setLocation(GeographicPoint locationIn) {
		location = locationIn;
	}

	public Double getDistance() {
		return distance;
	}

	public void setDistance(Double distanceIn) {
		distance = distanceIn;
	}
	
	public Double getProjDistance() {
		return distance;
	}

	public void setProjDistance(Double projDistanceIn) {
		distance = projDistanceIn;
	}
	

	public List<RoadNode> getNeighbors() {
		List<RoadNode> neighCopy = new ArrayList<>();
		neighCopy.addAll(neighbors);
		return neighCopy;
	}
	
	public void addNeighbor(RoadNode neighborIn) {
		neighbors.add(neighborIn);
	}
	
	
	public Double distanceFrom(RoadNode other) {
		return other.getLocation().distance(location);
	}
	
	public boolean isSameLocation(GeographicPoint pointIn) {
		if (!pointIn.equals(null)) {
		return(location.x == pointIn.x &&
		location.y == pointIn.y);
		}
		return false;
	}
}
