package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class RoadNode {
	private GeographicPoint location;
	private Double distance;
	private ArrayList<RoadNode> neighbors;
	
	RoadNode(GeographicPoint locationIn, Double distanceIn) {
		location = locationIn;
		distance = distanceIn;
		neighbors = new ArrayList<>();
	}
	
	RoadNode(GeographicPoint locationIn) {
		location = locationIn;
		distance = Double.POSITIVE_INFINITY;
		neighbors = new ArrayList<>();
	}
	
	public GeographicPoint getLocation() {
		GeographicPoint locCopy = new GeographicPoint(location.x, location.y);
		return locCopy;
	}

	public Double getDistance() {
		return distance;
	}
	
	public List<RoadNode> getNeighbors() {
		List<RoadNode> neighCopy = new ArrayList<>();
		neighCopy.addAll(neighbors);
		return neighCopy;
	}
	
	public void setDistance(Double distanceIn) {
		distance = distanceIn;
	}
	
	public void setLocation(GeographicPoint locationIn) {
		location = locationIn;
	}
	
	public void addNeighbor(RoadNode neighborIn) {
		neighbors.add(neighborIn);
	}
	
	
	public Double distanceFrom(RoadNode other) {
		return other.getLocation().distance(location);
	}
	
	public boolean isSameLocation(GeographicPoint pointIn) {
		return(location.x == pointIn.x &&
		location.y == pointIn.y);
	}
}
