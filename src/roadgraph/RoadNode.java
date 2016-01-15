package roadgraph;

import geography.GeographicPoint;

public class RoadNode {
	private GeographicPoint location;
	private Double distance;
	
	RoadNode(GeographicPoint locationIn, Double distanceIn) {
		location = locationIn;
		distance = distanceIn;
	}
	
	RoadNode(GeographicPoint locationIn) {
		location = locationIn;
		distance = Double.POSITIVE_INFINITY;
	}
	
	public GeographicPoint getLocation() {
		// method is not private
		// this class should be private
		return location;
	}

	public Double getDistance() {
		return distance;
	}
	public void setDistance(Double distanceIn) {
		distance = distanceIn;
	}
	public void setLocation(GeographicPoint locationIn) {
		location = locationIn;
	}
}
