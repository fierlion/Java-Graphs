package roadgraph;

import geography.GeographicPoint;

/**
 * @author Ray Allan
 * 
 * A class which represents a particular road edge
 * Each edge has a start and end point and may be reference be either
 *
 */
public class RoadEdge {
	private GeographicPoint fromPoint;
	private GeographicPoint toPoint;
	private String roadName;
	private String roadType;
	private double roadLength;
	
	public RoadEdge(GeographicPoint from, GeographicPoint to, String name, 
			String type, double length) {
		fromPoint = from;
		toPoint = to;
		roadName = name;
		roadType = type;
		roadLength = length;
	}
	
	public GeographicPoint getFrom() {
		return fromPoint;
	}
	
	public GeographicPoint getTo() {
		return toPoint;
	}
	
	public String getRoadName() {
		return roadName;
	}
	
	public String getRoadType() {
		return roadType;
	}
	
	public double getRoadLength() {
		return roadLength;
	}
	
	public void setFrom(GeographicPoint from) {
		fromPoint = from;
	}
	
	public void setTo(GeographicPoint to) {
		toPoint = to;
	}
	
	public void setRoadName(String name) {
		roadName = name;
	}
	
	public void setRoadType(String type) {
		roadType = type;
	}
	
	public void setRoadLength(double length) {
		roadLength = length;
	}
	
	
}
