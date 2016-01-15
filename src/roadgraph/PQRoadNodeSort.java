package roadgraph;

import java.util.Comparator;

public class PQRoadNodeSort implements Comparator<RoadNode> {
	@Override
	public int compare(RoadNode one, RoadNode two) {
		Double dist1 = one.getDistance();
		Double dist2 = two.getDistance();
		return dist1 < dist2 ? -1 : dist1 == dist2 ? 0 : 1;
	}
}
