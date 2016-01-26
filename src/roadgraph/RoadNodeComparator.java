package roadgraph;

import java.util.Comparator;

public class RoadNodeComparator implements Comparator<RoadNode>{
	@Override
	public int compare(RoadNode x, RoadNode y) {
		if (x.getDistance() > y.getDistance()) {
			return 1;
		}
		else if (x.getDistance() < y.getDistance()) {
			return -1;
		}
		return 0;
	}
}
