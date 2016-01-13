package roadgraph;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Test;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraphTest {

	@Test
	public void testAddVertices() {
		MapGraph testGraph = new MapGraph();
		GeographicPoint oneone = new GeographicPoint(1,1);
		GeographicPoint twotwo = new GeographicPoint(2,2);
		assert((testGraph.getNumVertices()==0) == true);
		testGraph.addVertex(oneone);
		testGraph.addVertex(twotwo);
		assert((testGraph.getNumVertices()==2) == true);
	}
	
	@Test
	public void testAddEdge() {
		MapGraph testGraph = new MapGraph();
		GeographicPoint oneone = new GeographicPoint(1,1);
		GeographicPoint twotwo = new GeographicPoint(2,2);
		testGraph.addVertex(oneone);
		testGraph.addVertex(twotwo);
		assert((testGraph.getNumEdges()==0) == true);
		String testName = "oneone to twotwo";
		String testType = "test road";
		double testDistance = 1.0;
		testGraph.addEdge(oneone, twotwo, testName, testType, testDistance);
		assert((testGraph.getNumEdges()==1) == true);
	}
	
	@Test
	public void testBfs() {
        MapGraph graph = new MapGraph();
        GraphLoader.loadRoadMap("data/graders/mod2/map1.txt", graph);
        GeographicPoint start = new GeographicPoint(0, 0);
        GeographicPoint end = new GeographicPoint(6, 6);
        List<GeographicPoint> bfsResult = graph.bfs(start, end);
        System.out.println(bfsResult);
        List<GeographicPoint> bfsReverse = graph.bfs(end, start);
        System.out.println(bfsReverse);
	}

}
