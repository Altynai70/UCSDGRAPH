package roadgraph;

import java.util.List;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapTester{

		public static void main(String[] args){
			
			System.out.print("Making a new map...");
			MapGraph theMap= new MapGraph();
			System.out.print("DONE.\nLoading the Map...");

			GraphLoader.loadRoadMap("data/simpletest.map", theMap);
		
			System.out.println ("DONE.");

		System.out.println("Num nodes:" + theMap.getNumVertices());
		System.out.println("Num nodes:" + theMap.getNumEdges());

		List<GeographicPoint>route = theMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println(route);
		}
	}
