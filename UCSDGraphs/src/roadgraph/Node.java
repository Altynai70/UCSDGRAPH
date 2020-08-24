package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class Node {
	GeographicPoint vertex;									//Location of the vertex.
	List<Edge> edges = new ArrayList<Edge>();			//List of edges emerging from that vertex.
	double distance;										//Dijkstra's algorithm distance.
	double projdist;										//Projected distance for AStar algorithm.
	
	//Constructor
	public Node(){
		vertex = new GeographicPoint(10,10);
		edges = new ArrayList<Edge>();
		distance = Double.POSITIVE_INFINITY;
		projdist = Double.POSITIVE_INFINITY;
	}
	
	//Creating a new MapNode
	public Node(GeographicPoint vertex, List<Edge> edges){
		vertex = this.vertex;
		edges = this.edges;
		distance = Double.POSITIVE_INFINITY;
		projdist = Double.POSITIVE_INFINITY;
	}
	
	public Node(GeographicPoint vertex, double distance)
	{
		vertex = this.vertex;
		edges = new ArrayList<Edge>();
		distance = this.distance;
		projdist = Double.POSITIVE_INFINITY;
	}

public Node(GeographicPoint vertex, double distance, double projdist)
{
	vertex = this.vertex;
	edges = new ArrayList<Edge>();
	distance = this.distance;
	projdist = this.projdist;
}

public double DistanceFrom(Node other)
{
	double thisx = this.vertex.getX();
	double thisy = this.vertex.getY();
	double otherx = other.vertex.getX();
	double othery = other.vertex.getY();
	double distance = Math.sqrt((thisx - otherx)*(thisx - otherx) + (thisy - othery)*(thisy - othery));
	return distance;
}



{

StringBuilder sb = new StringBuilder("animals");
sb.insert(7,"-");
sb.insert(0,"-");
sb.insert(4,"-");
System.out.println(sb);
}
}