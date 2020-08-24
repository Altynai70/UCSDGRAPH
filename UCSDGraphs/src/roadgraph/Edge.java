package roadgraph;

import geography.GeographicPoint;


public class Edge {
	
	private double timeToGetThrough;
	GeographicPoint start;
	GeographicPoint end;
	String VertexName;
	String vertexType;
	double length;
	
	//Constructor
	public Edge() {
		
		start = new GeographicPoint(10,10);
		end = new GeographicPoint(10,10);
		VertexName = "abc";      					// Random values initialized which could be helpful in debugging.
		vertexType = "def";
		length = 0;
		}
	
	//Create a new MapEdge
	public Edge(GeographicPoint start,GeographicPoint end,String VertexName,String vertexType,double length)
	{
		vertexType = this.vertexType;
		VertexName = this.VertexName;
		start = this.start;
		end = this.end;
		length = this.length;
		this.timeToGetThrough = length / getSpeedLimitByRoadType(vertexType);
	}
	
	//Constructor for defining location without name(Just given for debugging with my own map file.)
	public Edge(GeographicPoint start,GeographicPoint end)
	{
		start = this.start;
		end = this.end;
	}
	
        public static int getSpeedLimitByRoadType(String roadType){
        switch (roadType) {
            case "primary": return 50;
            case "residential": return 30;
            case "motorway": return 90;
            case "tertiary": return 30;
            case "secondary_link": return 40;
            case "motorway_link": return 50;
            default: return 50;
        }
    }

    public double getTimeToDriveThrough() {
		return timeToGetThrough;
    }

    public GeographicPoint getToPoint() {
        return this.end;
    }

    public GeographicPoint getFromPoint() {
        return this.start;
    }

    public String getRoadName(){
        return this.VertexName;
    }

    public String getRoadType(){
        return this.vertexType;
    }

    public double getLength(){
        return this.length;
    }

}
