package roadgraph;

import java.util.Comparator;

public class NodeComparator implements Comparator<Node>{
	
	@Override
	public int compare(Node one , Node two)
	{
		if(one.distance > two.distance)
			return 1;
		else if(one.distance <two.distance)
			return -1;
		return 0;
	}
			
}