
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;

/// A sample AI that takes a very suboptimal path.
/**
 * This is a sample AI that moves as far horizontally as necessary to reach the target,
 * then as far vertically as necessary to reach the target.  It is intended primarily as
 * a demonstration of the various pieces of the program.
 * 
 * @author Scott Madera
 */
public class DijkstraAI implements AIModule
{
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map)
    {
        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        Point currentPoint = map.getStartPoint();
        path.add(new Point(currentPoint));
        HashMap<Point,Boolean> closed = new HashMap<Point,Boolean>();
        Boolean pathVisited = new Boolean(false);
        while((map.getEndPoint().x != currentPoint.x) && (map.getEndPoint().y != currentPoint.y))
        {
            Point[] neighbors = map.getNeighbors(currentPoint);
            Point minNeighbor = new Point();
            Double minCost = Double.MAX_VALUE;
            for (Point neighbor : neighbors) {
                if (closed.get(neighbor) != null) {
                    continue;
                }
                Double tempCost = map.getCost(currentPoint, neighbor);
                if (tempCost < minCost) {
                    minCost = tempCost;
                    minNeighbor = neighbor;
                }
            }
            currentPoint = minNeighbor;
            closed.put(new Point(currentPoint), true);
            path.add(new Point(currentPoint));
        }

        /*
        // Keep moving vertically until we match the target.
        while(map.getEndPoint().y != CurrentPoint.y)
        {
            if(map.getEndPoint().y > CurrentPoint.y)
                ++CurrentPoint.y;
            else
                --CurrentPoint.y;
            path.add(new Point(CurrentPoint));
        }
        */

        // We're done!  Hand it back.
        return path;
    }
}
