import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
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
public class DijkstraAI implements AIModule {
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map) {
        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        Point currentPoint = map.getStartPoint();
        path.add(new Point(currentPoint));
        HashMap<Point, Double> open = new HashMap<>();
        HashMap<Point, Boolean> closed = new HashMap<Point, Boolean>();
        open.put(currentPoint, 0.0);
        Boolean pathVisited = new Boolean(false);
        while ((map.getEndPoint().x != currentPoint.x) && (map.getEndPoint().y != currentPoint.y)) {
            // verify if current point has been visited or not.
            // If so, take node from open list.
            if (open.isEmpty()) {
                throw new RuntimeException("Reached empty open list");
            }
            // set current point as the point associated
            // with the least cost in the frontier set
            currentPoint = extractMinPoint(open);

            // choose a different Point to search towards
            // if current node is already "explored"
            if (closed.containsKey(currentPoint)) {
                continue;
            }

            Point[] neighbors = map.getNeighbors(currentPoint);
            Point minNeighbor = new Point();
            Double minCost = Double.MAX_VALUE;
            for (Point neighbor : neighbors) {
                if (closed.get(neighbor) != null) {
                    continue;
                }
                Double tempCost = map.getCost(currentPoint, neighbor);
                open.put(neighbor, tempCost); // adding neighbors to frontier
                if (tempCost < minCost) {
                    minCost = tempCost;
                    minNeighbor = neighbor;
                }
            }

            closed.put(new Point(currentPoint), true);
            currentPoint = minNeighbor;
            path.add(new Point(currentPoint));
        }

        // We're done!  Hand it back.
        return path;
    }

    // find and return the Point associated with
    // the minimum cost in the frontier set.
    private Point extractMinPoint(HashMap<Point, Double> open) {
        Double minCost = Collections.min(open.values());
        for (Point key : open.keySet()) {
            if (open.get(key).equals(minCost)) {
                // extraction portion
                open.remove(key, minCost);
                return key;
            }
        }
        throw new RuntimeException("getMinPoint: no matching key.");
    }
}
