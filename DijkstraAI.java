import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.HashMap;
import java.util.PriorityQueue;
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
        // path.add(new Point(currentPoint));

        /* we create a PQ of MapNode objects that
        enables us to order the queue by costs from
        the StartPoint. A MapNode is a pair <Point, Double>,
        with the Double value representing the accumulated
        cost from StartPoint to the Point. */
        PriorityQueue<MapNode> open = new PriorityQueue<>();
        HashMap<Point, Boolean> closed = new HashMap<Point, Boolean>();
        HashMap<Point, Point> paths = new HashMap<Point,Point>();
        open.add(new MapNode(currentPoint, 0.0));
        Boolean pathVisited = new Boolean(false);
        while ((map.getEndPoint().x != currentPoint.x) || (map.getEndPoint().y != currentPoint.y)) {
            // verify if current point has been visited or not.
            // If so, take node from open list.
            if (open.isEmpty()) {
                throw new RuntimeException("Reached empty open list");
            }
            // set current point as the point associated
            // with the least cost in the frontier set
            currentPoint = open.poll().getPoint();

            Point[] neighbors = map.getNeighbors(currentPoint);
            Point minNeighbor = new Point();
            Double minCost = Double.MAX_VALUE;
            for (Point neighbor : neighbors) {
                if (closed.get(neighbor) != null) {
                    continue;
                }
                Double tempCost = map.getCost(currentPoint, neighbor);
                open.add(new MapNode(neighbor, tempCost)); // adding neighbors to frontier
                if (tempCost < minCost) {
                    minCost = tempCost;
                    minNeighbor = neighbor;
                }
            }

            closed.put(new Point(currentPoint), true);
            paths.put(new Point(minNeighbor), new Point(currentPoint));
        }

        // we're doing a bit of extra work here.
        // reverse is an O(n^2) operation, while if we just added each new Point to the beginning
        // of a list, we would accomplish the same result in only O(n) time

        // Are you sure? I looked up the Collections.sort() documentation,
        // and a lot of sources ended up saying it was O(n) anyway, not O(n^2).
        while ((map.getStartPoint().x != currentPoint.x) || (map.getStartPoint().y != currentPoint.y)) {
            path.add(paths.get(currentPoint));
            currentPoint = paths.get(currentPoint);
            System.out.println(currentPoint);
        }
        Collections.reverse(path);

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
                open.remove(key);
                return key;
            }
        }
        throw new RuntimeException("getMinPoint: no matching key.");
    }
}

class MapNode implements Comparable<MapNode> {
    private Point point;
    private Double cost;
    public MapNode() {
        this.point = new Point();
        this.cost = 0.0;
    }
    public MapNode(Point p, Double cost) {
        this.point = p;
        this.cost = cost;
    }
    @Override
    public int compareTo(MapNode mp) {
        return Double.compare(this.getCost(), mp.getCost());
    }
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MapNode)) {
            return false;
        }
        return ( (this.getPoint().equals(((MapNode)obj).getPoint()))
                && this.getCost().equals(((MapNode)obj).getCost()) );
    }
    public Point getPoint() {
        return point;
    }
    public Double getCost() {
        return cost;
    }
}