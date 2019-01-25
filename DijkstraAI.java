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
         * enables us to order the queue by costs from
         * the StartPoint. A MapNode is a pair <Point, Double>,
         * with the Double value representing the accumulated
         * cost from StartPoint to the Point. */
        PriorityQueue<MapNode> open = new PriorityQueue<>();
        HashMap<Point, Boolean> closed = new HashMap<>();

        /* distances will also hold the accumulated costs
         * from start point to the passed-in point much like open does,
         * with the passed-in point acting as key, cost acting as value.
         * We will use this to perform O(1) accesses instead of querying
         * the PriorityQueue, which can take O(log n) or O(n) operations.
         *
         * distances will have the following property:
         * if some point is found inside distances,
         * and has a non-null value associated with it,
         * then that key is inside the frontier. This will
         * be useful for later cost relaxation purposes. */
        HashMap<Point, Double> distances = new HashMap<>();
        HashMap<Point, Point> paths = new HashMap<>(); // format: <current, prev>

        // initialize collections to ensure loop correctness
        open.add(new MapNode(currentPoint, 0.0));
        distances.put(currentPoint, 0.0);
        closed.put(currentPoint, true);

        // Boolean pathVisited = new Boolean(false);
        while (!open.isEmpty()) {
            // set current point as the point associated
            // with the least cost in the frontier set
            currentPoint = open.poll().getPoint();
            if ((map.getEndPoint().x == currentPoint.x) && (map.getEndPoint().y == currentPoint.y)) {
                break;
            }

            Point[] neighbors = map.getNeighbors(currentPoint);
            // Point minNeighbor = new Point();
            Double minCost = Double.MAX_VALUE; // make compiler happy
            for (Point neighbor : neighbors) {
                if (closed.get(neighbor) != null) {
                    continue;
                }

                /* check if neighbor is in frontier. If distances has a non-null
                 * value for the given neighbor key, then neighbor is in frontier,
                 * meaning neighbor already has a cost value tied to it. Set this
                 * as the minCost. Otherwise set minCost as +infinity.
                 *
                 * This is much better than using an open.contains() check
                 * directly, as checking the PriorityQueue takes O(n) time,
                 * but querying the distances HashMap takes O(1) time. */
                boolean frontierFlag = false;
                if (distances.get(neighbor) != null) {
                    frontierFlag = true;
                    minCost = distances.get(neighbor);
                }
                else {
                    minCost = Double.MAX_VALUE;
                }
                Double tempCost = map.getCost(currentPoint, neighbor);
                if (tempCost + distances.get(currentPoint) < minCost) {
                    minCost = tempCost + distances.get(currentPoint);

                    /* entering this if statement means that there is
                     * a better path to the current neighbor, where neighbor
                     * is inside the frontier. Thus, remove the higher cost path
                     * of neighbor inside frontier, remove its entry in distances,
                     * and replace the entries back into the PriorityQueue and HashMap. */
                    if (frontierFlag) {
                        MapNode neighborInFrontier = new MapNode(neighbor, distances.get(neighbor));
                        distances.remove(neighbor); // O(1)?
                        open.remove(neighborInFrontier); // O(n)
                    }
                    distances.put(neighbor, minCost);
                    paths.put(neighbor, currentPoint);
                    open.add(new MapNode(neighbor, minCost)); // adding neighbors to frontier; O(log n)
                }
            }
            closed.put(currentPoint, true);
        }

        // we're doing a bit of extra work here.
        // reverse is an O(n^2) operation, while if we just added each new Point to the beginning
        // of a list, we would accomplish the same result in only O(n) time

        // Are you sure? I looked up the Collections.sort() documentation,
        // and a lot of sources ended up saying it was O(n) anyway, not O(n^2).
        while ((map.getStartPoint().x != currentPoint.x) || (map.getStartPoint().y != currentPoint.y)) {
            path.add(currentPoint);
            currentPoint = paths.get(currentPoint);
            // System.out.println(currentPoint);
        }
        path.add(currentPoint); // don't forget the StartPoint!
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