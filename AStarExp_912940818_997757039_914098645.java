import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.PriorityQueue;
/// A sample AI that takes an optimal path using A* search.
/**
 * This is a sample AI that intelligently searches through a path of points
 * using A* search, by means of a Dijkstra's algorithm implementation using
 * a PriorityQueue min-heap, as well as a heuristic function. This AI
 * specifically has its heuristic designed for the exponential cost function.
 * 
 * @author Cameron Lee
 * @author Marshall Fan
 * @author Scott Madera
 */
public class AStarExp_912940818_997757039_914098645 implements AIModule {
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map) {

        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        Point currentPoint = map.getStartPoint();

        /* we create a PQ of MapNode objects that
         * enables us to order the queue by costs from
         * the StartPoint. A MapNode is a pair <Point, Double>,
         * with the Double value representing the accumulated
         * cost from StartPoint to the Point. */
        PriorityQueue<MapNode> open = new PriorityQueue<MapNode>();
        HashMap<Point, Boolean> closed = new HashMap<Point, Boolean>();

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
        HashMap<Point, Double> distances = new HashMap<Point, Double>();
        HashMap<Point, Point> paths = new HashMap<Point, Point>(); // format: <current, prev>

        // initialize collections to ensure loop correctness
        distances.put(currentPoint, 0.0);
        open.add(new MapNode(currentPoint, getHeuristicCost(currentPoint, map.getEndPoint(), map)));
        closed.put(currentPoint, true);
        while (!open.isEmpty()) {
            // set current point as the point associated
            // with the least cost in the frontier set
            currentPoint = open.poll().getPoint();
            if ((map.getEndPoint().x == currentPoint.x) && (map.getEndPoint().y == currentPoint.y)) {
                break;
            }
            Double minCost;
            Point[] neighbors = map.getNeighbors(currentPoint);
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

                    /* entering below if statement means that there is
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
                    // f(n) + g(n) + h(n)
                    minCost += getHeuristicCost(neighbor, map.getEndPoint(), map);
                    paths.put(neighbor, currentPoint);
                    open.add(new MapNode(neighbor, minCost)); // adding neighbors to frontier; O(log n)
                }
            } // end of neighbors loop
            closed.put(currentPoint, true);
        } // end of Dijkstra while loop
        while ((map.getStartPoint().x != currentPoint.x) || (map.getStartPoint().y != currentPoint.y)) {
            path.add(0, currentPoint);
            currentPoint = paths.get(currentPoint);
        }
        path.add(0, currentPoint); // re-add start point

        // We're done!  Hand it back.
        return path;
    } // end of createPath()

    /** Given a current point, an end point, and a TerrainMap variable,
     *  returns the heuristic cost from current point to the end point
     *  by use of the Exponential Cost function. This function also
     * @param currentPoint: The (current) start point.
     * @param endpoint: The end point.
     * @return the minimum heuristic cost from currentPoint to endpoint.
     * **/
    private double getHeuristicCost(final Point currentPoint, final Point endpoint, TerrainMap map) {
        // First: compute min. # of moves from currentPoint to End,
        // assuming that height costs are ignored.
        int minNumOfMoves = getMinNumOfMoves(currentPoint, endpoint);
        double heightDiffToGoal = map.getTile(endpoint) - map.getTile(currentPoint);
        double idealCost;
        if (Math.abs(heightDiffToGoal) <= minNumOfMoves) {
            if (heightDiffToGoal < 0) {
                idealCost = 0.5*(-1*heightDiffToGoal) + (minNumOfMoves - heightDiffToGoal*(-1));
            }
            else {
                idealCost = 2 * heightDiffToGoal + (minNumOfMoves - heightDiffToGoal);
            }
        }
        else {
            idealCost = estimateIdealCost(minNumOfMoves, heightDiffToGoal);
        }
        return idealCost;
    }

    /** Given min number of moves to goal, and height difference to goal,
     *  returns the heuristic exponential cost from current point to goal point.
     *  This function specifically handles the case where |height difference|
     *  is greater than the minimum diagonal distance.
     * @param minSpaces: The minimum diagonal distance.
     * @param heightDiffToGoal: The estimated height difference from currentHeight to the goal height.
     * @return the minimum estimated exponential cost from current point to goal point.
     * **/
    private double estimateIdealCost(int minSpaces, double heightDiffToGoal) {
        // Create a list that holds all possible numbers and their quantities
        // that could sum to s.
        // Notice that a negative s will only have negative values in its
        // list and a positive s will only have positive values in its list.
        // This is because the optimal set of values, for our purposes, will
        // never make "backwards" progress.
        ArrayList<Double> heights = new ArrayList<Double>();
        double runningSum = 0;
        if (Math.abs(heightDiffToGoal) <= minSpaces) {
            for (int i = 0; i < minSpaces; i++) {
                if (Math.abs(runningSum) < Math.abs(heightDiffToGoal)) {
                    if (heightDiffToGoal < 0) {
                        heights.add(-1.0);
                        runningSum -= 1.0;
                    }
                    else if (heightDiffToGoal > 0) {
                        heights.add(1.0);
                        runningSum += 1.0;
                    }
                }
                else {
                    heights.add(0.0);
                }
            }
        }
        else {
            double optimalHeightChange = 0;
            double avgHeightDiff = heightDiffToGoal/minSpaces;
            for (int i = 0; i < minSpaces; i++) {
                if (runningSum == 0) {
                    optimalHeightChange = Math.ceil(avgHeightDiff);
                }
                else {
                    optimalHeightChange = Math.ceil((heightDiffToGoal - runningSum)/(minSpaces - i));
                }
                heights.add(optimalHeightChange);
                runningSum += optimalHeightChange;
            }
        }
        double totalCost = 0;
        for (double elevationDiff : heights) {
            totalCost += Math.pow(2.0, elevationDiff);
        }
        return totalCost;
    }

    /** Returns the diagonal distance between two nodes
     *  by comparing their coordinates and performing
     *  calculations on them.
     * @param start The (current) start point.
     * @param end The end point.
     * @return the minimum diagonal distance between start and end.
     * **/
    private int getMinNumOfMoves(Point start, Point end) {
        int xStart = start.x;
        int xEnd = end.x;
        int yStart = start.y;
        int yEnd = end.y;

        int numOfMoves;

        // case 1: (250,250) to (100,250)
        if ( (xStart == xEnd) || (yStart == yEnd) ) {
            if (xStart != xEnd) {
                numOfMoves = Math.abs(xEnd - xStart);
            }
            else { // yStart != yEnd
                numOfMoves = Math.abs(yEnd - yStart);
            }
        }
        // case 2: (250,250) to (450,450), perfect diagonal line
        else if ( (xStart == yStart) && (xEnd == yEnd) ) {
            numOfMoves = Math.abs(xEnd - xStart);
        }
        // case3: (250,200) to (400,450)
        else {
            int xDiff = Math.abs(xEnd - xStart);
            int yDiff = Math.abs(yEnd - yStart);
            int numOfDiagonalMoves = Math.min(xDiff, yDiff);
            int remainingMoves;
            if (xDiff > yDiff) {
                remainingMoves = xDiff - numOfDiagonalMoves;
            }
            else {
                remainingMoves = yDiff - numOfDiagonalMoves;
            }
            numOfMoves = numOfDiagonalMoves + remainingMoves;
        }
        return numOfMoves;
    } // end of getMinNumOfMoves

} // end of AStarClass

/// A class that associates a cost to a point.
/**
 * MapNode associates an accumulated cost away from a starting point
 * to some point in its object. This is specifically so that the
 * PriorityQueue object above will have a sorting property, where
 * least cost points are chosen for min extraction first, which is central
 * towards implementing Dijkstra's Algorithm.
 *
 * The sorting inside a PriorityQueue of MapNodes is done via comparator.
 * The overriden equals() method allows for proper remove() functionality
 * when wanting to remove specific points from the PriorityQueue.
 *
 */
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
