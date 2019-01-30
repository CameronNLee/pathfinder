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
 * specifically has its heuristic designed for the Bizarro division cost function.
 *
 * @author Cameron Lee
 * @author Marshall Fan
 * @author Scott Madera
 */
public class AStarDiv_912940818_997757039_914098645 implements AIModule {
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map) {

        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        Point currentPoint = map.getStartPoint();

        /* we create a PQ of MapVertex objects that
         * enables us to order the queue by costs from
         * the StartPoint. A MapVertex is a pair <Point, Double>,
         * with the Double value representing the accumulated
         * cost from StartPoint to the Point. */
        PriorityQueue<MapVertex> open = new PriorityQueue<MapVertex>();
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
        open.add(new MapVertex(currentPoint, getHeuristicCost(currentPoint, map.getEndPoint(), map)));
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
                        MapVertex neighborInFrontier = new MapVertex(neighbor, distances.get(neighbor));
                        distances.remove(neighbor); // O(1)?
                        open.remove(neighborInFrontier); // O(n)
                    }
                    distances.put(neighbor, minCost);
                    // f(n) + g(n) + h(n)
                    minCost += getHeuristicCost(neighbor, map.getEndPoint(), map);
                    paths.put(neighbor, currentPoint);
                    open.add(new MapVertex(neighbor, minCost)); // adding neighbors to frontier; O(log n)
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
     *  Returns the heuristic cost from current point to the end point
     *  by use of the Bizarro World division function. This function also
     *  takes into account the no wormhole assumption, in that no two points
     *  adjacent to each other can be of height = 0.
     * @param currentPoint: The (current) start point.
     * @param endpoint: The end point.
     * @return the minimum heuristic cost from currentPoint to endpoint.
     * **/
    private double getHeuristicCost(final Point currentPoint, final Point endpoint, TerrainMap map) {
        int minNumOfMoves = getMinNumOfMoves(currentPoint, endpoint);
        double heightDiffToGoal = map.getTile(endpoint) - map.getTile(currentPoint);
        double idealCost = 0.0;
        double updatedHeight = map.getTile(currentPoint);
        double estimatedGoalHeight = map.getTile(currentPoint) + heightDiffToGoal;
        if ( (map.getTile(currentPoint) == 0) && (map.getTile(endpoint) == 0)
                   && (heightDiffToGoal == 0) && (minNumOfMoves == 1) ) {
            return 0.0;
        }
        if (Math.abs(heightDiffToGoal) <= minNumOfMoves) {
            if (heightDiffToGoal <= 0) {
                // we know that there are more moves to make
                // than the height diff when we enter this if statement.
                // We also know that height diff is negative or 0.
                // Thus, when stepping down by 1 to minimize costs,
                // we always get n/n + n-1/n-1 + n-2/n-2 ...
                // which is just 1 + 1 + 1 ... + 1 = |heightDiff|.
                idealCost = (-1*heightDiffToGoal);
                double remainingMoves = minNumOfMoves - (-1*heightDiffToGoal);

                // taking special caution not to violate wormhole assumption.
                // here, we must alternate up and down by 1 height until we reach the goal.
                // In this case, alternating up and down creates the following alternating series:
                // 0/(1+1) + 1/(0+1) + 0/(1+1) + ... = 0 + 1 + 0 + ...
                // First number shows cost of moving from height 0 to height 1 = 0.
                // Second number shows cost of moving from height 1 to height 0 = 1.
                // This series goes on until remaining moves are all out.
                if (map.getTile(endpoint) == 0) {
                    if ((remainingMoves % 2 == 1) || (remainingMoves == minNumOfMoves)) {
                        // special case to prevent wormhole assumption. Example:
                        //  h=4      h=3      h=2      h=1      h=0      h=0
                        //(9,9) to (9,8) to (9,7) to (9,6) to (9,5) to (9,4) goal
                        //     4/4   +  3/3   +  2/2   +  1/1   +  0/2

                        // WRONG! Risks violating wormhole with an odd number of remaining moves.
                        // What we want:

                        //  h=4      h=3      h=2      h=1      h=1      h=0
                        //(9,9) to (9,8) to (9,7) to (9,6) to (9,5) to (9,4) goal
                        //     4/4   +  3/3   +  2/2   +  1/2   +  1/1
                        // =    1    +   1    +   1    +  0.5   +   1
                        // idealCost = 4.5
                        idealCost = (-1*heightDiffToGoal) + 0.5 + ((remainingMoves - 1) / 2.0);
                    }
                    else {
                        // Safe to take half of remaining moves and add it to
                        // the ideal cost (we take half because if there is, say, 4 remaining moves,
                        // 2 of them will be 1s (going from 0 to 1 = 0, then 1 to 0 = 1,
                        // then 0 to 1 = 0, then from 1 to 0 = 1). Discard the 0s.
                        idealCost += (remainingMoves / 2.0);
                    }
                }
                else { // can safely ride the flat line distance all the way to the end.
                    double flatCost = (remainingMoves * (estimatedGoalHeight / (estimatedGoalHeight + 1.0) ));
                    idealCost += flatCost;
                }
            }
            else { // heightDiff is positive
                while (updatedHeight != estimatedGoalHeight) {
                    idealCost += updatedHeight / ((updatedHeight+1.0)+1.0);
                    ++updatedHeight;
                    --minNumOfMoves;
                }
                // tally up the remaining moves by riding the flat line distance to endpoint.
                idealCost += (updatedHeight / (updatedHeight+1.0))*minNumOfMoves;
            }
        }
        else {
            idealCost = estimateBizarroCost(minNumOfMoves, heightDiffToGoal, map.getTile(currentPoint));
        }
        return idealCost * 0.9;
    }

    /** Given number of moves to goal, height difference to goal, and current height,
     *  returns the heuristic division cost from current point to goal point. This function
     *  specifically handles the case where |height difference| is greater than
     *  the minimum diagonal distance.
     * @param minSpaces: The minimum diagonal distance.
     * @param heightDiffToGoal: The estimated height difference from currentHeight to the goal height.
     * @param currentHeight: the current height.
     * @return the minimum estimated division cost from current point to goal point.
     * **/
    private double estimateBizarroCost(int minSpaces, double heightDiffToGoal, double currentHeight) {
        double avgHeightDiff = heightDiffToGoal/minSpaces;
        double runningSum = 0.0;
        double totalCost = 0;
        double updatedHeight = currentHeight;
        double optimalHeightChange;

        for (int i = 0; i < minSpaces; ++i) {
            if (runningSum == 0) {
                optimalHeightChange = Math.floor(avgHeightDiff);
            }
            else {
                optimalHeightChange = Math.floor((heightDiffToGoal - runningSum) / (minSpaces - i));
            }
            totalCost += updatedHeight / ((updatedHeight + optimalHeightChange) + 1.0);
            updatedHeight += optimalHeightChange;
            runningSum += optimalHeightChange;
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
    }
} // end of AStarClass

/// A class that associates a cost to a point.
/**
 * MapVertex associates an accumulated cost away from a starting point
 * to some point in its object. This is specifically so that the
 * PriorityQueue object above will have a sorting property, where
 * least cost points are chosen for min extraction first, which is central
 * towards implementing Dijkstra's Algorithm.
 *
 * The sorting inside a PriorityQueue of MapVertices is done via comparator.
 * The overriden equals() method allows for proper remove() functionality
 * when wanting to remove specific points from the PriorityQueue.
 *
 */
class MapVertex implements Comparable<MapVertex> {
    private Point point;
    private Double cost;
    public MapVertex() {
        this.point = new Point();
        this.cost = 0.0;
    }
    public MapVertex(Point p, Double cost) {
        this.point = p;
        this.cost = cost;
    }
    @Override
    public int compareTo(MapVertex mp) {
        return Double.compare(this.getCost(), mp.getCost());
    }
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MapVertex)) {
            return false;
        }
        return ( (this.getPoint().equals(((MapVertex)obj).getPoint()))
                && this.getCost().equals(((MapVertex)obj).getCost()) );
    }
    public Point getPoint() {
        return point;
    }
    public Double getCost() {
        return cost;
    }
}