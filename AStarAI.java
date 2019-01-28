import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.PriorityQueue;
/// A sample AI that takes a very suboptimal path.
/**
 * This is a sample AI that moves as far horizontally as necessary to reach the target,
 * then as far vertically as necessary to reach the target.  It is intended primarily as
 * a demonstration of the various pieces of the program.
 * 
 * @author Cameron Lee
 * @author Marshall Fan
 * @author Scott Madera
 */
public class AStarAI implements AIModule {
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map) {
        double testBest = heuristicCost(map.getStartPoint(), map.getEndPoint(), map, -255.0);
        double test1 = heuristicCost(map.getStartPoint(), map.getEndPoint(), map, 0);
        double test2 = heuristicCost(new Point(251, 251), map.getEndPoint(), map, -255.0);
        double test3 = heuristicCost(new Point(249, 249), map.getEndPoint(), map, -254.0);
        double test4 = heuristicCost(new Point(444, 443), map.getEndPoint(), map, 40);
        double test5 = heuristicCost(new Point(444, 443), map.getEndPoint(), map, 43);

        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        Point currentPoint = map.getStartPoint();
        // path.add(new Point(currentPoint));

        // debug to switch between Bizarro and Exp easily; remove in future
        // useBizarro: if false, use exponential cost. if true, use bizarro cost.
        // Obviously must still comment and uncomment whatever cost function
        // we want to use inside TerrainMap.java.
        // boolean useBizarro = false;

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
        // HeuristicNode hNode = new HeuristicNode(255.0);

        // initialize collections to ensure loop correctness
        distances.put(currentPoint, 0.0);
        open.add(new MapNode(currentPoint, heuristicCost(currentPoint, map.getEndPoint(), map/*, hNode, useBizarro*/)));
        closed.put(currentPoint, true);

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
                    // f(n) + g(n) + h(n)
                    distances.put(neighbor, minCost);
                    minCost += heuristicCost(neighbor, map.getEndPoint(), map/*, hNode, useBizarro*/);
                    paths.put(neighbor, currentPoint);
                    open.add(new MapNode(neighbor, minCost)); // adding neighbors to frontier; O(log n)
                }
            } // end of neighbors loop
            closed.put(currentPoint, true);

            // we update HeuristicNode height and count here
            // as we travel along the path.
            // Note: somehow adding the below if-else
            // INCREASED the costs very slightly AND nodes uncovered by 4000...
            // try commenting and uncommenting it out to compare
/*            if (useBizarro) { // update hNode members based on Bizarro scheme
                hNode.height = Math.floor(hNode.height / 2);
            }
            else { // update hNode members based on Exponent scheme
                if (hNode.height - hNode.count >= 0) {
                    //hNode.height = hNode.height - hNode.count;
                } else {
                    hNode.height = 0; // exhausted height, cannot go below height = 0 (range is 0 to 255)
                }
                //++hNode.count;
            }*/
        } // end of Dijkstra while loop
        while ((map.getStartPoint().x != currentPoint.x) || (map.getStartPoint().y != currentPoint.y)) {
            path.add(0, currentPoint);
            currentPoint = paths.get(currentPoint);
        }
        path.add(0, currentPoint); // re-add start point

        // We're done!  Hand it back.
        return path;
    } // end of createPath()

    // tester with additional heightDiff input, will remove once code is finalized
    public double heuristicCost(Point currentPoint, Point endpoint, TerrainMap map, double heightDiffToGoal/*, HeuristicNode hNode, boolean useBizarro*/) {
        // First: compute min. # of moves from currentPoint to End,
        // assuming that height costs are ignored.
        int minNumOfMoves = getMinNumOfMoves(currentPoint, endpoint);
        // double heightDiffToGoal = Math.abs(map.getTile(currentPoint) - map.getTile(endpoint));
        return estimateIdealCost(minNumOfMoves, heightDiffToGoal);
    }

    public double heuristicCost(Point currentPoint, Point endpoint, TerrainMap map/*, HeuristicNode hNode, boolean useBizarro*/) {
        // First: compute min. # of moves from currentPoint to End,
        // assuming that height costs are ignored.

        int minNumOfMoves = getMinNumOfMoves(currentPoint, endpoint);
        double heightDiffToGoal = map.getTile(endpoint) - map.getTile(currentPoint);
        return estimateIdealCost(minNumOfMoves, heightDiffToGoal);
        // return getLeastCostSumOfExponents(minNumOfMoves);

        //if (!useBizarro) {
/*            if (minNumOfMoves > 255) {
                int downwardMoves = minNumOfMoves - 255;
                return (minNumOfMoves * Math.pow(2.0, - 1.0)) + downwardMoves;
            } else {
                return minNumOfMoves * Math.pow(2.0, -1.0);
            }*/
        // }

        // Check the case where there is no longer
        // any downwards movement. If so, then treat
        // the remaining path to goal as a flat path,
        // with each move costing (2^0 = 1) per tile.
        /*if (hNode.height == 0) {
            return (double)minNumOfMoves;
        }*/

        // Next, calculate underestimated cost distance
        // from currentPoint to EndPoint, based on assumed
        // best-case height differences (2^some negative #)
        /*if (useBizarro) {
            return getDistanceToEndBizarro(minNumOfMoves, hNode);
        } else {
            return getDistanceToEndExp(minNumOfMoves, hNode);
        }*/
    }

    public double estimateIdealCost(int minSpaces, double heightDiffToGoal) {
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

    /**
     * Attempts to calculate an underestimate cost
     * from the given point to the EndPoint. Uses a
     * monotonically-decreasing scheme where costs are
     * accumulated until number of moves away from the goal
     * is zero, or if no more negative exponents can be produced.
     * Formula used:
     * summation(2^k) + number of remaining moves
     * where k is an increasing negative number.
     */
    public double getDistanceToEndExp(int minNumOfMoves, HeuristicNode hNode) {
        double sumCost = 0.0;
        double prevHeight = hNode.height;
        // we want a local copy of hNode.count. In the main dijkstra while loop
        // is when the actual value of hNode.count will be incremented.
        int count = hNode.count;
        double i = count;
        while (minNumOfMoves != 0) {
            if (prevHeight - i < 0) { // out of height; cannot go beyond 0
                sumCost += Math.pow(2.0, 0.0 - prevHeight); // "flatten" the remainder
                --minNumOfMoves; // "flattening" expended a step.
                sumCost += minNumOfMoves; // tally up remaining move costs (2^0 per move)
                break;
            }
            double tempCost = Math.pow(2.0, (prevHeight - i) - prevHeight);
            sumCost += tempCost;
            prevHeight = (prevHeight - i);
            ++count;
            i = count;
            --minNumOfMoves;
        }
        return sumCost;
    }

    public double getDistanceToEndBizarro(int minNumOfMoves, HeuristicNode hNode) {
        double sumCost = 0.0;
        double prevHeight = hNode.height;
        double half = hNode.height;
        while (minNumOfMoves != 0) {
            if (half == 0) {
                sumCost += minNumOfMoves; // tally up remaining move costs (2^0 per move)
                break;
            }
            // half = Math.floor((half / 2));
            half = half / 2.0;
            double tempCost = Math.pow(2.0, half - prevHeight);
            prevHeight = half;
            sumCost += tempCost;
            --minNumOfMoves;
        }
        return sumCost;
    }

    public double getDistanceToEndBizarro(int minNumOfMoves) {
        double sumCost = 0.0;
        double prevHeight = 255.0;
        double half = 255.0;
        while (minNumOfMoves != 0) {
            if (half == 0) {
                sumCost += minNumOfMoves; // tally up remaining move costs (2^0 per move)
                break;
            }
            // half = Math.floor((half / 2));
            half = half / 2.0;
            double tempCost = Math.pow(2.0, half - prevHeight);
            prevHeight = half;
            sumCost += tempCost;
            --minNumOfMoves;
        }
        return sumCost;
    }
    public double getLeastCostSumOfExponents(int minNumOfMoves) {
        double sumCost;
        double avgHeightDiff = 255.0 / (double) minNumOfMoves;
        sumCost = Math.pow(2.0, -avgHeightDiff) * minNumOfMoves;
        if (minNumOfMoves > 255) {
            int numFlatMoves = minNumOfMoves - 255;
            sumCost += numFlatMoves;
        }
        return sumCost;
    }

} // end of AStarClass

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

// cheaty way to perform pass by reference
class HeuristicNode {
    public double height;
    public int count;
    public HeuristicNode(double height) {
        this.height = height;
        this.count = 1;
    }
}