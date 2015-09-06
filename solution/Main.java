package solution;

import sun.jvm.hotspot.jdi.DoubleTypeImpl;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

/**
 * Find path from initial configuration to goal configuration of the joint robot using search algorithms
 *
 * @author Roman Yakobnyuk
 */
public class Main {

    /** The default value for maximum error */
    public static final double DEFAULT_MAX_ERROR = 1e-5;
    /** The maximum distance the base can move in one step */
    public static final double MAX_BASE_STEP = 0.001;
    /** The maximum change in joint angle in one step (radians) */
    public static final double MAX_JOINT_STEP = 0.1 * Math.PI / 180.0;

    public static void main(String[] argv) {

        // Ensure that a filename (or something) was given in argv[0]
        if (argv.length < 1 || argv.length > 2) {
            System.err.println("Usage: java ai-7702.jar inputFileName [outputFileName]");
            System.exit(1);
        }

        // Create new instance of the problem specification
        ProblemSpec problemSpec = new ProblemSpec();

        // Try to load problem file specified in argument 1
        try {
            problemSpec.loadProblem(argv[0]);
        } catch (IOException e) {
            say("Error occurred while trying to load problem file " + argv[0] + ":");
            say(e.getLocalizedMessage());
            System.exit(1);
        }

        say("Problem loaded: " + problemSpec.problemLoaded());
        say("Number of joints: " + problemSpec.getJointCount());
        say("Initial configuration: " + problemSpec.getInitialState());
        say("Goal configuration: " + problemSpec.getGoalState());

        int count = 1;
        for (Obstacle obstacle : problemSpec.getObstacles()) {
            say("Obstacle " + count + ":" + obstacle);
            count++;
        }

        // Construct graph
        // HashMap<String, Node> graph = new HashMap<String, Node>();

        Node initial = new Node (problemSpec.getInitialState());
        Node goal = new Node (problemSpec.getGoalState());

        Node path = buildRandomSearchTree(initial, goal, problemSpec);

        generatePath(path, goal);
    }

    // Find shortest path from root to goal node
    private static List<ArmConfig> generatePath(Node path, Node goal) {

        List<ArmConfig> verticies = new ArrayList<ArmConfig>();

        return verticies;
    }

    /**
     *
     * Build search tree from initial configuration to goal configuration
     * using Rapidly-exploring Random trees algorithm
     *
     * @param root - [Node] root node to start search from
     * @param goal - [Node] goal node to find path to
     *
     * @return - [Node] root node of the tree that contains path to the goal
     */
    private static Node buildRandomSearchTree(Node root, Node goal, ProblemSpec ps) {

        say("\nBuild tree from " + root.getConfiguration() + " to " + goal.getConfiguration());

        boolean found = false;

        // Check if path from initial to goal configuration is available
        // If available, add goal as a child of initial node and return

        while (!found) {
            // If not available, sample random point in the configuration space
            Node random = getRandomNode(ps);

            say("\nPicked random node: " + random.getConfiguration());

            // Traverse existing graph and find element with closest distance to the target node
            Node closest = findClosestNode(root, random);

            say("Found closest point to it: " + closest.getConfiguration());

            // Check that line between closest and target points are collision free
            if (!hasCollision(random.getConfiguration(), ps.getObstacles()) && isCollisionFreeLine(closest.getConfiguration(), random.getConfiguration(), ps.getObstacles())) {
                say("Line to random point has no collisions");

                // Add random node as a child of the closest node
                closest.addChild(random);

                // If so, check that line between random and goal configuration
                if (isCollisionFreeLine(random.getConfiguration(), goal.getConfiguration(), ps.getObstacles())) {
                    say("Line to the goal point has no collisions");
                    // Found valid path to the goal
                    found = true;
                    // Add goal node as a child of the random node
                    random.addChild(goal);
                }

            } else {
                // If it is not collision free, find closest collision free point
                say("Sample has collisions, putting closest point with no collisions on the tree");
            }
        }

        return root;
    }

    /**
     * Find closest node of the tree to the target
     *
     * @param root - [Node] - root element of the tree to traverse
     * @param target - [Node] - target element to check if path was found
     *
     * @return - [Node] closest node to the target in the given tree
     */
    private static Node findClosestNode(Node root, Node target) {
        return findClosestNode(root, target, root);
    }

    private static Node findClosestNode(Node root, Node target, Node closest) {
        //say("Distance from " + root.getConfiguration() + " to " + target.getConfiguration() + " is " + root.getConfiguration().maxDistance(target.getConfiguration()));

        // if root node is closer then closest distance, set is closest
        if (root.getConfiguration().maxDistance(target.getConfiguration()) < closest.getConfiguration().maxDistance(target.getConfiguration())) {
            closest = root;
        }

        // call function on every child
        for (Node child : root.getChildren()) {
            closest = findClosestNode(child, target, closest);
        }

        return closest;
    }

    private static Node getRandomNode(ProblemSpec ps) {
        double xvalue = Math.random();
        double yvalue = Math.random();

        String confString = "" + xvalue + " " + yvalue;

        ArmConfig testConfig = new ArmConfig(confString);

        return new Node(testConfig);
    }

    // Search for valid path between initial and goal configuration
//    private static void search(ProblemSpec problemSpec) {
//        // Graph that algorithm is building
//        HashMap<String, Node> graph = new HashMap<String, Node>();
//
//        // Create node with initial configuration
//        Node initial = new Node();
//        initial.label = "" + graph.size();
//        initial.configuration = problemSpec.getInitialState();
//        // Put in in the graph
//        graph.put(initial.label, initial);
//
//        // Check if straight pass between initial and goal configurations is available
//        if (isCollisionFreeLine(initial.configuration, problemSpec.getGoalState(), problemSpec.getObstacles())) {
//
//            say("Straight pass from initial to goal configuration is available");
//
//            // Build path from initial to goal path
//            try {
//                List<ArmConfig> path = createStraightPath(problemSpec.getInitialState(), problemSpec.getGoalState());
//
//                // Save this path to file
//                saveSolution("testfile.txt", path);
//
//            } catch (Exception e) {
//                say ("Failed to generate a solution path:");
//                say (e.getMessage());
//                System.exit(1);
//            }
//
//
//        } else {
//            say("There are obstacles");
//        }
//    }


    /**
     * Saves the current solution to a solution text file.
     *
     * @param filename
     *            the path of the text file to save to.
     * @throws IOException
     *             if the text file doesn't exist or doesn't meet the assignment
     *             specifications.
     */
    public static void saveSolution(String filename, List<ArmConfig> path) throws IOException {

        String ls = System.getProperty("line.separator");
        FileWriter output = new FileWriter(filename);
        output.write(String.format("%d %s", path.size() - 1, ls));
        for (ArmConfig cfg : path) {
            output.write(cfg + ls);
        }
        output.close();
    }


    /**
     * Create list of configurations that represent path from initial configuration to goal configuration.
     *
     * @param initial
     *        Initial configuration, first step of the path returned
     * @param goal
     *        Goal configuration, last step of the path returned
     * @return
     *        List of configurations between initial and goal configurations
     */
    private static List<ArmConfig> createStraightPath(ArmConfig initial, ArmConfig goal) throws Exception {

        // Create new path array
        List<ArmConfig> path = new ArrayList<ArmConfig>();

        // Add initial configuration as a start node
        path.add(initial);
        // Recursively create path
        createPath(initial, goal, path);
        // Add goal configuration as final node
        path.add(goal);


        // if path does not have invalid steps, return it. Otherwise error.
        if (getInvalidSteps(path).size() == 0) {

            return path;

        } else {
            throw new Exception("Failed to create valid path.");
        }
    }


    /**
     * Returns whether the given configs are collision free and straight line between them is collision-free.
     *
     * @param inital
     *            intial configuration to test
     * @param goal
     *            goal configuration to test
     * @param obstacles
     *            obstacles to check against
     *
     * @return (bool) whether the line between given configurations collides with any of the given
     *         obstacles.
     */
    private static boolean createPath(ArmConfig initial, ArmConfig goal, List<ArmConfig> path) {

        // Check if distance between initial and goal points is more then valid step
        if (isValidStep(initial, goal)) {
            // say("Valid step found - from " + initial + " to " + goal);
            // Path is collision free at that point
            path.add(fraction(0.5, initial, goal));

            return true;
        } else {

            createPath(initial, fraction(0.5, initial, goal), path);
            createPath(fraction(0.5, initial, goal), goal, path);

            return true;
        }

    }


    /**
     * Returns the preceding path indices of any invalid steps.
     *
     * @return the preceding path indices of any invalid steps.
     */
    private static List<Integer> getInvalidSteps(List<ArmConfig> path) {
        List<Integer> badSteps = new ArrayList<Integer>();
        ArmConfig state = path.get(0);

        for (int i = 1; i < path.size(); i++) {
            ArmConfig nextState = path.get(i);
            if (!isValidStep(state, nextState)) {
                badSteps.add(i - 1);
            }
            state = nextState;
        }

        return badSteps;
    }


    /**
     * Returns whether the given configs are collision free and straight line between them is collision-free.
     *
     * @param inital
     *            intial configuration to test
     * @param goal
     *            goal configuration to test
     * @param obstacles
     *            obstacles to check against
     *
     * @return (bool) whether the line between given configurations collides with any of the given
     *         obstacles.
     */
    private static boolean isCollisionFreeLine(ArmConfig initial, ArmConfig goal, List<Obstacle> obstacles) {

        //say("\nChecking initial " + initial + " and goal " + goal + " configurations");

        // Check if initial configuration collides with any of the obstacles
        //say("Initial " + initial + " has collisions: " + hasCollision(initial, obstacles));
        if (hasCollision(initial, obstacles)) {
            return false;
        }

        // Check if goal configuration is collision free
        //say("Goal " + goal + " has collisions: " + hasCollision(goal, obstacles));
        if (hasCollision(goal, obstacles)) {
            return false;
        }

        // Check if distance between initial and goal points is more then valid step
        if (isValidStep(initial, goal)) {
            // say("Valid step found - from " + initial + " to " + goal);
            // Path is collision free at that point
            return true;

        } else {

            return isCollisionFreeLine(initial, fraction(0.5, initial, goal), obstacles) && isCollisionFreeLine(fraction(0.5, initial, goal), goal, obstacles);

        }

    }


    /**
     * Returns a configuration that is on a specified fraction away on the line
     * between initial and goal configuration.
     * E.g. for values 0.5;0.5 and 0.5;0.9 it would give 0.5;0.7 point if fraction is 0.5
     *
     * @param fraction
     *        fraction of the line between initial and goal configuration to put a point on
     *
     * @param initial
     *        initial arm configuration
     *
     * @param goal
     *        goal arm configuration
     *
     * @return
     *        configuration that is fraction away from initial configuration.
     *        Returned configuration would have same joint angles as provided initial configuration
     */
    private static ArmConfig fraction(double fraction, ArmConfig initial, ArmConfig goal) {

        Point2D initialBase =  initial.getBase();
        Point2D goalBase = goal.getBase();

        // Calculate point that is fraction away on the line
        Point2D newConfig = new Point2D.Double(initialBase.getX() + fraction * (goalBase.getX() - initialBase.getX()), initialBase.getY() + fraction * (goalBase.getY() - initialBase.getY()));

        return new ArmConfig(newConfig, initial.getJointAngles());
    }


    /**
     * Returns whether the step from s0 to s1 is a valid primitive step.
     *
     * @param cfg0
     *            A configuration.
     * @param cfg1
     *            Another configuration.
     * @return whether the step from s0 to s1 is a valid primitive step.
     */
    public static boolean isValidStep(ArmConfig cfg0, ArmConfig cfg1) {
        if (cfg0.getJointCount() != cfg1.getJointCount()) {
            return false;
        } else if (cfg0.maxAngleDiff(cfg1) > MAX_JOINT_STEP + DEFAULT_MAX_ERROR) {
            return false;
        } else if (cfg0.getBase().distance(cfg1.getBase()) > MAX_BASE_STEP + DEFAULT_MAX_ERROR) {
            return false;
        }
        return true;
    }

    /**
     * Returns whether the given config collides with any of the given
     * obstacles.
     *
     * @param cfg
     *            the configuration to test.
     * @param obstacles
     *            the obstacles to test against.
     * @return whether the given config collides with any of the given
     *         obstacles.
     */
    public static boolean hasCollision(ArmConfig cfg, List<Obstacle> obstacles) {
        for (Obstacle o : obstacles) {
            if (hasCollision(cfg, o)) {
                return true;
            }
        }

        return false;
    }

    /**
     * Returns whether the given config collides with the given obstacle.
     *
     * @param cfg
     *            the configuration to test.
     * @param o
     *            the obstacle to test against.
     * @return whether the given config collides with the given obstacle.
     */
    public static boolean hasCollision(ArmConfig cfg, Obstacle o) {
        Rectangle2D lenientRect = grow(o.getRect(), -DEFAULT_MAX_ERROR);
        List<Line2D> links = cfg.getLinks();
        // If any of the links intersect obstacle - return true
        for (Line2D link : links) {
            if (link.intersects(lenientRect)) {
                return true;
            }
        }

        // If obstacle contains configuration base inside it - return true
        if (lenientRect.contains(cfg.getBase().getX(), cfg.getBase().getY())) {
            return true;
        }

        return false;
    }

    /**
     * Creates a new Rectangle2D that is grown by delta in each direction
     * compared to the given Rectangle2D.
     *
     * @param rect
     *            the Rectangle2D to expand.
     * @param delta
     *            the amount to expand by.
     * @return a Rectangle2D expanded by delta in each direction.
     */
    public static Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
                rect.getWidth() + delta * 2, rect.getHeight() + delta * 2);
    }

    // Log out text line to stdout
    public static void say(String line) {
        System.out.println(line);
    }
}