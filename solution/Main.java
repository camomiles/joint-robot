package solution;

import java.io.IOException;
import java.lang.Math;
import java.util.*;

/**
 * Find path from initial configuration to goal configuration of the joint robot using search algorithms
 *
 * @author Roman Yakobnyuk
 */
public class Main {
    /**
     * Application starting point.
     * Takes two arguments:
     * inputFileName - problem file to load
     * outputFileName - solution file to write to
     *
     * @param argv - arguments array
     */
    public static void main(String[] argv) {
        // Ensure that a filename (or something) was given in argv[0] and argv[1]
        if (argv.length != 2) {
            System.err.println("Usage: java ai-7702.jar inputFileName outputFileName");
            System.exit(1);
        }

        // Create new instance of the problem specification
        ProblemSpec problemSpec = new ProblemSpec();

        // Try to load problem file specified in argument 1
        try {
            problemSpec.loadProblem(argv[0]);

            say("Problem loaded: " + problemSpec.problemLoaded());
            say("Number of joints: " + problemSpec.getJointCount());
            say("Initial configuration: " + problemSpec.getInitialState());
            say("Goal configuration: " + problemSpec.getGoalState());

            int count = 1;
            for (Obstacle obstacle : problemSpec.getObstacles()) {
                say("Obstacle " + count + ":" + obstacle);
                count++;
            }
        } catch (IOException e) {
            say("Error occurred while trying to load problem file " + argv[0] + ":");
            say(e.getLocalizedMessage());
            System.exit(1);
        }

        // Run search for path solution
        search(problemSpec, argv[1]);

        // Set problem to test
        Tester.testAll(problemSpec);
    }

    /** Runs a Rapidly-exploring Random Trees algorithm on given problem specification to generate
     *  collision-free path in configuration space from initial configuration to goal configuration.
     *
     * @param problemSpec - Problem Spec to work on
     * @param filename - filename of the file to output solution to
     */
    private static void search(ProblemSpec problemSpec, String filename) {
        // Check if straight pass between initial and goal configurations is available
        say("Initial state:" + problemSpec.getInitialState());
        say("Goal state:" + problemSpec.getGoalState());
        say("Collision free: " + Tester.isCollisionFreeLine(problemSpec.getInitialState(), problemSpec.getGoalState(), problemSpec.getObstacles()));

        if (Tester.isCollisionFreeLine(problemSpec.getInitialState(), problemSpec.getGoalState(), problemSpec.getObstacles())) {
            say("Straight pass from initial to goal configuration is available.");
            // Build path from initial to goal path
            List<ArmConfig> path = createStraightPath(problemSpec.getInitialState(), problemSpec.getGoalState());
            // Save this path
            problemSpec.setPath(path);
            try {
                // Save solution to file
                problemSpec.saveSolution(filename);
            } catch (IOException e) {
                say("Failed to write solution to file. Error: ");
                say(e.getMessage());
                System.exit(1);
            }
        } else {
            // Create node with initial configuration
            Node initial = new Node(problemSpec.getInitialState());
            // Create node with goal configuration
            Node goal = new Node (problemSpec.getGoalState());
            // Find path tree
            Node path = buildRandomSearchTree(initial, goal, problemSpec);
            // Generate valid step path
            List<ArmConfig> fullPath = generatePath(path, goal);
            try {
                // Save solution
                problemSpec.setPath(fullPath);
                problemSpec.saveSolution(filename);
            } catch (IOException e) {
                say("Failed to write solution to file. Error:");
                say(e.getMessage());
                System.exit(1);
            }
        }
    }

    /**
     * Create list of configurations that represent path from initial configuration to goal configuration.
     * This method does not take obstacles into account, so before calling this method proper collision check for
     * this line segment should be done already.
     *
     * @param initial
     *        Initial configuration, first step of the path returned
     * @param goal
     *        Goal configuration, last step of the path returned
     * @return
     *        List of configurations between initial and goal configurations
     */
    private static List<ArmConfig> createStraightPath(ArmConfig initial, ArmConfig goal) {
        // Create new path array
        List<ArmConfig> path = new ArrayList<ArmConfig>();
        // Add initial configuration as a start node
        path.add(initial);
        // Recursively create path
        createPath(initial, goal, path);
        // Add goal configuration as final node
        path.add(goal);

        return path;
    }

    /**
     * Recursively split distance between initial and goal points until we get points that are valid steps.
     * Add valid steps to the path in order.
     *
     * @param initial initial configuration
     * @param goal goal configuration
     * @param path current path
     *
     * @return path from initial to goal configuration
     */
    private static List<ArmConfig> createPath(ArmConfig initial, ArmConfig goal, List<ArmConfig> path) {
        // Check if distance between initial and goal points is more then valid step
        if (Tester.isValidStep(initial, goal)) {
            // Path is collision free at that point
            path.add(Tester.fraction(0.5, initial, goal));

            return path;
        } else {
            // Split into two parts
            createPath(initial, Tester.fraction(0.5, initial, goal), path);
            createPath(Tester.fraction(0.5, initial, goal), goal, path);

            return path;
        }
    }

    /**
     * Build search tree from initial configuration to goal configuration
     * using Rapidly-exploring Random trees algorithm
     *
     * @param root - (Node) root node to start search from
     * @param goal - (Node) goal node to find path to
     *
     * @return - (Node) root node of the tree that contains path to the goal
     */
    private static Node buildRandomSearchTree(Node root, Node goal, ProblemSpec ps) {
        // found flag
        boolean found = false;

        // Repeat while solution is not found
        while (!found) {
            // If not available, sample random point in the configuration space
            Node random = getRandomNode(ps);

            say("\nPicked random node: " + random.getConfiguration());

            // Traverse existing graph and find element with closest distance to the target node
            Node closest = findClosestNode(root, random);

            say("Found closest point to it: " + closest.getConfiguration());

            // Check that line between closest and target points are collision free
            if (!Tester.hasCollision(random.getConfiguration(), ps.getObstacles()) &&
                    Tester.isCollisionFreeLine(closest.getConfiguration(), random.getConfiguration(), ps.getObstacles())) {

                say("Line from Closest to Random point has no collisions. Adding it to the tree...");

                // Add random node as a child of the closest node
                closest.addChild(random);

                // If so, check that line between random and goal configuration
                if (Tester.isCollisionFreeLine(random.getConfiguration(), goal.getConfiguration(), ps.getObstacles())) {
                    say("Line from Random to the Goal point has no collisions. Solution found.");
                    // Found valid path to the goal
                    found = true;
                    // Add goal node as a child of the random node
                    random.addChild(goal);
                }

            } else {
                // If it is not collision free, find closest collision free point
                say("Sample has collisions. Looking for closest collision-free node...");

                // Find closest collision free node
                Node collisionFreeNode = findClosetCollisionFreeNode(closest, random, ps.getObstacles());

                if (Tester.isCollisionFreeLine(closest.getConfiguration(), collisionFreeNode.getConfiguration(), ps.getObstacles())) {

                    // Check if closest point is the most closest collision free point
                    // in other words, it is the closest possible point to the obstacle already
                    if (closest.getConfiguration() == collisionFreeNode.getConfiguration()) {
                        say("Closest point is already on the path tree.");
                    } else {
                        say("Found closest collision-free node: " + collisionFreeNode.getConfiguration());
                        say("Adding it to the path tree...");

                        closest.addChild(collisionFreeNode);
                    }
                }
            }
        }

        return root;
    }

    /**
     *
     * @param ps - problem specification
     * @return random configuration node for number of joints from problem spec using
     */
    private static Node getRandomNode(ProblemSpec ps) {
        // Configuration string
        String config = "";

        // Build random configuration string
        for (int i = 0; i < 2 + ps.getJointCount(); i++) {
            config = config + Math.random();

            if (i < 1 + ps.getJointCount()) {
                config += " ";
            }
        }

        ArmConfig testConfig = new ArmConfig(config);

        return new Node(testConfig);
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

    /**
     * Recursively finds closest node to the target node
     *
     * @param root - current working element
     * @param target - target element to check against
     * @param closest - current closest element
     * @return - closest element
     */
    private static Node findClosestNode(Node root, Node target, Node closest) {
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

    /**
     * Finds a closest to the goal collision free point on the path
     * between initial and goal nodes by iterating over
     * a path of basic steps from initial to goal configuration
     * until it finds configuration that collides with obstacles
     *
     * @param initial - initial node
     * @param goal - goal node
     * @param obstacles - list of obstacles to check against
     *
     * @return - closest collision free point to the goal or initial goal in case of error
     */
    private static Node findClosetCollisionFreeNode(Node initial, Node goal, List<Obstacle> obstacles) {
        // check initial configuration for collisions
        if (Tester.hasCollision(initial.getConfiguration(), obstacles)) {
            return initial;
        }

        // create basic step path between configurations
        List<ArmConfig> path = createStraightPath(initial.getConfiguration(), goal.getConfiguration());

        // Assume first configuration is collision free (we checked before)
        ArmConfig closestFreeConf = initial.getConfiguration();

        // iterate over the path until first collision node
        for (ArmConfig conf : path) {
            if (Tester.hasCollision(conf, obstacles)) {
                // return first node before collision
                return new Node(closestFreeConf);
            } else {
                closestFreeConf = conf;
            }
        }

        return new Node(closestFreeConf);
    }

    /**
     * Given a root node of the tree that contains a goal node, and a goal node that contains a path to the
     * root node through its parents - generate all intermediate valid steps and make a path from initial to goal
     * node in the right order
     *
     * @param path - root node of the path
     * @param goal - goal node of the path
     * @return
     *         List of configurations that represent basic steps from initial to goal configuration through intermediate
     *         points in the tree.
     */
    // Find shortest path from root to goal node
    private static List<ArmConfig> generatePath(Node path, Node goal) {
        // Stack of nodes that make up a path
        Deque<Node> pathNodes = new ArrayDeque<Node>();
        // Point temp pointer at the goal node
        Node tempNode = goal;
        // Push it into stack
        pathNodes.push(goal);
        // while it has parents, push them to the stack
        while (tempNode.getParent() != null) {
            tempNode = tempNode.getParent();
            // push it to the stack
            pathNodes.push(tempNode);
        }

        // Create new configuration path out of nodes
        List<ArmConfig> fullPath = new ArrayList<ArmConfig>();

        // Pop first two nodes off the stack
        Node start = pathNodes.pop();
        Node next = pathNodes.pop();

        boolean flag = false;
        while (!flag) {
            // Create path for the line segment between two nodes
            List<ArmConfig> segmentPath = createStraightPath(start.getConfiguration(), next.getConfiguration());
            // Add segment path to the full path
            fullPath.addAll(segmentPath);
            // if array is empty, finish
            if (pathNodes.isEmpty()) {
                flag = true;
            } else {
                // Else pop next one
                start = next;
                next = pathNodes.pop();
            }
        }

        return fullPath;
    }

    // Log out text line to stdout
    private static void say(String line) {
        System.out.println(line);
    }
}