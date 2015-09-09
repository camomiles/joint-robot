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
     * Save link to the Problem Spec
     **/
    private static ProblemSpec problemSpec = new ProblemSpec();

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
            System.err.println("Usage: java -jar ai-7702.jar inputFileName outputFileName");
            System.exit(1);
        }

        // Create new instance of the problem specification
        problemSpec = new ProblemSpec();

        // Try to load problem file specified in argument 1
        try {
            problemSpec.loadProblem(argv[0]);

            say("Problem loaded: " + problemSpec.problemLoaded());
            say("Number of joints: " + problemSpec.getJointCount());
            say("Initial configuration: " + problemSpec.getInitialState());
            say("Goal configuration: " + problemSpec.getGoalState());

            int count = 1;
            for (Obstacle obstacle : problemSpec.getObstacles()) {
                say("Obstacle " + count + ": " + obstacle);
                count++;
            }
        } catch (IOException e) {
            say("Error occurred while trying to load problem file " + argv[0] + ":");
            say(e.getLocalizedMessage());
            System.exit(1);
        }

        say("\nSearching...");
        // Run search for path solution
        long startTime = System.currentTimeMillis();

        search();

        long stopTime = System.currentTimeMillis();
        long elapsedTime = stopTime - startTime;

        System.out.println("Time taken: " + elapsedTime / 1000 + " seconds.");

        // Set problem to test
        // Tester.testAll(problemSpec);

        try {
            // Save solution to file
            problemSpec.saveSolution(argv[1]);
        } catch (IOException e) {
            say("Failed to write solution to file. Error: ");
            say(e.getMessage());
            System.exit(1);
        }
    }

    /**
     * Runs a Rapidly-exploring Random Trees algorithm on given problem specification to generate
     * collision-free path in configuration space from initial configuration to goal configuration.
     */
    private static void search() {
        // Check if straight pass between initial and goal configurations is available
        if (Tester.isCollisionFreeLine(problemSpec.getInitialState(), problemSpec.getGoalState(), problemSpec.getObstacles())) {
            say("Straight pass from initial to goal configuration is available.");
            // Build path from initial to goal path
            List<ArmConfig> path = Tester.createStraightPath(problemSpec.getInitialState(), problemSpec.getGoalState());
            // Save this path
            problemSpec.setPath(path);
        } else {
            // Create node with initial configuration
            Node initial = new Node(problemSpec.getInitialState());
            // Create node with goal configuration
            Node goal = new Node(problemSpec.getGoalState());
            // Find path tree
            try {
                Node path = buildRandomSearchTree(initial, goal);
                // Generate valid step path
                List<ArmConfig> fullPath = generatePath(path, goal);
                // Set solution path in the problem spec
                problemSpec.setPath(fullPath);
            } catch (Exception e) {
                say("Failed to solve a problem. Error: ");
                e.printStackTrace();
                System.exit(1);
            }

        }
    }

    /**
     * Build search tree from initial configuration to goal configuration
     * using Rapidly-exploring Random trees algorithm
     *
     * @param root - (Node) root node to start search from
     * @param goal - (Node) goal node to find path to
     * @return - (Node) root node of the tree that contains path to the goal
     */
    private static Node buildRandomSearchTree(Node root, Node goal) {

        long timeRandomNodes = 0;
        long timeClosestNode = 0;
        long timeClosestCollisionFreeNode = 0;
        long timeCheckingIfLineIsFree = 0;
        // found flag
        boolean found = false;
        // Repeat while solution is not found
        while (!found) {
            // Sample random point in the configuration space
            long startTime = System.currentTimeMillis();

            Node random = getRandomNode();

            long stopTime = System.currentTimeMillis();
            long elapsedTime = stopTime - startTime;
            timeRandomNodes += elapsedTime;

            startTime = System.currentTimeMillis();
            // Traverse existing graph and find element with closest distance to the target node
            Node closest = findClosestNode(root, random);
            stopTime = System.currentTimeMillis();
            elapsedTime = stopTime - startTime;
            timeClosestNode += elapsedTime;

            // Check that line between closest and target points are collision free
            startTime = System.currentTimeMillis();
            if (!Tester.hasCollision(random.getConfiguration(), problemSpec.getObstacles()) &&
                    Tester.isCollisionFreeLine(closest.getConfiguration(), random.getConfiguration(), problemSpec.getObstacles())) {
                stopTime = System.currentTimeMillis();
                elapsedTime = stopTime - startTime;
                timeCheckingIfLineIsFree += elapsedTime;

                // Add random node as a child of the closest node
                closest.addChild(random);

                // If so, check that line between random and goal configuration
                if (Tester.isCollisionFreeLine(random.getConfiguration(), goal.getConfiguration(), problemSpec.getObstacles())) {
                    say("Solution found.");
                    say("Time spent taking samples: " + timeRandomNodes + " milliseconds.");
                    say("Time spent looking for closest node: " + timeClosestNode + " milliseconds.");
                    say("Time spent looking for collision free node: " + timeClosestCollisionFreeNode + " milliseconds.");
                    say("Time spent checking that line is collision free: " + timeCheckingIfLineIsFree + " milliseconds.");
                    // Found valid path to the goal
                    found = true;
                    // Add goal node as a child of the random node
                    random.addChild(goal);
                }
            } else {
                // If it is not collision free, find closest collision free point
                // Find closest collision free node
                startTime = System.currentTimeMillis();

                Node collisionFreeNode = findClosestCollisionFreeNode(closest, random, problemSpec.getObstacles());

                stopTime = System.currentTimeMillis();
                elapsedTime = stopTime - startTime;
                timeClosestCollisionFreeNode += elapsedTime;

                if (closest.getConfiguration().maxDistance(collisionFreeNode.getConfiguration()) > 0.0) {
                    closest.addChild(collisionFreeNode);
                }
            }
        }

        return root;
    }

    /**
     * @return random configuration node for number of joints from problem spec using
     */
    private static Node getRandomNode() {
        // Configuration string
        String config = "";

        // Build random configuration string
        for (int i = 0; i < 2 + problemSpec.getJointCount(); i++) {
            if (i < 2) {
                config = config + Math.random();
            } else {
                double randomSign = Math.random();

                double randomAngle = Math.random() * ((150 * Math.PI) / 180);

                if (randomSign >= 0.5) {
                    config = config + randomAngle;
                } else {
                    randomAngle = -randomAngle;
                    config = config + randomAngle;
                }
            }

            if (i < 1 + problemSpec.getJointCount()) {
                config += " ";
            }
        }

        // Bias search towards the goal
        double chance = Math.random();

        ArmConfig randomConfig;

        if (chance < 0.01) {
            randomConfig = problemSpec.getGoalState();
        } else {
            randomConfig = new ArmConfig(config);
        }

        // Check new random config for errors
        if (Tester.fitsBounds(randomConfig) && !Tester.hasSelfCollision(randomConfig) && Tester.hasValidJointAngles(randomConfig)) {
            return new Node(randomConfig);
        } else {
            return getRandomNode();
        }
    }

    /**
     * Find closest node of the tree to the target
     *
     * @param root   - [Node] - root element of the tree to traverse
     * @param target - [Node] - target element to check if path was found
     * @return - [Node] closest node to the target in the given tree
     */
    private static Node findClosestNode(Node root, Node target) {
        return findClosestNode(root, target, root);
    }

    /**
     * Recursively finds closest node to the target node
     *
     * @param root    - current working element
     * @param target  - target element to check against
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
     * @param initial   - initial node
     * @param goal      - goal node
     * @param obstacles - list of obstacles to check against
     * @return - closest collision free point to the goal or initial goal in case of error
     */
    private static Node findClosestCollisionFreeNode(Node initial, Node goal, List<Obstacle> obstacles) {
        if (Tester.isCollisionFreeLine(initial.getConfiguration(), goal.getConfiguration(), obstacles)) {
            return goal;
        } else {
            if (Tester.isValidStep(initial.getConfiguration(), goal.getConfiguration())) {
                return initial;
            }

            return findClosestCollisionFreeNode(initial, new Node(Tester.fraction(0.5, initial.getConfiguration(), goal.getConfiguration())), obstacles);
        }
    }

    /**
     * Given a root node of the tree that contains a goal node, and a goal node that contains a path to the
     * root node through its parents - generate all intermediate valid steps and make a path from initial to goal
     * node in the right order
     *
     * @param path - root node of the path
     * @param goal - goal node of the path
     * @return List of configurations that represent basic steps from initial to goal configuration through intermediate
     * points in the tree.
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
            List<ArmConfig> segmentPath = Tester.createStraightPath(start.getConfiguration(), next.getConfiguration());
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