package solution;

import java.awt.geom.*;
import java.util.*;

public class Tester {
    /** The maximum distance the base can move in one step */
    public static final double MAX_BASE_STEP = 0.001;
    /** The maximum change in joint angle in one step (radians) */
    public static final double MAX_JOINT_STEP = 0.1 * Math.PI / 180.0;
    /** Length of each link */
    public static final double LINK_LENGTH = 0.05;
    /** Minimum joint angle in radians */
    public static final double MIN_JOINT_ANGLE = -150.0 * Math.PI / 180.0;
    /** Maximum joint angle in radians */
    public static final double MAX_JOINT_ANGLE = 150 * Math.PI / 180;
    /** The workspace bounds */
    public static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
    /** The default value for maximum error */
    public static final double DEFAULT_MAX_ERROR = 1e-5;
    /** Remembers the specifications of the problem. */
    private static ProblemSpec ps;
    /** The workspace bounds, with allowable error. */
    private static Rectangle2D lenientBounds = grow(BOUNDS, DEFAULT_MAX_ERROR);

    /**
     * Set problem spec for testing
     *
     * @param problemSpec - problem spec
     */
    public static void setProblemSpec(ProblemSpec problemSpec) { ps = problemSpec; }


    /**
     * Returns whether the given configs are collision free and straight line between them is collision-free.
     *
     * @param initial (ArmConfig) initial configuration to test
     * @param goal (ArmConfig) goal configuration to test
     * @param obstacles (List<Obstacle>) obstacles to check against
     *
     * @return (bool) whether the line between given configurations collides with any of the given
     *         obstacles.
     */
    public static boolean isCollisionFreeLine(ArmConfig initial, ArmConfig goal, List<Obstacle> obstacles) {
        // Check if initial configuration collides with any of the obstacles
        if (hasCollision(initial, obstacles)) { return false; }

        // Check if goal configuration collides with any of the obstacles
        if (hasCollision(goal, obstacles)) { return false; }

        // If path between initial and goal is valid step, return true.
        // otherwise split in distance in half and compare
        ArmConfig half = fraction(0.5, initial, goal);

        return isValidStep(initial, goal) ||
                isCollisionFreeLine(initial, half, obstacles)  && isCollisionFreeLine(half, goal, obstacles);
    }

    /**
     * Returns a configuration that is on a specified fraction away on the line
     * between initial and goal configuration.
     * E.g. for values 0.5;0.5 and 0.5;0.9 it would give 0.5;0.7 point if fraction is 0.5
     *
     * @param fraction (double) fraction of the line between initial and goal configuration to put a point on
     *
     * @param initial (ArmConfig) initial arm configuration
     *
     * @param goal (ArmConfig) goal arm configuration
     *
     * @return (ArmConfig)
     *        configuration that is fraction away from initial configuration.
     *        Returned configuration would have same joint angles as provided initial configuration
     */
    public static ArmConfig fraction(double fraction, ArmConfig initial, ArmConfig goal) {
        Point2D initialBase =  initial.getBase();
        Point2D goalBase = goal.getBase();

        // Calculate point that is fraction away on the line
        Point2D newConfig = new Point2D.Double(initialBase.getX() + fraction * (goalBase.getX() - initialBase.getX()), initialBase.getY() + fraction * (goalBase.getY() - initialBase.getY()));

        return new ArmConfig(newConfig, initial.getJointAngles());
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

    /**
     * Checks that the first configuration in the solution path is the initial
     * configuration.
     */
    public static boolean testInitialFirst() {
        if (!hasInitialFirst()) {
            System.out.println("FAILED: "
                    + "Solution path must start at initial state.");
            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns whether the first cfg is the initial cfg.
     *
     * @return whether the first cfg is the initial cfg.
     */
    public static boolean hasInitialFirst() {
        double dist = ps.getPath().get(0).maxDistance(ps.getInitialState());
        return dist <= DEFAULT_MAX_ERROR && dist >= 0;
    }

    /**
     * Checks that the last configuration in the solution path is the goal
     * configuration.
     */
    public static boolean testGoalLast() {
        if (!hasGoalLast()) {
            System.out.println("FAILED: Solution path must end at goal state.");
            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns whether the last cfg is the goal cfg.
     *
     * @return whether the last cfg is the goal cfg.
     */
    public static boolean hasGoalLast() {
        List<ArmConfig> path = ps.getPath();
        double dist = path.get(path.size() - 1).maxDistance(ps.getGoalState());
        return dist <= DEFAULT_MAX_ERROR && dist >= 0;
    }

    /**
     * Checks that the steps in between configurations do not exceed the maximum
     * primitive step distance.
     */
    public static boolean testValidSteps() {
        List<Integer> badSteps = getInvalidSteps();
        if (!badSteps.isEmpty()) {
            System.out.println(String.format(
                    "FAILED: Step size limit exceeded for %d of %d step(s).",
                    badSteps.size(), ps.getPath().size() - 1));
            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns the preceding path indices of any invalid steps.
     *
     * @return the preceding path indices of any invalid steps.
     */
    public static List<Integer> getInvalidSteps() {
        List<Integer> badSteps = new ArrayList<Integer>();
        List<ArmConfig> path = ps.getPath();
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
     * Checks that joint angles are within the allowable range
     */
    public static boolean testJointAngles() {
        List<Integer> badStates = getInvalidJointAngleStates();
        if (!badStates.isEmpty()) {
            System.out.println(String.format(
                    "FAILED: Invalid joint angle for %d of %d state(s).",
                    badStates.size(), ps.getPath().size()));

            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns the path indices of any states with invalid joint angles.
     *
     * @return the path indices of any states with invalid joint angles.
     */
    public static List<Integer> getInvalidJointAngleStates() {
        List<Integer> badStates = new ArrayList<Integer>();
        List<ArmConfig> path = ps.getPath();
        for (int i = 0; i < path.size(); i++) {
            if (!hasValidJointAngles(path.get(i))) {
                badStates.add(i);
            }
        }
        return badStates;
    }

    /**
     * Checks if all joint angles are within the limits
     *
     * @param cfg
     * 			The configuration to test
     * @return true if all joint angles are within the limits
     */
    public static boolean hasValidJointAngles(ArmConfig cfg) {
        List<Double> jointAngles = cfg.getJointAngles();
        for (Double angle : jointAngles) {
            if (angle <= MIN_JOINT_ANGLE - DEFAULT_MAX_ERROR) {
                return false;
            } else if (angle >= MAX_JOINT_ANGLE + DEFAULT_MAX_ERROR) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks for collision between arm links
     */
    public static boolean testSelfCollision() {
        List<Integer> badStates = getSelfCollidingStates();
        if (!badStates.isEmpty()) {
            System.out.println(String.format(
                    "FAILED: Self collision for %d of %d state(s).",
                    badStates.size(), ps.getPath().size()));
            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns the path indices of any states with self collision.
     *
     * @return the path indices of any states with self collision.
     */
    public static List<Integer> getSelfCollidingStates() {
        List<Integer> badStates = new ArrayList<Integer>();
        List<ArmConfig> path = ps.getPath();
        for (int i = 0; i < path.size(); i++) {
            if (hasSelfCollision(path.get(i))) {
                badStates.add(i);
            }
        }
        return badStates;
    }

    /**
     * Checks if a configuration collides with itself. Uses a naive
     * method where each link is checked for intersection with other links.
     *
     * @param cfg
     * 			The arm configuration
     * @return true if there is a collision
     */
    public static boolean hasSelfCollision(ArmConfig cfg) {
        List<Line2D> links = cfg.getLinks();
        for (int i = 0; i < links.size(); i++) {
            for (int j = 0; j < i - 1; j++) {
                if (links.get(i).intersectsLine(links.get(j))) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Checks that each configuration fits within the workspace bounds.
     */
    public static boolean testBounds() {
        List<Integer> badStates = getOutOfBoundsStates();
        if (!badStates.isEmpty()) {
            System.out.println(String.format("FAILED: %d of %d"
                            + " state(s) go out of the workspace bounds.",
                    badStates.size(), ps.getPath().size()));

            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns the path indices of any states that are out of bounds.
     *
     * @return the path indices of any states that are out of bounds.
     */
    public static List<Integer> getOutOfBoundsStates() {
        List<ArmConfig> path = ps.getPath();
        List<Integer> badStates = new ArrayList<Integer>();
        for (int i = 0; i < path.size(); i++) {
            if (!fitsBounds(path.get(i))) {
                badStates.add(i);
            }
        }
        return badStates;
    }

    /**
     * Returns whether the given configuration fits wholly within the bounds.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the given configuration fits wholly within the bounds.
     */
    public static boolean fitsBounds(ArmConfig cfg) {
        if (!lenientBounds.contains(cfg.getBase())) {
            return false;
        }
        List<Line2D> links = cfg.getLinks();
        for (Line2D link : links) {
            if (!lenientBounds.contains(link.getP2())) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks that each configuration does not collide with any of the
     * obstacles.
     */
    public static boolean testCollisions() {
        List<Integer> badStates = getCollidingStates();
        if (!badStates.isEmpty()) {
            System.out.println(String.format(
                    "FAILED: %d of %d state(s) collide with obstacles.",
                    badStates.size(), ps.getPath().size()));

            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns the path indices of any states that collide with obstacles.
     *
     * @return the path indices of any states that collide with obstacles.
     */
    public static List<Integer> getCollidingStates() {
        List<ArmConfig> path = ps.getPath();
        List<Integer> badStates = new ArrayList<Integer>();
        for (int i = 0; i < path.size(); i++) {
            if (hasCollision(path.get(i), ps.getObstacles())) {
                badStates.add(i);
            }
        }
        return badStates;
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
        return lenientRect.contains(cfg.getBase().getX(), cfg.getBase().getY());
    }

    /**
     * Runs a specific test based on its name.
     */
    public static boolean testAll(ProblemSpec ps) {

        setProblemSpec(ps);

        System.out.println("\n");
        System.out.println("Test that solution has initial config first: " + testInitialFirst());
        System.out.println("Test that solution has goal config last: " + testGoalLast());
        System.out.println("Test that solution has only valid steps: " + testValidSteps());
        System.out.println("Test that joint angles are within range: " + testJointAngles());
        System.out.println("Test that there is no self collisions: " + testSelfCollision());
        System.out.println("Test that every configuration falls within bounds: " + testBounds());
        System.out.println("Test that there is no collisions with obstacles: " + testCollisions());

        return testInitialFirst() && testGoalLast() && testValidSteps() && testJointAngles()
                && testSelfCollision() && testBounds() && testCollisions();
    }
}
