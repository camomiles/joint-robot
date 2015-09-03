package solution;

import java.io.IOException;

/**
 * Find path from initial configuration to goal configuration of the joint robot using search algorithms
 *
 * @author Roman Yakobnyuk
 */
public class Main {

    public static void main(String[] argv) {

        // Ensure that a filename (or something) was given in argv[0]
        if (argv.length < 1 || argv.length > 2) {
            System.err.println("Usage: ai-7702 inputFileName [outputFileName]");
            System.exit(1);
        }

        say("Hello World!");

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

    }

    // Log out text line to stdout
    public static void say(String line) {
        System.out.println(line);
    }
}