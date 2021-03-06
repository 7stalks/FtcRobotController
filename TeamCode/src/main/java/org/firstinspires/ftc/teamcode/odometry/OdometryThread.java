package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class OdometryThread extends Thread {

    RobotHardware robot;

    double firstOLeft;
    double firstORight;
    double firstOMiddle;
    public double[] odometryInfo = {0, 0, 0};
    public double[] robotPosition = {0, 0, 0};
    public double[] lastIterationOdometryInfo = {0, 0, 0};

    public double encoderCountsPerIn = 306.3816404153158;
    // The two "big" constants. The wheel distance is the distance between L and R encoders and
    // determines theta. The tick per degree offset is for the middle encoder
    // TODO add "final" to each of these when done testing
    public double robotEncoderWheelDistance = 15.6176;
    public double horizontalEncoderTickPerDegreeOffset = -2120;

    // constructor
    public OdometryThread(RobotHardware myRobot) {
        robot = myRobot;
    }

    // Gets the h used in the odometry calculation
    // (AKA get the hypotenuse of the mini triangle made when driving)
    private double getHypOrDistance(double leftDistance, double rightDistance, double deltaTheta) {
        if (deltaTheta != 0) {
            double r = (leftDistance + rightDistance) / 2;
            return (r / deltaTheta)*(Math.sin(deltaTheta)/Math.cos(deltaTheta/2));
        } else {
            // returns the distance travelled, averages L and R just to be accurate.
            return (leftDistance + rightDistance) / 2;
        }
    }

    // Changes raw odometry info into useful changes in distance
    // Finds the delta and turns it to inches, Sort of a 2-in-1
    private double[] odometryInfoToDeltaInches(double[] odometryInfo) {
        // OLeft's encoder is reversed, hence the negative
        double deltaOLeft = -((odometryInfo[0]) - lastIterationOdometryInfo[0]) / encoderCountsPerIn;
        double deltaORight = (odometryInfo[1] - lastIterationOdometryInfo[1]) / encoderCountsPerIn;
        double deltaOMiddle = (odometryInfo[2] - lastIterationOdometryInfo[2]) / encoderCountsPerIn;
        // woooooaahhh. copies last odometryinfo onto lastiterodometryinfo
        System.arraycopy(odometryInfo, 0, lastIterationOdometryInfo, 0, 3);
        return new double[]{deltaOLeft, deltaORight, deltaOMiddle};
    }

    // This one is self explanatory, the change in theta (orientation)
    private double getDeltaTheta(double leftDistance, double rightDistance) {
        return (rightDistance - leftDistance) / robotEncoderWheelDistance;
    }

    // The main method. Will return the new (x, y) position. Feed it the old (x, y) position
    // that's retrieved from this method the last loop around (with initial position being
    // determined by Vuforia, hypothetically). odometryInfo will be the raw odometry values that
    // should be recorded right before this method is called
    public double[] getPosition(double[] oldPosition, double[] odometryInfo) {
        // assign names to the values in oldPosition
        double oldX = oldPosition[0];
        double oldY = oldPosition[1];
        double oldTheta = oldPosition[2];


        // get the changes (deltas) in distances/theta
        // deltaDistances has all 3 odometers (L, R, M)
        double[] deltaDistances = odometryInfoToDeltaInches(odometryInfo);
        double deltaTheta = getDeltaTheta(deltaDistances[0], deltaDistances[1]);

        // Get the new theta and make it look pretty too (doesn't hurt calculations to make look pretty)
        double newTheta = deltaTheta + oldTheta;

        // calculate horizontal change using the tick per degree offset and then proceed to get the
        // hypotenuse of the triangle made when moving
        double horizontalChange = deltaDistances[2] - ((horizontalEncoderTickPerDegreeOffset*deltaTheta)/encoderCountsPerIn);
        double h = getHypOrDistance(deltaDistances[0], deltaDistances[1], deltaTheta);

        // do a classic hyp * cos / sin to get x / y. also account for horizontal change
        double deltaX = (h * Math.cos(oldTheta+(deltaTheta/2))) - (horizontalChange * Math.sin(oldTheta + (deltaTheta/2)));
        double deltaY = (h * Math.sin(oldTheta+(deltaTheta/2))) - (horizontalChange * Math.cos(oldTheta + (deltaTheta/2)));

        return new double[]{deltaX + oldX, deltaY + oldY, newTheta, deltaDistances[0], deltaDistances[1], deltaTheta, horizontalChange};
    }

    public void inputVuforia(double x, double y, double rotation) {
        firstOLeft = robot.OLeft.getCurrentPosition();
        firstORight = robot.ORight.getCurrentPosition();
        firstOMiddle = robot.OMiddle.getCurrentPosition();
        robotPosition = new double[] {x, y, rotation};
    }

    private void queryOdometry() {
        odometryInfo = new double[]{
                robot.OLeft.getCurrentPosition() - firstOLeft,
                robot.ORight.getCurrentPosition() - firstORight,
                robot.OMiddle.getCurrentPosition() - firstOMiddle
        };
        robotPosition = getPosition(robotPosition, odometryInfo);
    }

    public void run() {
        while(!OdometryThread.currentThread().isInterrupted()) {
            queryOdometry();
        }
    }
}
