package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry {

    // length from left to right odometers
    // TODO add  for length from l to r odometer and change encoderOverMm
    double L = 15.625;
    double encoderCountOverIn = 307.699557;

    double[] lastIterationOdometryInfo = {0, 0, 0};

    // Gets the h used in the odometry calculation
    private double getHypOrDistance(double leftDistance, double rightDistance, double deltaTheta) {
        if (deltaTheta != 0) {
            double r = (leftDistance / deltaTheta) + (L / 2);
            return ((r * Math.sin(deltaTheta)) / Math.cos(deltaTheta / 2));
        } else {
            // returns the distance travelled, averages L and R just to be accurate.
            return (leftDistance + rightDistance) / 2;
        }
    }

    // Changes raw odometry info into useful changes in distance
    // Finds the delta and turns it to mm, Sort of a 2-in-1
    // TODO add encoderOverMm
    private double[] odometryInfoToDeltaIn(double[] odometryInfo) {
        double deltaOLeft = (-(odometryInfo[0]) - lastIterationOdometryInfo[0]) / encoderCountOverIn;
        double deltaORight = (odometryInfo[1] - lastIterationOdometryInfo[1]) / encoderCountOverIn;
        double deltaOMiddle = (-(odometryInfo[2]) - lastIterationOdometryInfo[2]) / encoderCountOverIn;
        // woooooaahhh. copies last odometryinfo onto lastiterodometryinfo
        System.arraycopy(odometryInfo, 0, lastIterationOdometryInfo, 0, 3);
        return new double[]{deltaOLeft, deltaORight, deltaOMiddle};
    }

    // this one is self explanatory. the change in theta
    private double getDeltaTheta(double leftDistance, double rightDistance) {
        return (rightDistance - leftDistance) / L;
    }

    // The main method. Will return the new (x, y) position. Feed it the old (x, y) position
    // that's retrieved from this method the last loop around (with initial position being
    // determined by Vuforia, hypothetically). odometryInfo will be the raw odometry values that
    // should be recorded right before this method is called
    public double[] getPosition(double[] oldPosition, double[] odometryInfo, Telemetry telemetry) {
        // assign names to the values in oldPosition
        double oldX = oldPosition[0];
        double oldY = oldPosition[1];
        double oldTheta = oldPosition[2];
//        telemetry.addData("Old X, Y, theta", oldPosition);

        // get the changes (deltas) in distances/theta
        // deltaDistances has all 3 odometers (L, R, M)
        double[] deltaDistances = odometryInfoToDeltaIn(odometryInfo);
        double deltaTheta = getDeltaTheta(deltaDistances[0], deltaDistances[1]);
//        telemetry.addData("delta distances (L, R, M)", deltaDistances);
//        telemetry.addData("deltaTheta", deltaTheta);

        // do the calculations
        double h = getHypOrDistance(deltaDistances[0], deltaDistances[1], deltaTheta);
        double deltaX = h * Math.cos((deltaTheta / 2) + oldTheta);
        double deltaY = h * Math.sin((deltaTheta / 2) + oldTheta);
//        telemetry.addData("h", h);
//        telemetry.addData("delta X", deltaX);
//        telemetry.addData("delta Y", deltaY);

        return new double[]{deltaX + oldX, deltaY + oldY, deltaTheta + oldTheta};
    }
}
