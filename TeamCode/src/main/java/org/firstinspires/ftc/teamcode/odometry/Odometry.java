package org.firstinspires.ftc.teamcode.odometry;

import android.sax.StartElementListener;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class Odometry {

    public double[] lastIterationOdometryInfo = {0, 0, 0};

    public double encoderCountsPerIn = 306.3816404153158;

    // The two "big" constants. The wheel distance is the distance between L and R encoders and
    // determines theta. The tick per degree offset is for the middle encoder
    // TODO add "final" to each of these when done testing
    public double robotEncoderWheelDistance = 15.6176;
    public double horizontalEncoderTickPerDegreeOffset = -2050;

    // TODO add a queryOdometry() method that uses the constructor below
//    RobotHardware robot;
//    Telemetry telemetry;
//
//    public Odometry(RobotHardware someRobot, Telemetry someTelemetry) {
//        robot = someRobot;
//        telemetry = someTelemetry;
//    }


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
    public double[] getPosition(double[] oldPosition, double[] odometryInfo, Telemetry telemetry) {
        // assign names to the values in oldPosition
        double oldX = oldPosition[0];
        double oldY = oldPosition[1];
        double oldTheta = oldPosition[2];
        telemetry.addData("Odometry info L", odometryInfo[0]);
        telemetry.addData("Odometry info R", odometryInfo[1]);
        telemetry.addData("Odometry info M", odometryInfo[2]);


        // get the changes (deltas) in distances/theta
        // deltaDistances has all 3 odometers (L, R, M)
        double[] deltaDistances = odometryInfoToDeltaInches(odometryInfo);
        telemetry.addData("deltaLeft", deltaDistances[0]);
        telemetry.addData("deltaRight", deltaDistances[1]);
        telemetry.addData("deltaMiddle", deltaDistances[2]);
        double deltaTheta = getDeltaTheta(deltaDistances[0], deltaDistances[1]);

        // Get the new theta and make it look pretty too (doesn't hurt calculations to make look pretty)
        double newTheta = deltaTheta + oldTheta;
//        if (newTheta > (2*Math.PI)) {
//            newTheta = newTheta - (2*Math.PI);
//        } else if (newTheta < -(2*Math.PI)) {
//            newTheta = newTheta + (2*Math.PI);
//        }

        // calculate horizontal change using the tick per degree offset and then proceed to get the
        // hypotenuse of the triangle made when moving
        double horizontalChange = deltaDistances[2] - ((horizontalEncoderTickPerDegreeOffset*deltaTheta)/encoderCountsPerIn);
        double h = getHypOrDistance(deltaDistances[0], deltaDistances[1], deltaTheta);

        // do a classic hyp * cos / sin to get x / y. also account for horizontal change
        double deltaX = (h * Math.cos(oldTheta+(deltaTheta/2))) - (horizontalChange * Math.sin(oldTheta + (deltaTheta/2)));
        double deltaY = (h * Math.sin(oldTheta+(deltaTheta/2))) - (horizontalChange * Math.cos(oldTheta + (deltaTheta/2)));

        return new double[]{deltaX + oldX, deltaY + oldY, newTheta, deltaDistances[0], deltaDistances[1], deltaTheta, horizontalChange};
    }

    // This is a future moveToPoint method. IN PROGRESS!
    //  Needs to be iterative if we want to do other processes. Thread???
    public void moveToPoint(double[] initialPosition, double[] finalPosition, double[] odometryInfo, GoBildaDrive drive, Telemetry telemetry) {
        double initialAngle;
        double pivotSpeed = .4;
        double rawAngleToPosition = Math.atan2(finalPosition[1] - initialPosition[1], finalPosition[0] - initialPosition[0]);

        // make initial angle positive
        if (initialPosition[2] < 0) {
            initialAngle = initialPosition[2] + 2 * Math.PI;
        } else {
            initialAngle = initialPosition[2];
        }

        // will have to negate in case odometry theta is negative
        double angleToPosition = initialAngle - rawAngleToPosition;

        if (angleToPosition < 0) {
            pivotSpeed = -pivotSpeed;
        }

        double[] currentPosition = initialPosition;
//        while (currentPosition[2] < )
        drive.circlepadMove(0, 0, pivotSpeed);
    }
}