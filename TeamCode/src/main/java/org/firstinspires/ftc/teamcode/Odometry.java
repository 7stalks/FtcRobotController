package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Odometry {

    OdometryCalibration calibration = new OdometryCalibration();

    // length from left to right odometers and horizontal tick offset per degree
    double[] lastIterationOdometryInfo = {0, 0, 0};
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    final public double robotEncoderWheelDistance = 15.66551181; //Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());// * calibration.encoderCountsPerIn;
    final public double horizontalEncoderTickPerDegreeOffset = 1662.60314922; //Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    //15.625; //15.75; //15.66551181; //
    //3313.00833716; //1662.60314922; //

    // Gets the h used in the odometry calculation
    private double getHypOrDistance(double leftDistance, double rightDistance, double deltaTheta) {
        if (deltaTheta != 0) {
            double r = (leftDistance + rightDistance) / 2;
            return (r / deltaTheta)*Math.sin(deltaTheta)/Math.cos(deltaTheta);
        } else {
            // returns the distance travelled, averages L and R just to be accurate.
            return (leftDistance + rightDistance) / 2;
        }
    }

    // Changes raw odometry info into useful changes in distance
    // Finds the delta and turns it to in, Sort of a 2-in-1
    private double[] odometryInfoToDeltaIn(double[] odometryInfo) {
        double deltaOLeft = -((odometryInfo[0]) - lastIterationOdometryInfo[0]) / calibration.encoderCountsPerIn;
        double deltaORight = (odometryInfo[1] - lastIterationOdometryInfo[1]) / calibration.encoderCountsPerIn;
        double deltaOMiddle = (odometryInfo[2] - lastIterationOdometryInfo[2]) / calibration.encoderCountsPerIn;
        // woooooaahhh. copies last odometryinfo onto lastiterodometryinfo
        System.arraycopy(odometryInfo, 0, lastIterationOdometryInfo, 0, 3);
        return new double[]{deltaOLeft, deltaORight, deltaOMiddle};
    }

    // this one is self explanatory. the change in theta
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

        // get the changes (deltas) in distances/theta
        // deltaDistances has all 3 odometers (L, R, M)
        double[] deltaDistances = odometryInfoToDeltaIn(odometryInfo);
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
        double horizontalChange = deltaDistances[2] - ((horizontalEncoderTickPerDegreeOffset*deltaTheta)/calibration.encoderCountsPerIn);
        double h = getHypOrDistance(deltaDistances[0], deltaDistances[1], deltaTheta);

        // do a classic hyp * cos / sin to get x / y. also account for horizontal change
        double deltaX = (h * Math.cos(oldTheta+(deltaTheta/2))) + (horizontalChange * Math.cos(oldTheta + (deltaTheta/2) - (Math.PI/2)));
        double deltaY = (h * Math.sin(oldTheta+(deltaTheta/2))) + (horizontalChange * Math.sin(oldTheta + (deltaTheta/2) - (Math.PI/2)));

        return new double[]{deltaX + oldX, deltaY + oldY, newTheta, deltaDistances[0], deltaDistances[1], deltaTheta, horizontalChange};
    }

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
    //////// THIS IS A MASSIVE MESS ////////

    public void swerveToPoint(double[] position, GoBildaDrive drioe) {

    }
}
//TODO: explain the code in cleaner fashion