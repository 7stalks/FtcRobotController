package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Odometry {

    OdometryCalibration calibration = new OdometryCalibration();
    public double horizontalChange = 0;

    // length from left to right odometers
    // TODO add  for length from l to r odometer and change encoderOverMm
    double[] lastIterationOdometryInfo = {0, 0, 0};
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    final public double robotEncoderWheelDistance = 15.625; //Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());// * calibration.encoderCountsPerIn;
    final public double horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    // Gets the h used in the odometry calculation
    private double getHypOrDistance(double leftDistance, double rightDistance, double deltaTheta) {
        if (deltaTheta != 0) {
            double r = (leftDistance / deltaTheta) + (robotEncoderWheelDistance / 2);
            return ((r * Math.sin(deltaTheta)) / Math.cos(deltaTheta / 2));
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

        // do the calculations
        double displayedTheta = deltaTheta + oldTheta;
        if (displayedTheta > (2*Math.PI)) {
            displayedTheta = displayedTheta - (2*Math.PI);
        } else if (displayedTheta < -(2*Math.PI)) {
            displayedTheta = displayedTheta + (2*Math.PI);
        }
        double horizontalChange = deltaDistances[2] - (horizontalEncoderTickPerDegreeOffset*deltaTheta);
        double h = getHypOrDistance(deltaDistances[0], deltaDistances[1], deltaTheta);
        double deltaX = (h * Math.sin(displayedTheta) + (horizontalChange * Math.cos(displayedTheta)));
        double deltaY = (h * Math.cos(displayedTheta) - (horizontalChange * Math.sin(displayedTheta)));

        return new double[]{deltaX + oldX, deltaY + oldY, displayedTheta, deltaDistances[0], deltaDistances[1], deltaTheta, horizontalChange};
    }
}
//TODO: explain the code in cleaner fashion


// more accurate encoder information (odometryInfoToDeltaIn???)
// !have to account for the horizontal encoder moving when spinning. this is the reason for the offset
//// per degree thingy, but NEED TO IMPLEMENT CORRECTLY!!!!!!!! AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH