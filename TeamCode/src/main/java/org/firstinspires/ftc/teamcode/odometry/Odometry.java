package org.firstinspires.ftc.teamcode.odometry;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.logging.Logger;

public class Odometry {

    RobotHardware robot = new RobotHardware();
    Telemetry telemetry;
    ElapsedTime brokenThingsTimer = new ElapsedTime();

    public Odometry(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public double[] lastIterationOdometryInfo = {0, 0, 0};
    public double[] robotPosition = {0, 0, 0};
    public double[] myPosition = new double[7];
    public double[] deltas = new double[3];
    public double[] encoderPositions = new double[3];
    double[] odometryInfo;
    double firstOLeft = 0;
    double firstORight = 0;
    double firstOMiddle = 0;
    public double encoderCountsPerIn = 306.3816404153158;
    public double inchesPerEncoderCounts = 1/encoderCountsPerIn;

    // The two "big" constants. The wheel distance is the distance between L and R encoders and
    // determines theta. The tick per degree offset is for the middle encoder
    // TODO add "final" to each of these when done testing
    public double robotEncoderWheelDistance = 15.5780;
    public double inverseEncoderWheelDistance = 1/15.5780;
    public double horizontalEncoderTickPerDegreeOffset = -2100;

    // Gets the h used in the odometry calculation
    // (AKA get the hypotenuse of the mini triangle made when driving)
    private double getHypOrDistance(double leftDistance, double rightDistance, double deltaTheta) {
        if (deltaTheta != 0) {
            double r = (leftDistance + rightDistance) * 0.5;
            return (r / deltaTheta)*(Math.sin(deltaTheta)/Math.cos(deltaTheta*0.5));
        } else {
            // returns the distance travelled, averages L and R just to be accurate.
            return (leftDistance + rightDistance) * 0.5;
        }
    }

    // Changes raw odometry info into useful changes in distance
    // Finds the delta and turns it to inches, Sort of a 2-in-1
    private double[] odometryInfoToDeltaInches(double[] odometryInfo) {
        // OLeft's encoder is reversed, hence the negative
        double deltaOLeft = -((odometryInfo[0]) - lastIterationOdometryInfo[0]) * inchesPerEncoderCounts;
        double deltaORight = -(odometryInfo[1] - lastIterationOdometryInfo[1]) * inchesPerEncoderCounts;
        double deltaOMiddle = -(odometryInfo[2] - lastIterationOdometryInfo[2]) * inchesPerEncoderCounts;
        // woooooaahhh. copies last odometryinfo onto lastiterodometryinfo
        System.arraycopy(odometryInfo, 0, lastIterationOdometryInfo, 0, 3);
        deltas[0] = deltaOLeft;
        deltas[1] = deltaORight;
        deltas[2] = deltaOMiddle;
        return deltas;
    }

    // This one is self explanatory, the change in theta (orientation)
    private double getDeltaTheta(double leftDistance, double rightDistance) {
        return (leftDistance - rightDistance) * inverseEncoderWheelDistance;
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
        double deltaTheta = getDeltaTheta(deltaDistances[0], deltaDistances[1]);

        // Get the new theta and make it look pretty too (doesn't hurt calculations to make look pretty)
        double newTheta = deltaTheta + oldTheta;

        // calculate horizontal change using the tick per degree offset and then proceed to get the
        // hypotenuse of the triangle made when moving
        double horizontalChange = deltaDistances[2] - ((horizontalEncoderTickPerDegreeOffset*deltaTheta)*inchesPerEncoderCounts);
        double h = getHypOrDistance(deltaDistances[0], deltaDistances[1], deltaTheta);

        // do a classic hyp * cos / sin to get x / y. also account for horizontal change
        double deltaX = (h * Math.cos(oldTheta+(deltaTheta/2))) + (horizontalChange * Math.sin(oldTheta + (deltaTheta/2)));
        double deltaY = (h * Math.sin(oldTheta+(deltaTheta/2))) - (horizontalChange * Math.cos(oldTheta + (deltaTheta/2)));

        myPosition[0] = deltaX + oldX;
        myPosition[1] = deltaY + oldY;
        myPosition[2] = newTheta;
        myPosition[3] = deltaDistances[0];
        myPosition[4] = deltaDistances[1];
        myPosition[5] = deltaTheta;
        myPosition[6] = horizontalChange;

        return myPosition;
    }

    double Left = 0;
    double Right = 0;
    double Middle = 0;
    public double[] getPositions() throws InterruptedException {
        Left = robot.OLeft.getCurrentPosition() - firstOLeft;
        Right = robot.ORight.getCurrentPosition() - firstORight;
        Middle = robot.OMiddle.getCurrentPosition() - firstOMiddle;
        encoderPositions[0] = Left;
        encoderPositions[1] = Right;
        encoderPositions[2] = Middle;
        return encoderPositions;
    }

    /**
     * Input vuforia info into odometry -- starts over from the new point
     * @param x the x from vuforia
     * @param y the y from vuforia
     * @param theta the theta from vuforia
     */
    public void inputVuforia(double x, double y, double theta) {
        robotPosition = new double[] {x, y, theta};
        firstOLeft = robot.OLeft.getCurrentPosition();
        firstORight = robot.ORight.getCurrentPosition();
        firstOMiddle = robot.OMiddle.getCurrentPosition();
        lastIterationOdometryInfo = new double[] {0, 0, 0};
    }

    /**
     * The main method -- updates odometry so that values are correct. Must be done in every loop!
     */
    public void queryOdometry() {
        int breakCounter = 0;
        while (true) {
            try {
                odometryInfo = getPositions();
                break;
            } catch (InterruptedException err) {
                telemetry.addData("err", err);
                breakCounter++;
                brokenThingsTimer.reset();
                while (brokenThingsTimer.milliseconds() < 30) {
                }
            }
            if (breakCounter>3) {
                break;
            }
        }

        robotPosition = getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);
        telemetry.update();
    }
}