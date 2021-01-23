package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class OdometryMove {

    RobotHardware robot;
    GoBildaDrive drive;
    Odometry odometry;
    LinearOpMode opMode;

    /**
     * Moves the robot using odometry
     * @param opMode the current opMode
     * @param robot the instantiated RobotHardware
     * @param odometry the current Odometry
     */
    public OdometryMove(LinearOpMode opMode, RobotHardware robot, Odometry odometry) {
        this.opMode = opMode;
        this.robot = robot;
        drive = new GoBildaDrive(robot);
        this.odometry = odometry;
    }

    // moves forwards backwards (x direction)
    public void goToPoint(double x, double rotation) {
        double moveSpeed = .85;
        double thetaSpeed = 0;
        while ((odometry.robotPosition[0] < (x-.1) || odometry.robotPosition[0] > (x+.1)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometry.robotPosition[2]+(rotation));
            if (odometry.robotPosition[0] < (x-.2)) {
                drive.circlepadMove(moveSpeed, 0, thetaSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[0] > (x+.2)) {
                drive.circlepadMove(-moveSpeed, 0, thetaSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[0] < (x-.1) || odometry.robotPosition[0] > (x+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(odometry.robotPosition[0] - x) < 16) {
                moveSpeed = .16 + (((.6-.15)/(16)) * (Math.abs(odometry.robotPosition[0] - x)));
            }
        }
        odometry.queryOdometry();
        drive.stop();
    }

    // OLD
    // moves left right (y direction)
    public void goToStrafePoint(double y, double rotation) {
        double moveSpeed = .75;
        double thetaSpeed = 0;
        while ((odometry.robotPosition[1] < (y-.1) || odometry.robotPosition[1] > (y+.1)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometry.robotPosition[2]+(rotation));
            if (odometry.robotPosition[1] < (y-.2)) {
                drive.circlepadMove(0, -moveSpeed, thetaSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[1] > (y+.2)) {
                drive.circlepadMove(0, moveSpeed, thetaSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[1] < (y-.1) || odometry.robotPosition[1] > (y+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(odometry.robotPosition[1] - y) < 10) {
                moveSpeed = .25 + (((.85-.25)/(12)) * (Math.abs(odometry.robotPosition[1] - y)));
            }
        }
        odometry.queryOdometry();
        drive.stop();
    }

    /**
     * Rotates the robot to wherever odometry thinks 0 theta is
     */
    public void rotateTo0() {
        double wantedAngle = (Math.round(odometry.robotPosition[2]/(2*Math.PI))) * 2 * Math.PI;
        double driveSpeed = .65;
        while (odometry.robotPosition[2] < wantedAngle - .0065 || odometry.robotPosition[2] > wantedAngle + .0065 && opMode.opModeIsActive()) {
            if (odometry.robotPosition[2] < wantedAngle - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[2] > wantedAngle + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
                odometry.queryOdometry();
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(odometry.robotPosition[2] - wantedAngle) < 1) {
                driveSpeed = .27 + (.2 * Math.abs(odometry.robotPosition[2] - wantedAngle));
            }
            odometry.queryOdometry();
        }
        drive.stop();
    }

    /**
     * Rotates the robot to a given theta
     * @param theta the angle (in radians) for the robot to move to relative to whatever odometry
     *              thinks "0 theta" is, positive being right and negative being left
     */
    public void rotate(double theta) {
        double driveSpeed = .45;
        while (odometry.robotPosition[2] < theta - .007 || odometry.robotPosition[2] > theta + .007 && opMode.opModeIsActive()) {
            if (odometry.robotPosition[2] < theta - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[2] > theta + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
                odometry.queryOdometry();
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(odometry.robotPosition[2] - theta) < 1) {
                driveSpeed = .27 + (.2 * Math.abs(odometry.robotPosition[2] - theta));
            }
            odometry.queryOdometry();
        }
        drive.stop();
    }

    /**
     * Rotates the robot a given amount
     * @param dTheta the angle (in radians) for the robot to move incrementally, positive being
     *               to the right and negative to the left
     */
    public void deltaRotate(double dTheta) {
        double driveSpeed = .45;
        double theta = odometry.robotPosition[2] + dTheta;
        while (odometry.robotPosition[2] < theta - .007 || odometry.robotPosition[2] > theta + .007 && opMode.opModeIsActive()) {
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(odometry.robotPosition[2] - theta) < 1) {
                driveSpeed = .13 + (.18 * Math.abs(odometry.robotPosition[2] - theta));
            }
            if (odometry.robotPosition[2] < theta - .007) {
                drive.circlepadMove(0, 0, driveSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[2] > theta + .007) {
                drive.circlepadMove(0, 0, -driveSpeed);
                odometry.queryOdometry();
            }
            odometry.queryOdometry();
        }
        drive.stop();
    }

    /**
     * Moves the robot to a given point diagonally at full speed with odometry theta = 0
     * @param x x value of the odometry-based point for the robot to go to
     * @param y y value of the oodometry-based point for the robot to go to
     */
    public void zeroThetaDiagonalToPoint(double x, double y) {
        rotateTo0();
        double hyp, driveX, driveY, distance;
        double thetaSpeed;
        while (((odometry.robotPosition[0] < x-.2 || odometry.robotPosition[0] > x+.2) || (odometry.robotPosition[1] < y-.2 || odometry.robotPosition[1] > y+.2)) && opMode.opModeIsActive()) {
            thetaSpeed = -odometry.robotPosition[2];
            hyp = Math.sqrt((x - odometry.robotPosition[0])*(x - odometry.robotPosition[0]) + (y - odometry.robotPosition[1])*(y - odometry.robotPosition[1]));
            driveX = (x - odometry.robotPosition[0]) / hyp;
            driveY = (y - odometry.robotPosition[1]) / hyp;
            odometry.queryOdometry();
            distance = Math.sqrt(Math.pow(Math.abs(odometry.robotPosition[0] - x), 2) + Math.pow(Math.abs(odometry.robotPosition[1] - y), 2));
            if (distance < 12) {
                driveX = driveX * (.325 + (.95-.325)*(distance/12));
                driveY = driveY * (.325 + (.95-.325)*(distance/12));
            }
            drive.circlepadMove(driveX, -driveY, thetaSpeed);
            odometry.queryOdometry();
        }
        drive.stop();
        rotateTo0();
    }

    /**
     * Moves the robot to a given point with a certain rotation (can be anything)
     * @param x x value of the odometry-based point for the robot to go to
     * @param y y value of the oodometry-based point for the robot to go to
     * @param rotation the rotation of the robot as it arrives at its destination (positive being
     *                 to the right, negative to the left) -- depends on what odometry thinks is
     *                 zero!!
     */
    public void diagonalToPoint(double x, double y, double rotation) {
        double hyp, initialX, initialY, thetaOfPoint, driveX, driveY, distance;
        double thetaSpeed = 0;
        while (((odometry.robotPosition[0] < x-.3 || odometry.robotPosition[0] > x+.3) || (odometry.robotPosition[1] < y-.3 || odometry.robotPosition[1] > y+.3)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometry.robotPosition[2]-(rotation));
            hyp = Math.sqrt((x - odometry.robotPosition[0])*(x - odometry.robotPosition[0]) + (y - odometry.robotPosition[1])*(y - odometry.robotPosition[1]));
            initialX = (x - odometry.robotPosition[0]) / hyp;
            initialY = (y - odometry.robotPosition[1]) / hyp;
            thetaOfPoint = Math.atan(initialY/initialX) + rotation;
            driveX = Math.cos(thetaOfPoint);
            driveY = Math.sin(thetaOfPoint);
            opMode.telemetry.addData("initialX", initialX);
            opMode.telemetry.addData("initialY", initialY);
            opMode.telemetry.addData("theta", thetaOfPoint);
            opMode.telemetry.addData("driveX", driveX);
            opMode.telemetry.addData("driveY", driveY);
            distance = Math.sqrt(Math.pow(Math.abs(odometry.robotPosition[0] - x), 2) + Math.pow(Math.abs(odometry.robotPosition[1] - y), 2));
            if (distance < 12) {
                driveX = driveX * (.325 + (.95-.325)*(distance/12));
                driveY = driveY * (.325 + (.95-.325)*(distance/12));
            }
            drive.circlepadMove(driveX, -driveY, thetaSpeed);
            odometry.queryOdometry();
        }
        drive.stop();
    }

    /**
     * A version of diagonalToPoint() but with wobble-carrying capabilities. Moves to a certain
     * point with a given rotation and also keeps the wobble rotator going at a certain position
     * @param x x value of the oodometry-based point for the robot to go to
     * @param y y value of the oodometry-based point for the robot to go to
     * @param rotation the rotation of the robot as it arrives at its destination (positive being
     *      *                 to the right, negative to the left) -- depends on what odometry thinks is
     *      *                 zero!!
     * @param wobblePosition the desired position of the wobble rotator
     * @param telemetry the opMode's telemetry
     */
    public void wobbleDiagonalToPoint(double x, double y, double rotation, int wobblePosition, Telemetry telemetry) {
        double hyp, initialX, initialY, thetaOfPoint, driveX, driveY, distance;
        double thetaSpeed = 0;
        while (((odometry.robotPosition[0] < x-.3 || odometry.robotPosition[0] > x+.3) || (odometry.robotPosition[1] < y-.3 || odometry.robotPosition[1] > y+.3)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometry.robotPosition[2]-(rotation));
            hyp = Math.sqrt((x - odometry.robotPosition[0])*(x - odometry.robotPosition[0]) + (y - odometry.robotPosition[1])*(y - odometry.robotPosition[1]));
            initialX = (x - odometry.robotPosition[0]) / hyp;
            initialY = (y - odometry.robotPosition[1]) / hyp;
            thetaOfPoint = Math.atan(initialY/initialX) + rotation;
            driveX = Math.cos(thetaOfPoint);
            driveY = Math.sin(thetaOfPoint);
            opMode.telemetry.addData("initialX", initialX);
            opMode.telemetry.addData("initialY", initialY);
            opMode.telemetry.addData("theta", thetaOfPoint);
            opMode.telemetry.addData("driveX", driveX);
            opMode.telemetry.addData("driveY", driveY);
            distance = Math.sqrt(Math.pow(Math.abs(odometry.robotPosition[0] - x), 2) + Math.pow(Math.abs(odometry.robotPosition[1] - y), 2));
            if (distance < 12) {
                driveX = driveX * (.325 + (.95-.325)*(distance/12));
                driveY = driveY * (.325 + (.95-.325)*(distance/12));
            }
            drive.circlepadMove(driveX, -driveY, thetaSpeed);
            odometry.queryOdometry();
            robot.wobbleToPosition(wobblePosition, telemetry);
        }
        drive.stop();
    }

    double closestShootingPosition;
    double farthestShootingPosition;
    double distanceFromFarthestToClosestShootingPositions;

    public boolean isRobotBelowTop(double x, double y) {
        return y < 10;
    }
    
//    public boolean isRobotAboveBottom(double x, double y) {
//        double yToBeat =
//    }

    public void goToShootingPosition(int myNumber) {

    }
}
