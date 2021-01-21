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

    public void doubleStrafeToPoint(double x, double y, double rotation) {
        rotateTo0();
        double hyp, driveX, driveY, distance;
        double thetaSpeed = 0;
        while (((odometry.robotPosition[0] < x-.2 || odometry.robotPosition[0] > x+.2) || (odometry.robotPosition[1] < y-.2 || odometry.robotPosition[1] > y+.2)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometry.robotPosition[2]+(rotation));
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

    public void testDoubleStrafeToPoint(double x, double y, double rotation) {
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

    public void wobbleTestDoubleStrafeToPoint(double x, double y, double rotation, int wobblePosition, Telemetry telemetry) {
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
}
