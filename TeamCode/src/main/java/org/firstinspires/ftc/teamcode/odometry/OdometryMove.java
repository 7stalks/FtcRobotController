package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        double moveSpeed = .6;
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
            if (Math.abs(odometry.robotPosition[0] - x) < 15) {
                moveSpeed = .15 + (((.6-.15)/(15)) * (Math.abs(odometry.robotPosition[0] - x)));
            }
        }
        odometry.queryOdometry();
        drive.stop();
    }

    // moves left right (y direction)
    public void goToStrafePoint(double y, double rotation) {
        double moveSpeed = .55;
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
            if (Math.abs(odometry.robotPosition[1] - y) < 9) {
                moveSpeed = .24 + (((.55-.24)/(9)) * (Math.abs(odometry.robotPosition[1] - y)));
            }
        }
        odometry.queryOdometry();
        drive.stop();
    }
}
