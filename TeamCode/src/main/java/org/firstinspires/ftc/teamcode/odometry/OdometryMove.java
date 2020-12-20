package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class OdometryMove {

    RobotHardware robot;
    GoBildaDrive drive;
    OdometryThread odometryThread;
    LinearOpMode opMode;

    public OdometryMove(LinearOpMode opMode, RobotHardware robot, OdometryThread odometryThread) {
        this.opMode = opMode;
        this.robot = robot;
        drive = new GoBildaDrive(robot);
        this.odometryThread = odometryThread;
    }

    // moves forwards backwards (x direction)
    public void goToPoint(double x, double rotation) {
        double moveSpeed = .6;
        double thetaSpeed = 0;
        while ((odometryThread.robotPosition[0] < (x-.1) || odometryThread.robotPosition[0] > (x+.1)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometryThread.robotPosition[2]+(rotation));
            if (odometryThread.robotPosition[0] < (x-.2)) {
                drive.circlepadMove(moveSpeed, 0, thetaSpeed);
            } else if (odometryThread.robotPosition[0] > (x+.2)) {
                drive.circlepadMove(-moveSpeed, 0, thetaSpeed);
            } else if (odometryThread.robotPosition[0] < (x-.1) || odometryThread.robotPosition[0] > (x+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(odometryThread.robotPosition[0] - x) < 16) {
                moveSpeed = .15 + (((.6-.15)/(16)) * (Math.abs(odometryThread.robotPosition[0] - x)));
            }
        }
        drive.stop();
    }

    // moves left right (y direction)
    public void goToStrafePoint(double y, double rotation) {
        double moveSpeed = .55;
        double thetaSpeed = 0;
        while ((odometryThread.robotPosition[1] < (y-.1) || odometryThread.robotPosition[1] > (y+.1)) && opMode.opModeIsActive()) {
            thetaSpeed = -(odometryThread.robotPosition[2]+(rotation));
            if (odometryThread.robotPosition[1] < (y-.2)) {
                drive.circlepadMove(0, -moveSpeed, thetaSpeed);
            } else if (odometryThread.robotPosition[1] > (y+.2)) {
                drive.circlepadMove(0, moveSpeed, thetaSpeed);
            } else if (odometryThread.robotPosition[1] < (y-.1) || odometryThread.robotPosition[1] > (y+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(odometryThread.robotPosition[1] - y) < 9) {
                moveSpeed = .24 + (((.55-.24)/(9)) * (Math.abs(odometryThread.robotPosition[1] - y)));
            }
        }
        drive.stop();
    }
}
