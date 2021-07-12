package org.firstinspires.ftc.teamcode.ultimategoal.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

@TeleOp(name = "Odometry Experimental Values Test")
public class OdometryExperimentTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    ElapsedTime timer = new ElapsedTime();

    double oldMiddleTicks = 0;

    void rotateToPoint(double dTheta) {
        // some constants to be referenced
        double initialTheta = odometry.robotPosition[2];
        double newTheta = initialTheta + dTheta;
        double driveSpeed = .85;

        // while outside of the bounds (less than bottom and greater than the top bounds), get there
        while (odometry.robotPosition[2] < newTheta - .01 || odometry.robotPosition[2] > newTheta + .01 && opModeIsActive()) {
            if (odometry.robotPosition[2] < newTheta - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
            } else if (odometry.robotPosition[2] > newTheta + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(odometry.robotPosition[2] - newTheta) < 1) {
                driveSpeed = .2 + (.2 * Math.abs(odometry.robotPosition[2] - newTheta));
            }
            odometry.queryOdometry();
        }
        drive.brake();
    }

    public void myDiagonalToPoint(double x, double y, double rotation) {
        double initialX, initialY, driveX, driveY, distance;
        double thetaSpeed;
        while (((odometry.robotPosition[0] < x-.1 || odometry.robotPosition[0] > x+.1) || (odometry.robotPosition[1] < y-.1 || odometry.robotPosition[1] > y+.1))
                || (odometry.robotPosition[2] > rotation - .01 || odometry.robotPosition[2] < rotation + .01) && opModeIsActive()) {
            thetaSpeed = -.7*(odometry.robotPosition[2]-rotation);
            distance = Math.sqrt(Math.pow(Math.abs(odometry.robotPosition[0] - x), 2) + Math.pow(Math.abs(odometry.robotPosition[1] - y), 2));
            initialX = (x - odometry.robotPosition[0]) / distance;
            initialY = (y - odometry.robotPosition[1]) / distance;
            driveX = initialX * Math.cos(-odometry.robotPosition[2]) - initialY * Math.sin(-odometry.robotPosition[2]);
            driveY = initialX * Math.sin(-odometry.robotPosition[2]) + initialY * Math.cos(-odometry.robotPosition[2]);
            if (distance < 12) {
                driveX = driveX * (.325 + (.95-.325)*(distance/12));
                driveY = driveY * (.325 + (.95-.325)*(distance/12));
            }
            drive.circlepadMove(driveX, -driveY, thetaSpeed);
            odometry.queryOdometry();
        }
        drive.brake();
    }

    void getSeparation() {
        rotateToPoint(6*Math.PI);
        while (opModeIsActive()) {
            telemetry.addLine("Change the wheel base separation");
            telemetry.addLine("Use the left stick's y to change it");
            telemetry.addLine("Press B to quit");
            if (gamepad1.left_stick_y < -.3) {
                odometry.robotEncoderWheelDistance += .01;
                sleep(200);
            } else if (gamepad1.left_stick_y > .3) {
                odometry.robotEncoderWheelDistance -= .01;
                sleep(200);
            }
            if (gamepad1.b) {
                break;
            }
            telemetry.addData("wheel separation", odometry.robotEncoderWheelDistance);
            telemetry.addData("number of pis", odometry.robotPosition[2]/Math.PI);
            odometry.queryOdometry();
        }
        for (int i = 0; i<=2; i++) {
            odometry.robotPosition[i] = 0;
        }
    }

    void getOffset(double oldTicks) {
        myDiagonalToPoint(0, 0, Math.PI/2);
        drive.brake();
        while (!gamepad1.b && opModeIsActive()) {
            drive.brake();
            telemetry.addLine("Press B to return");
            telemetry.addData("ticks per inch", 306.3816404153158);
            telemetry.addData("horizontal ticks", robot.OMiddle.getCurrentPosition() - oldTicks);
            telemetry.addData("horizontal ticks per degree", odometry.horizontalEncoderTickPerDegreeOffset);
            if (gamepad1.left_stick_y < -.3) {
                odometry.horizontalEncoderTickPerDegreeOffset += 5;
                sleep(100);
            } else if (gamepad1.left_stick_y > .3) {
                odometry.horizontalEncoderTickPerDegreeOffset -= 5;
                sleep(100);
            }
            odometry.queryOdometry();
        }
        for (int i = 0; i<=2; i++) {
            odometry.robotPosition[i] = 0;
        }
    }

    void mySpecialRotate(double rotation) {
//        while ()
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //init robot
        robot.init(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Press A for theta test and X for hor offset test");
            if (gamepad1.a) {
                getSeparation();
            }
            if (gamepad1.x) {
                getOffset(oldMiddleTicks);
                oldMiddleTicks = robot.OMiddle.getCurrentPosition();
            }
            if (gamepad1.y) {
                odometryMove.diagonalToPoint(12, 12, Math.PI/2);
                drive.brake();
            }
//            if (gamepad1.b) {
//                odometryMove.zeroThetaDiagonalToPoint(12, 12);
//                drive.brake();
//            }
            telemetry.addData("Left Odometer", robot.OLeft.getCurrentPosition());
            telemetry.addData("Right Odometer", robot.ORight.getCurrentPosition());
            telemetry.addData("Middle Odometer", robot.OMiddle.getCurrentPosition());
            odometry.queryOdometry();
        }
    }
}
