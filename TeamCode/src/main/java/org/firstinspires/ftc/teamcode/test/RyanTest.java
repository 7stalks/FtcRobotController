package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryThread;

@TeleOp(name = "RyanTest")
public class RyanTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    OdometryThread odometryThread = new OdometryThread(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    ElapsedTime timer = new ElapsedTime();

    double oldMiddleTicks = 0;

    void odometryTelemetry() {
        telemetry.addData("thread X", odometryThread.robotPosition[0]);
        telemetry.addData("thread Y", odometryThread.robotPosition[1]);
        telemetry.addData("thread Theta", odometryThread.robotPosition[2]);

    }

    void rotateToPoint(double dTheta) {
        // some constants to be referenced
        double initialTheta = odometryThread.robotPosition[2];
        double newTheta = initialTheta + dTheta;
        double driveSpeed = .6;

        // while outside of the bounds (less than bottom and greater than the top bounds), get there
        while (odometryThread.robotPosition[2] < newTheta - .01 || odometryThread.robotPosition[2] > newTheta + .01 && opModeIsActive()) {
            if (odometryThread.robotPosition[2] < newTheta - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
            } else if (odometryThread.robotPosition[2] > newTheta + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(odometryThread.robotPosition[2] - newTheta) < 1) {
                driveSpeed = .2 + (.2 * Math.abs(odometryThread.robotPosition[2] - newTheta));
            }
            odometryTelemetry();
        }
        drive.stop();
    }

    void getSeparation() {
        rotateToPoint(2*Math.PI);
        while (opModeIsActive()) {
            telemetry.addLine("Change the wheel base separation");
            telemetry.addLine("Use the left stick's y to change it");
            telemetry.addLine("Press B to quit");
            if (gamepad1.left_stick_y < -.3) {
                odometryThread.robotEncoderWheelDistance += .01;
                sleep(200);
            } else if (gamepad1.left_stick_y > .3) {
                odometryThread.robotEncoderWheelDistance -= .01;
                sleep(200);
            }
            if (gamepad1.b) {
                break;
            }
            telemetry.addData("wheel separation", odometryThread.robotEncoderWheelDistance);
            odometryTelemetry();
        }
        for (int i = 0; i<=2; i++) {
            odometryThread.robotPosition[i] = 0;
        }
    }

    void getOffset(double oldTicks) {
        rotateToPoint(.5*Math.PI);
        while (!gamepad1.b && opModeIsActive()) {
            telemetry.addLine("Press B to return");
            telemetry.addData("ticks per inch", 306.3816404153158);
            telemetry.addData("horizontal ticks", robot.OMiddle.getCurrentPosition() - oldTicks);
            telemetry.addData("horizontal ticks per degree", odometryThread.horizontalEncoderTickPerDegreeOffset);
            if (gamepad1.left_stick_y < -.3) {
                odometryThread.horizontalEncoderTickPerDegreeOffset += 5;
                sleep(100);
            } else if (gamepad1.left_stick_y > .3) {
                odometryThread.horizontalEncoderTickPerDegreeOffset -= 5;
                sleep(100);
            }
            odometryTelemetry();
        }
        for (int i = 0; i<=2; i++) {
            odometryThread.robotPosition[i] = 0;
        }
    }
    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};
    double firstOLeft = 0;
    double firstORight = 0;
    double firstOMiddle = 0;

    // kind of a central method. give it some time and it'll prolly be moved to Odometry.java
    private void queryOdometry() {
        odometryInfo = new double[]{
                robot.OLeft.getCurrentPosition() - firstOLeft,
                robot.ORight.getCurrentPosition() - firstORight,
                robot.OMiddle.getCurrentPosition() - firstOMiddle
        };
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);

        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //init robot
        robot.init(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        odometryThread.start();

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
            telemetry.addData("Left Odometer", robot.OLeft.getCurrentPosition());
            telemetry.addData("Right Odometer", robot.ORight.getCurrentPosition());
            telemetry.addData("Middle Odometer", robot.OMiddle.getCurrentPosition());
            odometryTelemetry();
            queryOdometry();
        }
        odometryThread.start();
    }
}
