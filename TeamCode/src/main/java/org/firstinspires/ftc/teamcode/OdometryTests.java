package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "OdometryTests")
public class OdometryTests extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry();
    ElapsedTime timer = new ElapsedTime();

    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};

    private void testImu() {
        // (goes before waitForStart())
        // obtain the heading (is this 0 degrees? test it please)
        double angle = robot.imu.getAngularOrientation().firstAngle;
        telemetry.addData("ANGLE", angle);

        // ONLY WORKS IF angle IS LESS THAN 270 TEST THIS
        if (gamepad1.a) {
            double initialAngle = robot.imu.getAngularOrientation().firstAngle;

            while (angle < (initialAngle + 90) && opModeIsActive()) {
                drive.circlepadMove(1, 0, .5);
                angle = robot.imu.getAngularOrientation().firstAngle;
                telemetry.addData("ANGLE", angle);
                telemetry.update();
            }
            drive.stop();
        }
    }

    //            if (gamepad1.a) {
//                timer.reset();
//                while (timer.time(TimeUnit.SECONDS) < 2 && opModeIsActive()) {
//                    drive.circlepadMove(.8, 0, 0);
//                    telemetry.addData("OLeft", robot.OLeft.getCurrentPosition());
//                    telemetry.addData("ORight", robot.ORight.getCurrentPosition());
//                    telemetry.addData("OMiddle", robot.OMiddle.getCurrentPosition());
//                    telemetry.update();
//                }
//                drive.stop();
//                sleep(1000);
//                telemetry.addData("Left divided by right", robot.OLeft.getCurrentPosition() / robot.ORight.getCurrentPosition());
//                telemetry.update();
//                sleep(10000);
//            }

    private void testOdometry() {
        drive.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drive.dpadMove(gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left,
                gamepad1.dpad_down);

        odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                robot.OMiddle.getCurrentPosition()};
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("OLeft", odometryInfo[0]);
        telemetry.addData("OMiddle", odometryInfo[2]);
        telemetry.addData("ORight", odometryInfo[1]);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);
        telemetry.addData("DeltaLeft", robotPosition[3]);
        telemetry.addData("DeltaRight", robotPosition[4]);
        telemetry.addData("deltatheta", robotPosition[5]);
        telemetry.addData("hor change", robotPosition[6]);
        telemetry.update();
    }

    private void queryOdometry() {
        odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                robot.OMiddle.getCurrentPosition()};
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);
        telemetry.update();
    }

    private void odometryRoutineA() {
        timer.reset();
        double initialAngle = robotPosition[2];
        while (timer.seconds() < 2 && opModeIsActive()) {
            drive.circlepadMove(-.35, 0, 0);
            queryOdometry();
        }
        //// why dont we get initialAngle here. See if it's more/less accurate
        while (robotPosition[2] < initialAngle + Math.PI && opModeIsActive()) {
            drive.circlepadMove(0, 0, .25);
            queryOdometry();
        }
        drive.stop();
        timer.reset();
        initialAngle = robotPosition[2];
        while (timer.seconds() < 2 && opModeIsActive()) {
            drive.circlepadMove(-.35, 0, 0);
            queryOdometry();
        }
        while (robotPosition[2] > initialAngle - Math.PI && opModeIsActive()) {
            drive.circlepadMove(0, 0, -.25);
            queryOdometry();
        }
        drive.stop();
        timer.reset();
        while (timer.seconds() < 30 && opModeIsActive()) {
            queryOdometry();
        }
    }

    private void odometryRoutineB() {
        timer.reset();
        queryOdometry();
        double initialAngle = robotPosition[2];
        while (timer.seconds() <= 7 && opModeIsActive()) {
            drive.circlepadMove(.53, 0, 0);
            queryOdometry();
        }
        drive.stop();
        timer.reset();
        while (timer.seconds() < 60 && opModeIsActive()) {
            queryOdometry();
            telemetry.addData("OLeft", odometryInfo[0]);
            telemetry.addData("OMiddle", odometryInfo[2]);
            telemetry.addData("ORight", odometryInfo[1]);
        }
    }

    private void odometryRoutineX() {
        queryOdometry();
        goToPoint(72);
//        timer.reset();
//        while (timer.seconds() < 20 && opModeIsActive()) {
//            queryOdometry();
//        }
    }

    void goToPoint(double x) {
        double moveSpeed = .5;
        if (x < 10) {
            moveSpeed = .2 + (.03*x);
        }
        while (robotPosition[0] < (x-.1) || robotPosition[0] > (x+.1) && opModeIsActive()) {
            if (robotPosition[0] < (x-.2)) {
                drive.circlepadMove(moveSpeed, 0, 0);
                queryOdometry();
            } else if (robotPosition[0] > (x+.2)) {
                drive.circlepadMove(-moveSpeed, 0, 0);
                queryOdometry();
            } else if (robotPosition[0] < (x-.1) || robotPosition[0] > (x+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(robotPosition[0] - x) < 10) {
                moveSpeed = .15 + (((.5-.15)/(10)) * (Math.abs(robotPosition[0] - x)));
            }
        }
    }

    void goToStrafePoint(double y) {
        while (robotPosition[1] < (y-.1) || robotPosition[1] > (y+.1) && opModeIsActive()) {
            if (robotPosition[1] < (y-.2)) {
                drive.circlepadMove(0, -.4, 0);
                queryOdometry();
            } else if (robotPosition[1] > (y+.2)) {
                drive.circlepadMove(0, .4, 0);
                queryOdometry();
            } else if (robotPosition[1] < (y-.1) || robotPosition[1] > (y+.1)) {
                drive.stop();
                break;
            }
        }
    }

    void pleaseRotate (double angle) {
        double initialAngle = robotPosition[2];
        while (robotPosition[2] < initialAngle + 2*Math.PI) {
            drive.circlepadMove(0, 0, .5);
            queryOdometry();
        }
        drive.stop();
    }

    // need to do rotate now

    private void odometryMaybeFindCountsPerIn() {
        drive.circlepadMove(-.5, 0, 0);
        
    }

    private void odometryAndVuforia() {

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
            if (gamepad1.a) {
                odometryRoutineA();
            } else if (gamepad1.b) {
                odometryRoutineB();
            } else if (gamepad1.x) {
                odometryRoutineX();
            }
            testOdometry();
        }
    }
}
