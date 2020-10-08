package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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


    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        robot.init(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            testOdometry();
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
        }
    }
}
