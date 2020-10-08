package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OrientationSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "OdometryTests")
public class OdometryTests extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry();

    double[] odometryInfo;

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


    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        robot.init(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(2);
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        double[] robotPosition = {0, 0, 0};

        waitForStart();

        while (opModeIsActive()) {
            drive.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            drive.dpadMove(gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left,
                    gamepad1.dpad_down);


            // TODO change from 0 to robot.OMiddle,getCurrentPosition()
            odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                    0};
            robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);

            telemetry.addData("OLeft", odometryInfo[0]);
            telemetry.addData("OMiddle", odometryInfo[2]);
            telemetry.addData("ORight", odometryInfo[1]);
            telemetry.addData("X", robotPosition[0]);
            telemetry.addData("Y", robotPosition[1]);
            telemetry.addData("Theta", robotPosition[2]);
            telemetry.update();

            if (gamepad1.a) {
                drive.stop();
                while (!gamepad1.b && opModeIsActive()) {
                }
            }
        }
    }
}
