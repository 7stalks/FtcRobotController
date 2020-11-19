package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.File;

@TeleOp(name = "loop de loop")
public class odometryCalibrationLoop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);

    ElapsedTime timer = new ElapsedTime();

    double encoderCountsPerIn = 306.3816404153158;
    double CumLeft = 0;
    double CumRight = 0;

    boolean doneTesting = false;

    double[] wheelBaseSeparationValues = new double[200];
    int counter = 0;

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        // set telemetry
        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting to be started");
        telemetry.update();

        waitForStart();

        while (!doneTesting && opModeIsActive()) {
            // initialize some values
            double top_angle = robot.top_imu.getAngularOrientation().firstAngle;
            double bottom_angle = robot.bottom_imu.getAngularOrientation().firstAngle;

            double angle = (top_angle + bottom_angle) / 2;
            double firstAngle = angle;

            // do the turn
            while (bottom_angle < 90 && opModeIsActive()) {
                if (angle < 60) {
                    drive.circlepadMove(0, 0, robot.PIVOT_SPEED);
                } else {
                    drive.circlepadMove(0, 0, robot.PIVOT_SPEED * .85);
                }
                top_angle = robot.top_imu.getAngularOrientation().firstAngle;
                bottom_angle = robot.bottom_imu.getAngularOrientation().firstAngle;
                angle = (top_angle + bottom_angle) / 2;
                telemetry.addData("top_angle", top_angle);
                telemetry.addData("bottom_angle", bottom_angle );
                telemetry.addData("angle", angle);
            }
            drive.stop();

            // Record IMU and encoder values to calculate the constants
            double newTopAngle = robot.top_imu.getAngularOrientation().firstAngle;
            double newBottomAngle = robot.bottom_imu.getAngularOrientation().firstAngle;

            double newAngle = (newTopAngle + newBottomAngle) / 2 ;
            double finalAngle = newAngle - firstAngle;
            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
                telemetry.addData("top_angle", top_angle);
                telemetry.addData("bottom_angle", bottom_angle );
                telemetry.addData("IMU Angle", newAngle);
                telemetry.addData("final angle", finalAngle);
                telemetry.update();
            }

            //TODO Rewrite this
//            double encoderDifference = Math.abs(robot.OLeft.getCurrentPosition()) +
//                    Math.abs(robot.ORight.getCurrentPosition());
//            double verticalEncoderTickOffsetPerDegree = encoderDifference / (robot.imu.getAngularOrientation().firstAngle - firstAngle); //changed from angle to imu.getangle

            double left = robot.ORight.getCurrentPosition() - CumLeft;
            double right = robot.OLeft.getCurrentPosition() - CumRight;
            CumLeft = robot.OLeft.getCurrentPosition();
            CumRight = robot.ORight.getCurrentPosition();
            double middle = robot.OMiddle.getCurrentPosition();
            double encoderPerDegree = -(left + right) / (2 * finalAngle);
            double wheelBaseSeparation = (encoderPerDegree * 360) / (Math.PI * encoderCountsPerIn);
            double horizontalTickOffset = (middle / finalAngle);

            telemetry.addData("Right", right);
            telemetry.addData("Left", left);
            telemetry.addData("Right minus left", right - left);
            telemetry.addData("Wheel base separation", wheelBaseSeparation);
            telemetry.addData("Horizontal tick offset per degree", horizontalTickOffset);
            telemetry.addLine("Press A to accept these values");
            telemetry.addLine("Press X to decline these values");
            telemetry.addLine("Press B and Y together to conclude testing");
            telemetry.update();
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    wheelBaseSeparationValues[counter] = wheelBaseSeparation;
                    counter += 1;
                    break;
                }
                if (gamepad1.x) {
                    break;
                }
                if (gamepad1.b && gamepad1.y) {
                    doneTesting = true;
                    break;
                }
            }
        }
    }
}
