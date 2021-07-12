package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

@TeleOp(name = "The Beeg One")
public class RemoteShootTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);

    double finalAngle = 0;
    double yInchesToGoal = 24;
    double xInchesToGoal = 60;
    double distanceToGoal = 0;

    public void fancyRotate(double angle) {
        double wantedAngle = ((Math.round(odometry.robotPosition[2]/(2*Math.PI))) * 2 * Math.PI) + angle;
        double driveSpeed = .65;
        while (odometry.robotPosition[2] < wantedAngle - .0065 || odometry.robotPosition[2] > wantedAngle + .0065 && opModeIsActive()) {
            if (odometry.robotPosition[2] < wantedAngle - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
                odometry.queryOdometry();
            } else if (odometry.robotPosition[2] > wantedAngle + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
                odometry.queryOdometry();
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .27
            if (Math.abs(odometry.robotPosition[2] - wantedAngle) < 1) {
                driveSpeed = .27 + (.2 * Math.abs(odometry.robotPosition[2] - wantedAngle));
            }
            odometry.queryOdometry();
        }
        drive.brake();
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {

            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad1.start) {
                odometry.inputVuforia(-80, 0, 0);
            }

            if (gamepad1.back) {
                // should be pointing towards 0, 0?
                finalAngle = -Math.atan(odometry.robotPosition[1] / -odometry.robotPosition[0]);
                if (odometry.robotPosition[1] < -10) {
                    finalAngle -= .02;
                }
                distanceToGoal = Math.sqrt(Math.pow(odometry.robotPosition[0], 2) + Math.pow(odometry.robotPosition[1], 2));
                if (odometry.robotPosition[1] > 20 && distanceToGoal >= 140) {
                    finalAngle += .02;
                }
                fancyRotate(finalAngle);
                if (distanceToGoal >= 100) {
                    if (odometry.robotPosition[1] > 10) {
                        robot.ShooterElevator.setPosition(.49);
                    } else {
                        robot.ShooterElevator.setPosition(.47);
                    }
                } else {
                    robot.ShooterElevator.setPosition(.52);
                }
                Log.v("REMOTE SHOOTER", "distance to goal: " + distanceToGoal);
                Log.v("REMOTE SHOOTER", "final angle: " + finalAngle);
            }

            if (gamepad2.right_trigger > 0.2) {
                robot.ShooterNew.setVelocity(83);
            } else {
                robot.ShooterNew.setVelocity(0);
            }
            if (gamepad2.left_trigger > 0.2) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
            } else {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
            }
            if  (gamepad2.right_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .0035);
            } else if (gamepad2.left_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .0035);
            }

            telemetry.addData("pidf coeffs", robot.ShooterNew.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

            telemetry.addData("shooter elevator", robot.ShooterElevator.getPosition());
            odometry.queryOdometry();
        }
    }
}
