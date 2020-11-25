package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.Odometry;

@TeleOp(name = "RyanTest")
public class RyanTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry();
    ElapsedTime timer = new ElapsedTime();

    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};

    void queryOdometry() {
        odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                robot.OMiddle.getCurrentPosition()};
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);

        telemetry.update();
    }

    void rotateToPoint(double dTheta) {
        // some constants to be referenced
        double initialTheta = robotPosition[2];
        double newTheta = initialTheta + dTheta;
        double driveSpeed = .6;

        // while outside of the bounds (less than bottom and greater than the top bounds), get there
        while (robotPosition[2] < newTheta - .01 || robotPosition[2] > newTheta + .01 && opModeIsActive()) {
            if (robotPosition[2] < newTheta - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
            } else if (robotPosition[2] > newTheta + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(robotPosition[2] - newTheta) < 1) {
                driveSpeed = .2 + (.2 * Math.abs(robotPosition[2] - newTheta));
            }
            queryOdometry();
        }
        drive.stop();
    }

    void getOffset() {
        rotateToPoint(2*Math.PI);
        while (!gamepad1.b && opModeIsActive()) {
            telemetry.addLine("Press B to return");
            telemetry.addData("horizontal ticks", robot.OMiddle.getCurrentPosition());
            telemetry.addData("ticks per inch", 306.3816404153158);
            queryOdometry();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //init robot
        robot.init(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        queryOdometry();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Press A for theta test and X for hor offset test");
            if (gamepad1.a) {
                rotateToPoint(2*Math.PI);
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
                    queryOdometry();
                }
                for (int i = 0; i<=2; i++) {
                    robotPosition[i] = 0;
                }
            }
            if (gamepad1.x) {
                getOffset();
            }
            queryOdometry();
        }
    }
}
