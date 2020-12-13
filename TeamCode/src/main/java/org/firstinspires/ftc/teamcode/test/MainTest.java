package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

@TeleOp(name = "Main test", group = "Robot")
public class MainTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime wobbleTimer = new ElapsedTime();
    VuforiaNavigation nav = new VuforiaNavigation();
    Odometry odometry = new Odometry();

    double wobblePosition = 0.0;
    boolean wobbleCaught = false;
    boolean intakeOn = false;
    boolean wobbleUp = true;

    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};
    double firstOLeft = 0;
    double firstORight = 0;
    double firstOMiddle = 0;
    double rotation = 0;


    ////
    // gamepad 1:
    // left and right sticks and dpad control the drive
    // gamepad 2:
    // A turns on and off the intake
    // B reverses and turns on/off the intake
    // left bumper makes the shooter servo go
    // right trigger makes the shooter motor go
    // dpad up/down makes the wobble stand go up/down
    // dpad left/right manipulate the servo on top of the wobble stand
    ////

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

    void rotateTo0() {
        double wantedAngle = (Math.round(robotPosition[2]/(2*Math.PI))) * 2 * Math.PI;
        double driveSpeed = .55;
        while (robotPosition[2] < wantedAngle - .01 || robotPosition[2] > wantedAngle + .01 && opModeIsActive()) {
            if (robotPosition[2] < wantedAngle - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
            } else if (robotPosition[2] > wantedAngle + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(robotPosition[2] - wantedAngle) < 1) {
                driveSpeed = .2 + (.2 * Math.abs(robotPosition[2] - wantedAngle));
            }
            queryOdometry();
        }
        drive.stop();
    }

    void doubleStrafeToPoint(double x, double y) {
        rotateTo0();
        double hyp, driveX, driveY;
        while (((robotPosition[0] < x-.4 || robotPosition[0] > x+.4) || (robotPosition[1] < y-.4 || robotPosition[1] > y+.4)) && opModeIsActive()) {
            hyp = Math.sqrt((x - robotPosition[0])*(x - robotPosition[0]) + (y - robotPosition[1])*(y - robotPosition[1]));
            driveX = .85 * (x - robotPosition[0]) / hyp;
            driveY = .85 * (y - robotPosition[1]) / hyp;
            if (Math.abs(robotPosition[0] - x) < 6) {
                driveX = driveX * .62;
                driveY = driveY * .62;
            }
            drive.circlepadMove(driveX, -driveY, 0);
            queryOdometry();
        }
        drive.stop();
        rotateTo0();
    }

    // there wasn't an override here before and i think it worked fine... oh well! we'll see
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.initVuforia(hardwareMap, telemetry);
        nav.navigationInit(robot);

        waitForStart();

        while (opModeIsActive()) {

            nav.navigationNoTelemetry();
            if (nav.targetVisible) {
                robotPosition = new double[] {nav.X+8, nav.Y, nav.Rotation + Math.PI/2};
                firstOLeft = robot.OLeft.getCurrentPosition();
                firstORight = robot.ORight.getCurrentPosition();
                firstOMiddle = robot.OMiddle.getCurrentPosition();
                odometry.lastIterationOdometryInfo = new double[] {0, 0, 0};
            }
            queryOdometry();

            if (gamepad1.start) {
                doubleStrafeToPoint(-3.5, -24.5);
            }

            // drive goes to gamepad 1. the left and right sticks control circlepad, dpad is for the fast move
            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            drive.dpadMove(gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_down);

            // gamepad 1 a can turn on and off the intake, b can reverse it and turn on/off
            if (intakeTimer.seconds() > .2) {
                if (!intakeOn) {
                    if (gamepad1.a) {
                        robot.TopIntake.setPower(1);
                        robot.BottomIntake.setPower(1);
                        intakeOn = true;
                        intakeTimer.reset();
                    } else if (gamepad1.b) {
                        robot.TopIntake.setPower(-1);
                        robot.BottomIntake.setPower(-1);
                        intakeOn = true;
                        intakeTimer.reset();
                    }
                } else if (gamepad1.a || gamepad1.b) {
                    robot.TopIntake.setPower(0);
                    robot.BottomIntake.setPower(0);
                    intakeOn = false;
                    intakeTimer.reset();
                }
            }

            // when the intake is on, you cannot change position of shooter; when it is off you can change position of shooter using bumpers
            if (intakeOn) {
                robot.ShooterElevator.setPosition(0);
            } else {
                if (gamepad2.right_bumper) {
                    robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .003);
                } else if (gamepad2.left_bumper) {
                    robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .003);
                }
            }
            telemetry.addData("shooter elevator position", robot.ShooterElevator.getPosition());

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if (gamepad2.left_trigger > .1) {
                if (robot.Shooter.getPower() >= .90) {
                    robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                } else {
                    robot.ShooterServo.setPosition(.6);
                }
            } else if (robot.ShooterServo.getPosition() >= robot.SHOOTER_SERVO_MAX) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
            }

            // gamepad 2 right trigger (analog) gets the shooter motor itself. has to hold down for it to work
            if (gamepad2.right_trigger > .1) {
                robot.Shooter.setPower(1);
                robot.ShooterElevator.setPosition(.36);
                robot.BottomIntake.setPower(0);
                robot.TopIntake.setPower(0);
                intakeOn = false;
            } else {
                robot.Shooter.setPower(0);
            }

            // gamepad 2 dpad up makes the wobble stand go up, dpad down makes it go down
            if (gamepad2.dpad_up) {
                robot.WobbleMotor.setPower(.8);
            } else {
                robot.WobbleMotor.setPower(0);
            }

            if (gamepad2.dpad_down) {
                robot.WobbleMotor.setPower(-.8);
            } else {
                robot.WobbleMotor.setPower(0);
            }

            // gamepad 2 dpad left and right manipulate the servo that's at the top of the wobble stand
            if (gamepad2.dpad_left) {
                wobblePosition = robot.WobbleServo.getPosition();
                telemetry.addData("dpad2 left", wobblePosition);
                if (wobblePosition < .44 && wobbleUp) {
                    robot.WobbleServo.setPosition(wobblePosition + .0008);
                    if (wobblePosition + .01 >= .44) {
                        wobbleUp = false;
                    }
                } else {
                    robot.WobbleServo.setPosition(wobblePosition - .0008);
                    if (wobblePosition - .01 <= .01) {
                        wobbleUp = true;
                    }
                }
            }
            telemetry.addData("wpbb;e servo", robot.WobbleServo.getPosition());
            telemetry.update();

            if (wobbleTimer.seconds() > .2) {
                if (gamepad2.dpad_right) {
                    if (!wobbleCaught) {
                        robot.WobbleCatcher.setPosition(.85);
                        wobbleCaught = true;
                    } else {
                        robot.WobbleCatcher.setPosition(.4);
                        wobbleCaught = false;
                    }
                    wobbleTimer.reset();
                }
            }
        }
    }
}