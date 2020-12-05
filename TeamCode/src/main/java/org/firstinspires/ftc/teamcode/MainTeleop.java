package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main")
public class MainTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();

    double wobblePosition = 0.0;
    boolean wobbleCaught = false;
    boolean intakeOn = false;
    boolean wobbleUp = true;

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

    // there wasn't an override here before and i think it worked fine... oh well! we'll see
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            // drive goes to gamepad 1. the left and right sticks control circlepad, dpad is for the fast move
            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            drive.dpadMove(gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_down);

            // gamepad 1 a can turn on and off the intake, b can reverse it and turn on/off
            if (intakeTimer.seconds() > .2) {
                if (!intakeOn) {
                    if (gamepad1.a) {
                        robot.TopIntake.setPower(.85);
                        robot.BottomIntake.setPower(1);
                        intakeOn = true;
                        intakeTimer.reset();
                    } else if (gamepad1.b) {
                        robot.TopIntake.setPower(-.85);
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

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if (gamepad2.left_trigger > .1) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
            } else if (robot.ShooterServo.getPosition() >= robot.SHOOTER_SERVO_MAX) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
            }

            // gamepad 2 right trigger (analog) gets the shooter motor itself. has to hold down for it to work
            if (gamepad2.right_trigger > .1) {
                robot.Shooter.setPower(1);
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
                if (wobblePosition < .55 && wobbleUp) {
                    robot.WobbleServo.setPosition(wobblePosition + .01);
                    if (wobblePosition + .01 >= .55) {
                        wobbleUp = false;
                    }
                } else {
                    robot.WobbleServo.setPosition(wobblePosition - .01);
                    if (wobblePosition - .01 <= .01) {
                        wobbleUp = true;
                    }
                }
            }

            if (gamepad2.dpad_right) {
                if (!wobbleCaught) {
                    robot.WobbleCatcher.setPosition(1);
                    wobbleCaught = true;
                    sleep(20);
                } else {
                    robot.WobbleCatcher.setPosition(0);
                    wobbleCaught = false;
                }
            }

            // gamepad 2 right bumper gets shooter elevator
            if (gamepad2.right_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .003);
            } else if (gamepad2.left_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .003);
            }
        }
    }
}