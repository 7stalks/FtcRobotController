package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main")
public class MainTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();

    boolean intakeOn = false;
    int position = 0;

    //// As of 31 December 2020:
    // gamepad 1 sticks: control drive
    // gamepad 1 A: turns on/off the intake
    // gamepad 1 B: reverses on/off the intake
    //
    // gamepad 2 right bumper: raises the shooter
    // gamepad 2 left bumper: lowers the shooter
    // gamepad 2 left trigger: turns on the shooter motor
    // gamepad 2 right trigger: moves the shooter servo when the motor is up and running
    // gamepad 2 dpad up/down: raises up/down the wobble rotator
    // gamepad 2 dpad left/right: closes/opens the wobble clamp
    // convenience buttons:
    // gamepad 2 A: aims the shooter at the high goal
    // gamepad 2 B: aims the shooter at the power shots
    // gamepad 2 X: raises wobble rotator to pickup position
    // gamepad 2 Y: raises the wobble rotator to lifting position


    // there wasn't an override here before and i think it worked fine... oh well! we'll see
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.openWobble();
        robot.initWobble();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // drive goes to gamepad 1. the left and right sticks control circlepad, dpad is for the fast move
            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            drive.dpadMove(gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down);

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

            // gamepad 2 right bumper raises the shooter, left bumper lowers it
            if (gamepad2.right_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .003);
            } else if (gamepad2.left_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .003);
            }
            telemetry.addData("shooter elevator position", robot.ShooterElevator.getPosition());

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if (gamepad2.left_trigger > .2) {
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
                robot.BottomIntake.setPower(0);
                robot.TopIntake.setPower(0);
                intakeOn = false;
            } else {
                robot.Shooter.setPower(0);
            }

            // quick shortcuts:
            // gamepad 2 a is for the high goal
            // gamepad 2 b is for the powershots
            if (gamepad2.a) {
                robot.ShooterElevator.setPosition(.32);
            }
            if (gamepad2.b) {
                robot.ShooterElevator.setPosition(.235);
            }

            if (gamepad2.dpad_up && position < 0) {
                position += 35;
            } else if (gamepad2.dpad_down && position > robot.wobbleRotatorMinimum) {
                position -= 35;
            }

            if (gamepad2.x) {
                position = robot.wobbleRotatorMinimum;
            } else if (gamepad2.y) {
                position = robot.wobbleRotatorFullUp;
            }
            telemetry.addData("wobble encoder 0", robot.wobbleEncoder0);
            telemetry.addData("position", position);
            telemetry.addData("wobble position", robot.getWobblePosition());
            robot.wobbleSetPosition(position);

            // the back servo goes from min to max
            // the front servo goes from max to min
            // dpad left closes clamp
            // dpad right opens it
            if (gamepad2.dpad_left) {
                if (!(robot.WobbleCatcherBack.getPosition() > robot.wobbleCatcherBackMax)) {
                    robot.WobbleCatcherBack.setPosition(robot.WobbleCatcherBack.getPosition() + robot.wobbleCatcherBackSpeed);
                    robot.WobbleCatcherFront.setPosition(robot.WobbleCatcherFront.getPosition() + robot.wobbleCatcherFrontSpeed);
                }
            } else if (gamepad2.dpad_right) {
                if (!(robot.WobbleCatcherBack.getPosition() < robot.wobbleCatcherBackMin)) {
                    robot.WobbleCatcherBack.setPosition(robot.WobbleCatcherBack.getPosition() - robot.wobbleCatcherBackSpeed);
                    robot.WobbleCatcherFront.setPosition(robot.WobbleCatcherFront.getPosition() - robot.wobbleCatcherFrontSpeed);
                }
            }
            if (!robot.topWobbleLimit.getState()) {
                robot.wobbleEncoder0 = robot.WobbleRotator.getCurrentPosition();
            }

            telemetry.addData("wobble catcher back position", robot.WobbleCatcherBack.getPosition());
            telemetry.addData("wobble catcher front position", robot.WobbleCatcherFront.getPosition());
            telemetry.update();
        }
    }
}