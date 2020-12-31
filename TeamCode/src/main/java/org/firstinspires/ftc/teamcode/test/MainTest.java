package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EncoderThread;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;
import org.firstinspires.ftc.teamcode.odometry.OdometryThread;

@TeleOp(name = "Main test", group = "Robot")
public class MainTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();
    VuforiaNavigation nav = new VuforiaNavigation();
    ElapsedTime sleepTimer = new ElapsedTime();
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    EncoderThread encoderThread = new EncoderThread(robot, this);
    ElapsedTime shooterTimer = new ElapsedTime();

    boolean intakeOn = false;

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
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        robot.initVuforia(hardwareMap, telemetry);
        nav.navigationInit(robot);
        robot.switchableCamera.setActiveCamera(robot.backWebcam);
        encoderThread.start();
        telemetry.setMsTransmissionInterval(1);

        waitForStart();

        while (opModeIsActive()) {

            nav.navigationNoTelemetry();
            if (nav.targetVisible && gamepad1.back) {
                double avgX = 0, avgY = 0, avgRot = 0, i;
                for (i=0; i<75; i++) {
                    avgX += (nav.X + 8);
                    avgY += nav.Y;
                    avgRot += (nav.Rotation + Math.PI/2);
                }
                avgX = avgX/i;
                avgY = avgY/i;
                avgRot = avgRot/i;
                odometry.inputVuforia(avgX, avgY, avgRot);
            }

            if (gamepad1.start) {
                odometryMove.doubleStrafeToPoint(-4, -8, 0);
            }

            if (gamepad1.x) {
                odometryMove.deltaRotate(.095);
            }

            // drive goes to gamepad 1. the left and right sticks control circlepad, dpad is for the fast move
            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            drive.dpadMove(gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down);

            // gamepad 1 a can turn on and off the intake, b can reverse it and turn on/off
            if (intakeTimer.seconds() > .2) {
                if (!intakeOn) {
                    if (gamepad1.a) {
                        robot.TopIntake.setPower(1);
                        robot.BottomIntake.setPower(1);
                        robot.ShooterElevator.setPosition(0);
                        intakeOn = true;
                        intakeTimer.reset();
                    } else if (gamepad1.b) {
                        robot.TopIntake.setPower(-1);
                        robot.BottomIntake.setPower(-1);
                        robot.ShooterElevator.setPosition(0);
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
            if (gamepad2.right_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .003);
            } else if (gamepad2.left_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .003);
            }
            telemetry.addData("shooter elevator position", robot.ShooterElevator.getPosition());

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if ((gamepad2.left_trigger > .1) && (encoderThread.revolutionsPerMinute > 4400)) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                shooterTimer.reset();
                telemetry.addData("Shooter status", "shooting");
            } else if (shooterTimer.milliseconds() > 325) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                telemetry.addData("Shooter status", "not shooting");
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
            if (gamepad2.a) {
                robot.ShooterElevator.setPosition(.36);
            }
            if (gamepad2.b) {
                robot.ShooterElevator.setPosition(.29);
            }

            if (gamepad2.dpad_up) {
                robot.WobbleRotator.setPosition(robot.WobbleRotator.getPosition() + .0015);
            }
            if (gamepad2.dpad_down) {
                robot.WobbleRotator.setPosition(robot.WobbleRotator.getPosition() - .0015);
            }

            // the back servo goes from min to max
            // the front servo goes from max to min
            // dpad left closees it
            if (gamepad2.dpad_left) {
                double backPosition = robot.WobbleCatcherBack.getPosition();
                double frontPosition = robot.WobbleCatcherFront.getPosition();
                if (!(backPosition > robot.wobbleCatcherBackMax)) {
                    robot.WobbleCatcherBack.setPosition(backPosition + robot.wobbleCatcherBackSpeed);
                    robot.WobbleCatcherFront.setPosition(frontPosition - robot.wobbleCatcherFrontSpeed);
                }
            }

            // dpad right opens it
            if (gamepad2.dpad_right) {
                double backPosition = robot.WobbleCatcherBack.getPosition();
                double frontPosition = robot.WobbleCatcherFront.getPosition();
                if (!(backPosition < robot.wobbleCatcherBackMin)) {
                    robot.WobbleCatcherBack.setPosition(backPosition - robot.wobbleCatcherBackSpeed);
                    robot.WobbleCatcherFront.setPosition(frontPosition + robot.wobbleCatcherFrontSpeed);
                }
            }

            if (gamepad2.x) {
                robot.WobbleRotator.setPosition(robot.wobbleRotatorPickup);
            }
            if (gamepad2.y) {
                robot.WobbleRotator.setPosition(robot.wobbleRotatorTop);
            }

            if (encoderThread.revolutionsPerMinute > 4400) {
                telemetry.addLine("Can shoot");
            } else {
                telemetry.addLine("Can NOT shoot");
            }
            telemetry.addData("revs per minute", encoderThread.revolutionsPerMinute);
            telemetry.addData("shooter servo position", robot.ShooterServo.getPosition());
            odometry.queryOdometry();
        }
        if (encoderThread.isAlive()) {
            encoderThread.join();
            encoderThread.quitThread = true;
        }
    }
}
