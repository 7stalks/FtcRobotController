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
    ElapsedTime wobbleTimer = new ElapsedTime();
    VuforiaNavigation nav = new VuforiaNavigation();
    ElapsedTime sleepTimer = new ElapsedTime();
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    EncoderThread encoderThread = new EncoderThread(robot, this);
    ElapsedTime shooterTimer = new ElapsedTime();

    double wobblePosition = 0.0;
    boolean wobbleCaught = false;
    boolean intakeOn = false;
    boolean wobbleUp = true;

    double[] odometryInfo;
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

    void timer_sleep(int milliseconds) {
        sleepTimer.reset();
        while (sleepTimer.milliseconds() < milliseconds && opModeIsActive()) {}
    }

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
            }
            if (shooterTimer.milliseconds() > 325) {
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
            telemetry.addData("wpbb;e servo", robot.WobbleServo.getPosition());
            if (encoderThread.revolutionsPerMinute > 4400) {
                telemetry.addLine("Can shoot");
            } else {
                telemetry.addLine("Can NOT shoot");
            }
            telemetry.addData("shooter servo position", robot.ShooterServo.getPosition());
            telemetry.addData("revs per minute", encoderThread.revolutionsPerMinute);
            odometry.queryOdometry();
        }
        if (encoderThread.isAlive()) {
            encoderThread.join();
            encoderThread.quitThread = true;
        }
    }
}
