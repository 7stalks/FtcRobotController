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

 @TeleOp(name = "Main test", group = "Robot")
public class MainTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();
    VuforiaNavigation nav = new VuforiaNavigation();
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    EncoderThread encoderThread = new EncoderThread(robot, this);
    ElapsedTime myShooterTimer = new ElapsedTime();
    ElapsedTime manualShooterTimer = new ElapsedTime();
    ElapsedTime manualWobbleTimer = new ElapsedTime();

    boolean intakeOn = false;
    int wobblePosition = 0;
    boolean wobbleRotatorOn = false;

    //// As of 31 December 2020:
    // gamepad 1 sticks: control drive
    // gamepad 1 A: turns on/off the intake
    // gamepad 1 B: reverses on/off the intake
    // gamepad 1 start: takes picture
    // gamepad 1 back: does all the powershot stuff after photos
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
    // gamepad 2 start: shoots 3 rings automatically

     void shoot(int numberOfRings, int timeout) {
         int i = 0;
         int numberOfFailedShots = 0;
         boolean attemptedShot = false;
         while (i < numberOfRings && numberOfFailedShots < timeout && opModeIsActive() && !gamepad2.start) {
             if (encoderThread.revolutionsPerMinute < 4450 && attemptedShot) {
                 i++;
                 attemptedShot = false;
                 robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
             }
             if (encoderThread.revolutionsPerMinute > 4600 && !attemptedShot) {
                 robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                 attemptedShot = true;
                 myShooterTimer.reset();
             }
             if (encoderThread.revolutionsPerMinute > 4500 && attemptedShot && myShooterTimer.milliseconds() > 350) {
                 attemptedShot = false;
                 numberOfFailedShots++;
                 robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                 sleep(200);
             }
             telemetry.addData("i", i);
             telemetry.addData("number of failed shots", numberOfFailedShots);
             telemetry.addData("attempted shot", attemptedShot);
             telemetry.addData("encoder", encoderThread.revolutionsPerMinute);
             telemetry.addLine("I'm inside of a while loop, hit BACK on GAMEPAD 2 to get out of it");
             telemetry.update();
         }
         robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
     }

     void shootPowerShots() {
         odometryMove.doubleStrafeToPoint(-4, -8.2, 0);
         odometryMove.rotateTo0();
         robot.ShooterElevator.setPosition(.2455);
         robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
         robot.Shooter.setPower(1);

         while (encoderThread.revolutionsPerMinute < 4600) {
             idle();
         }
         shoot(1, 2);
         robot.sleepTimer(75, this);
         odometryMove.rotate(0.11);
         robot.sleepTimer(175, this);
         shoot(1, 2);
         robot.sleepTimer(25, this);
         odometryMove.rotate(0.22);
         robot.sleepTimer(175, this);
         shoot(1, 2);
         robot.sleepTimer(50, this);
         robot.Shooter.setPower(0);

         odometryMove.rotateTo0();
     }

    // there wasn't an override here before and i think it worked fine... oh well! we'll see
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        robot.closWobble();
        robot.initVuforia(hardwareMap, telemetry);
        nav.navigationInit(robot);
        robot.switchableCamera.setActiveCamera(robot.backWebcam);
        encoderThread.start();
        manualWobbleTimer.reset();
        telemetry.setMsTransmissionInterval(1);

        waitForStart();

        while (opModeIsActive()) {

            nav.navigationNoTelemetry();
            if (nav.targetVisible && gamepad1.start) {
                double avgX = 0, avgY = 0, avgRot = 0, i;
                for (i=0; i<75; i++) {
                    avgX += (nav.X + 8);
                    avgY += nav.Y;
                    avgRot += (nav.Rotation + Math.PI/2);
                }
                avgX = avgX/i;
                avgY = avgY/i;
                avgRot = avgRot/i;
                odometry.inputVuforia(avgX, avgY, odometry.robotPosition[2]);
            }

            if (gamepad2.right_stick_button) {
                odometry.robotPosition[2] = 0;
            }

            if (gamepad1.back) {
                shootPowerShots();
            }

            if (gamepad1.y) {
                odometryMove.deltaRotate(.1);
            }
            if (gamepad1.x) {
                odometryMove.deltaRotate(-.1);
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

            if (gamepad2.right_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .003);
            } else if (gamepad2.left_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .003);
            }
            telemetry.addData("shooter elevator position", robot.ShooterElevator.getPosition());

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if ((gamepad2.left_trigger > .1) && (encoderThread.revolutionsPerMinute > 4500)) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                manualShooterTimer.reset();
            } else if (robot.ShooterServo.getPosition() < robot.SHOOTER_SERVO_START && manualShooterTimer.milliseconds() > 200){
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
                robot.ShooterElevator.setPosition(.285);
            }
            if (gamepad2.b) {
                robot.ShooterElevator.setPosition(.235);
            }


            // gamepad 2's dpad controls wobble stuff
            // up raises the entire apparatus, down lowers it
            if (gamepad2.dpad_up && wobblePosition < 192) {
                wobblePosition += 2;
            }
            if (gamepad2.dpad_down && wobblePosition > 0) {
                wobblePosition -= 2;
            }
            if (gamepad2.left_stick_button && manualWobbleTimer.milliseconds() > 200) {
                wobbleRotatorOn = !wobbleRotatorOn;
                manualWobbleTimer.reset();
            }
            if (wobbleRotatorOn) {
                robot.wobbleGoToPosition(wobblePosition, telemetry);
            }
            telemetry.addData("wobble position from dpad", wobblePosition);

            // the back servo goes from min to max
            // the front servo goes from max to min
            // dpad left closes clamp
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

            // quick shortcuts:
            // gamepad 2 x raises the wobble to pickup position
            // gamepad 2 y raises the wobble to lifting position
            if (gamepad2.x) {
                wobblePosition = robot.wobbleRotatorPickup;
            }
            if (gamepad2.y) {
                wobblePosition = robot.wobbleRotatorTop;
            }

            if (encoderThread.revolutionsPerMinute > 4400) {
                telemetry.addLine("Can shoot");
            } else {
                telemetry.addLine("Can NOT shoot");
            }
            telemetry.addData("revs per minute", encoderThread.revolutionsPerMinute);
            telemetry.addData("shooter servo position", robot.ShooterServo.getPosition());
            odometry.queryOdometry();

            if (gamepad2.back) {
                shoot(3, 3);
            }

        }
        if (encoderThread.isAlive()) {
            encoderThread.quitThread = true;
        }
        nav.targetsUltimateGoal.deactivate();
    }
}
