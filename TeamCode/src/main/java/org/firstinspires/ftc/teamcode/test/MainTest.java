 package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterRpmThread;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

import java.util.Arrays;

 @TeleOp(name = "Main test", group = "Robot")
public class MainTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();
    VuforiaNavigation nav = new VuforiaNavigation();
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    ShooterRpmThread encoderThread = new ShooterRpmThread(robot, this);
    ElapsedTime myShooterTimer = new ElapsedTime();
    ElapsedTime thisIsMyTimer = new ElapsedTime();
    ElapsedTime manualWobbleTimer = new ElapsedTime();
    ElapsedTime anotherShootTimer = new ElapsedTime();
    Runnable initWobbleRunnable = new Runnable() {
        @Override
        public void run() {
            while (robot.topWobbleLimit.getState()) {
                robot.WobbleRotator.setPower(.9);
            }
            robot.WobbleRotator.setPower(0);
            robot.wobbleEncoder0 = robot.WobbleRotator.getCurrentPosition();
        }
    };
    Thread initWobble = new Thread(initWobbleRunnable);

    boolean intakeOn = false;
    int counter = 0;
    int position = 0;

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

     double funThing() {
         int total = 0;
         for (int i=0; i<4; i++) {
             total += encoderThread.revolutionsPerMinute;
         }
         return total * .01;
     }

     void shooterTimerTime(int milliseconds) {
         anotherShootTimer.reset();
         while (opModeIsActive() && !gamepad2.back && anotherShootTimer.milliseconds() < milliseconds) {
             idle();
             Log.v("TAG", "waiting in the shootertimertime");
         }
     }

     double takeAverage(int n) {
         double total = 0;
         for (int i = 0; i<n; i++) {
             total += encoderThread.revolutionsPerMinute;
         }
         return total/n;
     }

     void shoot(int numberOfRings, int timeout) {
         robot.Shooter.setPower(1);
         while (encoderThread.revolutionsPerMinute < 4800) {
             idle();
         }
         int i = 0;
         int numberOfFailedShots = 0;
         boolean attemptedShot = false;
         myShooterTimer.reset();
         while (i < numberOfRings && numberOfFailedShots < timeout && opModeIsActive() && !gamepad2.back) {
             if (encoderThread.revolutionsPerMinute < 4400 && attemptedShot) {
                 shooterTimerTime(20);
                 double avg = takeAverage(100);
                 if (avg < 4400) {
                     telemetry.addData("revs per min while in if", encoderThread.revolutionsPerMinute);
                     Log.v("TAG", "revs per min in here " + encoderThread.revolutionsPerMinute);
                     i++;
                     attemptedShot = false;
                     robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                     shooterTimerTime(300);
                     if (i == 2) {
                         shooterTimerTime(200);
                     }
                 }
             }
             if (encoderThread.revolutionsPerMinute > 4900 && !attemptedShot) {
                 telemetry.addData("      revs per min while in if", encoderThread.revolutionsPerMinute);
                 Log.v("TAG", "     revs per min in here " + encoderThread.revolutionsPerMinute);
                 robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                 attemptedShot = true;
                 myShooterTimer.reset();
             }
             if (encoderThread.revolutionsPerMinute > 4900 && attemptedShot && myShooterTimer.milliseconds() > 400) {
                 telemetry.addData("           revs per min while in if", encoderThread.revolutionsPerMinute);
                 Log.v("TAG", "          revs per min in here " + encoderThread.revolutionsPerMinute);
                 attemptedShot = false;
                 numberOfFailedShots++;
                 robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                 shooterTimerTime(300);
             }
             telemetry.addData("i", i);
             telemetry.addData("number of failed shots", numberOfFailedShots);
             telemetry.addData("attempted shot", attemptedShot);
             telemetry.addData("encoder", encoderThread.revolutionsPerMinute);
             telemetry.addLine("I'm inside of a while loop, hit BACK on GAMEPAD 2 to get out of it");
             telemetry.update();
         }
     }

     void goToHighGoal() {
         double wantedAngle = (Math.round(odometry.robotPosition[2]/(2*Math.PI))) * 2 * Math.PI;
         odometryMove.dirtyDiagonalToPoint(0, 0, wantedAngle);
         odometryMove.rotateTo0();
     }

    // there wasn't an override here before and i think it worked fine... oh well! we'll see
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        initWobble.start();
        robot.closeWobble();
        encoderThread.start();
        manualWobbleTimer.reset();
        telemetry.setMsTransmissionInterval(5);
        thisIsMyTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            //// odometry convenience stuffs and drive
            if (gamepad1.back) {
                goToHighGoal();
            }
            if (gamepad1.start) {
                odometry.inputVuforia(0, 0, 0);
            }
            if (gamepad1.y) {
                odometryMove.deltaRotate(0.097);
            }
            if (gamepad1.x) {
                odometryMove.deltaRotate(-0.097);
            }

            // drive goes to gamepad 1. the left and right sticks control circlepad, dpad is for the fast move
            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            drive.dpadMove(gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down);

            //// intake
            // gamepad 1 a can turn on and off the intake, b can reverse it and turn on/off
            if (intakeTimer.seconds() > .2) {
                if (!intakeOn) {
                    if (gamepad1.a) {
                        robot.ShooterElevator.setPosition(0);
                        robot.sleepTimer(25, this);
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

            //// shooter
            if (gamepad2.right_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .006);
            } else if (gamepad2.left_bumper) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .006);
            }
            telemetry.addData("shooter elevator position", robot.ShooterElevator.getPosition());

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if ((gamepad2.left_trigger > .1) && (encoderThread.revolutionsPerMinute > 4800)) {

                //// THIS IS PROBABLY A VERY BAD IDEA
                if (funThing() > 4800) {
                    Log.v("SHOOTER", "timer: " + thisIsMyTimer.milliseconds());
                    // myList.append(timer.milliseconds() - lastTime)
                    // lastTime = thisIsMyTimer.milliseconds();
                    // if (allThreeOfmyList is small) {
                    //      wait;
                    // }
                    robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                }
            } else if (robot.ShooterServo.getPosition() < robot.SHOOTER_SERVO_START) {
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
                robot.ShooterElevator.setPosition(.52);
            }
            if (gamepad2.b) {
                robot.ShooterElevator.setPosition(.46);
            }

            //// wobble
            // change wobble position
            if (gamepad2.dpad_up && position < 0) {
                position += 35;
            } else if (gamepad2.dpad_down && position > robot.wobbleRotatorMinimum) {
                position -= 35;
            }

            // convenience
            if (gamepad2.x || gamepad1.left_bumper) {
                position = robot.wobbleRotatorMinimum;
            } else if (gamepad2.y || gamepad1.right_bumper) {
                position = robot.wobbleRotatorFullUp;
            }
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
            telemetry.addData("revs per minute", encoderThread.revolutionsPerMinute);
            telemetry.addData("shooter servo position", robot.ShooterServo.getPosition());
            odometry.queryOdometry();

            //// super secret convenient shoot
            if (gamepad2.start) {
                shoot(3, 3);
            }
        }
        if (encoderThread.isAlive()) {
            encoderThread.quitThread = true;
        }
    }
}
