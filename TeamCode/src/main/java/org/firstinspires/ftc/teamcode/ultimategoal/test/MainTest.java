package org.firstinspires.ftc.teamcode.ultimategoal.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.ShooterRpmThread;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

@TeleOp(name = "Main test", group = "Robot")
public class MainTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime intakeTimer = new ElapsedTime();
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    ShooterRpmThread encoderThread = new ShooterRpmThread(robot, this);
    ElapsedTime shooterTimer = new ElapsedTime();
    ElapsedTime manualWobbleTimer = new ElapsedTime();
    ElapsedTime myElapsedTime = new ElapsedTime();
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
    boolean setBackToStart = true;
    boolean didNotShoot = true;
    int position = 0;

    boolean myBoolean = false;

    // gamepad 1 sticks: control drive
    // gamepad 1 A: turns on/off the intake
    // gamepad 1 B: reverses on/off the intake
    // gamepad 1 X: rotate a lil to the left
    // gamepad 1 Y: rotate a lil to the right
    // gamepad 1 start: marks the high goal position
    // gamepad 1 back: goes to high goal position
    // gamepad 1 left bumper: wobble to pickup-ready position
    // gamepad 1 right bumper: wobble to lifted position
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

    double averageOfLastRPMs() {
        int total = 0;
        for (int i = 0; i < 50; i++) {
            total += encoderThread.revolutionsPerMinute;
        }
        return total * .02;
    }

    void goToHighGoal() {
        double wantedAngle = (Math.round(odometry.robotPosition[2] / (2 * Math.PI))) * 2 * Math.PI;
        odometryMove.dirtyDiagonalToPoint(0, 0, wantedAngle);
        odometryMove.rotateTo0();
    }

    double[] deltaTimes = new double[]{1000, 1000, 1000};
    int x = 0;
    double lastTime = 0;
    double currentDeltaTime = 0;

    void updateDeltaTimesList() {
        currentDeltaTime = shooterTimer.milliseconds() - lastTime;
        if (currentDeltaTime > 4000) {
            x = 0;
            deltaTimes[1] = 1000;
            deltaTimes[2] = 1000;
            Log.v("SHOOTER", "just set x to 0 normally");

        } else if (x > 2) {
            x = 0;
            Log.v("SHOOTER", "just set x to 0 because it was >2");
        }
        deltaTimes[x] = currentDeltaTime;
        Log.v("SHOOTER", "here is my current delta time: " + currentDeltaTime);
        x++;
        lastTime = shooterTimer.milliseconds();
    }

    boolean twoRingsShot() {
        return deltaTimes[0] > 1400 && deltaTimes[1] < 800 && deltaTimes[2] < 800;
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
        shooterTimer.reset();
        myElapsedTime.reset();

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

            //// intake
            // gamepad 1 a can turn on and off the intake, b can reverse it and turn on/off
            if (intakeTimer.seconds() > .2) {
                if (!intakeOn) {
                    if (gamepad1.a) {
                        robot.ShooterElevator.setPosition(robot.SHOOTER_ELEVATOR_MIN);
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
            if (gamepad2.right_bumper && robot.ShooterElevator.getPosition() < 1) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() + .008);
            } else if (gamepad2.left_bumper && robot.ShooterElevator.getPosition() > robot.SHOOTER_ELEVATOR_MIN) {
                robot.ShooterElevator.setPosition(robot.ShooterElevator.getPosition() - .008);
            }
            telemetry.addData("shooter elevator position", robot.ShooterElevator.getPosition());

            // gamepad 2 left trigger gets the servo that hits the rings into the shooter wheel
            if ((gamepad2.left_trigger > .1) && (encoderThread.revolutionsPerMinute > 4900)) {
                Log.v("SHOOTER", "RPM is greater than 4800");
                Log.v("SHOOTER", "didNotShoot: " + didNotShoot);

                if (averageOfLastRPMs() > 4900 && didNotShoot) {
                    Log.v("SHOOTER", "timer: " + shooterTimer.milliseconds());

                    updateDeltaTimesList();
                    if (twoRingsShot()) {
                        robot.sleepTimer(200, this);
                        Log.v("SHOOTER", "I AM HERE RIGHT AFTER THE DELAY");
                    }

                    robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                    robot.sleepTimer(150, this);
                    setBackToStart = false;
                    didNotShoot = false;
                    myElapsedTime.reset();
                }

            } else if (robot.ShooterServo.getPosition() <= robot.SHOOTER_SERVO_START) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                telemetry.addData("Hey shooter", "I'm inside the servo start place");
                setBackToStart = true;
            }

            if (myElapsedTime.milliseconds() > 200 && setBackToStart) {
                didNotShoot = true;
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
                robot.ShooterElevator.setPosition(.53);
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
            if (gamepad2.x) {
                position = robot.wobbleRotatorMinimum;
                robot.WobbleCatcherFront.setPosition(robot.wobbleCatcherFrontMax);
                robot.WobbleCatcherBack.setPosition(robot.wobbleCatcherBackMin);
            } else if (gamepad2.y) {
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

            if (gamepad1.right_bumper) {
                odometryMove.rotate(Math.PI/9);
            }

            telemetry.addData("wobble catcher back position", robot.WobbleCatcherBack.getPosition());
            telemetry.addData("wobble catcher front position", robot.WobbleCatcherFront.getPosition());
            telemetry.addData("revs per minute", encoderThread.revolutionsPerMinute);
            telemetry.addData("shooter servo position", robot.ShooterServo.getPosition());
            odometry.queryOdometry();
        }
        if (encoderThread.isAlive()) {
            encoderThread.quitThread = true;
        }
    }
}
