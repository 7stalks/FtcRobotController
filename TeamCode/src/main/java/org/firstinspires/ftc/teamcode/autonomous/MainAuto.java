package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ShooterRpmThread;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.WobbleThread;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

import java.util.List;
import java.util.Arrays;

@Autonomous(name = "Main Autonomous")
public class MainAuto extends LinearOpMode {

    // core:
    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    VuforiaNavigation nav = new VuforiaNavigation();
    // timers:
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime myShooterTimer = new ElapsedTime();
    ElapsedTime anotherShootTimer = new ElapsedTime();
    // threads:
    Runnable switchCamera =
            new Runnable() {
                public void run() {
                    robot.switchableCamera.setActiveCamera(robot.backWebcam);
                    nav.navigationInit(robot);
                }
            };
    Thread switchCameraThread = new Thread(switchCamera);
    WobbleThread wobbleThread = new WobbleThread(robot, this);
    ShooterRpmThread encoderThread = new ShooterRpmThread(robot, this);

    // yeah yeah, plucked straight from TensorTest... but it works!!!
    public String checkForRings(double seconds) {
        String numberOfRings = "";
        List<Recognition> updatedRecognitions;
        timer.reset();
        while (numberOfRings.equals("") && timer.seconds() < seconds && opModeIsActive()) {
            updatedRecognitions = robot.tensorFlowEngine.getUpdatedRecognitions();
            try {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
            } catch (NullPointerException err) {
                telemetry.addData("# Object Detected", 0);
                continue;
            }
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                if (recognition.getLabel().equals("Quad") || recognition.getLabel().equals("Single")) {
                    telemetry.addLine(String.format("I found a %s", recognition.getLabel()));
                    numberOfRings = recognition.getLabel();
                    break;
                }
            }
            telemetry.update();
        }
        return numberOfRings;
    }

    void shooterTimerTime(int milliseconds) {
        anotherShootTimer.reset();
        while (opModeIsActive() && anotherShootTimer.milliseconds() < milliseconds) {
            idle();
        }
    }

    void shoot(int numberOfRings, int timeout) {
        int i = 0;
        int numberOfFailedShots = 0;
        boolean attemptedShot = false;
        while (i<numberOfRings && numberOfFailedShots < timeout && opModeIsActive() && !gamepad2.back) {
            if (encoderThread.revolutionsPerMinute < 4300 && attemptedShot) {
                i++;
                attemptedShot = false;
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                shooterTimerTime(100);
                if (i==2) {
                    shooterTimerTime(300);
                }
            }
            if (encoderThread.revolutionsPerMinute > 4900 && !attemptedShot) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                attemptedShot = true;
                myShooterTimer.reset();
            }
            if (encoderThread.revolutionsPerMinute > 4800 && attemptedShot && myShooterTimer.milliseconds() > 300) {
                attemptedShot = false;
                numberOfFailedShots++;
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                shooterTimerTime(300);
            }
        }
    }

    void shootHighGoal() {
        odometryMove.zeroThetaDiagonalToPoint(-4, -21.5);
        robot.ShooterElevator.setPosition(0.345);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.Shooter.setPower(1);
        shoot(3, 3);
        robot.Shooter.setPower(0);
    }

    void shootPowerShots() {
        odometryMove.zeroThetaDiagonalToPoint(-5, -6.5);
        odometryMove.rotateTo0();
        odometryMove.deltaRotate(-0.0425);
        robot.ShooterElevator.setPosition(.472);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.Shooter.setPower(1);

        while (encoderThread.revolutionsPerMinute < 5000) {
            idle();
        }
        shoot(1, 3);
        robot.sleepTimer(100, this);
        odometryMove.deltaRotate(0.095);
        robot.sleepTimer(100, this);
        shoot(1, 3);
        robot.sleepTimer(100, this);
        odometryMove.deltaRotate(0.097);
        robot.sleepTimer(100, this);
        shoot(1, 3);
        robot.sleepTimer(50, this);
        robot.Shooter.setPower(0);

        odometryMove.rotateTo0();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            runMyOpMode();
        } catch (InterruptedException err) {
            if (encoderThread.isAlive()) {
                encoderThread.quitThread = true;
            }
            if (wobbleThread.isAlive()) {
                wobbleThread.quitThread = true;
            }
            throw err;
        }
        if (encoderThread != null && encoderThread.isAlive()) {
            encoderThread.quitThread = true;
        }
        if (wobbleThread != null && wobbleThread.isAlive()) {
            wobbleThread.quitThread = true;
        }
    }

    public void runMyOpMode() throws InterruptedException {
        // initialization things
        robot.init(hardwareMap, telemetry);
        robot.closeWobble();
        robot.initWobble();
        robot.initVuforia(hardwareMap, telemetry);
        robot.initTFOD(telemetry);
        robot.tensorFlowEngine.activate();
        telemetry.update();
        sleep(200);
        telemetry.setMsTransmissionInterval(5);

        // loop through tensor during init to see if we have anything
        List<Recognition> beginningUpdatedRecognitions;
        while (!isStarted() && !isStopRequested()) {
            beginningUpdatedRecognitions = robot.tensorFlowEngine.getUpdatedRecognitions();
            if (beginningUpdatedRecognitions != null) {
                for (Recognition recognition : beginningUpdatedRecognitions) {
                    telemetry.addData("Recognition", recognition.getLabel());
                }
                if (beginningUpdatedRecognitions.size() == 0) {
                    telemetry.addData("Recognition", "None");
                }
                if (isStopRequested()) {
                    break;
                }
            }
            telemetry.update();
        }
        //// the start ends right here

        // tensor section. gets the number of rings (we'll have to fine tune the number of seconds)
        // before turning it into an int because that makes me more comfortable
        // then it turns off tensor so it stops eating away our power
        String stringNumberOfRings = checkForRings(1);
        int numberOfRings = 0;
        if (stringNumberOfRings.equals("Quad")) {
            numberOfRings = 4;
        } else if (stringNumberOfRings.equals("Single")) {
            numberOfRings = 1;
        }
        robot.tensorFlowEngine.deactivate();
        switchCameraThread.start();

        //vuforia time! gotta move over to the picture too. odometry time
        odometryMove.goToPoint(3, 0);
        odometryMove.zeroThetaDiagonalToPoint(24, 24);
        odometryMove.rotateTo0();

        // waits for the camera to switch. as soon as it's done it joins the thread
        while (nav == null && opModeIsActive()) {
            idle();
        }
        encoderThread.start();
        wobbleThread.start();
        wobbleThread.position = robot.wobbleRotatorUp;

        // waits until it sees a target and then averages 100 snapshots
        double avgX = 0, avgY = 0, avgRot = 0;
        while (!nav.targetVisible && opModeIsActive()) {
            nav.navigationNoTelemetry();
        }
        int sampleSize = 200;
        for (int i = 0; i < sampleSize; i++) {
            avgX += nav.X + 8;
            avgY += nav.Y;
            avgRot += nav.Rotation + Math.PI/2;
        }
        nav.targetsUltimateGoal.deactivate();
        robot.vuforia.close();
        avgX = avgX / sampleSize;
        avgY = avgY / sampleSize;
        avgRot = avgRot / sampleSize;
        telemetry.addData("avg x, y, z", Arrays.toString(new double[]{avgX, avgY, avgRot}));
        telemetry.update();
        Log.v("VUFORAISTUFF", "avgX: " + avgX);
        Log.v("VUFORAISTUFF", "avgY + .6: " + (avgY+0.6));
        Log.v("VUFORAISTUFF", "robotPosition[2]: " + odometry.robotPosition[2]);
        odometry.inputVuforia(avgX, avgY-.7, odometry.robotPosition[2]);

        // shoots the rings
        shootPowerShots();
        encoderThread.quitThread = true;

        try {
            robot.vuforia.close();
            if (robot.vuforia != null) {
                robot.vuforia = null;
            }
        } catch (NullPointerException err) {
            telemetry.addData("err", err);
            Log.i("vu error", err.toString());
        }

        // calculates where it needs to go to drop wobble using numberOfRings from earlier
        int wobbleX, wobbleY;
        if (numberOfRings == 0) {
            wobbleX = 8;
            wobbleY = -48;
        } else if (numberOfRings == 1) {
            wobbleX = 32;
            wobbleY = -24;
        } else {
            wobbleX = 56;
            wobbleY = -48;
        }
        if (encoderThread.isAlive()) {
            encoderThread.quitThread = true;
            if (encoderThread != null) {
                encoderThread = null;
            }
        }

        // proceeds to go to that point and drop the wobble goal
        odometryMove.zeroThetaDiagonalToPoint(wobbleX+2, wobbleY+4);
        wobbleThread.position = robot.wobbleRotatorMinimum-225;
        while (robot.getWobblePosition() > robot.wobbleRotatorMinimum + 100) {
            idle();
        }
        robot.openWobble();
        robot.sleepTimer(600, this);

        // moves a little to the right
        timer.reset();
        while (timer.milliseconds() < 750 && opModeIsActive()) {
            drive.circlepadMove(0, -1, 0);
        }
        drive.brake();

        // move up the wobble rotator to pickup position and hightail it to the other wobble
        wobbleThread.position = robot.wobbleRotatorMinimum-125;
        odometryMove.diagonalToPoint(-30, -51, -Math.PI/2);
        drive.brake();

        // inch into the wobble goal and then clamp onto it before moving it up
        odometryMove.diagonalToPoint(-37.2, -51, -Math.PI/2);
        drive.brake();
        robot.sleepTimer(100, this);
        robot.closeWobble();
        robot.sleepTimer(300, this);
        wobbleThread.position = robot.wobbleRotatorUp;
        odometryMove.diagonalToPoint(-39, -35, -Math.PI/2);

        // get back to the drop zone and drop the wobble
        odometryMove.diagonalToPoint(wobbleX-7, wobbleY + 24, 0);
        odometryMove.diagonalToPoint(wobbleX+1, wobbleY + 11, 0);
        drive.brake();
        wobbleThread.position = robot.wobbleRotatorMinimum-250;
        robot.sleepTimer(250, this);
        wobbleThread.quitThread = true;

        robot.openWobble();
        robot.sleepTimer(250, this);

        // move onto the line and then finish
        timer.reset();
        while (timer.milliseconds() < 350 && opModeIsActive()) {
            drive.circlepadMove(0, -1, 0);
        }
        drive.brake();

        odometryMove.goToPoint(10, 0);
        if (wobbleThread != null) {
                wobbleThread.quitThread = true;
        }
    }
}
