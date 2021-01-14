package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.EncoderThread;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.WobbleThread;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

import java.util.List;
import java.util.Arrays;

@Autonomous(name = "Blue Close Shoot 3 Encoder")
public class BlueShoot3EncoderTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    VuforiaNavigation nav = new VuforiaNavigation();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime myShooterTimer = new ElapsedTime();
    Runnable switchCamera =
            new Runnable() {
                public void run() {
                    robot.switchableCamera.setActiveCamera(robot.backWebcam);
                    nav.navigationInit(robot);
                }
            };
    Thread switchCameraThread = new Thread(switchCamera);
//    int wobblePositionThread = -165;
//    Runnable carryWobble = new Runnable() {
//        public void run() {
//            robot.wobbleToPosition(wobblePositionThread, telemetry);
//        }
//    };
//    Thread wobbleThread = new Thread(carryWobble);
    WobbleThread wobbleThread = new WobbleThread(robot, this);
    EncoderThread encoderThread = new EncoderThread(robot, this);

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

    void shootHighGoal() {
        odometryMove.doubleStrafeToPoint(-4, -21.5, 0);
        robot.ShooterElevator.setPosition(0.345);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.Shooter.setPower(1);
        robot.sleepTimer(1000, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        robot.sleepTimer(500, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.sleepTimer(450, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        robot.sleepTimer(500, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.sleepTimer(450, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        robot.sleepTimer(500, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.sleepTimer(450, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        robot.sleepTimer(500, this);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
//        robot.sleepTimer(700, this);
//        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
//        robot.sleepTimer(400, this);
//        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.Shooter.setPower(0);
    }

//    void shoot(int numberOfShots) {
//        int i = 0;
//        boolean shot = false;
//        boolean start = false;
//        boolean resetTheTimer = false;
//        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
//        sleep(75);
//        maxTimer.reset();
//        while (opModeIsActive() && i < numberOfShots) {
//            if (encoderThread.revolutionsPerMinute > 4600 && start) {
//                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
//                shot = true;
//                start = false;
//                maxTimer.reset();
//            }
//
//            if (maxTimer.milliseconds() > 300)  {
//                shot = false;
//            }
//
//            if (shot && encoderThread.revolutionsPerMinute < 4000 && maxTimer.milliseconds() < 300) {
//                i++;
//                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
//                shooterTimer.reset();
//                sleep(100);
//            }
//
//            if (maxTimer.milliseconds() > 300 || encoderThread.revolutionsPerMinute < 4200) {
//                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
//                if (!start && !resetTheTimer) {
//                    shooterTimer.reset();
//                    resetTheTimer = true;
//                }
//            }
//            if (shooterTimer.milliseconds() > 200) {
//                start = true;
//            }
//
//            if (encoderThread.revolutionsPerMinute > 4400) {
//                telemetry.addLine("can shoot");
//            } else {
//                telemetry.addLine("can not shoot");
//            }
//            telemetry.addData("rpm", encoderThread.revolutionsPerMinute);
//            telemetry.addData("start", start);
//            telemetry.addData("shpoter timer", shooterTimer.milliseconds());
//            telemetry.addData("i", i);
//            telemetry.addLine(encoderThread.output);
//            telemetry.update();
//        }
//    }

    void shoot(int numberOfRings) {
        int i = 0;
        int numberOfFailedShots = 0;
        boolean attemptedShot = false;
        while (i<numberOfRings && numberOfFailedShots < 3 && opModeIsActive() && !gamepad2.back) {
            if (encoderThread.revolutionsPerMinute < 4000 && attemptedShot) {
                i++;
                attemptedShot = false;
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                sleep(50);
            }
            if (encoderThread.revolutionsPerMinute > 4000 && !attemptedShot) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                attemptedShot = true;
                myShooterTimer.reset();
            }
            if (encoderThread.revolutionsPerMinute > 4000 && attemptedShot && myShooterTimer.milliseconds() > 250) {
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

//    void shootForReals() {
//        boolean shot = false;
//        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
//        sleep(150);
//        boolean atStart = true;
//        int counter = 0;
//        while (opModeIsActive()) {
//            if (encoderThread.revolutionsPerMinute > 4400 && atStart) {
//                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
//                sleep(300);
//                atStart = false;
//                shot = true;
//                counter++;
//            }
//            if (encoderThread.revolutionsPerMinute < 4100 && shot) {
//                break;
//            }
//            if (encoderThread.revolutionsPerMinute > 4100 && shot) {
//                shot = false;
//                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
//                sleep(300);
//                atStart = true;
//            }
//            if (counter > 3) {
//                break;
//            }
//        }
//    }


    void shootPowerShots() {
        odometryMove.doubleStrafeToPoint(-4, -9, 0);
        odometryMove.rotateTo0();
        robot.ShooterElevator.setPosition(.245);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        robot.Shooter.setPower(1);

        shoot(1);
        robot.sleepTimer(25, this);
        odometryMove.rotate(0.1003);
        robot.sleepTimer(125, this);
        shoot(1);
        robot.sleepTimer(25, this);
        odometryMove.rotate(0.2006);
        robot.sleepTimer(125, this);
        shoot(1);
        robot.sleepTimer(50, this);
        robot.Shooter.setPower(0);

        odometryMove.rotateTo0();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            runMyOpMode();
        } catch (InterruptedException err) {
            if (switchCameraThread.isAlive()) {
                switchCameraThread.join();
            }
            if (encoderThread.isAlive()) {
                encoderThread.join();
            }
            if (wobbleThread.isAlive()) {
                wobbleThread.join();
                wobbleThread.quitThread = true;
            }
        }
    }


    public void runMyOpMode() throws InterruptedException {
        // initialization things
        robot.init(hardwareMap, telemetry);
        robot.closWobble();
        robot.initVuforia(hardwareMap, telemetry);
        robot.initTFOD(telemetry);
        robot.tensorFlowEngine.activate();
        telemetry.update();
        sleep(1000);
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
        String stringNumberOfRings = checkForRings(.5);
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
        odometryMove.doubleStrafeToPoint(12, 24, 0);

        // waits for the camera to switch. as soon as it's done it joins the thread
        while (nav == null && opModeIsActive()) {
            idle();
        }
        switchCameraThread.join();
        encoderThread.start();

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
        odometry.inputVuforia(avgX, avgY, avgRot);

        // shoots the rings
        shootPowerShots();
        encoderThread.quitThread = true;

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
        // proceeds to go to that point and drop the wobble goal
        odometryMove.doubleStrafeToPoint(wobbleX+2, wobbleY+2, 0);
        while (robot.WobbleRotator.getCurrentPosition() > -65) {
            robot.WobbleRotator.setPower(-1);
        }
        while (robot.WobbleRotator.getCurrentPosition() > -160) {
            robot.WobbleRotator.setPower(-.4);
        }
        robot.WobbleRotator.setPower(0);
        robot.openWobble();
        robot.sleepTimer(600, this);

        // moves a little to the right and then back to the line
        timer.reset();
        while (timer.milliseconds() < 600 && opModeIsActive()) {
            drive.circlepadMove(0, -.9, 0);
        }
        drive.stop();

        // move up the wobble rotator to pickup position and hightail it to the other wobble
        wobbleThread.position = -180;
        wobbleThread.start();
        odometryMove.testDoubleStrafeToPoint(-30, -50, -Math.PI/2);
        drive.stop();
//        odometryMove.rotate(-Math.PI/2);
//        odometryMove.testDoubleStrafeToPoint(-35, -38.5, -Math.PI/2);
//        drive.stop();

        // inch into the wobble goal and then clamp onto it before moving it up
        odometryMove.testDoubleStrafeToPoint(-37, -50, -Math.PI/2);
        drive.stop();
        robot.closWobble();
        robot.sleepTimer(300, this);
        wobbleThread.position = -150;
        robot.sleepTimer(250, this);

        // get back to the drop zone and drop the wobble
//        odometryMove.testDoubleStrafeToPoint(-30, -43, -Math.PI/2);
//        odometryMove.rotate(0);
        odometryMove.doubleStrafeToPoint(wobbleX+1, wobbleY + 15, 0);
        drive.stop();
//        wobbleThread.join();
        wobbleThread.quitThread = true;

        robot.WobbleRotator.setPower(0);
        robot.openWobble();
//        while(robot.WobbleRotator.getCurrentPosition() > -185) {
//            robot.WobbleRotator.setPower(-.4);
//        }
        robot.sleepTimer(300, this);

        // move onto the line and then finish
        timer.reset();
        while (timer.milliseconds() < 400 && opModeIsActive()) {
            drive.circlepadMove(0, -1, 0);
        }
        odometryMove.goToPoint(10, 0);
//        while (opModeIsActive()) {
//            odometry.queryOdometry();
//        }
        if (wobbleThread.isAlive()) {
            wobbleThread.join();
            wobbleThread.quitThread = true;
        }
    }
}
