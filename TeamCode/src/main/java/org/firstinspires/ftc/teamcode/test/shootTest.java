package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterRpmThread;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

import java.util.Iterator;
import java.util.Set;

@TeleOp(name = "shoot test")
public class shootTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);
    ShooterRpmThread encoderThread = new ShooterRpmThread(robot, this);
    ElapsedTime myShooterTimer = new ElapsedTime();

    void shoot(int numberOfRings) {
        int i = 0;
        int numberOfFailedShots = 0;
        boolean attemptedShot = false;
        while (i<numberOfRings && numberOfFailedShots < 3 && opModeIsActive() && !gamepad2.back) {
            if (encoderThread.revolutionsPerMinute < 4000 && attemptedShot) {
                robot.sleepTimer(50, this);
                if (encoderThread.revolutionsPerMinute < 4200) {
                    i++;
                    attemptedShot = false;
                    robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                    sleep(50);
                }
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

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        encoderThread.start();
        telemetry.setMsTransmissionInterval(5);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Set<Thread> threadSet = Thread.getAllStackTraces().keySet();
            Iterator<Thread> it = threadSet.iterator();
            int i = 1;
            while (it.hasNext()) {
                telemetry.addData("thread " + i, it.next());
                i++;
            }
            telemetry.addData("encoder", encoderThread.revolutionsPerMinute);
            telemetry.update();

            if (gamepad1.a) {
                odometryMove.rotateTo0();
                robot.ShooterElevator.setPosition(.245);
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                robot.Shooter.setPower(1);

                shoot(1);
                robot.sleepTimer(125, this);
                odometryMove.rotate(0.1065);
                shoot(1);
                robot.sleepTimer(125, this);
                odometryMove.rotate(0.213);
                shoot(1);
                robot.sleepTimer(50, this);
                robot.Shooter.setPower(0);

                odometryMove.rotateTo0();
            }
        }
    }
}
