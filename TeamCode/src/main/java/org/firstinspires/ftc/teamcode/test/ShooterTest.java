package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ShooterRpmThread;

import java.util.Arrays;

@TeleOp (name = "shooter test")
@Disabled
public class ShooterTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ShooterRpmThread encoderThread = new ShooterRpmThread(robot, this);
    ElapsedTime shooterTimer = new ElapsedTime();
    ElapsedTime timeoutShooterTimer = new ElapsedTime();

    int rings = 0;
    String TAG = "ShooterTest";

    void shooterTimerTime(int milliseconds) {
        shooterTimer.reset();
        while (opModeIsActive() && !gamepad2.back && shooterTimer.milliseconds() < milliseconds) {
            idle();
        }
    }

    void shoot(int numberOfRings) {
        robot.ShooterElevator.setPosition(.324);
        robot.Shooter.setPower(1);
        int i = 0;
        int numberOfFailedShots = 0;
        boolean attemptedShot = false;
        timeoutShooterTimer.reset();
        while (i < numberOfRings && numberOfFailedShots < 3 && opModeIsActive() && !gamepad2.back) {
            if (encoderThread.revolutionsPerMinute < 4700 && attemptedShot) {
                Log.v(TAG, "Inside of the i++ part");
                i++;
                attemptedShot = false;
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                shooterTimerTime(100);
                if (i==2) {
                    shooterTimerTime(200);
                }
            }
            if (encoderThread.revolutionsPerMinute > 4900 && !attemptedShot) {
                Log.v(TAG, "Inside fo the shooting part, about to put the shooter to max position");

                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
                attemptedShot = true;
                timeoutShooterTimer.reset();
//                 shooterTimerTime(25);
            }
            if (encoderThread.revolutionsPerMinute > 4800 && attemptedShot && timeoutShooterTimer.milliseconds() > 300) {
                Log.v(TAG, "I just timed out, 500 ms passed since last shot");
                attemptedShot = false;
                numberOfFailedShots++;
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
                shooterTimerTime(300);
            }
            Log.v(TAG, "--------");
            Log.v(TAG, "time: " + System.currentTimeMillis());
            Log.v(TAG, "i: " + i);
            Log.v(TAG, "failed shots: " + numberOfFailedShots);
            Log.v(TAG, "attempted shot?: " + attemptedShot);
            Log.v(TAG, "rpm: " + encoderThread.revolutionsPerMinute);
            Log.v(TAG, "timeout timer: " + timeoutShooterTimer.milliseconds());
            Log.v(TAG, "position of the servo: " + robot.ShooterServo.getPosition());
        }
        telemetry.update();
        robot.Shooter.setPower(0);
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        encoderThread.start();
        waitForStart();

        while (opModeIsActive()) {
            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.dpad_up && rings < 3) {
                rings++;
            }
            if (gamepad1.dpad_down && rings > 1) {
                rings--;
            }

            if (gamepad1.start) {
                shoot(rings);
            }

            telemetry.addData("rings", rings);
            telemetry.update();
        }
        encoderThread.quitThread = true;
    }
}
