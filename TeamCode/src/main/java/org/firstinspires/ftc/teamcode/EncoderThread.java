package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.OdometryThread;

public class EncoderThread extends Thread {

    // the 1:1 motor goes 6000 rpm, which is 100 rots per sec, 0.1 rots per ms
    // ONE ROT PER 10 MS

    // TODO the rpm needs to be tested, as well as the ticks per revolution
    // 28 ticks per revolution

    RobotHardware robot;
    LinearOpMode opMode;
    public double encoderDifferencePerMillisecond;
    public double encoderDifferencePerSecond;
    public double revolutionsPerSecond;
    public double revolutionsPerMinute;
    double lastPosition = 0;
    double lastTime = 0;
    public boolean quitThread = false;

    // constructor to get the robotHardware
    public EncoderThread(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    public void run() {
        while(!EncoderThread.currentThread().isInterrupted() && opMode.opModeIsActive()) {

            // gives encoder ticks per ms using the precise time before sleeping for 5 ms
            encoderDifferencePerMillisecond = (Math.abs(robot.Shooter.getCurrentPosition()) - lastPosition)/(System.currentTimeMillis() - lastTime);

            // get some values for us (the last position/time and a conversion to seconds)
            lastPosition = Math.abs(robot.Shooter.getCurrentPosition());
            lastTime = System.currentTimeMillis();
            encoderDifferencePerSecond = encoderDifferencePerMillisecond * 1000;
            revolutionsPerSecond = encoderDifferencePerSecond / 28;
            revolutionsPerMinute = revolutionsPerSecond * 60;

            // sleep for 5 ms
            try {
                Thread.sleep(15);
            } catch (InterruptedException ignored) {}

            if (quitThread) {
                break;
            }
        }
        if (quitThread) {
            try {
                EncoderThread.currentThread().join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
