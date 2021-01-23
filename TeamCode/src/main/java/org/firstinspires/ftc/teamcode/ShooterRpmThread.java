package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.OdometryThread;

public class ShooterRpmThread extends Thread {

    // the 1:1 motor goes 6000 rpm, which is 100 rots per sec, 0.1 rots per ms
    // ONE ROT PER 10 MS

    // 28 ticks per revolution

    RobotHardware robot;
    LinearOpMode opMode;
    public double encoderDifferencePerMillisecond;
    public double encoderDifferencePerSecond;
    public double revolutionsPerSecond;
    public volatile double revolutionsPerMinute;
    double lastPosition = 0;
    double lastTime = 0;
    public volatile boolean quitThread = false;

    /**
     * Uses the shooter's encoder to calculate its rpm
     * @param robot the instantiated RobotHardware
     * @param opMode the current opMode
     */
    public ShooterRpmThread(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    public void run() {
        while(!isInterrupted() && !quitThread) {

            // gives encoder ticks per ms using the precise time before sleeping for 5 ms
            encoderDifferencePerMillisecond = (Math.abs(robot.Shooter.getCurrentPosition()) - lastPosition) / (System.currentTimeMillis() - lastTime);

            // get some values for us (the last position/time and a conversion to seconds)
            lastPosition = Math.abs(robot.Shooter.getCurrentPosition());
            lastTime = System.currentTimeMillis();
            encoderDifferencePerSecond = encoderDifferencePerMillisecond * 1000;
            revolutionsPerSecond = encoderDifferencePerSecond / 28;
            revolutionsPerMinute = revolutionsPerSecond * 60;

            // sleep for 10 ms
            try {
                Thread.sleep(10);
            } catch (InterruptedException ignored) { }
        }
    }
}
