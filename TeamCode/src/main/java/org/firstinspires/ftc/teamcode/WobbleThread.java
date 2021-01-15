package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class WobbleThread extends Thread {
    RobotHardware robot;
    LinearOpMode opMode;

    public volatile int position = 0;
    public volatile boolean quitThread = false;
    public volatile boolean moveWobble = true;

    public WobbleThread(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted() && !quitThread) {
            if (moveWobble) {
                robot.wobbleToPosition(position, opMode.telemetry);
            }
        }
        robot.WobbleRotator.setPower(0);
    }
}
