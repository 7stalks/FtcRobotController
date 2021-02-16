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
        while (!isInterrupted() && !quitThread) {
            if (moveWobble) {
                robot.wobbleSetPosition(position);
            }
        }
        robot.WobbleRotator.setPower(0);
    }
}
