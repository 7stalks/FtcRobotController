package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class WobbleThread extends Thread {
    RobotHardware robot;
    LinearOpMode opMode;

    public int position = 0;
    public boolean quitThread = false;
    public boolean moveWobble = true;

    public WobbleThread(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    public void run() {
        while (!WobbleThread.currentThread().isInterrupted() && opMode.opModeIsActive()) {
            if (moveWobble) {
                robot.wobbleToPosition(position, opMode.telemetry);
            }
            if (quitThread) {
                robot.WobbleRotator.setPower(0);
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
