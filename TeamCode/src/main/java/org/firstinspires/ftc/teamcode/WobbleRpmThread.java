package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashSet;

public class WobbleRpmThread extends Thread {

    RobotHardware robot = new RobotHardware();
    public boolean quitThread = false;
    public ArrayList<Integer> myList = new ArrayList<>();

    public WobbleRpmThread (RobotHardware robot) {
        this.robot = robot;
    }

    public void run() {
        while (!quitThread) {
            myList.add(robot.WobbleRotator.getCurrentPosition());
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (myList.size() > 100) {
                myList.remove(0);
            }
        }
    }

    public boolean isStuck() {
        return new HashSet<>(myList).size() <= 1;
    }

    public boolean isTooFast() {
        return new HashSet<>(myList).size() >= 3;
    }
}