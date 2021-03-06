package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class GoBildaDrive {

    // introduce variables and members
    private final RobotHardware robot;

    double r;
    double robotAngle;
    double rFront;
    double rBack;
    double lFront;
    double lBack;

    // constructor
    public GoBildaDrive(RobotHardware givenRobot) {
        robot = givenRobot;
    }

    /**
     * Put your circlepad sticks into this to move the robot in full 360 degree motion
     * @param leftStickY the raw left stick's y -- make sure to negate!!
     * @param leftStickX the raw left stick's x -- make sure to negate!!
     * @param rightStickX the raw right stick's x, don't negate
     */
    public void circlepadMove(double leftStickY, double leftStickX, double rightStickX) {
        rightStickX = -rightStickX;
        // Make sure that the circle pad sticks don't accidentally move the robot
        if (Math.abs(leftStickX) < robot.stickThres && Math.abs(leftStickY) < robot.stickThres
                && Math.abs(rightStickX) < robot.stickThres) {
            this.brake();
        } else {
            // Create the magnitude (or radius, r) and angle of the sticks
            r = Math.hypot(leftStickX, leftStickY);
            robotAngle = Math.atan2(leftStickY, leftStickX) - (Math.PI / 4);

            // Set power to the motors corresponding to the angle and magnitude
            rFront = r * Math.cos(robotAngle) + rightStickX;
            rBack = r * Math.sin(robotAngle) + rightStickX;
            lFront = r * Math.sin(robotAngle) - rightStickX;
            lBack = r * Math.cos(robotAngle) - rightStickX;
            robot.RightFront.setPower(-rFront);
            robot.RightBack.setPower(-rBack);
            robot.LeftFront.setPower(-lFront);
            robot.LeftBack.setPower(-lBack);
        }
    }

    /**
     * Allows for faster movement forwards, backwards, left, and right. circlepadMove() isn't as
     * fast as possible when going in these directions
     * @param right the right dpad
     * @param up the up dpad
     * @param left the left dpad
     * @param down the down dpad
     */
    public void dpadMove(boolean right, boolean up, boolean left, boolean down) {
        if (right) {
            robot.RightFront.setPower(1);
            robot.RightBack.setPower(-1);
            robot.LeftFront.setPower(-1);
            robot.LeftBack.setPower(1);
        } else if (up) {
            robot.RightFront.setPower(1);
            robot.RightBack.setPower(1);
            robot.LeftFront.setPower(1);
            robot.LeftBack.setPower(1);
        } else if (left) {
            robot.RightFront.setPower(-1);
            robot.RightBack.setPower(1);
            robot.LeftFront.setPower(1);
            robot.LeftBack.setPower(-1);
        } else if (down) {
            robot.RightFront.setPower(-1);
            robot.RightBack.setPower(-1);
            robot.LeftFront.setPower(-1);
            robot.LeftBack.setPower(-1);
        }
    }

    /**
     * Stops the drive. Sets power to 0 on all drive motors (effectively braking)
     */
    public void brake() {
        robot.RightFront.setPower(0);
        robot.RightBack.setPower(0);
        robot.LeftFront.setPower(0);
        robot.LeftBack.setPower(0);
    }

    // Test one motor at a time
    public void motorTest(boolean up, boolean right, boolean down, boolean left) {
        if (up) {
            robot.RightFront.setPower(1);
        } else if (right) {
            robot.RightBack.setPower(1);
        } else if (down) {
            robot.LeftBack.setPower(1);
        } else if (left) {
            robot.LeftFront.setPower(1);
        } else {
            brake();
        }
    }
}
