package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "OdometryTests")
public class OdometryTests extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    ElapsedTime timer = new ElapsedTime();

    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};

    private void testOdometry() {
        drive.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drive.dpadMove(gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left,
                gamepad1.dpad_down);

        odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                robot.OMiddle.getCurrentPosition()};
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("OLeft", odometryInfo[0]);
        telemetry.addData("OMiddle", odometryInfo[2]);
        telemetry.addData("ORight", odometryInfo[1]);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);
        telemetry.addData("DeltaLeft", robotPosition[3]);
        telemetry.addData("DeltaRight", robotPosition[4]);
        telemetry.addData("deltatheta", robotPosition[5]);
        telemetry.addData("hor change", robotPosition[6]);
        telemetry.update();
    }

    private void queryOdometry() {
        odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                robot.OMiddle.getCurrentPosition()};
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);

        telemetry.update();
    }

    private void odometryRoutineA() {
        timer.reset();
        double initialAngle = robotPosition[2];
        while (timer.seconds() < 2 && opModeIsActive()) {
            drive.circlepadMove(-.35, 0, 0);
            queryOdometry();
        }
        //// why dont we get initialAngle here. See if it's more/less accurate
        while (robotPosition[2] < initialAngle + Math.PI && opModeIsActive()) {
            drive.circlepadMove(0, 0, .25);
            queryOdometry();
        }
        drive.brake();
        timer.reset();
        initialAngle = robotPosition[2];
        while (timer.seconds() < 2 && opModeIsActive()) {
            drive.circlepadMove(-.35, 0, 0);
            queryOdometry();
        }
        while (robotPosition[2] > initialAngle - Math.PI && opModeIsActive()) {
            drive.circlepadMove(0, 0, -.25);
            queryOdometry();
        }
        drive.brake();
        timer.reset();
        while (timer.seconds() < 30 && opModeIsActive()) {
            queryOdometry();
        }
    }

    private void odometryRoutineB() {
        timer.reset();
        queryOdometry();
        double initialAngle = robotPosition[2];
        while (timer.seconds() <= 7 && opModeIsActive()) {
            drive.circlepadMove(.53, 0, 0);
            queryOdometry();
        }
        drive.brake();
        timer.reset();
        while (timer.seconds() < 60 && opModeIsActive()) {
            queryOdometry();
            telemetry.addData("OLeft", odometryInfo[0]);
            telemetry.addData("OMiddle", odometryInfo[2]);
            telemetry.addData("ORight", odometryInfo[1]);
        }
    }

    private void odometryRoutineX() {
        queryOdometry();
        goToStrafePoint(12);
        goToPoint(-48);
        drive.brake();
        boolean middle = false;
        boolean top = false;
        double wobbleX;
        double wobbleY;

        shoot();

        timer.reset();
        while ((timer.seconds() < 4) && opModeIsActive()) {
            telemetry.addData("X", robotPosition[0]);
            telemetry.addData("Y", robotPosition[1]);
            telemetry.addData("Theta", robotPosition[2]);
            telemetry.addData("Status", "shooting");
            telemetry.addLine("NOTHING FOR BOTTOM, A FOR MIDDLE, B FOR TOP");
            telemetry.update();
            if (gamepad1.a) {
                middle = true;
            }
            if (gamepad1.b) {
                top = true;
            }
        }
        if (middle) {
            wobbleX = -96;
            wobbleY = 12;
        } else if (top) {
            wobbleX = -120;
            wobbleY = 36;
        } else {
            wobbleX = -72;
            wobbleY = 36;
        }

        goToPoint(wobbleX);
        goToStrafePoint(wobbleY);
        drive.brake();

        timer.reset();
        while ((timer.seconds() < 3) && opModeIsActive()) {
            telemetry.addData("X", robotPosition[0]);
            telemetry.addData("Y", robotPosition[1]);
            telemetry.addData("Theta", robotPosition[2]);
            telemetry.addData("Status", "wobble goal");
            telemetry.update();
        }
    }

    void shoot() {
        robot.Shooter.setPower(1);
        sleep(1000);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
        sleep(500);
        robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
        sleep(500);
        robot.Shooter.setPower(0);
    }

    void goToPoint(double x) {
        double moveSpeed = .7;
        double thetaSpeed = 0;
        while ((-robotPosition[0] < (x-.1) || -robotPosition[0] > (x+.1)) && opModeIsActive()) {
            thetaSpeed = -robotPosition[2];
            if (-robotPosition[0] < (x-.2)) {
                drive.circlepadMove(-moveSpeed, 0, thetaSpeed);
                queryOdometry();
            } else if (-robotPosition[0] > (x+.2)) {
                drive.circlepadMove(moveSpeed, 0, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[0] < (x-.1) || robotPosition[0] > (x+.1)) {
                drive.brake();
                break;
            }
            if (Math.abs(-robotPosition[0] - x) < 15) {
                moveSpeed = .15 + (((.7-.15)/(15)) * (Math.abs(-robotPosition[0] - x)));
            }
        }
    }

    void goToStrafePoint(double y) {
        double moveSpeed = .55;
        double thetaSpeed = 0;
        while ((robotPosition[1] < (y-.1) || robotPosition[1] > (y+.1)) && opModeIsActive()) {
            thetaSpeed = -robotPosition[2];
            if (robotPosition[1] < (y-.2)) {
                drive.circlepadMove(0, -moveSpeed, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[1] > (y+.2)) {
                drive.circlepadMove(0, moveSpeed, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[1] < (y-.1) || robotPosition[1] > (y+.1)) {
                drive.brake();
                break;
            }
            if (Math.abs(robotPosition[1] - y) < 7) {
                moveSpeed = .3 + (((.55-.3)/(7)) * (Math.abs(robotPosition[1] - y)));
            }
        }
    }

    void pleaseRotate (double angle) {
        double initialAngle = robotPosition[2];
        while (robotPosition[2] < initialAngle + 2*Math.PI && opModeIsActive()) {
            drive.circlepadMove(0, 0, .5);
            queryOdometry();
        }
        drive.brake();
    }

    void rotateTo0() {
        double wantedAngle = (Math.round(robotPosition[2]/(2*Math.PI))) * 2 * Math.PI;
        double driveSpeed = .55;
        while (robotPosition[2] < wantedAngle - .01 || robotPosition[2] > wantedAngle + .01 && opModeIsActive()) {
            if (robotPosition[2] < wantedAngle - .01) {
                drive.circlepadMove(0, 0, driveSpeed);
            } else if (robotPosition[2] > wantedAngle + .01) {
                drive.circlepadMove(0, 0, -driveSpeed);
            }
            // once there's a radian to go, start proportionally reducing drivespeed to .3
            if (Math.abs(robotPosition[2] - wantedAngle) < 1) {
                driveSpeed = .2 + (.2 * Math.abs(robotPosition[2] - wantedAngle));
            }
            queryOdometry();
        }
        drive.brake();
    }

    void diagonalToPoint(double x, double y) {
        rotateTo0();
        double hyp = Math.sqrt((x - robotPosition[0])*(x - robotPosition[0]) + (y - robotPosition[1])*(y - robotPosition[1]));
        double first_drive_x = (x - robotPosition[0]) / hyp;
        double first_drive_y = (y - robotPosition[1]) / hyp;
        double drive_x = first_drive_x;
        double drive_y = first_drive_y;
        while ((robotPosition[0] < x-.2 || robotPosition[0] > x+.2) && opModeIsActive()) {
            drive.circlepadMove(drive_x, drive_y, 0);
            if (Math.abs(robotPosition[0] - x) < 6) {
                drive_x = first_drive_x * .5;
                drive_y = first_drive_y * .5;
            } else {
                drive_x = first_drive_x;
                drive_y = first_drive_y;
            }
            queryOdometry();
        }
        drive.brake();
        telemetry.addData("hyp", hyp);
        telemetry.addData("first x", first_drive_x);
        telemetry.addData("first y", first_drive_y);
        queryOdometry();
        sleep(20000);
    }

    void doubleStrafeToPoint(double x, double y) {
        rotateTo0();
        double hyp, driveX, driveY;
        while (((robotPosition[0] < x-.4 || robotPosition[0] > x+.4) || (robotPosition[1] < y-.4 || robotPosition[1] > y+.4)) && opModeIsActive()) {
            hyp = Math.sqrt((x - robotPosition[0])*(x - robotPosition[0]) + (y - robotPosition[1])*(y - robotPosition[1]));
            driveX = .85 * (x - robotPosition[0]) / hyp;
            driveY = .85 * (y - robotPosition[1]) / hyp;
            if (Math.abs(robotPosition[0] - x) < 6) {
                driveX = driveX * .62;
                driveY = driveY * .62;
            }
            drive.circlepadMove(driveX, -driveY, 0);
            queryOdometry();
        }
        drive.brake();
        rotateTo0();
    }

    void odometryRoutineY() {
        doubleStrafeToPoint(24, 12);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //init robot
        robot.init(hardwareMap, telemetry);
        robot.initIMU(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                odometryRoutineA();
            } else if (gamepad1.b) {
                odometryRoutineB();
            } else if (gamepad1.x) {
                odometryRoutineX();
            } else if (gamepad1.y) {
                odometryRoutineY();
            }
            drive.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Left Odometer", robot.OLeft.getCurrentPosition());
            telemetry.addData("Right Odometer", robot.ORight.getCurrentPosition());
            telemetry.addData("Middle Odometer", robot.OMiddle.getCurrentPosition());
            queryOdometry();
        }
    }
}
