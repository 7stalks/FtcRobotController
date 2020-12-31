package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

import java.util.List;

@Autonomous(name = "Blue Far Shoot 3")
public class BlueFarShoot3 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    VuforiaNavigation nav = new VuforiaNavigation();
    ElapsedTime timer = new ElapsedTime();
    Runnable switchCamera =
            new Runnable(){
                public void run(){
                    robot.switchableCamera.setActiveCamera(robot.backWebcam);
                    nav.navigationInit(robot);
                }
            };
    Thread switchCameraThread = new Thread(switchCamera);


    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};
    double firstOLeft = 0;
    double firstORight = 0;
    double firstOMiddle = 0;
    double rotation = 0;

    // yeah yeah, plucked straight from TensorTest... but it works!!! hypothetically
    public String checkForRings(int seconds) {
        String numberOfRings = "";
        List<Recognition> updatedRecognitions;
        timer.reset();
        while (numberOfRings.equals("") && timer.seconds() < seconds && opModeIsActive()) {
            updatedRecognitions = robot.tensorFlowEngine.getUpdatedRecognitions();
            try {telemetry.addData("# Object Detected", updatedRecognitions.size());}
            catch (NullPointerException err) {
                telemetry.addData("# Object Detected", 0);
                continue;
            }
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                if (recognition.getLabel().equals("Quad") || recognition.getLabel().equals("Single")) {
                    telemetry.addLine(String.format("I found a %s", recognition.getLabel()));
                    numberOfRings = recognition.getLabel();
                    break;
                }
            }
            telemetry.update();
        }
        return numberOfRings;
    }

    // kind of a central method. give it some time and it'll prolly be moved to Odometry.java
    private void queryOdometry() {
        odometryInfo = new double[]{
                robot.OLeft.getCurrentPosition() - firstOLeft,
                robot.ORight.getCurrentPosition() - firstORight,
                robot.OMiddle.getCurrentPosition() - firstOMiddle
        };
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);

        telemetry.update();
    }

    // moves forwards backwards (x direction)
    void goToPoint(double x) {
        double moveSpeed = .7;
        double thetaSpeed = 0;
        while ((robotPosition[0] < (x-.1) || robotPosition[1] > (x+.1)) && opModeIsActive()) {
            thetaSpeed = -(robotPosition[2]+(rotation));
            if (robotPosition[0] < (x-.2)) {
                drive.circlepadMove(moveSpeed, 0, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[0] > (x+.2)) {
                drive.circlepadMove(-moveSpeed, 0, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[0] < (x-.1) || robotPosition[1] > (x+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(robotPosition[0] - x) < 15) {
                moveSpeed = .15 + (((.7-.15)/(15)) * (Math.abs(robotPosition[0] - x)));
            }
        }
        drive.stop();
    }

    // moves left right (y direction)
    void goToStrafePoint(double y) {
        double moveSpeed = .55;
        double thetaSpeed = 0;
        while ((robotPosition[1] < (y-.1) || robotPosition[1] > (y+.1)) && opModeIsActive()) {
            thetaSpeed = -(robotPosition[2]+(rotation));
            if (robotPosition[1] < (y-.2)) {
                drive.circlepadMove(0, -moveSpeed, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[1] > (y+.2)) {
                drive.circlepadMove(0, moveSpeed, thetaSpeed);
                queryOdometry();
            } else if (robotPosition[1] < (y-.1) || robotPosition[1] > (y+.1)) {
                drive.stop();
                break;
            }
            if (Math.abs(robotPosition[1] - y) < 9) {
                moveSpeed = .2 + (((.55-.2)/(9)) * (Math.abs(robotPosition[1] - y)));
            }
        }
        drive.stop();
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

    @Override
    public void runOpMode() {
        // initialization things. we'll have to see if it's too heavy for the robot to handle
        robot.init(hardwareMap, telemetry);
        robot.WobbleRotator.setPosition(robot.wobbleRotatorPickup+.07);
        robot.initVuforia(hardwareMap, telemetry);
        robot.initTFOD(telemetry);
        robot.tensorFlowEngine.activate();

        telemetry.update();

        List<Recognition> beginningUpdatedRecognitions;
        while (!isStarted() && !isStopRequested()) {
            beginningUpdatedRecognitions = robot.tensorFlowEngine.getUpdatedRecognitions();
            if (beginningUpdatedRecognitions != null) {
                for (Recognition recognition : beginningUpdatedRecognitions) {
                    telemetry.addData("Recognition", recognition.getLabel());
                }
                if (beginningUpdatedRecognitions.size() == 0) {
                    telemetry.addData("Recognition", "None");
                }
            }
            telemetry.update();
        }

        // the start ends right here
        //
        //

        // TODO: will there be any moving necessary to pick up the three rings? may need 2 opmodes for this

        // tensor section. gets the number of rings (we'll have to fine tune the number of seconds)
        // before turning it into an int because that makes me more comfortable
        // then it turns off tensor so it stops eating away our power

        //TODO take this out and use the value from init
        String stringNumberOfRings = checkForRings(1);
        int numberOfRings = 0;
        if (stringNumberOfRings.equals("Quad")) {
            numberOfRings = 4;
        } else if (stringNumberOfRings.equals("Single")) {
            numberOfRings = 1;
        }
        robot.tensorFlowEngine.deactivate();
        switchCameraThread.start();

        //vuforia time! gotta move over to the picture too. odometry time
        goToPoint(6);
        goToStrafePoint(50);
        sleep(500);
        // unsure if this will work. we'll find out
        while (!nav.targetVisible && !isStopRequested()) {
            nav.navigationNoTelemetry();
        }

        robotPosition = new double[] {nav.X+8, nav.Y, nav.Rotation + Math.PI/2};
        firstOLeft = robot.OLeft.getCurrentPosition();
        firstORight = robot.ORight.getCurrentPosition();
        firstOMiddle = robot.OMiddle.getCurrentPosition();
        odometry.lastIterationOdometryInfo = new double[] {0, 0, 0};
        queryOdometry();
        sleep(3000);

        goToPoint(-3.5);
        goToStrafePoint(-22);
//        drive.circlepadMove(0, 0, -.4);
//        sleep(100);
        drive.stop();

        robot.ShooterElevator.setPosition(0.33);
        sleep(300);
        shoot();
        sleep(100);

        int wobbleX, wobbleY;
        if (numberOfRings == 0) {
            wobbleX = -8;
            wobbleY = -48;
        } else if (numberOfRings == 1) {
            wobbleX = -32;
            wobbleY = -24;
        } else {
            wobbleX = -56;
            wobbleY = -48;
        }
        sleep(10000);
    }
}
