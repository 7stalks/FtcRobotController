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

@Autonomous(name = "Blue Shoot 3")
public class BlueShoot3 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry();
    VuforiaNavigation nav = new VuforiaNavigation();
    ElapsedTime timer = new ElapsedTime();

    double[] odometryInfo;
    double[] robotPosition = {0, 0, 0};

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
        odometryInfo = new double[]{robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(),
                robot.OMiddle.getCurrentPosition()};
        robotPosition = odometry.getPosition(robotPosition, odometryInfo, telemetry);
        telemetry.addData("X", robotPosition[0]);
        telemetry.addData("Y", robotPosition[1]);
        telemetry.addData("Theta", robotPosition[2]);

        telemetry.update();
    }

    // moves forwards backwards (x direction)
    void goToPoint(double x) {
        x = -x;
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
                drive.stop();
                break;
            }
            if (Math.abs(-robotPosition[0] - x) < 15) {
                moveSpeed = .15 + (((.7-.15)/(15)) * (Math.abs(-robotPosition[0] - x)));
            }
        }
    }

    // moves left right (y direction)
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
                drive.stop();
                break;
            }
            if (Math.abs(robotPosition[1] - y) < 7) {
                moveSpeed = .3 + (((.55-.3)/(7)) * (Math.abs(robotPosition[1] - y)));
            }
        }
    }



    @Override
    public void runOpMode() {
        // initialization things. we'll have to see if it's too heavy for the robot to handle
        robot.init(hardwareMap, telemetry);
        robot.initVuforia(hardwareMap, telemetry);
        robot.initTFOD(telemetry);
        robot.tensorFlowEngine.activate();
        nav.navigationInit(robot);
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

        // TODO: will there be any moving necessary to pick up the three rings? may need 2 opmodes for this

        // tensor section. gets the number of rings (we'll have to fine tune the number of seconds)
        // before turning it into an int because that makes me more comfortable
        // then it turns off tensor so it stops eating away our power
        String stringNumberOfRings = checkForRings(2);
        int numberOfRings = 0;
        if (stringNumberOfRings.equals("Quad")) {
            numberOfRings = 4;
        } else if (stringNumberOfRings.equals("Single")) {
            numberOfRings = 1;
        }
        robot.tensorFlowEngine.deactivate();

        //vuforia time! gotta move over to the picture too. odometry time
        goToPoint(6);
        goToStrafePoint(26);
        // unsure if this will work. we'll find out
        while (!nav.targetVisible) {
            nav.navigationNoTelemetry();
        }
        robotPosition = new double[] {nav.X, nav.Y, nav.Rotation};
        queryOdometry();
        sleep(7);
    }
}
