package org.firstinspires.ftc.teamcode.ultimategoal.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

@Disabled
@TeleOp(name = "Tensor Test")
public class TensorTestTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime timer = new ElapsedTime();

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

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.initVuforia(hardwareMap, telemetry);
        robot.initTFOD(telemetry);
        robot.tensorFlowEngine.activate();
        telemetry.update();

        while (!isStarted()) {
            List<Recognition> updatedRecognitions;
            updatedRecognitions = robot.tensorFlowEngine.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Recognition", recognition.getLabel());
                }
            }
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addLine("Press A to test");
            telemetry.update();
            if (gamepad1.a) {
                String numberOfRings = checkForRings(5);
                timer.reset();
                while (timer.seconds() < 3) {
                    telemetry.addData("Number of rings", numberOfRings);
                    telemetry.update();
                }
            }
        }
    }
}