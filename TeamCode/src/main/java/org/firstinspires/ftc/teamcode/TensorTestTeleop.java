package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@TeleOp(name = "Tensor Test")
public class TensorTestTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    ElapsedTime timer = new ElapsedTime();

    public String checkForRings(int seconds) {
        String numberOfRings = "";
        List<Recognition> updatedRecognitions = robot.tensorFlowEngine.getUpdatedRecognitions();
        timer.reset();
        while (numberOfRings.equals("") && timer.seconds() < seconds) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
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
        robot.tensorFlowEngine.activate();
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            String numberOfRings = checkForRings(5);
            telemetry.addData("Number of rings", numberOfRings);
        }
    }
}