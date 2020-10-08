package org.firstinspires.ftc.teamcode.usercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Connor Servo move", group = "Connor")
public class Connor extends LinearOpMode {
    Servo servo;

    double counter = 0.01;
    double max_pos = 1.0;
    double min_pos = 0.0;

    boolean going_up = true;

    @Override
    public void runOpMode() throws InterruptedException {
        int foo = 0;
        servo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.left_trigger>0){
                // left trigger goes from 0 > 1
                // at zero hold position
                // stick at 1/2 servo == .5
                // a
            }
            if (gamepad1.left_trigger > 0){
                counter += 0.01;
                if (counter >= max_pos){
                    counter = max_pos;
                }
            }else if (gamepad1.right_trigger > 0){
                counter -= 0.01;
                if (counter <= min_pos){
                    counter = min_pos;
                }
            }
            servo.setPosition(counter);
            telemetry.addData("Running", counter);
            telemetry.update();

        }
    }
}
