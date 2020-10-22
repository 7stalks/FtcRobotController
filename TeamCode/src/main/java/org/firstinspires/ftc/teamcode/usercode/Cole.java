package org.firstinspires.ftc.teamcode.usercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Cole Servo move", group = "Cole")
public class Cole extends LinearOpMode {
    Servo servo;
    DcMotor motor;
    DigitalChannel sensor;

    double counter = 0;
    double max_pos = 1.0;
    double min_pos = 0;

    boolean going_up = true;


    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");
        sensor = hardwareMap.get(DigitalChannel.class, "sensor");
        sensor.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while (opModeIsActive()) {


            if (sensor.getState() == false) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                motor.setPower(0);
            } else {
                if (gamepad1.left_stick_y > 0.01 || gamepad1.left_stick_y < -0.01) {
                    motor.setPower(gamepad1.left_stick_y);
                } else {
                    motor.setPower(0);
                }
            }

            if (sensor.getState() == false) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                if (gamepad1.left_stick_x != 0) {

                    if (gamepad1.left_stick_x > 0) {
                        counter += 0.001;
                        if (counter > max_pos) {
                            counter = max_pos;
                        }
                    } else {
                        counter -= 0.001;
                        if (counter < min_pos) {
                            counter = min_pos;
                        }
                    }
                    servo.setPosition((counter));
                }

            }

            telemetry.addData("Digital Touch", "Is Pressed");
            telemetry.addData("no", counter);
            telemetry.update();

        }

    }
}

