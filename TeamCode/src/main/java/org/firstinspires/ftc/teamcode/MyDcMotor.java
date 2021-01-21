package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public abstract class MyDcMotor implements DcMotor {

    String TAG = "motor thing";

    @Override
    public int getCurrentPosition() {
        Log.v(TAG, "hi");
        return getCurrentPosition();
    }
}