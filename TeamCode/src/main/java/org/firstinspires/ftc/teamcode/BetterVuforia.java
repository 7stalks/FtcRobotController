package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class BetterVuforia extends VuforiaLocalizerImpl {

    String TAG = "Vuforia Close";

    public BetterVuforia(Parameters parameters) {
        super(parameters);
    }

    public void close() {
        Log.v(TAG, "Before the close");
        super.close();
        Log.v(TAG, "After the close");
    }
}
