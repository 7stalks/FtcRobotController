package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class BetterVuforia extends VuforiaLocalizerImpl {
    public BetterVuforia(Parameters parameters) {
        super(parameters);
    }

    public void close() {
        super.close();
    }
}
