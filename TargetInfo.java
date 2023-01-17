package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;


public class TargetInfo {
    public String name;
    double x, y, z;

    public TargetInfo(VuforiaTrackable trackable) {
        name = trackable.getName();
        VuforiaTrackableDefaultListener identifiedTrackable = ((VuforiaTrackableDefaultListener) trackable.getListener());
        VectorF trans = identifiedTrackable.getVuforiaCameraFromTarget().getTranslation();
        x = trans.get(0);
        y = trans.get(1);
        z = trans.get(2);
    }
}
