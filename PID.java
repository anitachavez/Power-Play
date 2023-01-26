package org.firstinspires.ftc.teamcode;

public class PID {
    private final double P, I, D;

    public PID(double p, double i, double d){
        this.P = p;
        this.I = i;
        this.D = d;
    }
    /**
     * Correction values assumed
     * It is not the best method, but derives from the literal concept of PID.
     * */
    public double correctionValue(double currentValue, double expectedValue, double previousError, double DELTA){
        double currentError = currentValue - expectedValue, correction;
        correction = currentError * P;
        correction += this.I * (previousError + currentError) * DELTA / 2;
        correction += this.D * (previousError + currentError) / DELTA;
        return correction;
    }
    public double correctionValue(double currentValue, double expectedValue, double previousError){
        double DELTA = 0.001;
        return correctionValue(currentValue, expectedValue, previousError, DELTA);
    }
}
