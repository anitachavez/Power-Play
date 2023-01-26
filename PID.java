package org.firstinspires.ftc.teamcode;

public class PID {
    private final double P, I, D;

    public PID(double p, double i, double d){
        this.P = p;
        this.I = i;
        this.D = d;
    }
    /**
     * Numerical approximations of a PID controller without modelling a function
     * @param currentValue current value of the sensor
     * @param expectedValue expected value for the sensor
     * @param previousError previous significant error
     * @param DELTA the time delta used for every measure
     * @return the correction value for this sensor
     * */
    public double correctionValue(double currentValue, double expectedValue, double previousError, double DELTA){
        double currentError = currentValue - expectedValue, correction;
        correction = currentError * P;
        correction += this.I * (previousError + currentError) * DELTA / 2;
        correction += this.D * (previousError + currentError) / DELTA;
        return correction;
    }
    /**
     * Assumption of a time delta of 0.001
     * @param currentValue current value of the sensor
     * @param expectedValue expected value for the sensor
     * @param previousError previous significant error
     * @return the correction error considering a delta of 0.001
     * */
    public double correctionValue(double currentValue, double expectedValue, double previousError){
        double DELTA = 0.001;
        return correctionValue(currentValue, expectedValue, previousError, DELTA);
    }
}
