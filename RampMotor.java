package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.Range;

public class RampMotor extends DcMotorImpl {

    private static final double INCREMENT = 0.05, MIN = -1.0, MAX = 1.0;
    private boolean rampUp = true;
    private double power;

    public RampMotor(DcMotor motor){
        this(motor.getController(), motor.getPortNumber(), Direction.FORWARD);
    }

    public RampMotor(DcMotor motor, Direction direction){
        this(motor.getController(), motor.getPortNumber(), direction);
    }

    private RampMotor(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    private void setRamp(double power){
        if(this.power <= 0 && power >= 0) rampUp = true;
        else if(this.power >= 0 && power <= 0) rampUp = false;
    }
    @Override
    public void setPower(double power){
        setRamp(power);
        if(rampUp){
            this.power+=INCREMENT;
            if(this.power >= power) this.power = power;
        }
        else{
            this.power-=INCREMENT;
            if(this.power <= power) this.power = power;
        }
        super.setPower(Range.clip(this.power, MIN, MAX));
    }
}
