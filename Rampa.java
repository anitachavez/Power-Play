
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ConceptRampMotorSpeed")

public class ConceptRampMotorSpeed extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  0.8;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -0.8;     // Maximum REV power applied to motor

    // Define class members
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    double  power   = 0;
    boolean rampUp  = true;
    int ramp = 0;



    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the start button
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

        if(-gamepad1.left_stick_y < -0.01) ramp = -1;
        else if(-gamepad1.left_stick_y > 0.01) ramp = 1;
        else if(gamepad1.right_stick_x < -0.01) ramp = -1;
        else if(gamepad1.left_stick_x > 0.01) ramp = 1;

        else ramp = 0;
         // Ramp the motors, according to the rampUp variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT ;
                if (power <= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT ;
                if (power >= MAX_REV ) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

        telemetry.addData("POW:", String.format("%.2f",power));
        move();
}
    private double move(){
            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.update();

            // Set the motor to the new power and pause;
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);

            sleep(CYCLE_MS);
            idle();

            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);

        telemetry.addData(">", "Done");
        telemetry.update();
        }
    }
}
