package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

    final double MOTOR_UPDATE_PERIOD_MS = 50;
    final double MOTOR_POWER_INCREMENT = 0.065;

    private ElapsedTime timer = new ElapsedTime();
    private double lastMotorUpdateTime;
    private LinearOpMode opMode;

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initializeHardware(HardwareMap hardwareMap) {
        opMode.telemetry.addData("Status", "Initializing...");
        opMode.telemetry.update();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lastMotorUpdateTime = timer.milliseconds();

        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
    }

    public void move(double drive, double lateral, double yaw) {
        double currentTime = timer.milliseconds();
        if (currentTime - lastMotorUpdateTime > MOTOR_UPDATE_PERIOD_MS) {
            double leftFrontTarget  = drive - lateral - yaw;
            double rightFrontTarget = drive + lateral + yaw;
            double leftBackTarget   = drive + lateral - yaw;
            double rightBackTarget  = drive - lateral + yaw;
            // Normalise velocities
            double max = Math.max(Math.abs(leftFrontTarget), Math.abs(rightFrontTarget));
            max = Math.max(max, Math.abs(leftBackTarget));
            max = Math.max(max, Math.abs(rightBackTarget));
            if (max > 1.0) {
                leftFrontTarget  /= max;
                rightFrontTarget /= max;
                leftBackTarget   /= max;
                rightBackTarget  /= max;
            }
            // Ramp velocities
            double leftFrontPower  = getIncreasedPower(leftFrontDrive.getPower(), leftFrontTarget);
            double rightFrontPower = getIncreasedPower(rightFrontDrive.getPower(), rightFrontTarget);
            double leftBackPower   = getIncreasedPower(leftBackDrive.getPower(), leftBackTarget);
            double rightBackPower  = getIncreasedPower(rightBackDrive.getPower(), rightBackTarget);
            // Update velocities
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            opMode.telemetry.addData("LF", leftFrontTarget + " " + leftFrontPower);
            opMode.telemetry.addData("RF", rightFrontTarget + " " + rightFrontPower);
            opMode.telemetry.addData("LB", leftBackTarget + " " + leftBackPower);
            opMode.telemetry.addData("RB", rightBackTarget + " " + rightBackPower);
            lastMotorUpdateTime = currentTime;
        }
    }

    private double getIncreasedPower(double current, double target) {
        double diff = target - current;
        if(diff == 0)
            return current;
        double absDiff = Math.abs(diff);
        double sign = diff / absDiff;
        double valueToIncrement = sign * Math.min(absDiff, MOTOR_POWER_INCREMENT);
        return current + valueToIncrement;
    }

    public void logChasisPowers() {
        opMode.telemetry.addData("LF", leftFrontDrive.getPower());
        opMode.telemetry.addData("RF", rightFrontDrive.getPower());
        opMode.telemetry.addData("LB", leftBackDrive.getPower());
        opMode.telemetry.addData("RB", rightBackDrive.getPower());
    }
    
    public void resetEncoders() {
        setChasisRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setChasisRunMode(DcMotor.RunMode runmode) {
        leftFrontDrive.setMode(runmode);
        leftBackDrive.setMode(runmode);
        rightFrontDrive.setMode(runmode);
        rightBackDrive.setMode(runmode);
    }
}