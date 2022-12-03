package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    final double UPDATE_PERIOD_MS = 50;
    final double MOTOR_POWER_INCREMENT = 1;

    private ElapsedTime timer = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();
        double lastUpdateTime = timer.milliseconds();

        while (opModeIsActive()) {
            double currentTime = timer.milliseconds();
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            double leftFrontTarget  = axial - lateral + yaw;
            double rightFrontTarget = axial + lateral - yaw;
            double leftBackTarget   = axial + lateral + yaw;
            double rightBackTarget  = axial - lateral - yaw;

            double max = Math.max(Math.abs(leftFrontTarget), Math.abs(rightFrontTarget));
            max = Math.max(max, Math.abs(leftBackTarget));
            max = Math.max(max, Math.abs(rightBackTarget));

            if (max > 1.0) {
                leftFrontTarget  /= max;
                rightFrontTarget /= max;
                leftBackTarget   /= max;
                rightBackTarget  /= max;
            }
            double leftFrontPower = getIncreasedPower(leftFrontDrive.getPower(), leftFrontTarget);
            double rightFrontPower = getIncreasedPower(rightFrontDrive.getPower(), rightFrontTarget);
            double leftBackPower = getIncreasedPower(leftBackDrive.getPower(), leftBackTarget);
            double rightBackPower = getIncreasedPower(rightBackDrive.getPower(), rightBackTarget);
            if (currentTime - lastUpdateTime > UPDATE_PERIOD_MS){
                leftFrontDrive.setPower(leftFrontTarget);
                rightFrontDrive.setPower(rightFrontTarget);
                leftBackDrive.setPower(leftBackTarget);
                rightBackDrive.setPower(rightBackTarget);
                lastUpdateTime = currentTime;
            }
            telemetry.addData("LF", leftFrontDrive.getPower() + " " + leftFrontPower);
            telemetry.addData("RF", rightFrontDrive.getPower() + " " + rightFrontPower);
            telemetry.addData("LB", leftBackDrive.getPower() + " " + leftBackPower);
            telemetry.addData("RB", rightBackDrive.getPower() + " " + rightBackPower);
            telemetry.update();
        }
    }
    
    double getIncreasedPower(double current, double target) {
        double diff = target - current;
        double absDiff = Math.abs(diff);
        double sign = diff / absDiff;
        telemetry.addData("", sign);
        double valueToIncrement = sign * Math.min(absDiff, MOTOR_POWER_INCREMENT);
        return current + valueToIncrement;
    }
}
