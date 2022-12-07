package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleoperado")
public class Teleoperado extends LinearOpMode {
    final double UPDATE_PERIOD_MS = 50;
    final double MOTOR_POWER_INCREMENT = 1;

    final int LEFT_FRONT_POWER = 0;
    final int RIGHT_FRONT_POWER = 1;
    final int LEFT_BACK_POWER = 2;
    final int RIGHT_BACK_POWER = 3;

    private ElapsedTime timer = new ElapsedTime();
    private RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.initializeHardware(hardwareMap);

        waitForStart();
        timer.reset();
        double lastUpdateTime = timer.milliseconds();

        while (opModeIsActive()) {
            double currentTime = timer.milliseconds();
            if (currentTime - lastUpdateTime > UPDATE_PERIOD_MS){
                double[] chasisPowers = getChasisPowers();
                robot.leftFrontDrive.setPower(chasisPowers[LEFT_FRONT_POWER]);
                robot.rightFrontDrive.setPower(chasisPowers[RIGHT_FRONT_POWER]);
                robot.leftBackDrive.setPower(chasisPowers[LEFT_BACK_POWER]);
                robot.rightBackDrive.setPower(chasisPowers[RIGHT_BACK_POWER]);
                lastUpdateTime = currentTime;
            }
            
            telemetry.update();
        }
    }

    double[] getChasisPowers() {
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     = gamepad1.right_stick_x;

        double leftFrontTarget  = axial - lateral - yaw;
        double rightFrontTarget = axial + lateral + yaw;
        double leftBackTarget   = axial + lateral - yaw;
        double rightBackTarget  = axial - lateral + yaw;

        double max = Math.max(Math.abs(leftFrontTarget), Math.abs(rightFrontTarget));
        max = Math.max(max, Math.abs(leftBackTarget));
        max = Math.max(max, Math.abs(rightBackTarget));

        if (max > 1.0) {
            leftFrontTarget  /= max;
            rightFrontTarget /= max;
            leftBackTarget   /= max;
            rightBackTarget  /= max;
        }
        double[] chasisPowers = new double[4];
        chasisPowers[LEFT_FRONT_POWER] = getIncreasedPower(robot.leftFrontDrive.getPower(), leftFrontTarget);
        chasisPowers[RIGHT_FRONT_POWER] = getIncreasedPower(robot.rightFrontDrive.getPower(), rightFrontTarget);
        chasisPowers[LEFT_BACK_POWER] = getIncreasedPower(robot.leftBackDrive.getPower(), leftBackTarget);
        chasisPowers[RIGHT_BACK_POWER] = getIncreasedPower(robot.rightBackDrive.getPower(), rightBackTarget);
        telemetry.addData("LF", robot.leftFrontDrive.getPower() + " " + chasisPowers[LEFT_FRONT_POWER]);
        telemetry.addData("RF", robot.rightFrontDrive.getPower() + " " + chasisPowers[RIGHT_FRONT_POWER]);
        telemetry.addData("LB", robot.leftBackDrive.getPower() + " " + chasisPowers[LEFT_BACK_POWER]);
        telemetry.addData("RB", robot.rightBackDrive.getPower() + " " + chasisPowers[RIGHT_BACK_POWER]);
        return chasisPowers;
    }
    
    double getIncreasedPower(double current, double target) {
        double diff = target - current;
        if(diff == 0)
            return current;
        double absDiff = Math.abs(diff);
        double sign = diff / absDiff;
        double valueToIncrement = sign * Math.min(absDiff, MOTOR_POWER_INCREMENT);
        return current + valueToIncrement;
    }
}