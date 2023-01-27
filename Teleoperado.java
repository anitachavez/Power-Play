package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Teleoperado")
public class Teleoperado extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private double errorElevator = 0;

    @Override
    public void runOpMode() {
        robot.initializeHardware(hardwareMap);

        waitForStart();
        boolean wasClickedStart = false, usePot = true;

        while (opModeIsActive()) {
            // **************************
            // *     CONTROL CHASIS     *
            // **************************
            double powerMultiplier = 0.6;
            double turnSensitivity = 0.8;
            if(gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0)
                powerMultiplier = 0.9;
            else if(gamepad1.right_trigger > 0)
                powerMultiplier *= 0.5;
            double axial   = getTangencialJoystick(-gamepad1.left_stick_y) * powerMultiplier;
            double lateral =  getTangencialJoystick(gamepad1.left_stick_x) * powerMultiplier;
            double yaw     =  gamepad1.right_stick_x * powerMultiplier * turnSensitivity;
            robot.move(axial, lateral, yaw);
            // ****************************
            // *     CONTROL ELEVADOR     *
            // ****************************
            // Decide if use encoders or pot
            if(gamepad2.start && wasClickedStart != gamepad2.start)
                usePot = !usePot;

            ElevatorPositions targetElevatorPosition = null;
            if(gamepad2.right_trigger > 0)
                robot.moveElevator(robot.ELEVATOR_RISE_POWER);
            else if(gamepad2.left_trigger > 0)
                robot.moveElevator(robot.ELEVATOR_LOWER_POWER);
            else if(gamepad2.dpad_down) targetElevatorPosition = ElevatorPositions.LOW;
            else if(gamepad2.dpad_right) targetElevatorPosition = ElevatorPositions.MEDIUM;
            else if(gamepad2.dpad_up) targetElevatorPosition = ElevatorPositions.HIGH;
            else if(gamepad2.dpad_left) targetElevatorPosition = ElevatorPositions.GROUND;
            else robot.moveElevator(0);
            if(usePot)
                errorElevator = robot.usePot(targetElevatorPosition, errorElevator);
            else errorElevator = robot.moveToPosition(targetElevatorPosition, errorElevator);
            robot.showPotVoltage();
            robot.showElevatorTicks();
            // **************************
            // *     CONTROL INTAKE     *
            // **************************
            if(gamepad2.a) robot.openIntake();
            else robot.closeIntake();
            telemetry.addData("Using pot", usePot);
            telemetry.update();
        }
    }

    private double getTangencialJoystick(double joystickInput) {
        return 1.2 * Math.tan(0.7 * joystickInput);
    }

    private void getConnectionInfo(){
        telemetry.addLine(robot.leftFrontDrive.getConnectionInfo());
        telemetry.addLine(robot.rightFrontDrive.getConnectionInfo());
        telemetry.addLine(robot.leftBackDrive.getConnectionInfo());
        telemetry.addLine(robot.rightBackDrive.getConnectionInfo());
    }
    private void motorInfo(){
        telemetry.addData("LF", robot.leftFrontDrive.getPower()) ;
        telemetry.addData("RF", robot.rightFrontDrive.getPower());
        telemetry.addData("LB", robot.leftBackDrive.getPower());
        telemetry.addData("RB", robot.rightBackDrive.getPower());
    }
    private void gamepadStatus(){
        telemetry.addData("LeftStick_x: %.2f", gamepad1.left_stick_x);
        telemetry.addData("LeftStick_y: %.2f", gamepad1.left_stick_y);
        telemetry.addData("RightStick_x: %.2f", gamepad1.right_stick_x);
    }
}