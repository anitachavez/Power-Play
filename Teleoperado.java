package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Teleoperado")
public class Teleoperado extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.initializeHardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double powerMultiplier = 0.8;
            double turnSensitivity = 0.8;
            double axial   = -gamepad1.left_stick_y * powerMultiplier;
            double lateral =  gamepad1.left_stick_x * powerMultiplier;
            double yaw     =  gamepad1.right_stick_x * powerMultiplier * turnSensitivity;
            robot.move(axial, lateral, yaw);
            telemetry.update();
        }
    }
    
}
