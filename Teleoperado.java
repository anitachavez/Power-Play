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
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            robot.move(axial, lateral, yaw);
            telemetry.update();
        }
    }
}