package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author ErnoMitrovic <https://github.com/ErnoMitrovic>
 * @version 1
 * @since 11/12/2022
 */
public class ChassisRamp {
    private final ElapsedTime time;
    private static final byte CYCLE_MS = 50;
    RampMotor frontLeft, frontRight, backLeft, backRight;
    public ChassisRamp(HardwareMap hm){
        frontLeft = new RampMotor(hm.get(DcMotor.class, "front_left"),
                DcMotorSimple.Direction.REVERSE);
        frontRight = new RampMotor(hm.get(DcMotor.class, "front_right"));
        backLeft = new RampMotor(hm.get(DcMotor.class, "back_left"),
                DcMotorSimple.Direction.REVERSE);
        backRight = new RampMotor(hm.get(DcMotor.class, "back_right"));
        time = new ElapsedTime();
    }
    public void move(Gamepad gamepad){
        if(time.milliseconds() >= CYCLE_MS) {
            double drive = -gamepad.left_stick_y,
                    lateral = gamepad.left_stick_x,
                    turn = gamepad.right_stick_x;
            frontLeft.setPower(drive + lateral + turn);
            frontRight.setPower(drive - lateral - turn);
            backLeft.setPower(drive - lateral + turn);
            backRight.setPower(drive + lateral - turn);
            time.reset();
        }
    }
    @SuppressLint("DefaultLocale")
    public String status(){
        String stats = "";
        stats += String.format("Front left power: %.2f\n", frontLeft.getPower());
        stats += String.format("Front right power: %.2f\n", frontRight.getPower());
        stats += String.format("Back left power: %.2f\n", backLeft.getPower());
        stats += String.format("Back right power: %.2f\n", backRight.getPower());

        return stats;
    }
}
