package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp")
public class TeleOperado extends OpMode{
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // todo: write your code here
    
     @Override
        public void init(){
            frontLeft = hardwareMap.get(DcMotor.class, "front_left");
            frontRight = hardwareMap.get(DcMotor.class, "front_right");
            backLeft = hardwareMap.get(DcMotor.class, "back_left");
            backRight = hardwareMap.get(DcMotor.class, "back_right");  
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
     }
    @Override
    public void loop(){
        moveChasis(gamepad1);
    }
        private void moveChasis(Gamepad gamepad) {
            double drive = -gamepad.left_stick_y;
            double lateral = gamepad.right_stick_x;
            double turn = gamepad.left_stick_x;
            double powerMultiplier = getPowerMultiplier(gamepad);
            move(drive, lateral, turn, powerMultiplier);
            
        }
        
        private double getPowerMultiplier(Gamepad gamepad){
            if(gamepad.right_bumper && gamepad.left_bumper)
                return 0.2;
            if(gamepad.right_bumper)
                return 0.6;
            if(gamepad.left_bumper)
                return 0.4;
            return 0.8;
        }
    
       /* private void controlCarousel(Gamepad gamepad){
            double carouselPower = 0;
            if (gamepad.right_stick_y < 0) {
                carouselPower = -1;
          } else if (gamepad.right_stick_y > 0) {
                carouselPower = 1;
          } else {
                carouselPower = 0;
          }
            carouselServomotor.setPower(carouselPower);
    }*/
    
        public void move(double drive, double lateral, double turn, double multiplier){
            double frontLeftPower = (drive + lateral + turn) * multiplier;
            double frontRightPower = (drive - lateral - turn) * multiplier;
            double backLeftPower = (drive - lateral + turn) * multiplier;
            double backRightPower = (drive + lateral - turn) * multiplier;
            frontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
            frontRight.setPower(Range.clip(frontRightPower, -1, 1));
            backLeft.setPower(Range.clip(backLeftPower, -1, 1));
            backRight.setPower(Range.clip(backRightPower, -1, 1));
    }
    
    
    
    
    
}
