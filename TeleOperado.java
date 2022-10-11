package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp")
public class TeleOperado extends LinearOpMode{
    final double UPDATE_PERIOD = 1000;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    
    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");  
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        ElapsedTime timer = new ElapsedTime();
        double lastUpdateTime = timer.milliseconds();
        waitForStart();
        while(opModeIsActive()) {
            double currentTime = timer.milliseconds();
            if (currentTime - lastUpdateTime > UPDATE_PERIOD){
                moveChasis(gamepad1);
                telemetry.addData("actualizar motor", "");
                lastUpdateTime = currentTime;
            }
            telemetry.update();
        }
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
                return 0.4;
            if(gamepad.right_bumper)
                return 0.8;
            if(gamepad.left_bumper)
                return 0.6;
            return 1.0;
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
            
            if (frontRight.getPower()<frontRightPower){
            frontRightPower = frontRightPower + 0.1;
            }
            if (frontLeft.getPower()<frontLeftPower){
            frontLeftPower = frontLeftPower + 0.1;
            }
            if(backRight.getPower()<backRightPower){
            backRightPower = backRightPower + 0.1;
            }
            if (backLeft.getPower()<backLeftPower){
            backLeftPower = backLeftPower + 0.1;
            }
            if (frontRight.getPower()>frontRightPower){
            frontRightPower = frontRightPower - 0.1;
            }
            if (frontLeft.getPower()>frontLeftPower){
            frontLeftPower = frontLeftPower - 0.1;
            } 
            if(backRight.getPower()>backRightPower){
            backRightPower = backRightPower - 0.1;
            } 
            if (backLeft.getPower()>backLeftPower){
            backLeftPower = backLeftPower - 0.1;
            
            }else
            frontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
            frontRight.setPower(Range.clip(frontRightPower, -1, 1));
            backLeft.setPower(Range.clip(backLeftPower, -1, 1));
            backRight.setPower(Range.clip(backRightPower, -1, 1));
            }
    }
    
