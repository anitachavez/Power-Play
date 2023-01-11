package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomo")

public class AutonomoPrueba extends LinearOpMode{
    int TICKS_PER_CM = 25;
    double AUTONOMOUS_SPEED = 0.2;
    BNO055IMU imu;
    Orientation angles;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class,"front_left");
        frontRight = hardwareMap.get(DcMotor.class,"front_right");
        backLeft = hardwareMap.get(DcMotor.class,"back_left");
        backRight = hardwareMap.get(DcMotor.class,"back_right");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        initIMU();
        waitForStart();
            //****ADELANTE*****
        //moveAutonomous(0);
            //****ATRAS****
        // moveAutonomous(-0);
            //****LATERAL IZQ****
        //moveAutonomousLateral(0);
          //****LATERAL DER****
        //moveAutonomousLateral(-0);
          //****GIRO DER****
        //turnRight(0);
          //****GIRO IZQ****
        //turnLeft(0);
    }
     public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
    }
    public void setChasisRunMode(DcMotor.RunMode runmode) {
        frontLeft.setMode(runmode);
        frontRight.setMode(runmode);
        backLeft.setMode(runmode);
        backRight.setMode(runmode);
    }
    public void resetEncoders() {
        setChasisRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void initAutoDrive() {
        setChasisRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (
            frontLeft.isBusy() &&
            frontRight.isBusy() &&
            backLeft.isBusy() &&
            backRight.isBusy()
            ) {
                sleep(100);
            }
    }
    public void moveAutonomous(double distance) {
        resetEncoders();
        int targetPosition = (int) Math.round(distance*TICKS_PER_CM);
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        backLeft.setTargetPosition(targetPosition);
        backRight.setTargetPosition(targetPosition);
        frontLeft.setPower(AUTONOMOUS_SPEED);
        frontRight.setPower(AUTONOMOUS_SPEED);
        backLeft.setPower(AUTONOMOUS_SPEED);
        backRight.setPower(AUTONOMOUS_SPEED);
        initAutoDrive();
    }
    public void moveAutonomousLateral(double distance) {
       resetEncoders();
        int targetPosition = (int) Math.round(distance*TICKS_PER_CM);
        frontLeft.setTargetPosition(-targetPosition);
        frontRight.setTargetPosition(-targetPosition);
        backLeft.setTargetPosition(targetPosition);
        backRight.setTargetPosition(targetPosition);
        frontLeft.setPower(AUTONOMOUS_SPEED);
        frontRight.setPower(AUTONOMOUS_SPEED);
        backLeft.setPower(AUTONOMOUS_SPEED);
        backRight.setPower(AUTONOMOUS_SPEED);
        initAutoDrive();
    }
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void turnRight(double angleToRotate) {
        setChasisRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentAngle = getAngle();
        double targetAngle = currentAngle - angleToRotate;
        while (currentAngle > targetAngle) {
            frontLeft.setPower(AUTONOMOUS_SPEED);
            frontRight.setPower(-AUTONOMOUS_SPEED);
            backLeft.setPower(AUTONOMOUS_SPEED);
            backRight.setPower(-AUTONOMOUS_SPEED);
            currentAngle = getAngle();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeft(double angleToRotate) {
        setChasisRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentAngle = getAngle();
        double startAngle = currentAngle; //borrar
        double targetAngle = angleToRotate + getAngle();
        while (currentAngle < targetAngle) {
            telemetry.addData("Start angle: ", startAngle);
            telemetry.addData("Target angle: ", targetAngle);
            telemetry.addData("Current angle: ", currentAngle);
            frontLeft.setPower(-AUTONOMOUS_SPEED);
            frontRight.setPower(AUTONOMOUS_SPEED);
            backLeft.setPower(-AUTONOMOUS_SPEED);
            backRight.setPower(AUTONOMOUS_SPEED);
            currentAngle = getAngle();
            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
