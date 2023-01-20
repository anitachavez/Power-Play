package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
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
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private double lastTracakbleSearchTime;
    private TargetInfo identifiedTrackable;

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

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lastMotorUpdateTime = timer.milliseconds();
        lastTracakbleSearchTime = lastMotorUpdateTime;

        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
    }

    // ******************************************
    // *                 CHASIS                 *
    // ******************************************

    public void move(double drive, double lateral, double yaw) {
        double currentTime = timer.milliseconds();
        if (currentTime - lastMotorUpdateTime > MOTOR_UPDATE_PERIOD_MS) {
            double leftFrontTarget  = drive + lateral + yaw;
            double rightFrontTarget = drive - lateral - yaw;
            double leftBackTarget   = drive - lateral + yaw;
            double rightBackTarget  = drive + lateral - yaw;
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
            leftFrontDrive.setPower(leftFrontTarget);
            rightFrontDrive.setPower(rightFrontTarget);
            leftBackDrive.setPower(leftBackTarget);
            rightBackDrive.setPower(rightBackTarget);
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

    // ******************************************
    // *                 VISION                 *
    // ******************************************
    private WebcamName webcamName;
    public void initVuforia() {
        webcamName = opMode.hardwareMap.get(WebcamName.class, "camara");
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATjrkEL/////AAABmfR2/BPftkOFvL9kl5ElbHswfU6Tuno4QSB4aHpVUmWaWqKdEUps2CsnGbmjoGqMAfOjyPlhrew8njlemEsarH9XKySF9i0egaUhOiT2fE0MivatYaT037ZwPe1bOkI1GGmd2CsWL8GeupcT91XQkGhRcMyTS3ZfmDYu1/HmcRxCy4zxwbiyPVcoHtsh+KPfjI29mv9YfMStiB4/o8FgefPbTGtX6L9zeoyUemNIMN1WcaMi6wSM7rB7kF3VnUJCrXAca6YmFNEr6GEdJX4G7JhO5EiD6K/e1+wZ0fLtWiQDWe09Bgxxpp2n+qHeccA06zA8nNTo2F07UORoM40ZK29vMj4eh0GjyNMAOmWcuQeI";
        parameters.useExtendedTracking = false;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {

    }

    public void logIdentifiedTarget() {
        TargetInfo trackable = getVisibleTrackable();
        if (trackable == null)
            opMode.telemetry.addData("Trackable:", "No visible target");
        else {
            opMode.telemetry.addData("Trackable:", trackable.name);
            opMode.telemetry.addData("Trackable:", "X: (%.2f)  Y: (%.2f)  Z: (%.2f)", trackable.x, trackable.y, trackable.z);
        }
    }
    
    public void followTarget() {
        final double MM_PER_INCH = 25.40;
        TargetInfo trackable = getVisibleTrackable();
        if (trackable != null){
            double targetX = trackable.x / MM_PER_INCH;
            double targetY = trackable.z / MM_PER_INCH;
            final double DESIRED_DISTANCE = 9;
            final double SPEED_GAIN =   0.030 ;
            final double TURN_GAIN  =   0.045 ; 

            double targetRange = Math.hypot(targetX, targetY);

            double targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));
            opMode.telemetry.addData("Trackable:", trackable.name);
            opMode.telemetry.addData("Trackable:", "X: (%.2f) Z: (%.2f)", trackable.x, trackable.z);
            
            double  rangeError   = (targetRange - DESIRED_DISTANCE);
            double  headingError = targetBearing;

            double drive = rangeError * SPEED_GAIN;
            double turn  = headingError * TURN_GAIN ;
            
            move(drive, 0, turn);
        }
    }

    public void startTracking() {
        trackables = this.vuforia.loadTrackablesFromAsset("PowerPlay");
        trackables.get(0).setName("Red Audience Wall");
        trackables.get(1).setName("Red Rear Wall");
        trackables.get(2).setName("Blue Audience Wall");
        trackables.get(3).setName("Blue Rear Wall");
        trackables.activate();
    }

    private TargetInfo getVisibleTrackable() {
        double currentTime = timer.milliseconds();
        if(currentTime - lastTracakbleSearchTime > 100) {
            lastTracakbleSearchTime = currentTime;
            for (VuforiaTrackable trackable : trackables) {
                VuforiaTrackableDefaultListener targetListener = ((VuforiaTrackableDefaultListener) trackable.getListener());
                boolean isTrackableVisible = targetListener.isVisible();
                if(isTrackableVisible){
                    identifiedTrackable = new TargetInfo(trackable);
                    return identifiedTrackable;
                }
                    
            }
            return null;
        }
        return identifiedTrackable;
    }
}