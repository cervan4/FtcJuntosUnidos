package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
public class OdometryAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0;
    private double deltaLeftEncoder = 0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0;
    private double deltaRightEncoder = 0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;
    private double deltaCenterEncoder = 0;

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    double OPcircumference = 2.0*Math.PI*(16.0/25.4);
    private boolean rightStop = false;
    private boolean leftStop = false;

    @Override
    public void runOpMode() {
        SetupHardware();

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        GoForward(FORWARD_SPEED,1.5);
        }
    //This method is used to setup the hardware motors and sensors need to be setup here.
    private void SetupHardware() {
        FrontLeft  = hardwareMap.get(DcMotor.class, "frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "frontright");
        BackLeft  = hardwareMap.get(DcMotor.class, "backleft");
        BackRight = hardwareMap.get(DcMotor.class, "backright");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "frontleft");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "frontright");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "backleft");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    private void GoForward(double power, double time){
        // Step 1:  Drive forward for 3 seconds
        FrontLeft.setPower(power);
        BackLeft.setPower(power);
        FrontRight.setPower(power);
        BackRight.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }

    public void driveForward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(power);

        telemAllTicks("Forward");

        while (!(rightStop && leftStop)) {
            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
        }

        telemAllTicks("Forward");

        rightStop = false;
        leftStop = false;

        stopAllPower();
        resetTicks();

        sleep(1000*sleep);
    }

    public void driveBackward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Backward");

        while (!(rightStop && leftStop)) {
            if (Math.abs(getRightTicks()) >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (Math.abs(getLeftTicks()) >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
        }

        telemAllTicks("Backward");

        rightStop = false;
        leftStop = false;
        stopAllPower();
        resetTicks();

        sleep(1000*sleep);
    }

    public void strafeRight(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(power);

        telemAllTicks("Right");

        while (getCenterTicks() < targetTicks){

        }

        telemAllTicks("Right");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(1000*sleep);
    }

    public void strafeLeft(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Left");

        while (Math.abs(getCenterTicks()) < targetTicks){
            telemAllTicks("Left");
        }

        telemAllTicks("Left");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(1000*sleep);
    }


    public void setNormalDrive(){
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setStrafingDrive(){
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
    }

    public void setAllPower(double p){
        FrontLeft.setPower(p);
        FrontRight.setPower(p);
        BackLeft.setPower(p);
        BackRight.setPower(p);
    }

    public void stopAllPower(){
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void setLeftPower(double p){
        FrontLeft.setPower(p);
        FrontRight.setPower(p);
    }

    public void setRightPower(double p){
        FrontRight.setPower(p);
        BackRight.setPower(p);
    }

    public void stopLeftPower(){
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
    }

    public void stopRightPower(){
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (-(leftEncoderMotor.getCurrentPosition() - leftEncoderPos));
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public double getRightTicks(){
        return (rightEncoderMotor.getCurrentPosition() - rightEncoderPos);
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }

    public double getCenterTicks(){
        return (centerEncoderMotor.getCurrentPosition() - centerEncoderPos);
    }

    public void telemAllTicks(String dir){
        telemetry.addData("Direction", dir);
        telemetry.addData("Left pos", getLeftTicks());
        telemetry.addData("Right pos", getRightTicks());
        telemetry.addData("Center pos", getCenterTicks());
        telemetry.update();
    }

    public double TicksToInches(double ticks){
        double rev = (double)ticks/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToTicks(double inches){
        double rev = inches/OPcircumference;
        double tick = 2000*rev;
        return tick;
    }

}

