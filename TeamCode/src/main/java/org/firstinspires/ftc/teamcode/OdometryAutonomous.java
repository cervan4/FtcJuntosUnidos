package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Odometry", group="Robot")
public class OdometryAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    @Override
    public void runOpMode() {
        SetupHardware();

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        resetTicks();
        int targetTick = calculateTicksForOneFoot(1.88976,2000);
        telemAllTicks("None");

        //driveForward(targetTick, 0.2, 1);
        MoveLeft(targetTick,0.2,1);

        telemAllTicks("None");
    }
    public static int calculateTicksForOneFoot(double wheelDiameterInches, int ticksPerRotation) {
        // 1 foot = 12 inches
        double distanceToMoveInInches = 10.0;

        // Calculate the circumference of the wheel
        double circumferenceInches = Math.PI * wheelDiameterInches;

        // Calculate the number of wheel rotations needed to move 1 foot (12 inches)
        double rotationsNeeded = distanceToMoveInInches / circumferenceInches;

        // Calculate the number of ticks for 1 foot
        int ticksNeeded = (int) (rotationsNeeded * ticksPerRotation);

        return ticksNeeded;
    }

    //This method is used to setup the hardware motors and sensors need to be setup here.
    private void SetupHardware() {
        FrontLeft  = hardwareMap.get(DcMotor.class, "frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "frontright");
        BackLeft  = hardwareMap.get(DcMotor.class, "backleft");
        BackRight = hardwareMap.get(DcMotor.class, "backright");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "frontleft");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "frontright");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "backleft");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void MoveLeft(double targetTicks, double power, long sleep){
        rightStop = false;
        leftStop = false;
        resetTicks();
        telemAllTicks("Moving Left");
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);

        while (!rightStop && !leftStop) {

            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopAllPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopAllPower();
            }
            telemetry.addData("Right tick", getRightTicks());
            telemetry.addData("Left tick", getLeftTicks());
            telemetry.addData("Target tick", targetTicks);
            telemetry.update();

        }
        sleep(1000*sleep);

    }

    public void driveForward(double targetTicks, double power, long sleep) {
        rightStop = false;
        leftStop = false;
        resetTicks();

        telemAllTicks("Forward");
        setAllPower(power);

        while (!rightStop && !leftStop) {
            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopAllPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopAllPower();
            }
            telemetry.addData("Right tick", getRightTicks());
            telemetry.addData("Left tick", getLeftTicks());
            telemetry.addData("Target tick", targetTicks);
            telemetry.update();
        }

        telemAllTicks("Forward");

        rightStop = false;
        leftStop = false;

        stopAllPower();
        resetTicks();

        sleep(1000*sleep);
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

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (leftEncoderMotor.getCurrentPosition() - leftEncoderPos);
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
}

