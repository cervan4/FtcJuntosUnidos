package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;


@Autonomous(name="Odometry", group="Robot")
public class OdometryAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private DcMotor VerticalLiftMotor = null;
    private IMU Imu = null;
    private DcMotor IntakeVerticalMotor = null;
    private DcMotor IntakeHorizontalMotor = null;
    private Servo ClawServo = null;
    private CRServo IntakeServo = null;
    static final double OPEN = 1.0;
    static final double CLOSE = 0.5;

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
        telemAllTicks("None");
        driveForward(calculateTicks(24), 0.5, 1);
        //MoveLeft(targetTick,0.2,1);


        telemAllTicks("None");
    }
    //will calculate ticks for one foot
    //for the robot to move 1 foot call calculateTicks 10 inches
    public static int calculateTicks(int distanceToMoveInInches) {
        double wheelDiameterInches = 1.88976;
        int ticksPerRotation = 2000;

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
        ClawServo = hardwareMap.get (Servo.class,"Clawservo");
        IntakeServo = hardwareMap.get (CRServo.class,"intakeservo");
        VerticalLiftMotor = hardwareMap.get(DcMotor.class, "verticalLift");
        IntakeVerticalMotor =  hardwareMap.get(DcMotor.class, "intakeVertical");
        IntakeHorizontalMotor = hardwareMap.get(DcMotor.class, "intakeHorizontal");
        Imu = hardwareMap.get(IMU.class, "imu");


        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void liftClaw(double power, int time){
        VerticalLiftMotor.setPower(power);
        sleep(time);
        HoldInPlace();
    }
    public void descentClaw(double power, int time){
        power = -power;
        VerticalLiftMotor.setPower(power);
        sleep(time);
        HoldInPlace();
    }
    public void rotate(double targetDegrees, double power){
        double currentRotation = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double FrontLeftPower;
        double FrontRightPower;
        double BackLeftPower;
        double BackRightPower;
        if(targetDegrees == 0){
            return;
        }
        else if(targetDegrees < 0){
            FrontLeftPower = -power;
            FrontRightPower = power;
            BackLeftPower = -power;
            BackRightPower = power;
        }else{
            FrontLeftPower = power;
            FrontRightPower = -power;
            BackLeftPower = power;
            BackRightPower = -power;
        }
        while(currentRotation < targetDegrees){
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);

            currentRotation = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        StopRobot();
    }
    public void StopRobot(){
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);
    }
    public void HoldInPlace(){
        VerticalLiftMotor.setPower(0.1);
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

