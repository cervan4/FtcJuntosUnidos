package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
public class RobotAutoDriveByTime_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

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

}

