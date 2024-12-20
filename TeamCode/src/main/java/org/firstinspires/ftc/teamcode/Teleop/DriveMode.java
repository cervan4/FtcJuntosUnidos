package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Drive Mode", group="Drive Mode")
public class DriveMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private DcMotor VerticalLiftMotor = null;

    private Servo Clawservo = null;
    private Servo IntakeServo = null;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.5;

    @Override
    public void runOpMode() {
        SetupHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            DriveFunctionality();
        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    //This function controls basic function such as moving forward, Backward, Left and Right.
    private void DriveFunctionality() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_y;
        double yaw     =  -gamepad1.right_stick_x;
        boolean open = gamepad1.right_bumper;
        boolean close = gamepad1.left_bumper;

        if (open){
            Clawservo.setPosition(MAX_POS);
                }
        if (close){
            Clawservo.setPosition(MIN_POS);
        }
        if(gamepad1.dpad_up){
            VerticalLiftMotor.setPower(1);
        }else if(gamepad1.dpad_down){
            VerticalLiftMotor.setPower(-1);
        }else if(gamepad1.y){
            VerticalLiftMotor.setPower(0.1);
        }else {
            VerticalLiftMotor.setPower(0);
        }
        if(gamepad1.x){
            IntakeServo.setDirection(Servo.Direction.REVERSE);
            IntakeServo.setPosition(MAX_POS);
        }else{
            IntakeServo.setPosition(0);//do nothing
        }



        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        FrontLeft.setPower(leftFrontPower);
        FrontRight.setPower(rightFrontPower);
        BackLeft.setPower(leftBackPower);
        BackRight.setPower(rightBackPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }

    //This method is used to setup the hardware motors and sensors need to be setup here.
    private void SetupHardware() {
        FrontLeft  = hardwareMap.get(DcMotor.class, "frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "frontright");
        BackLeft  = hardwareMap.get(DcMotor.class, "backleft");
        BackRight = hardwareMap.get(DcMotor.class, "backright");
        Clawservo = hardwareMap.get (Servo.class,"Clawservo");
        IntakeServo = hardwareMap.get (Servo.class,"intakeservo");
        VerticalLiftMotor = hardwareMap.get(DcMotor.class, "verticalLift");
        SetupNormalDrive();
        VerticalLiftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void SetupNormalDrive(){
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

