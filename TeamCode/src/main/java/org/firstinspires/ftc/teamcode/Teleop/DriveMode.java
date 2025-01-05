package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Drive Mode", group="Drive Mode")
public class DriveMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private DcMotor VerticalLiftMotor = null;
    private DcMotor IntakeVerticalMotor = null;
    private DcMotor IntakeHorizontalMotor = null;
    private Servo ClawServo = null;
    private CRServo IntakeServo = null;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.5;

    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "frontright");
        BackLeft = hardwareMap.get(DcMotor.class, "backleft");
        BackRight = hardwareMap.get(DcMotor.class, "backright");
        ClawServo = hardwareMap.get(Servo.class, "Clawservo");
        IntakeServo = hardwareMap.get(CRServo.class, "intakeservo");
        VerticalLiftMotor = hardwareMap.get(DcMotor.class, "verticalLift");
        IntakeVerticalMotor = hardwareMap.get(DcMotor.class, "intakeVertical");
        IntakeHorizontalMotor = hardwareMap.get(DcMotor.class, "intakeHorizontal");


        VerticalLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);//r
        BackLeft.setDirection(DcMotor.Direction.REVERSE);//r
        FrontRight.setDirection(DcMotor.Direction.FORWARD);//f
        BackRight.setDirection(DcMotor.Direction.FORWARD);//f

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SetupHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_y;
            double yaw = -gamepad1.right_stick_x;

            boolean open = gamepad1.right_bumper;
            boolean close = gamepad1.left_bumper;

            if (open) {
                ClawServo.setPosition(MAX_POS);
            } else if (close) {
                ClawServo.setPosition(MIN_POS);
            }

            if (gamepad1.x) {
                IntakeServo.setPower(-1);
            } else if (gamepad1.b) {
                IntakeServo.setPower(1);
            } else {
                IntakeServo.setPower(0);
            }

            if (gamepad1.dpad_up) {
                VerticalLiftMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                VerticalLiftMotor.setPower(-1);
            } else if (gamepad1.y) {
                VerticalLiftMotor.setPower(0.1);
            } else {
                VerticalLiftMotor.setPower(0);
            }

            if (gamepad1.dpad_right) {
                IntakeHorizontalMotor.setPower(-1);
            } else if (gamepad1.dpad_left) {
                IntakeHorizontalMotor.setPower(1);
            } else {
                IntakeHorizontalMotor.setPower(0);
            }


            IntakeVerticalMotor.setPower(-gamepad1.right_trigger);
            IntakeVerticalMotor.setPower(gamepad1.left_trigger);


            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
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

    }
}

