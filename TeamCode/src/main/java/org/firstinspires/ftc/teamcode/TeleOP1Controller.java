package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name= "TeleOP1controller", group="Linear Opmode")
//@Disabled
public class TeleOP1Controller extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lr = null;
    private DcMotor rr = null;
    private DcMotor lift = null;
    private Servo wobble = null;
    private Servo left = null;
    private Servo right = null;
    private boolean buttonPrev = false;

    @Override
    public void runOpMode() {

        wobble = hardwareMap.get(Servo.class, "scoop_servo");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        lf = hardwareMap.get(DcMotor.class, "left_front");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        lr = hardwareMap.get(DcMotor.class, "left_rear");
        rr = hardwareMap.get(DcMotor.class, "right_rear");
        lift = hardwareMap.get(DcMotor.class, "lift");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("currPos", OfficialAuto.currPos);
        telemetry.update();
        left.setDirection(Servo.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        right.setPosition(.3);
        left.setPosition(.3);
        while (opModeIsActive()) {

            if (gamepad1.a) {

                left.setPosition(.4);
                right.setPosition(.4);
            }
            if (gamepad1.dpad_up) {

                left.setPosition(.65);
                right.setPosition(.65);
            }
            if(gamepad1.b)
            {
                left.setPosition(.18);
                right.setPosition(.18);
            }

            if ((lift.getCurrentPosition() < 10) && -gamepad1.right_stick_y < 0) {
                lift.setPower(0);
            } else {
                lift.setPower(-gamepad1.right_stick_y);
            }
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = (-gamepad1.right_stick_x);
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double x = 1;

            lf.setPower(v1 * x);
            rf.setPower(v2 * x);
            lr.setPower(v3 * x);
            rr.setPower(v4 * x);

        }
    }
}
  