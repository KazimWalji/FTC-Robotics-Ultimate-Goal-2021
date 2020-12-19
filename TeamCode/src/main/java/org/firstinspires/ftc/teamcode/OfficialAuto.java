
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class OfficialAuto extends LinearOpMode {
    OpenCvCamera phoneCam;
    StageSwitchingPipeline pipeline;

    BNO055IMU imu;

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lr = null;
    private DcMotor rr = null;
    private DcMotor lift = null;
    private Servo wobble = null;
    private Servo left = null;
    private Servo right = null;
    public static int currPos = -1;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        pipeline = new StageSwitchingPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("pos: ", pipeline.getAnalysis());
        telemetry.addData("thresh: ", pipeline.getThreshold());
        telemetry.update();
        waitForStart();
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setDirection(Servo.Direction.REVERSE);
        double threshold = pipeline.getThreshold();
        RingDetectionCV.StageSwitchingPipeline.ringPos pos = pipeline.getAnalysis();
        left.setPosition(.2);
        right.setPosition(.2);
        sleep(500);
        right.setPosition(.6);
        left.setPosition(.6);
        currPos = lift.getCurrentPosition();
        drive.setPoseEstimate(new Pose2d(-63, -53, 0));
        Pose2d startpos = new Pose2d(-63, -53, 0);


        while (opModeIsActive()) {
            if (pos == RingDetectionCV.StageSwitchingPipeline.ringPos.Four) {
                telemetry.addData("pos: ", pos);
                telemetry.addData("thresh: ", threshold);
                telemetry.update();
                Trajectory traj = drive.trajectoryBuilder(startpos)
                        .lineTo(new Vector2d(50, -67)).build();

                drive.followTrajectory(traj);

                sleep(200);

                Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                        .lineToLinearHeading(new Pose2d(-25, -8, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(traj1);

                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .back(39)
                        .build();

                drive.followTrajectory(traj2);

                Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        .strafeRight(18.5)
                        .build();

                drive.followTrajectory(traj3);


                Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .forward(11)
                        .build();

                drive.followTrajectory(traj4);

                sleep(1000);
                right.setPosition(.18);
                left.setPosition(.3);
                sleep(500);

                Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                        .strafeRight(29)
                        .build();

                drive.followTrajectory(traj5);

                Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                        .lineTo(new Vector2d(40, -65))
                        .build();

                drive.followTrajectory(traj6);

                right.setPosition(.5);
                left.setPosition(.5);
                sleep(1000);

                Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                        .back(30)
                        .build();

                drive.followTrajectory(traj7);

                sleep(30000);

            } else if (pos == RingDetectionCV.StageSwitchingPipeline.ringPos.One) {
                telemetry.addData("pos: ", pos);
                telemetry.addData("thresh: ", threshold);
                telemetry.update();
                Trajectory traj = drive.trajectoryBuilder(startpos)
                        .lineTo(new Vector2d(30, -40)).build();

                drive.followTrajectory(traj);

                sleep(2000);

                Trajectory add = drive.trajectoryBuilder(traj.end()) .lineToLinearHeading(new Pose2d(25, -46, Math.toRadians(0))).build();
                Trajectory traj1 = drive.trajectoryBuilder(add.end())
                        .back(95)
                        .build();

                drive.followTrajectory(traj1);


                Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(22)
                        .build();

                drive.followTrajectory(traj3);

                Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .lineTo(new Vector2d(22, -35)).build();

                drive.followTrajectory(traj4);

                sleep(2000);

                Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                        .back(15)
                        .build();

                drive.followTrajectory(traj5);

                sleep(30000);
            } else if (pos == RingDetectionCV.StageSwitchingPipeline.ringPos.None) {
                telemetry.addData("pos: ", pos);
                telemetry.addData("thresh: ", threshold);
                telemetry.update();
                telemetry.addData("pos: ", pos);
                telemetry.addData("thresh: ", threshold);
                telemetry.update();

                Trajectory traj = drive.trajectoryBuilder(startpos)
                        .lineTo(new Vector2d(-1, -67)).build();

                drive.followTrajectory(traj);

                Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                        .back(60).build();

                drive.followTrajectory(traj1);

                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(42).build();

                drive.followTrajectory(traj2);

                right.setPosition(.18);
                Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        .lineTo(new Vector2d(0, -61)).build();

                drive.followTrajectory(traj3);

                Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .back(20).build();

                drive.followTrajectory(traj4);

                Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                        .strafeLeft(30).build();

                drive.followTrajectory(traj5);

                Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                        .forward(30).build();

                drive.followTrajectory(traj6);
                sleep(30000);
            }
        }
    }

    public static class StageSwitchingPipeline extends OpenCvPipeline {
        enum ringPos {
            Four,
            One,
            None
        }

        static final Point topLeft = new Point(52, 190);
        static final int width = 25;
        static final int height = 35;
        final int fourRingsThreshold = 107;
        final int oneRingThreshold = 119;
        Point pointA = new Point(topLeft.x, topLeft.y);
        Point pointB = new Point(topLeft.x + width, topLeft.y + height);

        Mat box = new Mat();
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgCb;

        private volatile RingDetectionCV.StageSwitchingPipeline.ringPos pos = RingDetectionCV.StageSwitchingPipeline.ringPos.Four;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);
            box = Cb.submat(new Rect(pointA, pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avgCb = (int) Core.mean(box).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    pointA, // First point which defines the rectangle
                    pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), 1);
            if (avgCb < oneRingThreshold) {
                pos = RingDetectionCV.StageSwitchingPipeline.ringPos.One;
            }
            if (avgCb < fourRingsThreshold) {
                pos = RingDetectionCV.StageSwitchingPipeline.ringPos.Four;
            } else if (avgCb > 122) {
                pos = RingDetectionCV.StageSwitchingPipeline.ringPos.None;
            }
            return input;
        }

        public RingDetectionCV.StageSwitchingPipeline.ringPos getAnalysis() {
            return pos;
        }

        public int getThreshold() {
            return avgCb;
        }
    }
}
