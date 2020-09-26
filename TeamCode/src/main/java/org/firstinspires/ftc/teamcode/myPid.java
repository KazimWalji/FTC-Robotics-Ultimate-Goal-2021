/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous
public class myPid extends LinearOpMode {

    BNO055IMU imu;

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lr = null;
    private DcMotor rr = null;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            encoderPidTrap(3000,lf.getCurrentPosition(), 3000,rf.getCurrentPosition(), 3000,lr.getCurrentPosition(), 3000, rr.getCurrentPosition(), .9, 7.0);
            turn_to_heading(10,0);
            sleep(20000);
            encoderPidTrap(1000,lf.getCurrentPosition(), 1000,rf.getCurrentPosition(), 1000,lr.getCurrentPosition(), 1000, rr.getCurrentPosition(), .8, 7.0);
            sleep(1000);
            turn_to_heading(150, -25);
            encoderPidTrap(3000,lf.getCurrentPosition(), 3000,rf.getCurrentPosition(), 3000,lr.getCurrentPosition(), 3000, rr.getCurrentPosition(), .9, 7.0);
            sleep(1000);
            turn_to_heading(345,-25);
            encoderPidTrap(4000,lf.getCurrentPosition(), 4000,rf.getCurrentPosition(), 4000,lr.getCurrentPosition(), 4000, rr.getCurrentPosition(), .9, 7.0);
            encoderPidTrap(-1000,lf.getCurrentPosition(), -1000,rf.getCurrentPosition(), -1000,lr.getCurrentPosition(), -1000, rr.getCurrentPosition(), .9, 1.5);
            sleep(30000);



        }
    }
    public void turn_to_heading(double target_heading, double speedModifier) {
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean right;
        double currentHeading;
        double degreesToTurn;
        double wheelPower = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        currentHeading = getHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        right = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            right = !right;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        while (degreesToTurn > .7 && opModeIsActive() && timeoutTimer.seconds() < 4) {
            telemetry.addData("currPos: ", getHeading());
            telemetry.addData("target", target_heading);
            telemetry.update();
            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 17) / -speedModifier, 3) + 5) / 100;
            }
            if (speedModifier == 0) {
                wheelPower = (Math.pow(((degreesToTurn) / 30) + 1, 4) + 10) / 100;
            }

            if (right) {
                wheelPower = -wheelPower;
            }

            lf.setPower(wheelPower);
            lr.setPower(wheelPower);
            rr.setPower(-wheelPower);
            rf.setPower(-wheelPower);

            currentHeading = getHeading();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            right = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                right = !right;
                degreesToTurn = 360 - degreesToTurn;
            }

        }
        lf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
        rf.setPower(0);

    }
    public void encoderPidTrap(int lF, int curlf, int rF, int currf, int lR, int curlr, int rR, int currr, double maxPow, double sec) {
        double totalDist = ((lF+lR+rR+rF)/4);
        double minPow = .45;
        boolean backwards = false;
        double prevHeading = getHeading();
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        while ((Math.abs((rr.getCurrentPosition()-currr)-rR) > 5 ||Math.abs((lr.getCurrentPosition()-curlr)-lR) > 5 || Math.abs((rf.getCurrentPosition()-currf)-rF) > 5 || Math.abs((lf.getCurrentPosition()-curlf)-lF) > 5) && opModeIsActive() && runTime.seconds() < 10.0) {
            double dist = (((lF-(lf.getCurrentPosition()-curlf))+(rF-rf.getCurrentPosition())+(lR-(lr.getCurrentPosition()-curlr))+(rR-(rr.getCurrentPosition()-currr)))/4)/totalDist;
            if(dist<0)
            {
                dist = -dist;
                backwards = true;
            }
            double power = -16* Math.pow((dist-.5),4) + 1;
            power = power * maxPow;
            if(power<=minPow && dist>0.8)
            {
                power = minPow;
            }

            if(dist<0.015)
            {
                lr.setPower(0);
                rr.setPower(0);
                rf.setPower(0);
                lf.setPower(0);
                break;
            }
            if(!((Math.abs(rr.getCurrentPosition()-rR) > 5 ||Math.abs(lr.getCurrentPosition()-lR) > 5 || Math.abs(rf.getCurrentPosition()-rF) > 5 || Math.abs(lf.getCurrentPosition()-lF) > 5) && opModeIsActive() && runTime.seconds() < 10.0)) {
                lf.setPower(0);
                lr.setPower(0);
                rr.setPower(0);
                rf.setPower(0);
                break;
            }
            telemetry.addData("dist", dist);
            telemetry.addData("power", rr.getPower());
            telemetry.addData("back", backwards);
            telemetry.addData("curr", rr.getCurrentPosition());
            telemetry.addData("targ", rR);
            telemetry.addData("time", runTime.seconds());
            telemetry.update();
            if(backwards)
            {
                power = -power;
                lf.setPower(power);
                rf.setPower(power);
                rr.setPower(power);
                lr.setPower(power);
            }
            else {
                lf.setPower(power);
                rf.setPower(power);
                rr.setPower(power);
                lr.setPower(power);
            }
        }
        lf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
        rf.setPower(0);
        turn_to_heading(prevHeading,0);
    }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle + 360) % 360;
    }

}
