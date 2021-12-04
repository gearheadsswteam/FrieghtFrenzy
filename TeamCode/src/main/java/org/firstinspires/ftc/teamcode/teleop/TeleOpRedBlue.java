package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ValueStorage;

@TeleOp(name = "TeleOpOneController", group = "Mecanum")
public class TeleOpRedBlue extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx intake;
    DcMotorEx lift;
    Servo arm;
    Servo claw;
    Servo bucket;
    Servo gate;
    CRServo spinner;
    BNO055IMU gyro;
    RevColorSensorV3 bucketSensor;
    double initialHeading = ValueStorage.lastPose.getHeading() - redMultiplier * PI / 2;
    double robotHeading;
    double joystickAngle;
    double joystickMagnitude;
    double turn;
    int intakeState = lastIntakeState;
    int liftState = lastLiftState;
    int armState = lastArmState;
    int detectionFrames = 0;
    ElapsedTime intakeStateTime = new ElapsedTime();
    ElapsedTime armStateTime = new ElapsedTime();
    boolean aPressed = false;
    boolean bPressed = false;
    boolean yPressed = false;
    boolean lbPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean yReleased = true;
    boolean lbReleased = true;
    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        gate = hardwareMap.get(Servo.class, "gate");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        bucketSensor = hardwareMap.get(RevColorSensorV3.class, "bucketSensor");
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setDirection(Direction.REVERSE);
        br.setDirection(Direction.REVERSE);
        intake.setDirection(Direction.REVERSE);
        spinner.setDirection(Direction.REVERSE);
        lift.setTargetPosition(liftPositions[liftState]);
        lift.setMode(RunMode.RUN_TO_POSITION);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);
        waitForStart();
        lift.setPower(1);
        intakeStateTime.reset();
        armStateTime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad1.b) {
                bPressed = bReleased;
                bReleased = false;
            } else {
                bPressed = false;
                bReleased = true;
            }
            if (gamepad1.y) {
                yPressed = yReleased;
                yReleased = false;
            } else {
                yPressed = false;
                yReleased = true;
            }
            if (gamepad1.left_bumper) {
                lbPressed = lbReleased;
                lbReleased = false;
            } else {
                lbPressed = false;
                lbReleased = true;
            }
            if (gamepad1.dpad_up) {
                initialHeading -= robotHeading;
            }
            if (gamepad1.right_bumper) {
                spinner.setPower(redMultiplier);
            } else {
                spinner.setPower(0);
            }
            if (bucketSensor.getDistance(DistanceUnit.CM) < bucketSensorThreshold) {
                detectionFrames++;
            } else {
                detectionFrames = 0;
            }
            switch (intakeState) {
                case 0:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[0][liftState]) {
                        gate.setPosition(gateUp);
                        bucket.setPosition(bucketRest);
                        if (intakeStateTime.milliseconds() >= intakeStateTimes[0][0]) {
                            lift.setTargetPosition(liftPositions[0]);
                        }
                    } else {
                        intake.setPower(1);
                        if (aPressed || bPressed || yPressed) {
                            if (aPressed) {
                                liftState = 0;
                            } else if (bPressed) {
                                liftState = 1;
                            } else {
                                liftState = 2;
                            }
                            intakeState = 1;
                            intakeStateTime.reset();
                        }
                    }
                    break;
                case 1:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[1][liftState]) {
                        intake.setPower(1);
                        gate.setPosition(gateDown);
                        lift.setTargetPosition(liftPositions[0]);
                    } else if (detectionFrames >= bucketDetectionFrames || aPressed) {
                        intakeState = 2;
                        intakeStateTime.reset();
                    }
                    break;
                case 2:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[2][liftState]) {
                        intake.setPower(-0.5);
                        bucket.setPosition(bucketUp);
                        lift.setTargetPosition(liftPositions[liftState]);
                    } else {
                        intake.setPower(0);
                        if (bPressed) {
                            intakeState = 3;
                            intakeStateTime.reset();
                        }
                    }
                    break;
                case 3:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[3][liftState]) {
                        bucket.setPosition(bucketDown);
                    } else if (bPressed) {
                        intakeState = 0;
                        intakeStateTime.reset();
                    }
                    break;
            }
            switch (armState) {
                case 0:
                    if (armStateTime.milliseconds() < armStateTimes[0]) {
                        arm.setPosition(armRest);
                        claw.setPosition(clawClosed);
                    } else if (lbPressed) {
                        armState = 1;
                        armStateTime.reset();
                    }
                    break;
                case 1:
                    if (armStateTime.milliseconds() < armStateTimes[1]) {
                        arm.setPosition(armUp);
                    } else if (lbPressed) {
                        armState = 2;
                        armStateTime.reset();
                    }
                    break;
                case 2:
                    if (armStateTime.milliseconds() < armStateTimes[2]) {
                        claw.setPosition(clawOpen);
                    } else if (lbPressed) {
                        armState = 3;
                        armStateTime.reset();
                    }
                    break;
                case 3:
                    if (armStateTime.milliseconds() < armStateTimes[3]) {
                        arm.setPosition(armRest);
                        claw.setPosition(clawOpen);
                    } else if (lbPressed) {
                        armState = 4;
                        armStateTime.reset();
                    }
                    break;
                case 4:
                    if (armStateTime.milliseconds() < armStateTimes[4]) {
                        arm.setPosition(armDown);
                    } else if (lbPressed) {
                        armState = 5;
                        armStateTime.reset();
                    }
                    break;
                case 5:
                    if (armStateTime.milliseconds() < armStateTimes[5]) {
                        claw.setPosition(clawClosed);
                    } else if (lbPressed) {
                        armState = 0;
                        armStateTime.reset();
                    }
                    break;
            }
            robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialHeading;
            joystickAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            joystickMagnitude = pow(gamepad1.left_stick_x, 2) + pow(gamepad1.left_stick_y, 2);
            turn = gamepad1.right_stick_x;
            fr.setPower(Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1));
            fl.setPower(Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1));
            br.setPower(Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1));
            bl.setPower(Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1));
        }
    }
}