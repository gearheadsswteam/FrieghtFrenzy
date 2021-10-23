package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.MecanumDrive;


@TeleOp(name = "Mecanum TeleOp Mode", group = "Mecanum")
//@Disabled
public class TeleOpMecanumOpMode extends LinearOpMode {

    //Reference for Josh's code: https://docs.google.com/document/d/1nJ-Rro6GFyXt1vbN69c-Y5u8U8c_oHpr-_ET3eomAbA/edit

    /* Declare OpMode members. */
    private GearheadsMecanumRobotRR robot;   // Use gearheads robot hardware




    private MecanumDrive mecanum;
    private BNO055IMU gyro;

    private double turn;
    private double forwardPower;
    private double sidePower;
    private boolean capStoneGrabed=false;


    /**
     * Constructor
     */
    public TeleOpMecanumOpMode() {
        robot = new GearheadsMecanumRobotRR(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        initOpMode();

        while (opModeIsActive()) {
            adjustForFOV();

            dampenSpeed();
            //Move The robot
            //moveRobot();

            //operateIntake();

            operateCapstoneArm();

            //operateDelivery();

        }
    }




    /**
     * Initialize the opmode
     */

    private void initOpMode() {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        telemetry.addData("Status", "Started");

        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleOp(hardwareMap);


        gyro = robot.imu;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //DRIVING
        DcMotor fl_motor = robot.fl_motor;
        DcMotor fr_motor = robot.fr_motor;
        DcMotor rl_motor = robot.rl_motor;
        DcMotor rr_motor = robot.rr_motor;

        mecanum = new MecanumDrive(fl_motor, fr_motor, rl_motor, rr_motor, gyro);
    }


    /**
     * Adjust teleop driving with FOV mode
     */
    private void adjustForFOV() {


        double angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;


        double tempForwardPower = gamepad1.left_stick_y;
        double tempSidePower = gamepad1.left_stick_x;

        sidePower = tempForwardPower * Math.cos(angle) + tempSidePower * Math.sin(angle);
        forwardPower = -tempForwardPower * Math.sin(angle) + tempSidePower * Math.cos(angle);
        turn = gamepad1.right_stick_x;
    }

    /**
     * Dampen the Robot driving movements if right trigger is pressed
     */
    private void dampenSpeed() {
        float speedDamper = gamepad1.right_trigger;

        if (speedDamper == 1) {
            speedDamper = (float) 0.8;
        }

        forwardPower = forwardPower * (1 - speedDamper);
        sidePower = sidePower * (1 - speedDamper);
        turn = turn * (1 - speedDamper);
    }

    private void operateIntake(){
        if(gamepad1.a) {
            robot.intakesystem.startInTake();
        }else if(gamepad1.b) {
            robot.intakesystem.stopInTake();
        }else if(gamepad1.x) {
            robot.intakesystem.startReverseInTake();
        }
    }

    private void operateDelivery(){
        if (gamepad2.x) {
            robot.deliveryArmSystem.setLiftElevatorLow();
        }else if (gamepad2.a) {
            robot.deliveryArmSystem.setLiftElevatorMedium();
        }else if (gamepad2.b) {
            robot.deliveryArmSystem.setLiftElevatorHigh();
        }
    }
    private void operateCapstoneArm(){
        if (gamepad1.y) {
            if (!capStoneGrabed) {
                robot.capstoneArmSystem.grabCapstone();
                capStoneGrabed = true;
            } else if (capStoneGrabed) {
                robot.capstoneArmSystem.ungrabCapstone();
                capStoneGrabed = false;
            }
        }
    }

    /**
     * Drive the robot
     */
    private void moveRobot() {
        //Joystick Movement
        mecanum.move(forwardPower, sidePower, turn);
        //Push data
        pushTelemetry();
    }

    /**
     * Drive the robot
     */
    private void stopRobot() {
        //Joystick Movement
        mecanum.move(0, 0, 0);
        //Push data
        pushTelemetry();
    }


    private void pushTelemetry() {
        telemetry.addData("Gyro Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Drive Data", mecanum.getDataString());
        telemetry.update();
    }

}