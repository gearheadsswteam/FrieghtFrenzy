package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneArmSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneDetector;
import org.firstinspires.ftc.teamcode.robot.actionparts.DeliveryArmSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.DuckrotationSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GearheadsMecanumRobotRR {

    //drive train for TelOp only
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor rl_motor;
    public DcMotor rr_motor;

    //Gyro
    public BNO055IMU imu;

    //servo used to raise capstone arm
    private Servo liftServo;

    //Servo used to grab the capstone
    private Servo grabServo;

    // Motor used for the intake system
    private DcMotor intakeMotor;

    //The Motor to lift the Elevator
    private DcMotor liftElevator;

    //Servo to tilt the bucket
    private Servo tiltBucket;

    private Servo duckServo;

    public CapstoneArmSystem capstoneArmSystem;
    public Intakesystem intakesystem;
    public DeliveryArmSystem deliveryArmSystem;
    public DuckrotationSystem duckrotationSystem;

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    public HardwareMap hwMap = null;

    public CapstoneDetector capstoneDetector;

    /* Constructor */
    public GearheadsMecanumRobotRR(LinearOpMode opMode) {
        this.curOpMode = opMode;
        hwMap = opMode.hardwareMap;
    }



    /**
     * Initializes the Gyro
     *
     * @param calibrate
     */
    private void initGyro(boolean calibrate) {
        imu = hwMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
        calibrate = false;
        if (calibrate) {
            curOpMode.telemetry.addData("Mode", "calibrating...");
            curOpMode.telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!curOpMode.isStopRequested() && !imu.isGyroCalibrated()) {
                curOpMode.sleep(10);
                curOpMode.idle();
            }
        }

        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        curOpMode.telemetry.update();
    }


    /**
     * Initializes the drive train
     */
    private void initDriveMotors() {

        //DRIVING
        fl_motor = hwMap.dcMotor.get("fl");
        fr_motor = hwMap.dcMotor.get("fr");
        rl_motor = hwMap.dcMotor.get("bl");
        rr_motor = hwMap.dcMotor.get("br");


        //This is based on how motors have been mounted
        fr_motor.setDirection(DcMotor.Direction.FORWARD);
        rr_motor.setDirection(DcMotor.Direction.FORWARD);
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        rl_motor.setDirection(DcMotor.Direction.REVERSE);// BL motor works inverted...not sure why

        fr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize Capstone Arm System
     */
    private void initCapstoneArmSystem() {

        //read hardware
        liftServo = hwMap.servo.get("liftServo");
        grabServo = hwMap.servo.get("grabServo");

        capstoneArmSystem = new CapstoneArmSystem(liftServo, grabServo);

        capstoneArmSystem.initialize();
    }

    /**
     * Initialize Capstone Arm System
     */
    private void initIntakeSystem() {

        //read hardware
        intakeMotor = hwMap.dcMotor.get("intakeMotor");

        intakesystem = new Intakesystem(intakeMotor);

        intakesystem.initialize();

    }

    /**
     * Initialize Capstone Arm System
     */
    private void initDuckRotationystem() {

        //read hardware
        duckServo = hwMap.servo.get("duckServo");

        duckrotationSystem = new DuckrotationSystem(duckServo);

        duckrotationSystem.initialize();

    }


    private void initDeliveryArmSystem() {

        liftElevator = hwMap.dcMotor.get("liftElevator");
        tiltBucket = hwMap.servo.get("tiltBucket");

        deliveryArmSystem = new DeliveryArmSystem (liftElevator , tiltBucket);

        deliveryArmSystem.initialize();
    }

    private void initCapstoneDetector(){
        capstoneDetector = new CapstoneDetector();
        capstoneDetector.intitalize(curOpMode);
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleopRR(HardwareMap ahwMap) {
        init(ahwMap);
        initGyro(true);
    }

    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap, String teamType) {
        init(ahwMap);
        initGyro(true);
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        init(ahwMap);
        initGyro(true);
        initDriveMotors();
    }

    private void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initDriveMotors();
        initCapstoneArmSystem();
        initIntakeSystem();
        initDeliveryArmSystem();
        //initCapstoneDetector();
    }
}


