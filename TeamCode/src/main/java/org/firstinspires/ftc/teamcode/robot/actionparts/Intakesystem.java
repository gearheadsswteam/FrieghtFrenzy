package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class the represents the Intake System
 */
public class Intakesystem {
    //DC motor used by the intake system
    public DcMotor intakeMotor;
    public Servo intakeServo;

    private double servoIntakeBlockPosition = 0.65;
    private double servoIntakeOpenPosition = 0.31;

    private boolean intakeOpen = true;
    /**
     * Constructor
     * @param intakeMotor
 */
    public Intakesystem(DcMotor intakeMotor, Servo intakeServo) {
        this.intakeMotor = intakeMotor;
        this.intakeServo = intakeServo;
    }

    public void toggleIntakeOpenClosePosition() {
        if (intakeOpen) {
            intakeServo.setPosition(servoIntakeBlockPosition);
            intakeOpen = false;
        } else {
            intakeServo.setPosition(servoIntakeOpenPosition);
            intakeOpen = true;
        }
    }

    /**
     * Initialize the system
     */
    public void initialize(){
        intakeServo.setPosition(servoIntakeBlockPosition);
        intakeOpen = false;
    }

    /**
     * Start the intake system
     */
    public void startInTake() {
        intakeMotor.setPower(-0.5);
       // intakeServo.setPosition(servoIntakeBlockPosition);
    }

    /**
     * Stop the intake system
     */
    public void stopInTake() {
        intakeMotor.setPower(0);
    }

    /**
     * Start the intake system
     */
    public void startReverseInTake() {
        intakeMotor.setPower(1.0);
    }
}
