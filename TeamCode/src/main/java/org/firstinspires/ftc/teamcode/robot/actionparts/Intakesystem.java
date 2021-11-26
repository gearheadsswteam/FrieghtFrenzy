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

    private double gateUp = 0.65;
    private double gateDown = 0.31;

    private boolean intakeOpen = true;

    /**
     * Constructor
     *
     * @param intakeMotor
     */
    public Intakesystem(DcMotor intakeMotor, Servo intakeServo) {
        this.intakeMotor = intakeMotor;
        this.intakeServo = intakeServo;
    }

    public void toggleIntakeOpenClosePosition() {
        if (intakeOpen) {
            intakeServo.setPosition(gateUp);
            intakeOpen = false;
        } else {
            intakeServo.setPosition(gateDown);
            intakeOpen = true;
        }
    }

    public void intakeGateOpen() {
        intakeServo.setPosition(gateDown);
        intakeOpen = true;
    }

    public void intakeGateClosed() {
        intakeServo.setPosition(gateUp);
        intakeOpen = false;
    }

    /**
     * Initialize the system
     */
    public void initialize() {
        intakeServo.setPosition(gateDown);
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
