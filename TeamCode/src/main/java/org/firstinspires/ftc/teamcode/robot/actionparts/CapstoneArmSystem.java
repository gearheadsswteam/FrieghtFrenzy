package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneArmSystem {

    //The Servo to lift the arm up and down
    private Servo liftServo;

    //Servo used to grab the capstone
    private Servo grabServo;

    //Servo Positions
    //todo change positions
    private final double UP_POSITION = 0.56;
    private final double DOWN_POSITION = 0.41;
    private final double CLOSED_POSITION = 0.0;
    private final double OPEN_POSITION = 0.5;


    /**
     * Constructor
     * @param liftServo
     * @param grabServo
     */
    public CapstoneArmSystem(Servo liftServo, Servo grabServo) {
        this.liftServo = liftServo;
        this.grabServo = grabServo;
    }

    /**
     * Initialize the Capstone System
     */
    public void initialize(){

    }

    /**
     * Grabs the wobble goal
     */
    public void grabCapstone() {
        grabServo.setPosition(CLOSED_POSITION);
    }

    /**
     * Ungrabs the wobble goal
     */
    public void ungrabCapstone() {
        grabServo.setPosition(OPEN_POSITION);
    }

    /**
     * Lifts the wobble goal post
     */

    public void liftArm() {
        liftServo.setPosition(UP_POSITION);
    }

    /**
     * Sets the wobble goal post down
     */
    public void resetArm() {
        liftServo.setPosition(DOWN_POSITION);
    }

    /**
     * The position to move the arm to
     *
     * @param leftArmState
     */

}
