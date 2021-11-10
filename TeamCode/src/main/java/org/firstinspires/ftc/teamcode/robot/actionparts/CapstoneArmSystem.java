package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneArmSystem {

    //The Servo to lift the arm up and down
    private Servo liftServo;

    //Servo used to grab the capstone
    private Servo grabServo;

    //Servo Positions
    private final double UP_POSITION = 0.0;
    private final double DOWN_POSITION = 1.0;
    private final double CLOSED_POSITION = 0.6;
    private final double OPEN_POSITION = 0.47;


    /**
     * Constructor
     *
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
    public void initialize() {
        ungrabCapstone();
        armUp();
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

    public void armUp() {
        liftServo.setPosition(UP_POSITION);
    }

    /**
     * Sets the wobble goal post down
     */
    public void armDown() {
        liftServo.setPosition(DOWN_POSITION);
    }


    public void moveCapstoneArm(int capstonearmState) {

        switch (capstonearmState) {
            case 0:
                ungrabCapstone();
                armUp();
                break;

            case 1:
                ungrabCapstone();
                armDown();
                break;

            case 2:
                grabCapstone();
                armDown();
                break;

            case 3:
                grabCapstone();
                armUp();
                break;
        }
    }
}
