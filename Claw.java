import lejos.nxt.*;
import lejos.util.*;

/**
 * Grabs, lifts and drops objects
 * @author  Anass Al-Wohoush
 * @version 0.5
 */
public class Claw {
    // OBJECTS
    public NXTRegulatedMotor claw;
    public boolean isDown = true;

    /**
     * Claw constructor
     *
     * @param claw       motor port controlling claw
     */
    public Claw(NXTRegulatedMotor claw) {
        this.claw = claw;
    }

    /**
     * Lowers arm and opens claw
     */
    public void drop() {
        isDown = true;
        claw.setSpeed(360);
        claw.rotate(8*180);
    }

    /**
     * Grabs object and lifts arm
     */
    public void grab() {
        claw.setSpeed(360);
        claw.rotate(-8*180);
        isDown = false;
    }
}
