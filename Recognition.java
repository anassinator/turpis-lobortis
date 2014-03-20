/**
 * Recognition.java
 * Recognizes different objects
 * @author  Anass Al-Wohoush
 * @version 0.2
 */

import lejos.nxt.*;
import lejos.util.*;
import lejos.robotics.*;

public class Recognition {
    private ColorSensor sensor;
    private Color color;
    private int red = 0, green = 0, blue = 0, sum = 0;

    // OBSTACLE TYPES
    private final int IDK = -1;
    private final int WOOD = 0;
    private final int RED = 1;
    private final int DARK_BLUE = 2;
    private final int LIGHT_BLUE = 3;
    private final int YELLOW = 4;
    private final int WHITE = 5;

    /**
     * Recognition constructor
     */
    public Recognition(Robot robot) {
        this.sensor = robot.centerColor;
    }

    /**
     * Returns recognized object
     *
     * <TABLE BORDER=1>
     * <TR><TH>Block Code</TH><TH>Wooden Block</TH></TR>
     * <TR><TD>-1</TD><TD>Not Recognized</TD></TR>     
     * <TR><TD>0</TD><TD>Red Block</TD></TR>
     * <TR><TD>1</TD><TD>Red Block</TD></TR>
     * <TR><TD>2</TD><TD>Dark Blue Block</TD></TR>
     * <TR><TD>3</TD><TD>Light Blue Block</TD></TR>
     * <TR><TD>4</TD><TD>Yellow  Block</TD></TR>
     * <TR><TD>5</TD><TD>White Block</TD></TR>
     * </TABLE>
     *
     * @return the block code
     */
    public int recognize() {
        while (true) {
            color = sensor.getColor();

            red = color.getRed();
            green = color.getGreen();
            blue = color.getBlue();
            sum = red + green + blue;

            LCD.drawInt(red, 0, 0);
            LCD.drawInt(green, 0, 1);
            LCD.drawInt(blue, 0, 2);

            if (isWood()) {
                LCD.drawString("WOOD      ", 0, 3);
                return WOOD;
            } else if (isRed()) {
                LCD.drawString("RED       ", 0, 3);
                return RED;
            } else if (isDarkBlue()) {
                LCD.drawString("DARK BLUE ", 0, 3);
                return DARK_BLUE;
            } else if (isLightBlue()) {
                LCD.drawString("LIGHT BLUE", 0, 3);
                return LIGHT_BLUE;
            } else if (isYellow()) {
                LCD.drawString("YELLOW    ", 0, 3);
                return YELLOW;
            } else if (isWhite()) {
                LCD.drawString("WHITE     ", 0, 3);
                return WHITE;
            } else {
                LCD.drawString("IDK       ", 0, 3);
                return IDK;
            }
        }
    }

    /**
     * Returns whether block is red
     *
     * @return <code>true</code> if block is red; <code>false</code> otherwise
     */
    public boolean isRed() {
        return (red / (double) sum) > 0.60;
    }

    /**
     * Returns whether block is dark blue
     *
     * @return <code>true</code> if block is dark blue; <code>false</code> otherwise
     */
    public boolean isDarkBlue() {
        return (blue / (double) sum) > 0.45;
    }

    /**
     * Returns whether block is light blue
     *
     * @return <code>true</code> if block is light blue; <code>false</code> otherwise
     */
    public boolean isLightBlue() {
        return (red / (double) sum) < (green / (double) sum) && (red / (double) sum) < (blue / (double) sum);
    }

    /**
     * Returns whether block is yellow
     *
     * @return <code>true</code> if block is yellow; <code>false</code> otherwise
     */
    public boolean isYellow() {
        return (red / (double) sum) > 0.45 && (blue / (double) sum) < 0.15;
    }

    /**
     * Returns whether block is white
     *
     * @return <code>true</code> if block is white; <code>false</code> otherwise
     */
    public boolean isWhite() {
        return (red / (double) sum) > 0.30 && (green / (double) sum) > 0.30 && (blue / (double) sum) > 0.30;
    }

    /**
     * Returns whether block is wooden
     *
     * @return <code>true</code> if block is wooden; <code>false</code> otherwise
     */
    public boolean isWood() {
        return (red / (double) sum) > 0.40 && (green / (double) sum) > 0.20 && (blue / (double) sum) > 0.20;
    }
}
