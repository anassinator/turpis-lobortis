/**
 * Recognition.java
 * Recognizes different objects
 * @author  Anass Al-Wohoush
 * @version 0.1
 */

import lejos.nxt.*;
import lejos.util.*;
import lejos.robotics.*;

public class Recognition {
    private ColorSensor sensor;
    private Color color;
    private int red = 0, green = 0, blue = 0, sum = 0;

    public Recognition(Robot robot) {
        this.sensor = robot.centerColor;
    }

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

            if (isWood())
                LCD.drawString("WOOD      ", 0, 3);
            else if (isRed())
                LCD.drawString("RED       ", 0, 3);
            else if (isDarkBlue())
                LCD.drawString("DARK BLUE ", 0, 3);
            else if (isLightBlue())
                LCD.drawString("LIGHT BLUE", 0, 3);
            else if (isYellow())
                LCD.drawString("YELLOW    ", 0, 3);
            else if (isWhite())
                LCD.drawString("WHITE     ", 0, 3);
            else
                LCD.drawString("IDK       ", 0, 3);
        }
    }

    public boolean isRed() {
        return (red / (double) sum) > 0.60;
    }

    public boolean isDarkBlue() {
        return (blue / (double) sum) > 0.45;
    }

    public boolean isLightBlue() {
        return (red / (double) sum) < (green / (double) sum) && (red / (double) sum) < (blue / (double) sum);
    }

    public boolean isYellow() {
        return (red / (double) sum) > 0.45 && (blue / (double) sum) < 0.15;
    }

    public boolean isWhite() {
        return (red / (double) sum) > 0.30 && (green / (double) sum) > 0.30 && (blue / (double) sum) > 0.30;
    }

    public boolean isWood() {
        return (red / (double) sum) > 0.40 && (green / (double) sum) > 0.20 && (blue / (double) sum) > 0.20;
    }
}
