/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
* 
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition
*/
import lejos.nxt.Button;
import lejos.nxt.LCD;
import bluetooth.*;

public class BTTest {
	// example call of the transmission protocol
	// The print function is just for debugging to make sure data is received correctly
	// make sure to import the bluetooth.BluetoothConnection library
	@SuppressWarnings("unused")
	public static void main(String [] args) {
		BluetoothConnection conn = new BluetoothConnection();
		
		// as of this point the bluetooth connection is closed again, and you can pair to another NXT (or PC) if you wish
		
		// example usage of Tranmission class
		Transmission t = conn.getTransmission();
		if (t == null) {
			LCD.drawString("Failed to read transmission", 0, 5);
		} else {
			PlayerRole role = t.role;
			StartCorner corner = t.startingCorner;
			int greenZoneLL_X = t.greenZoneLL_X;
			int greenZoneLL_Y = t.greenZoneLL_Y;
			int redZoneLL_X = t.redZoneLL_X;
			int redZoneLL_Y = t.redZoneLL_Y;
			int greenDZone_X = t.greenDZone_X;
			int greenDZone_Y = t.greenDZone_Y;
			int redDZone_X = t.redDZone_X;
			int redDZone_Y = t.redDZone_Y;
			int greenFlag = t.greenFlag;
			int	redFlag = t.redFlag;
		
			// print out the transmission information
			conn.printTransmission();
		}
		// stall until user decides to end program
		Button.ESCAPE.waitForPress();
	}
}
