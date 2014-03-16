/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
* 
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition
*/
package bluetooth;

public enum PlayerRole {
	GREEN(1, "G"),
	RED(2, "R"),
	NULL(0, "");
	
	private int role;
	private String str;
	
	private PlayerRole(int rl, String str) {
		this.role = rl;
		this.str = str;
	}
	
	public String toString() {
		return this.str;
	}
	
	public int getId() {
		return this.role;
	}
	
	public static PlayerRole lookupRole(int rl) {
		for (PlayerRole role : PlayerRole.values())
			if (role.getId() == rl)
				return role;
		return PlayerRole.NULL;
	}
}
