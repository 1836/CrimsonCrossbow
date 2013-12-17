package com.milkenknights.crimsoncrossbow;

import edu.wpi.first.wpilibj.Joystick;

public class JStick {
	public static final int XBOX_A = 1;
	public static final int XBOX_B = 2;
	public static final int XBOX_X = 3;
	public static final int XBOX_Y = 4;
	public static final int XBOX_LB = 5;
	public static final int XBOX_RB = 6;
	public static final int XBOX_BACK = 7;
	public static final int XBOX_START = 8;
	public static final int XBOX_LJ = 9;
	public static final int XBOX_RJ = 10;

	public static final int XBOX_LSX = 1; // left stick x
	public static final int XBOX_LSY = 2; // left stick y
	public static final int XBOX_TRIG = 3; // left is positive
	public static final int XBOX_RSX = 4; // right stick x
	public static final int XBOX_RSY = 5; // right stick y
	public static final int XBOX_DPAD = 6; // buggy

	public static final int JOYSTICK_KNOB = 3;
	
	public static final int MAX_BUTTONS = 12; // as specified in the docs
	public static final int MAX_AXES = 6; // as specificed in the docs

	private Joystick jstick;
	private boolean[] buttonPressed;
	private boolean[] buttonLastPressed;
	private double[] axes;
	private double[] slowAxes;

	private double slow;

	public JStick(int port) {
		// initialize everything
		jstick = new Joystick(port);
		buttonPressed = new boolean[MAX_BUTTONS+1];
		buttonLastPressed = new boolean[MAX_BUTTONS+1];
		axes = new double[MAX_AXES+1];
		slowAxes = new double[MAX_AXES+1];
		slow = 2;
	}

	public void update() {
		for(int i = 1; i < buttonPressed.length; ++i) {
			buttonLastPressed[i] = buttonPressed[i];
			buttonPressed[i] = jstick.getRawButton(i);
		}

		for(int i = 1; i < axes.length; ++i) {
			double newAxis = jstick.getRawAxis(i);
			
			if (newAxis - axes[i] > slow) {
				slowAxes[i] += slow;
			} else if (axes[i] - newAxis > slow) {
				slowAxes[i] -= slow;
			} else {
				slowAxes[i] = newAxis;
			}
			axes[i] = newAxis;
		}
	}

	/**
	 * The output of joystick axes can be slowed down
	 * so that after each update its output will only
	 * deviate from previous value at a maximum of the
	 * slow value.
	 *
	 * @param r The new slow value. This should be a positive number.
	 */
	public void setSlow(double s) {
		slow = Math.abs(s);
	}

	public double getSlow() {
		return slow;
	}
	
	/**
	 * Gets the button value
	 *
	 * @param b The button number to be read.
	 * @return The state of the button.
	 */
	public boolean isPressed(int b) {
		if(b >= 0 && b < buttonPressed.length)
			return buttonPressed[b];
		else return false;
	}

	/**
	 * Gets whether or not the button is being released
	 *
	 * @param b The button number to be read.
	 * @return True if the button was pressed in the last update but not now.
	 */
	public boolean isReleased(int b) {
		if(b >= 0 && b < buttonPressed.length)
			return !buttonPressed[b] && buttonLastPressed[b];
		else return false;
	}

	/**
	 * Gets the value of the axis.
	 *
	 * @param b The axis to read.
	 * @return The value of the axis.
	 */
	public double getAxis(int b) {
		if(b >= 0 && b < axes.length)
			return axes[b];
		else return 0;
	}

	public double getSlowedAxis(int b) {
		if(b >= 0 && b < axes.length)
			return slowAxes[b];
		else return 0;
	}
	
	/**
	 * If the absolute value of the input is in the jitterRange, return 0.
	 *
	 * @param in The input
	 * @param jitterRange the range that the input shouldn't be in. Should be positive.
	 * @return The input with jitter removed if necessary
	 */
	public static double removeJitter(double in, double jitterRange) {
		if (Math.abs(in) > jitterRange) {
			return in;
		} else {
			return 0;
		}
	}
}
