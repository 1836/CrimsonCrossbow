package com.milkenknights.crimsoncrossbow;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class TalonPair implements SpeedController {
	private Talon ma;
	private Talon mb;

	// whether or not the Talon should be reversed
	private boolean reva;
	private boolean revb;

	private double speed;

	/**
	 * Creates a pair of Talons
	 *
	 * @param a The PWM channel that the first Talon is attached to.
	 * @param b The PWM channel that the second Talon is attached to.
	 */
	public TalonPair(int a, int b) {
		// if we aren't told that the motors are reversed or not,
		// set them to false by default
		this(a,false,b,false);
	}

	/**
	 * Creates a pair of Talons, with options to reverse motors.
	 *
	 * @param a The PWM channel that the first Talon is attached to.
	 * @param arev If Talon A should be reversed.
	 * @param b The PWM channel that the second Talon is attached to.
	 * @param brev If Talon B should be reversed.
	 */
	public TalonPair(int a, boolean arev, int b, boolean brev) {
		ma = new Talon(a);
		ma.set(0);
		reva = arev;

		mb = new Talon(b);
		mb.set(0);
		revb = brev;
	}

	/**
	 * Get the recently set value of the Talons
	 *
	 * @return The most recently set value
	 */
	public double get() {
		return speed;
	}

	public double getA() {
		return ma.get();
	}

	public double getB() {
		return mb.get();
	}

	/**
	 * Set the PWM value for the Talons.
	 *
	 * @param newspeed The speed value between -1.0 and 1.0 to set.
	 */
	public void set(double newspeed) {
		speed = newspeed;
		ma.set(reva ? -newspeed : newspeed);
		mb.set(revb ? -newspeed : newspeed);
	}

	public void disable() {
		ma.disable();
		mb.disable();
	}

	// we have to implement these methods as part of SpeedController
	// even though they're not really used
	public void pidWrite(double output) {
		ma.pidWrite(reva ? -output : output);
		mb.pidWrite(revb ? -output : output);
	}
	public void set(double speed, byte syncGroup) {
		set(speed);
	}
}
