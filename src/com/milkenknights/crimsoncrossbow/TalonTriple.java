package com.milkenknights.crimsoncrossbow;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class TalonTriple implements SpeedController {
	private Talon ma;
	private Talon mb;
	private Talon mc;

	// whether or not the Talon should be reversed
	private boolean reva;
	private boolean revb;
	private boolean revc;

	private double speed;
	
	/**
	 * Creates a triple of Talons
	 *
	 * @param a The PWM channel that the first Talon is attached to.
	 * @param b The PWM channel that the second Talon is attached to.
	 * @param c The PWM channel that the third Talon is attached to.
	 */
	public TalonTriple(int a, int b, int c) {
		this(a,false,b,false,c,false);
	}

	/**
	 * Creates a triple of Talons, with options to reverse motors.
	 *
	 * @param a The PWM channel that the first Talon is attached to.
	 * @param arev If Talon A should be reversed.
	 * @param b The PWM channel that the second Talon is attached to.
	 * @param brev If Talon B should be reversed.
	 * @param c The PWM channel that the third Talon is attached to.
	 * @param crev If Talon C should be reversed.
	 */
	public TalonTriple(int a, boolean arev, int b, boolean brev, int c, boolean crev) {
		ma = new Talon(a);
		ma.set(0);
		reva = arev;

		mb = new Talon(b);
		mb.set(0);
		revb = brev;
		
		mc = new Talon(c);
		mc.set(0);
		revc = crev;
	}

	public void disable() {
		ma.disable();
		mb.disable();
		mc.disable();
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
	public double getC() {
		return mc.get();
	}

	/**
	 * Set the PWM value for the Talons.
	 *
	 * @param newspeed The speed value between -1.0 and 1.0 to set.
	 */
	public void set(double newspeed) {
		ma.set(reva ? -newspeed : newspeed);
		mb.set(revb ? -newspeed : newspeed);
		mc.set(revc ? -newspeed : newspeed);
	}

	// we have to implement these methods as part of SpeedController
	// even though they're not really used
	public void set(double newspeed, byte arg1) {
		set(newspeed);
	}
	public void pidWrite(double output) {
		ma.pidWrite(reva ? -output : output);
		mb.pidWrite(revb ? -output : output);
		mc.pidWrite(revc ? -output : output);
	}

	
}
