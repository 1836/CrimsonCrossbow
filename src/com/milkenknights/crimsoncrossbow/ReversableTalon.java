package com.milkenknights.crimsoncrossbow;

import edu.wpi.first.wpilibj.Talon;

public class ReversableTalon extends Talon {
	boolean isReversed;
	
	public ReversableTalon(int channel, boolean isReversed) {
		super(channel);
		this.isReversed = isReversed;
		// TODO Auto-generated constructor stub
	}
	
	public void set(double speed) {
		if (isReversed) {
			speed = -speed;
		}
		super.set(speed);
	}
	public double get() {
		double speed = super.get();
		if (isReversed) {
			speed = -speed;
		}
		return speed;
	}

}
