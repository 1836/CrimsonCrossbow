package com.milkenknights.crimsoncrossbow;

// TODO: add comments to this file
public class SolenoidXANDPair extends SolenoidPair {
	public SolenoidXANDPair(int a, int b) {
		this(a, b, false);
	}

	public SolenoidXANDPair(int a, int b, boolean initial) {
		super(a, b, initial);
	}

	public void set(boolean on) {
		super.set(on);
		sa.set(get());
		sb.set(get());
	}
}
