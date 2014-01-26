package com.milkenknights.crimsoncrossbow;

import java.util.Enumeration;
import java.util.Vector;

import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
//import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.RGBImage;

public class Vision {
	AxisCamera camera;
	public Vision() {
		camera = AxisCamera.getInstance();
	}
	
	public int isHot() {
		int o;
		try {
			o = analyseTarget(camera.getImage());
		} catch (AxisCameraException e) {
			o = 0;
		} catch (NIVisionException e) {
			o = 0;
		}
		return o;
	}
	
	public void diagnostic() {
		for (int i = 1; i <= 6; i++) {
			System.out.println("testing image "+i);
			try {
				analyseTarget(new RGBImage("/test"+i+".jpg"));
			} catch (NIVisionException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	// returns 0 for no target, 1 for left, 2 for right
	public int analyseTarget(ColorImage img) {
		//ColorImage color;
		try {
			//color = camera.getImage();
			ParticleAnalysisReport[] report = img.thresholdHSL(0,255,0,16,238,255).
					//removeSmallObjects(true,7).
					convexHull(true).
					getOrderedParticleAnalysisReports();
			
			System.out.println("total particles "+report.length);
			int l = report.length;
			
			Vector horizontalCandidates = new Vector();
			Vector verticalCandidates = new Vector();			
			
			for (int i = 0; i<l; i++) {
				if (report[i].particleToImagePercent > 0.3) {
					System.out.println("L "+report[i].boundingRectLeft);
					System.out.println("T "+report[i].boundingRectTop);
					System.out.println("W "+report[i].boundingRectWidth);
					System.out.println("H "+report[i].boundingRectHeight);
					// check for horizontal criteria
					if (report[i].boundingRectWidth > report[i].boundingRectHeight) {
						double ratio = report[i].boundingRectWidth/report[i].boundingRectHeight;
						if (ratio > ((23.5/4) - 1.5) && ratio < ((23.5/4) + 1.5)) {
							horizontalCandidates.addElement(report[i]);
						}
						System.out.println("R "+ratio);
					} else {
						double ratio = report[i].boundingRectHeight/report[i].boundingRectWidth;
						if (ratio > ((32/4) - 3) && ratio < ((32/4) + 2)) {
							verticalCandidates.addElement(report[i]);
						}
						System.out.println("R "+ratio);
					}
				}
			}
			
			System.out.println("H "+horizontalCandidates.size());
			System.out.println("V "+verticalCandidates.size());
			// attempt to pair horizontal candidates with verticals
			// if we find a pair, the goal is hot
			for (Enumeration he = horizontalCandidates.elements(); he.hasMoreElements();) {
				ParticleAnalysisReport h = (ParticleAnalysisReport) he.nextElement();
				
				for (Enumeration ve = verticalCandidates.elements(); ve.hasMoreElements();) {
					ParticleAnalysisReport v = (ParticleAnalysisReport) ve.nextElement();
					
					// tape width check
					double ratioScore = getScore(((double)h.boundingRectHeight)
							/v.boundingRectWidth,1);
					
					// the distance between the center of the horizontal and closest vertical
					// edge should be 1.2 times larger than the width of the horizontal
					// first determine if we are looking at the left or right goal
					double leftRight = 0;
					boolean isLeft = true;
					if (h.center_mass_x > v.center_mass_x) {
						leftRight = getScore((h.center_mass_x - v.boundingRectLeft + v.boundingRectWidth) /
							h.boundingRectWidth, 1.2);
						isLeft = false;
					} else {
						leftRight = getScore((v.boundingRectLeft - h.center_mass_x) / h.boundingRectWidth,1.2);
					}
					// we can get the vertical score by taking the distance between the top edge of the vertical
					// target and the center of the horizontal target, then divide it by four times the height
					// of the horizantal target
					double vertical = 1 - (v.boundingRectTop-h.center_mass_y)/(h.boundingRectHeight*4);

					double totalScore = ratioScore + leftRight + vertical;
					System.out.println(""+totalScore);
					if (totalScore > 1.35) {
						return isLeft ? 1 : 2;
					}
				}
			}			
		} catch (NIVisionException e) {
			e.printStackTrace();
		}
		return 0;
	}

	private double getScore(double actual, double expected) {
		// score is just 1 minus the percent error
		return 1-((Math.abs(actual-expected))/expected);
	}
}
