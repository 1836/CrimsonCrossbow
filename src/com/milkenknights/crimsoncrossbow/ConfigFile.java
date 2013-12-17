package com.milkenknights.crimsoncrossbow;

import com.sun.squawk.io.BufferedReader;
import com.sun.squawk.microedition.io.FileConnection;

import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Hashtable;

import javax.microedition.io.Connector;

public class ConfigFile extends RobotConfig {
	private String fileName;
	private Hashtable configMap;
	
	public ConfigFile(String file) {
		fileName = file;
		configMap = new Hashtable();
	}

	public void loadFile() {
		FileConnection file = null;

		try {
			file = (FileConnection) Connector.open("file:///"+fileName, Connector.READ);

			if (file.exists()) {
				BufferedReader reader = new BufferedReader(new InputStreamReader(file.openInputStream()));
				
				String l;
				while((l = reader.readLine()) != null) {
					// we don't have String.split so we have to do it manually
					int equalPos = l.indexOf('=');
					if (equalPos != -1) {
						configMap.put(l.substring(0,equalPos),l.substring(equalPos+1));
					}
				}
				reader.close();
			}

		} catch (IOException ex) {
			ex.printStackTrace();
		} finally {
			if (file != null) {
				try {
					file.close();
				} catch (IOException ex) {}
			}
		}
		System.out.println("loaded robot config");
		System.out.println(configMap.toString());
	}

	public String customGet(Object k) {
		return configMap.get(k).toString();
	}

}
