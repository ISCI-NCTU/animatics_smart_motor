package ca.etsmtl.capra.smartmotor.io;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;

import java.util.Enumeration;

public class Connector {

	private SerialPort serialPort;
	private String portName;
	
	public Connector(String portName){
		this.portName = portName;
	}
	
	public SerialPort getSerialPort(){
		return serialPort;
	}
	
	// ---------------- Port Check ---------------

	public boolean checkPort(){

		System.out.println("Checking port...");
		Enumeration<?> thePorts = CommPortIdentifier.getPortIdentifiers();
		while (thePorts.hasMoreElements()) {
			CommPortIdentifier com = (CommPortIdentifier) thePorts.nextElement();
			System.out.println("on a un port! " + com.getName());
			try {
				CommPort thePort = com.open("CommUtil", 50);
				thePort.close();
				System.out.println("Ca connecte!");
				return true;

			} catch (PortInUseException e) {
				System.out.println("Port, "  + com.getName() + ", is in use.");
				return false;
			} catch (Exception e) {
				System.err.println("Failed to open port " +  com.getName());
				e.printStackTrace();
				return false;
			}
		}
		return true;
	}

	// ----------- CONNECTION --------------


	//Choix du port, connection et setup du port
	public boolean connect(){

		boolean success = false;

		try{
			CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);

			CommPort commPort = portIdentifier.open("SerialRead",2000);

			serialPort = (SerialPort) commPort;
			serialPort.setSerialPortParams(9600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);

			success = true;
		} catch (PortInUseException e){
			System.out.println("SerialManager: Connector: Port " + portName + " in use");
		} catch (NoSuchPortException e){
			System.out.println("SerialManager: Connector: Port " + portName + " not found");
//			e.printStackTrace();

		} catch (Exception e){
			e.printStackTrace();
		}

		return success;

	}

	public void disconnect() {
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		serialPort.close();
	}
}