package ca.etsmtl.capra.smartmotor.io;
import gnu.io.SerialPort;

public class MotorController
{
	private InputControl inputControl;
	private SerialCom serialCom;
	
	private SerialPort serialPort;
	private Connector connector;

	//Singleton
	private static MotorController instance;
	private MotorController(){
	}
	
	public static MotorController getInstance(){
		if(instance == null)
			instance = new MotorController();
		
		return instance;
	}

	public void endThreads(){
		serialCom.stop();
		
		disconnect();
	}

	public void write(String command){
		serialCom.write(command);
	}
	
	double[] newCountRL = {0,0};
	
	public void setWaitForEcho(boolean val){
		serialCom.setWaitForEcho(val);
	}

	public boolean connect(int nbMotors, String portName){
		
		inputControl = new InputControl(nbMotors);
		serialCom = new SerialCom(inputControl);
		
		try {
			connector = new Connector(portName);
			if ( connector.connect() )
			{
				System.out.println("SerialManager: Port " + connector.getSerialPort().getName() + " allocated");
			    serialPort = connector.getSerialPort();
			}
		} 
		catch (Exception e1) 
		{
			e1.printStackTrace();
			return false;
		}

		if(serialPort == null)
			return false;

		serialCom.connect(serialPort);
		Thread threadRequests = new Thread(serialCom);
		threadRequests.start();
	
		return true;
	}

	public void disconnect()
	{
		connector.disconnect();
	}
	
	public InputControl getInputControl(){
		return inputControl;
	}

}