package ca.etsmtl.capra;

import java.util.Observable;
import java.util.Observer;

import ca.etsmtl.capra.smartmotorlib.Configuration;
import ca.etsmtl.capra.smartmotorlib.listeners.PositionListener;
import ca.etsmtl.capra.smartmotorlib.robots.Robot;
import ca.etsmtl.capra.smartmotorlib.Position;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import ca.etsmtl.capra.smartmotorlib.MotorController;

public class SmartMotor extends AbstractNodeMain
{
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////// Parameters //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private static String PARAM_NAME_N_MOTORS = "~n_motors";
	private static String PARAM_NAME_PORT_NAME = "~port";
	private static String PARAM_NAME_WATCHDOG_FREQ = "~watchdog_rate";
	private static String PARAM_NAME_COVARIANCE = "~covariance";
	
	private static String DEFAULT_PORT_NAME = "/dev/ttyUSB1002";
	private static int DEFAULT_N_MOTORS = 2;
	private static int DEFAULT_WATCHDOG_RATE = 5;
	private static double DEFAULT_COVARIANCE = 100.0;
	
	private int nMotors;
	private String portName;
	private int watchdogFreq = DEFAULT_WATCHDOG_RATE;	
	private double covariance = 100.0;
	
	ParameterTree params;
	
    ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Topics ///////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private static String TOPIC_NAME_ODOM = "odom";
	private static String TOPIC_NAME_CMD_VEL = "~cmd_vel";
	
	private Publisher<nav_msgs.Odometry> odomPublisher;
	private Subscriber<geometry_msgs.Twist> cmdVelSubscriber;

    ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Other ////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private ConnectedNode node;
	private MotorController motors;
	
	private CommandType commandType = CommandType.CMD_VEL;
	
	private float commandedLinearVelocity = 0;
	private float commandedAngularVelocity = 0;
	private long lastVelocityUpdate = 0;
	
	@Override
	public GraphName getDefaultNodeName()
	{
		return GraphName.of("capra_smartmotor");
	}
	
	private void initTopics()
	{
		odomPublisher = node.newPublisher(TOPIC_NAME_ODOM, nav_msgs.Odometry._TYPE);
		
		cmdVelSubscriber = node.newSubscriber(TOPIC_NAME_CMD_VEL, geometry_msgs.Twist._TYPE);
		cmdVelSubscriber.addMessageListener(new cmdVelListener());
	}
	
	private void loadParams ( )
	{
		params = node.getParameterTree();
		
		nMotors = params.getInteger(PARAM_NAME_N_MOTORS, DEFAULT_N_MOTORS);
		portName = params.getString(PARAM_NAME_PORT_NAME, DEFAULT_PORT_NAME);
		watchdogFreq = params.getInteger(PARAM_NAME_WATCHDOG_FREQ, DEFAULT_WATCHDOG_RATE);
		covariance = params.getDouble(PARAM_NAME_COVARIANCE, DEFAULT_COVARIANCE);
	}
	
	private class cmdVelListener implements MessageListener<geometry_msgs.Twist>
	{
		@Override
		public void onNewMessage(Twist cmd_vel) 
		{
			float newLinear = (float)cmd_vel.getLinear().getX();
			float newAngular = (float)cmd_vel.getAngular().getZ();
			
			lastVelocityUpdate = System.currentTimeMillis();
			commandType = CommandType.CMD_VEL;
			
			if ( commandedLinearVelocity != newLinear || commandedAngularVelocity != newAngular )
			{
				commandedLinearVelocity = newLinear;
				commandedAngularVelocity = newAngular;
				motors.setVelocity(commandedLinearVelocity, commandedAngularVelocity);
			}
		}
	}
	
	private void initServices()
	{
		
	}
	
	private boolean connectToMotors()
	{
        Robot robot = Configuration.getRobot();
        robot.setPortName(portName);
        robot.setNbMotors(nMotors);
        motors = new MotorController();

		return motors.init();
	}
	
	private void initWatchdog ( )
	{
		node.executeCancellableLoop(new CancellableLoop()
		{
			@Override
			protected void loop() throws InterruptedException
			{
                if ( motors.isMoving() && System.currentTimeMillis() - lastVelocityUpdate > 2 * (1000 / watchdogFreq)
					 && commandType == CommandType.CMD_VEL )
				{
					commandedLinearVelocity = 0;
					commandedAngularVelocity = 0;
					motors.setVelocity(commandedLinearVelocity, commandedAngularVelocity);
				}
				
				Thread.sleep(1000 / watchdogFreq);		
			}
		});
	}
	
	private void initTelemetryPublishers()
	{
        motors.setPositionListener(new PositionListener() {
            @Override
            public void onNewPosition(Position pos) {
                nav_msgs.Odometry odom = node.getTopicMessageFactory().newFromType(nav_msgs.Odometry._TYPE);

                odom.getHeader().setFrameId("odom");
                odom.setChildFrameId("base_link");

                // Position
                geometry_msgs.Point position = odom.getPose().getPose().getPosition();
                position.setX(pos.getX());
                position.setY(pos.getY());
                position.setZ(0);

                // Orientation
                Quaternion quaternion = QuaternionUtils.createQuaternionMsgFromYaw(node, pos.getTheta());
                odom.getPose().getPose().setOrientation(quaternion);

                // Velocity
                //Todo: Modifier pour avoir la vraie valeur
                odom.getTwist().getTwist().getLinear().setX(commandedLinearVelocity);
                odom.getTwist().getTwist().getAngular().setZ(commandedAngularVelocity);

                // Covariance
                //Todo Trouver la vraie covariance
                double[] covariance_matrix = new double[]{  covariance, 0, 0, 0, 0, 0,
                                                            0, covariance, 0, 0, 0, 0,
                                                            0, 0, covariance, 0, 0, 0,
                                                            0, 0, 0, covariance, 0, 0,
                                                            0, 0, 0, 0, covariance, 0,
                                                            0, 0, 0, 0, 0, covariance};
                odom.getPose().setCovariance(covariance_matrix);
                odom.getTwist().setCovariance(covariance_matrix);

                odomPublisher.publish(odom);
            }
        });
	}

	private void init ( )
	{
		initTopics();
		initServices();
		initWatchdog();
		initTelemetryPublishers();
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode)
	{
		node = connectedNode;
		loadParams();
		
		if ( connectToMotors() )
			init();
		else 
			throw new RuntimeException("Error connecting to the motors");
	}
}
