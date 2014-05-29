package ca.etsmtl.capra.motors.smartmotor;

import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.Quaternion;

public class QuaternionUtils
{
    public static Quaternion fromYaw(double yaw)
    {
        double halfYaw = yaw * 0.5;
        double cosYaw = Math.cos(halfYaw);
        double sinYaw = Math.sin(halfYaw);
        return new Quaternion(0, 0, sinYaw, cosYaw);
    }

    public static geometry_msgs.Quaternion quaternionToMsg(ConnectedNode node, Quaternion q)
    {
        q = q.normalize();

        geometry_msgs.Quaternion quaternion = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
        quaternion.setX(q.getX());
        quaternion.setY(q.getY());
        quaternion.setZ(q.getZ());
        quaternion.setW(q.getW());

        return quaternion;
    }

    public static geometry_msgs.Quaternion createQuaternionMsgFromYaw(ConnectedNode node, double yaw)
    {
        return quaternionToMsg(node, fromYaw(yaw));
    }
}
