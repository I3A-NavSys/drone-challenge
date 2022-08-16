ros_master_ip = "192.168.18.14";
ros_master_port = 11311;

%Tópicos
topics_odometry = "/quadcopter/+/odometry";

%Conectamos con el master de ROS
rosinit(ros_master_ip, ros_master_port)

%Suscripcion al tópico
sub = rossubscriber(topics_odometry, @printTopicMessage);

function printTopicMessage(src, msg)
    fprintf("X: %s \n Y: %s \n Z: %s \n \n", msg.Pose.Position.X, msg.Pose.Position.Y, msg.Pose.Position.Z);
%     rosshutdown;
end
