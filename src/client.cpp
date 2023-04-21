#include "ros/ros.h"
#include <tf/transform_broadcaster.h>


//Header of the service message. 
//	The service message belongs to the classwork4 package
#include "classwork4/transformation.h"

using namespace std;

int main(int argc, char **argv) {

	//Init the ROS node with service_client name
	ros::init(argc, argv, "transformation_client");
	ros::NodeHandle n;

	//Init the service client. Data to use for the service (the .srv file) and the name of the service
	ros::ServiceClient client = n.serviceClient<classwork4::transformation>("transformation");
    
	string frame_a;
    string frame_b;
    
    if (argc != 3) {
        ROS_ERROR("need two reference frame names as argument"); 
        return -1;
    }
    frame_a=argv[1];
    frame_b=argv[2];
    
    classwork4::transformation frame_transformation;
    frame_transformation.request.frame_a.data = frame_a;
	frame_transformation.request.frame_b.data = frame_b;

	
	//Wait that in the ROS network, the service sum is advertised
	//	If you call a service and the service has not been advertised, you will have back an error
	ROS_INFO("Waiting for the client server");
	client.waitForExistence();
	ROS_INFO("Client server up now");
	
	
	tf::TransformBroadcaster br;
    tf::Transform transform;
	ros::Rate rate(100.0);

	while (ros::ok()){
	//Call the service callback
	//	The return value is false if:
	//		- the callback returns false
	//		- the service has not been found in the ROS network
	if (!client.call(frame_transformation)) {
		ROS_ERROR("Error calling the service");
		return 1;
	}
	

	tf::poseMsgToTF(frame_transformation.response.pose,transform);
	//transform( tf::Vector3(frame_transformation.response.pose.position.x, frame_transformation.response.pose.position.y, frame_transformation.response.pose.position.z) );
	//transform.setRotation(tf::Quaternion(frame_transformation.response.pose.orientation.x, frame_transformation.response.pose.orientation.y, frame_transformation.response.pose.orientation.z, frame_transformation.response.pose.orientation.w));
	
	
	
	
	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),frame_a,frame_b));

	
	//Just print the output
    ROS_INFO_STREAM(" Transform: Position:" << 
    
        transform.getOrigin().getX() << ", " << 
        transform.getOrigin().getY() << ", " <<
		transform.getOrigin().getZ() << ", " << ", \tOrientation(Quaternion):" << 
        transform.getRotation().getX() << ", " <<
		transform.getRotation().getY()<< ", " <<
		transform.getRotation().getZ() << ", " <<
		transform.getRotation().getW()<<"\n";
    );

		rate.sleep();
	}
	
	return 0;
}
