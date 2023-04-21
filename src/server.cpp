#include "ros/ros.h"
//Header of the service message. 
//	The service message belongs to the ros_service package
#include<tf/transform_listener.h>
#include "classwork4/transformation.h"

using namespace std;

//Callback function
//	Return value: boolean
//		If this function returns true, the service function has been corretly called
//		You can use this value to check if the function has been called with correct parameters
//		i.e. call a service calculating the square of a number, calling the service with a negative number
//	Input values:  the request part of the service 
//				   the output of the service to fill
bool service_callback( classwork4::transformation::Request &req, classwork4::transformation::Response &res) {


    tf::TransformListener listener;
        tf::StampedTransform transform;
            try {
                listener.waitForTransform(req.frame_a.data, req.frame_b.data, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(req.frame_a.data, req.frame_b.data, ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }


//DEVO PROVARE A CONVERTIRE DIVERSAMENTE|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
            
            //tf::pointTFToMsg(transform.getOrigin(), res.pose.position);
            //tf::quaternionTFToMsg(transform.getRotation(), res.pose.orientation);

            
            res.pose.position.x=transform.getOrigin().x();
            res.pose.position.y=transform.getOrigin().y();
            res.pose.position.z=transform.getOrigin().z();

            res.pose.orientation.x= transform.getRotation().x();
            res.pose.orientation.y= transform.getRotation().y();
            res.pose.orientation.z= transform.getRotation().z();
            res.pose.orientation.w=transform.getRotation().w();

            ROS_INFO_STREAM(" Transform: Position: " << 
        
                res.pose.position.x << ", " << 
                res.pose.position.y << ", " <<
                res.pose.position.z << ", \tOrientation(Quaternion):" << 
                res.pose.orientation.x << ", " << 
                res.pose.orientation.y << ", " << 
                res.pose.orientation.z << ", "<<
                res.pose.orientation.w
            );
            return true;

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "transformation_service");
	ros::NodeHandle node;

	//Initialize the service object: name of the service and callback function
	//	Like subscribers, also tje callback function can be declared as a class function
	ros::ServiceServer service = node.advertiseService("transformation", service_callback);

	//Call the spin function to maintain the node alive

	ros::spin();

	return 0;
}







