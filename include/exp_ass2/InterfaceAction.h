#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"

namespace KCL_rosplan {
	class MoveInterface: public RPActionInterface
	{
	private:
	public:
		/* constructor */
		MoveInterface(ros::NodeHandle &nh);
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};

	class CheckHypInterface: public RPActionInterface
	{
	private:
	public:
		/* constructor */
		CheckHypInterface(ros::NodeHandle &nh);
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
	
		class HypReadyInterface: public RPActionInterface
	{
	private:
	public:
		/* constructor */
		HypReadyInterface(ros::NodeHandle &nh);
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
	
		class MoveToHintInterface: public RPActionInterface
	{
	private:
	public:
		/* constructor */
		MoveToHintInterface(ros::NodeHandle &nh);
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
	
		class MoveToOracleInterface: public RPActionInterface
	{
	private:
	public:
		/* constructor */
		MoveToOracleInterface(ros::NodeHandle &nh);
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
	
			class TakeHintInterface: public RPActionInterface
	{
	private:
	public:
		/* constructor */
		TakeHintInterface(ros::NodeHandle &nh);
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};

}

