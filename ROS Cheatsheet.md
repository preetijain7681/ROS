# ROS COMMANDS

## Running server:
<!-- Always source before starting -->
    source /opt/ros/melodic/setup.bash
<!-- roscore will start both rosmaster and parametere server -->
    roscore
## ROS Commands for debugging:
<!-- To check active number -->
    rostopic list
    rosparam list
    rosservice list
    rosnode list
<!-- displays sub pub and topic connection -->
    rqt_graph
<!-- displays log -->
    rqt_console
<!-- plots numerical data on ROS topic over time -->
    rqt_plot
<!-- generates a PDF of ROS dependencies -->
    rqt_deps
<!-- Information for debuging -->
    rosnode info <node name>
    rostopic echo <topic name>
    rostopic type <topic name>
    rosservice type /demo_service
    rosservice info /demo_service
<!-- To display content -->
    rosmsg show <msg name>
    rossrv show <srv name>
<!-- rosbash commands -->
    roscd 
<!-- change directory starting with package, stack, or location name -->
    rospd
 <!-- pushd equivalent of roscd -->
    rosd 
<!-- lists directories in the directory-stack -->
    rosls
<!-- list files of a ros package -->
    rosed
 <!-- edit a file in a package -->
    roscp
<!-- copy a file from a package -->
    rosrun 
<!-- run executables of a ros package -->

## File Structure:
<!-- File structure inside ROS package -->
    .
    ├── <workspace>        
        ├── build                       
        ├── devel               
        ├── src
            ├── CMakeLists.txt            
            ├── <package>      
                ├── action     
                ├── msg            
                ├── srv
                ├── src               
                ├── launch 
                ├── urdf
                ├── meshes          
                ├── scripts                
                ├── worlds
                ├── ReadMe.md
                ├── package.xml           
                └── CMakeLists.txt   
        

## Creating ROS package:    
<!-- Create workspace folder along with src in it -->
    mkdir ~/book_ws/src 
    cd ~/book_ws/src 
<!-- Initiate workspace -->
    catkin_init_workspace 
    cd ~/book_ws
<!-- Build after every changes -->
    catkin_make
<!-- Always source the workspace setup -->
    source devel/setup.bash
    cd ~/book_ws/src 
<!-- Create the package mentioning dependencies -->
    catkin_create_pkg demo_pkg roscpp rospy std_msgs actionlib actionlib_msgs
    cd ~/book_ws
<!-- Build after creating package -->
    catkin_make

## Creating ROS nodes:
<!-- Inside package create src and place all the cpp files there, whereas scripts for python files -->
    cd ~/book_ws/src/demo_pkg/src
<!-- create pub file -->
    gedit demo_publisher.cpp
        #include "ros/ros.h"
        #include "std_msgs/Int32.h"
        #include <iostream>
        int main(int argc, char **argv)
        {
            ros::init(argc, argv,"demo_publisher");
            ros::NodeHandle node_obj;
            ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers",800);
            ros::Rate loop_rate(10);
            int number_count =0;
            while (ros::ok())
            {
                std_msgs::Int32 msg;
                msg.data = number_count;
                ROS_INFO("%d",msg.data);
                number_publisher.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
                ++number_count;
            }
            return 0;
        }
<!-- create sub file -->
    gedit demo_subscriber.cpp
        #include "ros/ros.h"
        #include "std_msgs/Int32.h"
        #include <iostream>
        void number_callback(const std_msgs::Int32::ConstPtr& msg)
        {
            ROS_INFO("Received  [%d]",msg->data);
        }
        int main(int argc, char **argv)
        {
            ros::init(argc, argv,"demo_subscriber");
            ros::NodeHandle node_obj;
            ros::Subscriber number_subscriber = node_obj.subscribe("/numbers",10,number_callback);
            ros::Rate loop_rate(10);
            ros::spin();
            return 0;
        }
    cd ~/book_ws/src/demo_pkg 
<!--   edit cmakelist -->
    gedit CMakeLists.txt
    (edit the following lines)
        include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
        )

        add_executable(demo_publisher src/demo_publisher.cpp)
        add_executable(demo_subscriber src/demo_subscriber.cpp)

        target_link_libraries(demo_publisher ${catkin_LIBRARIES})
        target_link_libraries(demo_subscriber ${catkin_LIBRARIES})
    cd ~/book_ws
<!-- build -->
    catkin_make demo_pkg
<!-- run -->
    roscore 
    rosrun demo_pkg demo_publisher
    rosrun demo_pkg demo_subscriber
<!-- ## Checking running nodes:
    rqt_graph
    rosnode list
    rosnode info demo_publisher
    rosnode info demo_subscriber
    rostopic echo /numbers
    rostopic type /numbers -->
## Creating custom msg:
<!-- create a msg folder in package file -->
    cd ~/book_ws/src/demo_pkg/msg
<!-- create msg file -->
    gedit demo_msg.msg
        string greet
        int32 number
    cd ~/book_ws/src/demo_pkg
<!-- edit package file and cmakelist -->
    gedit package.xml
    (uncomment compile time runtime and execution time command for message generation)
        <build_depend>message_generation</build_depend>
        <build_export_depend>message_generation</build_export_depend>
        <exec_depend>message_runtime</exec_depend>
    gedit CMakeLists.txt
    (uncomment or add the following changes)
        find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        roscpp
        rospy
        std_msgs
        message_generation
        )

        ## Generate messages in the 'msg' folder
        add_message_files(
        FILES
        demo_msg.msg
        )

        ## Generate added messages and services with any dependencies listed here
        generate_messages(
        DEPENDENCIES
        actionlib_msgs std_msgs
        )
<!-- build -->
    cd ~/book_ws
    catkin_make
<!-- ## Check msg:
    rosmsg show demo_pkg/demo_msg -->
## Creating custom srv:
<!-- create a srv folder in package file -->
    cd ~/book_ws/src/demo_pkg/srv
<!-- create srv file -->
    gedit demo_srv.srv
        string in
        ---
        string out
<!-- edit package file and cmakelist -->
    cd ~/book_ws/src/demo_pkg
    gedit package.xml
        uncomment compile time runtime and execution time command for message generation
        <build_depend>message_generation</build_depend>
        <build_export_depend>message_generation</build_export_depend>
        <exec_depend>message_runtime</exec_depend>
    gedit CMakeLists.txt
    (uncomment or add the following changes)
        catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs message_runtime 
        )

        find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        roscpp
        rospy
        std_msgs
        message_generation
        )

        ## Generate messages in the 'msg' folder
        add_message_files(
        FILES
        demo_msg.msg
        )

        ## Generate added messages and services with any dependencies listed here
        generate_messages(
        DEPENDENCIES
        actionlib_msgs std_msgs
        )
 <!-- build  -->
    cd ~/book_ws
    catkin_make    
<!-- ## Checking the content
    rossrv show demo_pkg/demo_srv -->
## Running ROS services
<!-- navigate to src folder -->
    cd ~/book_ws/src/demo_pkg/src
<!-- create service server file -->
    gedit demo_service_server.cpp
        #include "ros/ros.h"
        #include "demo_pkg/demo_srv.h"
        #include <iostream>
        #include <sstream>


        //Defining namespace using in this code
        using namespace std;
        //using namespace ros;
        //using namespace std_msgs;
        //using namespace demo_pkg;


        bool demo_service_callback(demo_pkg::demo_srv::Request  &req,
                demo_pkg::demo_srv::Response &res)
        {


        //  ROS_INFO("From Client  [%s], Server says [%s]",req.in.c_str(),ss.c_str());


        std::stringstream ss;
        ss << "Received  Here";
        res.out = ss.str();

        ROS_INFO("From Client  [%s], Server says [%s]",req.in.c_str(),res.out.c_str());

        return true;
<!-- create client file -->
    gedit demo_service_client.cpp
        #include "ros/ros.h"
        #include "std_msgs/Int32.h"
        #include <iostream>

        #include "demo_pkg/demo_srv.h"
        #include <iostream>
        #include <sstream>


        //Defining namespace using in this code
        using namespace std;
        //using namespace ros;
        //using namespace std_msgs;
        //using namespace demo_pkg;


        int main(int argc, char **argv)
        {
        ros::init(argc, argv, "demo_service_client");
        ros::NodeHandle n;
        ros::Rate loop_rate(10);

        ros::ServiceClient client = n.serviceClient<demo_pkg::demo_srv>("demo_service");

            while (ros::ok())
            {


            demo_pkg::demo_srv srv;
            std::stringstream ss;
            ss << "Sending from Here";
            srv.request.in = ss.str();


            if (client.call(srv))
            {

                ROS_INFO("From Client  [%s], Server says [%s]",srv.request.in.c_str(),srv.response.out.c_str());

            }
            else
            {
                ROS_ERROR("Failed to call service");
                return 1;
            }

            ros::spinOnce();
            //Setting the loop rate
            loop_rate.sleep();

            }
        return 0;
        }
<!-- build -->
    cd ~/book_ws
    catkin_make
<!-- run -->
    rosrun demo_pkg demo_service_server
    rosrun demo_pkg demo_service_client

<!-- ## Checking ROS service
    rosservice list
    rosservice type /demo_service
    rosservice info /demo_service -->

## Running actionlib
<!-- create action folder in  -->
    mkdir ~/book_ws/src/demo_pkg/action
    cd ~/action
    gedit demo_action.action
        #goal definition
        int32 count
        ---
        #result definition
        int32 final_count
        ---
        #feedback
        int32 current_number
<!-- create server file -->
    cd ~/book_ws/src/demo_pkg/src
    gedit demo_action_server.cpp
        #include "ros/ros.h"
        #include "std_msgs/Int32.h"
        #include <actionlib/server/simple_action_server.h>
        #include "demo_pkg/Demo_actionAction.h"
        #include <iostream>
        #include <sstream>

        class Demo_actionAction
        {
        protected:
        ros::NodeHandle nh_;
        // NodeHandle instance must be created before this line. Otherwise strange error may occur.
        actionlib::SimpleActionServer<demo_pkg::Demo_actionAction> as; 
        // create messages that are used to published feedback/result
        demo_pkg::Demo_actionFeedback feedback;
        demo_pkg::Demo_actionResult result;

        std::string action_name;
        int goal;
        int progress;

        public:
        Demo_actionAction(std::string name) :
            as(nh_, name, boost::bind(&Demo_actionAction::executeCB, this, _1), false),
            action_name(name)
        {
            as.registerPreemptCallback(boost::bind(&Demo_actionAction::preemptCB, this));
            as.start();
        }

        ~Demo_actionAction(void)
        {
        }

        void preemptCB(){
            ROS_WARN("%s got preempted!", action_name.c_str());
            result.final_count = progress;
            as.setPreempted(result,"I got Preempted"); 

        }
        void executeCB(const demo_pkg::Demo_actionGoalConstPtr &goal)
        {
            if(!as.isActive() || as.isPreemptRequested()) return;
            ros::Rate rate(5);
            ROS_INFO("%s is processing the goal %d", action_name.c_str(), goal->count);
            for(progress = 1 ; progress <= goal->count; progress++){
                //Check for ros
                if(!ros::ok()){
                    result.final_count = progress;
                    as.setAborted(result,"I failed !");
                    ROS_INFO("%s Shutting down",action_name.c_str());
                    break;

                }
            
                if(!as.isActive() || as.isPreemptRequested()){
                    return;
                }	

                if(goal->count <= progress){
                    ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->count);
                    result.final_count = progress;
                    as.setSucceeded(result);

                }else{
                    ROS_INFO("Setting to goal %d / %d",feedback.current_number,goal->count);
                    feedback.current_number = progress;
                    as.publishFeedback(feedback);
                }
                rate.sleep();
            }	
        }
        };

        int main(int argc, char** argv)
        {

        ros::init(argc, argv, "demo_action");
        ROS_INFO("Starting Demo Action Server");
        Demo_actionAction demo_action_obj(ros::this_node::getName());
        ros::spin();
        return 0;
        }
<!-- create client file -->
    gedit demo_action_client.cpp
<!-- edit cmakelist and package file -->
    cd ~/book_ws/src/demo_pkg/src
    gedit CMakeLists.txt
<!-- add the code here -->  
    gedit package.xml
<!-- add the code here -->
    roscore
    rosrun demo_pkg demo_action_server
    rosrun demo_pkg demo_action_client 50 4

## Creating launch files
<!-- create launch file -->
    roscd demo_pkg
    mkdir launch
    cd launch
    gedit demo_topic.launch
<!-- add the code here -->
    roslaunch demo_pkg demo_demo_topic.launch

## Checking
    rosnode list
    rqt_console

## Releasing ROS package
<!-- install bloom tool -->
    sudo apt-get install python-bloom
<!--create repo with package(upstream repo) -->
<!-- create empty repo(release repo) -->
    catkin_generate_changelog
<!-- git add commit push -->
    catkin_prepare_release
<!-- bloom-release --rosdistro <rosdistro> --track <rosdistro> repository_name -->
<!-- fill in necessary info -->
<!-- create pull request  -->

## Creating a Wiki page
<!-- register to wiki -->
<!-- login -->
<!-- enter username link and build ur page -->

---    
## Creating URDF model
<!-- create a package  -->
    catkin_create_pkg robot_description_pkg roscpp tf geometry_msgs urdf rviz xacro 
<!-- create 3 folders: urdf, meshes, launch -->
<!-- navigate to urdf folder  -->
<!-- create urdf file -->
#### Interaction with links and joints
<!-- include joint state publisher node in launch file -->
<!-- in the joint tag include limits -->
<!-- include robot state publisher node in launch file, these helps to publish the state to tf -->
#### Additional properties in URDF
<!-- include tags like color material geometry collision mass interia  -->
## Debugging URDF
    check_urdf <urdf name>
<!-- see the structure of robot links, it generates urdf.gv and urdf.pdf file -->
    urdf_to_graphiz <urdf name>
<!-- it shows urdf.pdf file -->
    evince <urdf name>.pdf
<!-- navigate to launch folder and create a launch file for rviz -->
<!-- launch the file to visualize the urdf in rviz -->
---
## Creating Xacros
<!--  -->
## Conversion of xacro to urdf
    rosrun xacro xacro.py <xacro file> > <urdf file>
## Conversion of urdf to sdf
    gz sdf -p <URDF file> > <sdf file>
---
## Creating differential drive robot
<!-- for one find dat -->
---
## Gazebo Simulation
<!-- create a launch file for gazebo  -->
## Adding sensors in Gazebo
## Using controllers in gazebo
## Simulating a differential wheel drive in gazebo
## Adding ROS teleop node
---
## Generating MoveIt! configuration file with setup
    roslaunch moveit_setup_assistant setup_assistant.launch
<!-- generate self collision matrix -->
<!-- add virtual joints -->
<!-- add planning group of kinematic groups and non kinematic groups -->
<!-- add robot poses -->
<!-- add robot end effector -->
<!-- add passive joints -->
<!-- generate configuration files -->
<!-- to invoke the demo launch fil -->


---
