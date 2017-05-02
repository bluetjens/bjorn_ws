
/* perception_neuron_tf_broadcaster.cpp
 *
 * Copyright (C) 2015 Alexander Rietzler, Simon Haller
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

// access to global callback_queue needed for own spinner
#include <ros/callback_queue.h>

// Frequency measurement
ros::Time lastReceiveTime;
int counter;

class NeuronBroadcaster
{
public:
    NeuronBroadcaster(ros::NodeHandle & nh)
        :nh_(nh)
    {        

         // Incoming child data sequence is determined by perception_neuron DataReader APK
         // OverheadElement is not included in the documentation and is number 59 concerning "Sequence in Data Block" = 60th element
         link_children_names_=std::vector<std::string>{"Hips","RightUpLeg","RightLeg","RightFoot",
                                                     "LeftUpLeg","LeftLeg","LeftFoot","Spine","Spine1","Spine2","Spine3","Neck","Head",
                                                     "RightShoulder","RightArm","RightForeArm","RightHand",
                                                     "RightHandThumb1","RightHandThumb2","RightHandThumb3",
                                                     "RightInHandIndex","RightHandIndex1","RightHandIndex2","RightHandIndex3",
                                                     "RightInHandMiddle","RightHandMiddle1","RightHandMiddle2","RightHandMiddle3",
                                                     "RightInHandRing","RightHandRing1","RightHandRing2","RightHandRing3",
                                                     "RightInHandPinky","RightHandPinky1","RightHandPinky2","RightHandPinky3",
                                                     "LeftShoulder","LeftArm","LeftForeArm","LeftHand",
                                                     "LeftHandThumb1","LeftHandThumb2","LeftHandThumb3",
                                                     "LeftInHandIndex","LeftHandIndex1","LeftHandIndex2","LeftHandIndex3",
                                                     "LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2","LeftHandMiddle3",
                                                     "LeftInHandRing","LeftHandRing1","LeftHandRing2","LeftHandRing3",
                                                     "LeftInHandPinky","LeftHandPinky1","LeftHandPinky2","LeftHandPinky3", "OverheadElement"
                                                     };

        link_parents_names_=std::vector<std::string>{"WorldPerceptionNeuron","Hips","RightUpLeg","RightLeg",
                                                     "Hips","LeftUpLeg","LeftLeg","Hips","Spine","Spine1","Spine2","Spine3","Neck",
                                                     "Spine3","RightShoulder","RightArm","RightForeArm",
                                                     "RightHand","RightHandThumb1","RightHandThumb2",
                                                     "RightHand","RightInHandIndex","RightHandIndex1","RightHandIndex2",
                                                     "RightHand","RightInHandMiddle","RightHandMiddle1","RightHandMiddle2",
                                                     "RightHand","RightInHandRing","RightHandRing1","RightHandRing2",
                                                     "RightHand","RightInHandPinky","RightHandPinky1","RightHandPinky2",
                                                     "Spine3","LeftShoulder","LeftArm","LeftForeArm",
                                                     "LeftHand","LeftHandThumb1","LeftHandThumb2",
                                                     "LeftHand","LeftInHandIndex","LeftHandIndex1","LeftHandIndex2",
                                                     "LeftHand","LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2",
                                                     "LeftHand","LeftInHandRing","LeftHandRing1","LeftHandRing2",
                                                     "LeftHand","LeftInHandPinky","LeftHandPinky1","LeftHandPinky2", "WorldPerceptionNeuron"
                                                    };

        std::vector<std::string> topic_names={"/perception_neuron/data_1",
                                              "/perception_neuron/data_2",
                                              "/perception_neuron/data_3"};

        // own callback queues:
        //own callbackQueues
        // TODO convert to array:
        // 3 = number of data topics
        ros::CallbackQueue cbqueue[3];

        //so müsste alles auf cbqeue[0] landen.
        ros::SubscribeOptions subOpt1;
        for(int i=0; i < subscribers_.size(); i++){
            subOpt1.init(topic_names.at(i), 6, boost::bind(&NeuronBroadcaster::bvhtotf,this, _1,i));
        }
        subOpt1.callbackQueue = &cbqueue[0];

        // 3 = number of topics
        subscribers_.resize(3);

        //list topic_names is exactly 3 big, s.t. it runs from 0 to 2
        //boost::bind lets us give i as a variable to call function callback_i
        for(int i=0; i < subscribers_.size(); i++){
            subscribers_.at(i)=nh_.subscribe<std_msgs::Float64MultiArray>(subOpt1);
        }
        /*
        // old subscribers:
        // 3 = number of topics
        subscribers_.resize(3);

        //list topic_names is exactly 3 big, s.t. it runs from 0 to 2
        //boost::bind lets us give i as a variable to call function callback_i
        for(int i=0; i < subscribers_.size(); i++){
            subscribers_.at(i)=nh_.subscribe<std_msgs::Float64MultiArray>(topic_names.at(i), 5, boost::bind(&NeuronBroadcaster::callback_i,this, _1,i));
        }
        */

    }

    void sendStaticTransform(){
        //ROS_INFO_STREAM("inside sendStaticTransform");

        //Sending Static transformation to ROS World
        tf::Transform world_frame;
        world_frame.setOrigin(tf::Vector3(0,0,0));
        world_frame.setRotation(tf::Quaternion(0.70711,0,0,0.70711));
        tf_broadcaster_.sendTransform(tf::StampedTransform(world_frame, ros::Time::now(), "world", "WorldPerceptionNeuron"));
    }

    ~NeuronBroadcaster(){};

private:

     ros::NodeHandle nh_;
     std::vector<ros::Subscriber> subscribers_;
     std::vector<std::string> link_children_names_, link_parents_names_;
     tf::TransformBroadcaster tf_broadcaster_;



    // converts rotation given in degrees to quaternion data
    void eulerToQuaternion(float eulerY, float eulerX, float eulerZ, tf::Quaternion & q){

        Eigen::Matrix3f rxyz,rx,ry,rz;

        rx=Eigen::AngleAxisf(eulerX*M_PI/180, Eigen::Vector3f::UnitX());
        ry=Eigen::AngleAxisf(eulerY*M_PI/180, Eigen::Vector3f::UnitY());
        rz=Eigen::AngleAxisf(eulerZ*M_PI/180, Eigen::Vector3f::UnitZ());

        //Check Ordering in Axis Neuron! Here = YXZ
        rxyz =  ry*rx*rz;

        Eigen::Quaternionf qf(rxyz);

        q.setW(qf.w());
        q.setX(qf.x());
        q.setY(qf.y());
        q.setZ(qf.z());
    }

/*
    void callback_i(const std_msgs::Float64MultiArrayConstPtr & bone_data, int i){
        // 0 <= i <= 2
        cbqueue[i].addCallback(boost::bind(&NeuronBroadcaster::bvhtotf,bone_data, _1,i),0);


    }
    */
    // i gives reference to the topic names: data_1, data_2 or data_3
        //theses are seperated, because of three different data streames. The message is simly too long to put it into one message
    // j gives the indices into the bone_data array to the position and rotation data.
    //startInd gives reference to the start of bone_data array position regarding the current limb
    //link_index is indicating which parent belongs to which child. Link_index gives the parent number.:
        //link_index = 0 : WorldPerceptionNeuron
        //link_index = 1 : HipsPosition
        //link_index = 2 : Hips
        //...
    void bvhtotf(const std_msgs::Float64MultiArrayConstPtr & bone_data, int i){


        uint startIdx=0;
        uint link_index=0;

        for(uint j=0; j < bone_data->data.size()/6; j++){

            startIdx=j*6;

            tf::Transform pose;
            float eulerY,eulerX,eulerZ;
            tf::Quaternion rotation;
            tf::Vector3 position;

            position.setX(0.01*bone_data->data[startIdx]); //conversion to meters
            position.setY(0.01*bone_data->data[startIdx+1]);
            position.setZ(0.01*bone_data->data[startIdx+2]);

            eulerY=bone_data->data[startIdx+3];
            eulerX=bone_data->data[startIdx+4];
            eulerZ=bone_data->data[startIdx+5];

            eulerToQuaternion(eulerY,eulerX,eulerZ,rotation);

            pose.setOrigin(position);
            pose.setRotation(rotation);

            link_index=j + i*20;

            // set Hips onto World Perception Neuron and rotate by PI/2 rad around y axis of WorldPerceptionNeuron to be equal to have the correct position of ur5 arm in correlation to body
            if(i == 0 && j == 0){
                pose.setOrigin(tf::Vector3(0, 0, 0));
                //eulerToQuaternion(0, 0, 0 , rotation);
                rotation.setRPY(0, 3*M_PI/2, 0);
                pose.setRotation( rotation );
            }

            //print out perception_neuron/data_1
            //j= 16 for RightHand , j = 0 for Hips
            if(i == 0 && j == 16){
                // print out incoming data
                //ROS_INFO_STREAM("indices i: " << i << " j: "  << j << "link_index: " << link_index);
                // ROS_INFO_STREAM("parent node: " << link_parents_names_.at(link_index) << " child node: " << link_children_names_.at(link_index));
                // ROS_INFO_STREAM("Broadcasting transform: Rotation: Xr: " << eulerX << " Yr: " << eulerY << " Zr " << eulerZ << " Rotation: " << *rotation);
                // ROS_INFO_STREAM("Broadcasting transform: Transformation, : X: " << 0.01*bone_data->data[startIdx] << " Y: " << 0.01*bone_data->data[startIdx+1] << " Z " << 0.01*bone_data->data[startIdx+2]);

                // print out tf data
                 ROS_INFO_STREAM("Incoming transform: getOrigin: y: " << pose.getOrigin().y() << " x: " << pose.getOrigin().x() << " z: " << pose.getOrigin().z() << "rotation x: " << pose.getRotation().getAxis().x() << " y: " << pose.getRotation().getAxis().y() << " z: " << pose.getRotation().getAxis().z());
                 ROS_INFO_STREAM("Incoming transform: getRotation: qy: " << pose.getRotation().getAxis().y() << " qx: " << pose.getRotation().getAxis().x() << " qz: " << pose.getRotation().getAxis().z() << " qw: " << pose.getRotation().getW());
                // ROS_INFO_STREAM("Incoming transform: Quaternion Angle in rad: " << pose.getRotation().getAngle());

            }

            tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), link_parents_names_.at(link_index), link_children_names_.at(link_index)));

        }
        // TODO: check for i==0 again
        //if( i == 0 ){  // first topic
        if(1){  // first topic
            ROS_INFO_STREAM("i: " << i << "frameNumber: " << counter << ", Time to last tf broadcast: " << ros::Time::now().toSec() - lastReceiveTime.toSec() << "s");
            lastReceiveTime = ros::Time::now();
            counter++;
        }
    }

};

int main(int argc, char** argv){
    ROS_INFO_STREAM("front of main");
    // TODO
    ROS_WARN("ros_warn.");
    ROS_INFO_STREAM("ros_info_stream.");
    ROS_ERROR_STREAM("ros_error_stream." );
    ROS_INFO("ros_info.");


    ros::init( argc, argv, "perception_neuron_tf_broadcaster_node", ros::init_options::AnonymousName );

    // TODO: evaluate if I need this spinner:
    //ros::AsyncSpinner spinner(3);
    //spinner.start();

    ros::NodeHandle nh;

    // Frequency measurement
    lastReceiveTime = ros::Time::now();
    counter = 0;

    NeuronBroadcaster neuronBroadcaster(nh);

    // OWN SPINNER:
    ros::Time frameTime = ros::Time::now();
    ros::Rate rate(60);
    ros::CallbackQueue::CallOneResult callOneResult;
    ros::CallbackQueue::CallOneResult callOneResult2;
    ros::CallbackQueue::CallOneResult callOneResult3;
    while(nh.ok()){
        // check how full the Global callback queue is each 16.7ms
        // while(ros::getGlobalCallbackQueue()->empty() != 1)

        ros::getGlobalCallbackQueue() -> callAvailable();


        //callOneResult = ros::getGlobalCallbackQueue() -> callOne();


        //callOneResult = ros::getGlobalCallbackQueue() -> callOne();
        //callOneResult2 = ros::getGlobalCallbackQueue() -> callOne();
        //callOneResult3 = ros::getGlobalCallbackQueue() -> callOne();
        //ros::getGlobalCallbackQueue() -> clear();


        /*int l = 0;
        while( !ros::getGlobalCallbackQueue() -> isEmpty() ){
            ros::getGlobalCallbackQueue() -> callOne();
            l++;
        }
        ROS_INFO_STREAM(l << " callbacks have been called");
        */
        neuronBroadcaster.sendStaticTransform();
        bool rateCorrect = rate.sleep();
        //ros::Duration dur(0.016);
        //dur.sleep();

        ROS_INFO_STREAM("callOneResult: " << callOneResult << callOneResult2 << callOneResult3 << " frameTime: " << ros::Time::now().toSec() - frameTime.toSec() << " cycleTime: " << rate.cycleTime());
        frameTime = ros::Time::now();

        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }

    // TODO: evaluate if I need this spinner:
    //ros::Rate rate(125);
    //while (nh.ok()){
    //    neuronBroadcaster.sendStaticTransform();
    //    rate.sleep();
    //}

    ROS_INFO_STREAM("This is a utility to publish perception neuron bone data to tf");
    ROS_INFO_STREAM("Type any key when you are done");
    std::string mode;
    std::cin >> mode;

    ROS_INFO_STREAM("Bye!");
    return 0;
}
