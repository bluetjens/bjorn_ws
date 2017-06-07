//TODO:
// - unnötige Kommentare entfernen
// - evaluate how to spin: if i should use async spinner of callAvailable or callOnce
// - alles mit link_index auskommentieren
// - rename j to link_index or joint_index
// - eliminate global variables

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
// changed datatype to float32MultiArray, as perception neuron data has maximum precision of float32
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <chrono> //high precision event timer

// access to global callback_queue needed for own spinner
#include <ros/callback_queue.h>

// Frequency measurement
ros::Time lastReceiveTime;
int counter;
int oldid = 0;

int testCallback(const std_msgs::Float32MultiArrayConstPtr & bone_data){
    // ROS_INFO_STREAM("internal frameNumber: " << counter << ", Time to last tf broadcast: " << (ros::Time::now().toSec() - lastReceiveTime.toSec())*1000 << "ms");
    std::string::size_type sz;
    int id = std::stoi(bone_data->layout.dim[0].label, &sz);
    if( id - oldid > 1){ // skipped a message
        ROS_INFO_STREAM("LOST A MESSAGE/n/n/n");
        //usleep(1000);
    }
    oldid = id;
    ROS_INFO_STREAM("FrameID:" << id << " Time to last tf broadcast: " << (ros::Time::now().toSec() - lastReceiveTime.toSec())*1000 << "ms");
    lastReceiveTime = ros::Time::now();
    //counter++;
}
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
        std::string topic_name = "/perception_neuron/data";
        subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>(topic_name, 5, boost::bind(&NeuronBroadcaster::callback_i,this, _1));

        /*
        std::vector<std::string> topic_names={"/perception_neuron/data_1",
                                              "/perception_neuron/data_2",
                                              "/perception_neuron/data_3"};

        // 3 = number of topics
        subscribers_.resize(3);

        //list topic_names is exactly 3 big, s.t. it runs from 0 to 2
        //boost::bind lets us give i as a variable to call function callback_i
        // lutjens: added queue_size from 5 to 9 = 3* topics.
        for(int i=0; i < subscribers_.size(); i++){
            subscribers_.at(i)=nh_.subscribe<std_msgs::Float32MultiArray>(topic_names.at(i), 9, boost::bind(&NeuronBroadcaster::callback_i,this, _1,i));
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
     ros::Subscriber subscriber_;
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

    // i gives reference to the topic names: data_1, data_2 or data_3
        //theses are seperated, because of three different data streames. The message is simly too long to put it into one message
    // j gives the indices into the bone_data array to the position and rotation data.
    //startInd gives reference to the start of bone_data array position regarding the current limb
    //link_index is indicating which parent belongs to which child. Link_index gives the parent number.:
        //link_index = 0 : WorldPerceptionNeuron
        //link_index = 1 : HipsPosition
        //link_index = 2 : Hips
        //...
    void callback_i(const std_msgs::Float32MultiArrayConstPtr & bone_data){


        uint startIdx=0;
        // uint link_index=0;
        // bone_data->data.size() = (59(bones)+1(overhead)) * 6(3rot+3trans) = 360
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

            // Changed to deal with only one message
            // link_index=j;// + i*20;

            // set Hips onto World Perception Neuron and rotate by PI/2 rad around y axis of WorldPerceptionNeuron to be equal to have the correct position of ur5 arm in correlation to body
            //if(i == 0 && j == 0){
            if(j == 0){
                pose.setOrigin(tf::Vector3(0, 0, 0));
                //eulerToQuaternion(0, 0, 0 , rotation);
                rotation.setRPY(0, 3*M_PI/2, 0);
                pose.setRotation( rotation );
            }

            //print out perception_neuron/data_1
            //j= 16 for RightHand , j = 0 for Hips
            //if(i == 0 && j == 16){

            //if(j == 16){
                // print out incoming data
                //ROS_INFO_STREAM("indices i: " << i << " j: "  << j << "link_index: " << link_index);
                // ROS_INFO_STREAM("parent node: " << link_parents_names_.at(link_index) << " child node: " << link_children_names_.at(link_index));
                //ROS_INFO_STREAM("Received bvh data: Rotation: Xr: " << eulerX << " Yr: " << eulerY << " Zr " << eulerZ << " Rotation: " << *rotation);
                // ROS_INFO_STREAM("Received bvh data: Transformation, : X: " << 0.01*bone_data->data[startIdx] << " Y: " << 0.01*bone_data->data[startIdx+1] << " Z " << 0.01*bone_data->data[startIdx+2]);

                // print out tf data
                // ROS_INFO_STREAM("Outgoing tf transform: getOrigin: y: " << pose.getOrigin().y() << " x: " << pose.getOrigin().x() << " z: " << pose.getOrigin().z() << "rotation x: " << pose.getRotation().getAxis().x() << " y: " << pose.getRotation().getAxis().y() << " z: " << pose.getRotation().getAxis().z());
                // ROS_INFO_STREAM("Outgoing tf transform: getRotation: qy: " << pose.getRotation().getAxis().y() << " qx: " << pose.getRotation().getAxis().x() << " qz: " << pose.getRotation().getAxis().z() << " qw: " << pose.getRotation().getW());
                // ROS_INFO_STREAM("Incoming transform: Quaternion Angle in rad: " << pose.getRotation().getAngle());
            //}

            // sizeof stampedtransform is 152 byte
            // ROS_INFO_STREAM("size of tf transform: " << sizeof(tf::StampedTransform(pose, ros::Time::now(), link_parents_names_.at(j), link_children_names_.at(j))));
            tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), link_parents_names_.at(j), link_children_names_.at(j)));

        }
        // TODO: check for i==0 again
        //if( i == 0 ){  // first topic
        if(1){  // first topic
            ROS_INFO_STREAM("percNeuron frameNumber: " << bone_data->layout.dim[0].label << "internatl frameNumber: " << counter << ", Time to last tf broadcast: " << ros::Time::now().toSec() - lastReceiveTime.toSec() << "s");
            lastReceiveTime = ros::Time::now();
            counter++;

            std::string::size_type sz;
            int id = std::stoi(bone_data->layout.dim[0].label, &sz);
            if( id - oldid > 1){ // skipped a message
                ROS_INFO_STREAM("LOST A MESSAGE/n/n/n");
                //usleep(1000);
            }
            oldid = id;

        }
    }

};

int main(int argc, char** argv){
    ROS_INFO_STREAM("started perc_neuron_tf_broadcaster");

    ros::init( argc, argv, "perception_neuron_tf_broadcaster_node", ros::init_options::AnonymousName );

    // TODO: evaluate if I need this spinner:
    // ros::AsyncSpinner spinner(3); 3 Spinner unter Simon Haller
    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    ros::NodeHandle nh;

    // Frequency measurement
    lastReceiveTime = ros::Time::now();
    counter = 0;

    /*
    // test subscriber>
    // ^^^^^^^^^^^^^^^^^^^^
    std::string test_topic = "/test_data";
    ros::Subscriber subscriber;
    //subscriber = nh.subscribe<std_msgs::Float32MultiArray>(test_topic, 5, boost::bind(testCallback));
    subscriber = nh.subscribe<std_msgs::Float32MultiArray>(test_topic, 5, &testCallback);

    // high precision timer
    std::chrono::high_resolution_clock::time_point testLastPublishTime = std::chrono::high_resolution_clock::now();
    double testFrameRate = 60;
    double testPublishInterval = 1/testFrameRate; // in seconds

    double testVarianceOfStdSleep = 0.002; // variance of ros::Duration::sleep() function in seconds

    while(nh.ok()){
        ros::getGlobalCallbackQueue() -> callOne();
        // testRate.sleep();

        // establish high precision timer
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        //std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        std::chrono::duration<double> elapsedTimeSincePublish = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - testLastPublishTime);
        std::chrono::duration<double> callbackLoopTime = elapsedTimeSincePublish; //ROS_INFO_STREAM("elapsedTimeSincePublish in ms before Sleep: " << 1000*elapsedTimeSincePublish.count());
        std::chrono::duration<double> timeAfterStdSleep;
        // 2ms variance of ros::Duration::sleep function
        if( testPublishInterval - elapsedTimeSincePublish.count() - testVarianceOfStdSleep > 0){
            ros::Duration dur(testPublishInterval - elapsedTimeSincePublish.count()- testVarianceOfStdSleep); // sleep 15ms
            dur.sleep();
            //usleep(15000);
            elapsedTimeSincePublish = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - testLastPublishTime);
            timeAfterStdSleep = elapsedTimeSincePublish;
        }
        // spin until 17ms is reached
        while(elapsedTimeSincePublish.count() < testPublishInterval){
            elapsedTimeSincePublish = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - testLastPublishTime);
        }
        ROS_INFO_STREAM("Total time since publish: " << 1000*elapsedTimeSincePublish.count() << " Time for callback: " << callbackLoopTime.count()*1000 << " after StdSleep: " << timeAfterStdSleep.count()*1000);
        testLastPublishTime = std::chrono::high_resolution_clock::now();

        //ros::Duration dur(0.016);
        //dur.sleep();

        //ROS_INFO_STREAM("cumulative time since last publish in ms: " << (ros::Time::now().toSec() - testframeTime.toSec())*1000 << " cycleTime: " << testRate.cycleTime());
        //    testframeTime = ros::Time::now();
        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }
    */


    NeuronBroadcaster neuronBroadcaster(nh);

    // high precision timer
    // ^^^^^^^^^^^^^^^^^^^^
    std::chrono::high_resolution_clock::time_point lastPublishTime = std::chrono::high_resolution_clock::now();
    double frameRate = 60;
    double publishInterval = 1/frameRate; // in seconds

    double varianceOfStdSleep = 0.002; // variance of ros::Duration::sleep() function in seconds


    // OWN SPINNER:
    ros::Time frameTime = ros::Time::now();
    // TODO: RATE zuruck auf 60 setzen
    ros::Rate rate(120);
    ros::CallbackQueue::CallOneResult callOneResult;
    while(nh.ok()){
        // check how full the Global callback queue is each 16.7ms
        // while(ros::getGlobalCallbackQueue()->empty() != 1)

        //ros::getGlobalCallbackQueue() -> callAvailable();

        callOneResult = ros::getGlobalCallbackQueue() -> callOne();
        //ros::getGlobalCallbackQueue() -> clear();

        /*
        int l = 0;
        while( !ros::getGlobalCallbackQueue() -> isEmpty() ){
            ros::getGlobalCallbackQueue() -> callOne();
            l++;
            rate.sleep();
        }
        ROS_INFO_STREAM(l << " callbacks have been called");
        */



        neuronBroadcaster.sendStaticTransform();
        // rate.sleep();

        // establish high precision timer
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        std::chrono::duration<double> elapsedTimeSincePublish = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - lastPublishTime);
        std::chrono::duration<double> callbackLoopTime = elapsedTimeSincePublish; //ROS_INFO_STREAM("elapsedTimeSincePublish in ms before Sleep: " << 1000*elapsedTimeSincePublish.count());
        std::chrono::duration<double> timeAfterStdSleep;
        // 2ms variance of ros::Duration::sleep function
        if( publishInterval - elapsedTimeSincePublish.count() - varianceOfStdSleep > 0){
            ros::Duration dur(publishInterval - elapsedTimeSincePublish.count()- varianceOfStdSleep); // equivalate time s.t. varianceOfStdSleep time is left over for high precision timer
            dur.sleep();
            //usleep(15000);
            elapsedTimeSincePublish = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - lastPublishTime);
            timeAfterStdSleep = elapsedTimeSincePublish;
        }
        // spin until 17ms is reached
        while(elapsedTimeSincePublish.count() < publishInterval){
            elapsedTimeSincePublish = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - lastPublishTime);
        }
        ROS_INFO_STREAM("Total time since publish: " << 1000*elapsedTimeSincePublish.count() << " Time for callback: " << callbackLoopTime.count()*1000 << " after StdSleep: " << timeAfterStdSleep.count()*1000);
        lastPublishTime = std::chrono::high_resolution_clock::now();

        ROS_INFO_STREAM("callOneResult: " << callOneResult << "cumulative time since last publish in ms: " << (ros::Time::now().toSec() - frameTime.toSec())*1000 << " cycleTime: " << rate.cycleTime());
        if (callOneResult==0){ // frame published
            frameTime = ros::Time::now();
        }
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
