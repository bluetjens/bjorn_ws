/* perception_neuron_tf_broadcaster.cpp
*
* Takes the position of the Perception Neuron hand and lets the UR10 end effector follow it.

* Takes the TF data, which was produced by the Perception Neuron suit and sent to tf by 
* perc_neuron_tf_broadcaster. Calculates continually the goal position of the UR10, checks it to 
* be inside joint restrictions on angle and velocity and publishes them to the UR10 controller.
*
* Copyright (C) 2016 Bjorn Lutjens (bjoern.luetjens@gmail.com), Emmanuel Dean
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD license.  See the LICENSE file for details.
*/

//TODO:
// do i have more getParam() which do not include /follow_hand/ prefix
// Evaluate to either use only right hand or both hands
// - adapt to use private and public member variables
// - initialize struct variables in constructer; e.g. bodyJointsRightHand, bodyJointsLeftHand
//  - dann nur noch schreiben
// - schauen ob lastPosition -> lastPubPosition keinen error verursacht hat.
// - Ref Variablen einheitlich benennen
// - rename jointPubRate to jointPubInterval
// - disable interpolate = 1 variable in checkAndPublishDesJointPos
// brei
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include <math.h>
#include <stdio.h>

#include <ctime>  // runtime measurement

#include <qt4/Qt/qvector.h>

#include <tumtools/Math/MathTools.h>  // Calculation of robot trajectory during initialization
// TODO: GESCHEIDEN PFAD EINBINDEN
//#include <../../follow_hand/include/tumtools/include/tumtools/Math/MathTools.h>  // Calculation of robot trajectory during initialization

#include <ur_kinematics/ur_kin.h>

using namespace std;


// Looks up a tf transfrom and pushes them into an array for calculating
// the goal transform
// Takes the tf from transOrigin to transDestination in tfListener and
// pushes them in transArray
// Returns 0 for success, -1 for failure
int pushIntoTransformArray(std::string transOrigin,
                           std::string transDestination,
                           std::vector<tf::Transform>* transArray,
                           tf::TransformListener* tfListener) {
  tf::StampedTransform transform;
  try {
    tfListener->lookupTransform(transOrigin, transDestination, ros::Time(0),
                                transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return -1;
  }
  transArray->push_back(
      tf::Transform(transform.getRotation(), transform.getOrigin()));
  return 0;
}

struct robotRestrictions {
  // Maximum joint velocity in rad per sec
  double maxVel;
  bool checkElbowRestriction;
  // angle between base link and 3rd joint=
  // minimum inner angle of elbow of robot is in interval [0, M_PI]
  double innerElbowMin;
  double innerElbowMax;
};

// Interface to publish messages to robot controller
struct pubIntf {
  bool publishToController;  // True: node publishes to robot controller, False:
                             // robot publishes directly to joint states
  ros::Publisher jointPosPublisher;  // Publishes desired joint values
  sensor_msgs::JointState
      jointMsg;       // Published message with desired joint values
  double jointPubRate;  // Rate at which joint values are published
};

// Interface to tf Data, which is published by Perception Neuron TF Broadcaster
struct tfPercNeuronInterface {
  bool followLeftHand = 0; //default program follows RightHand
  std::vector<std::string> bodyJointsRightHand;  // Array with tf names of Perception
                                        // Neuron joints. Necessary to listen to
                                        // them with tfListener
  std::vector<std::string> bodyJointsLeftHand;
  tf::TransformListener
      tfListener;  // tf listener to data from external tf broadcaster
  tf::Transform transBaseLinkRightHand;  // goal Transform for IK calculations,
                                         // robot base_link to RightHand
                                         // dpushIntoTransformArrayirectly
  tf::TransformBroadcaster
      tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

  // Get tf Transform / Vector from base_link of ur# to RightHand of PercNeuron
  // The UR# End Effector follows this transform
  // returns 0 for success, -1 for failure
  int getTransBaseLinkRightHand() {
    tf::Transform
        transHipsRightHand;  // transform from Hips to RightHand directly
    std::vector<tf::Transform> transformArray;
    int success =
        0;  // = 0 for success of getTransBaseLinkRightHand; < 0 for failure

    // Get Transform from Hips to RightHand of PercNeuron
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // fill transformArray with transforms from "Hips" to "RightHand"; i=0 would
    // equal Hips
    if(followLeftHand == 0){
        for (int i = 0; i < bodyJointsRightHand.size() - 1; i++) {
          success += pushIntoTransformArray(
              bodyJointsRightHand.at(i), bodyJointsRightHand.at(i + 1), &transformArray,
              &tfListener);  // returns 0 for success -1 for failure
        }
    }
    else{
        for (int i = 0; i < bodyJointsLeftHand.size() - 1; i++) {
          success += pushIntoTransformArray(
              bodyJointsLeftHand.at(i), bodyJointsLeftHand.at(i + 1), &transformArray,
              &tfListener);  // returns 0 for success -1 for failure
        }
    }
    if (success != 0) {
      printf("\nNo tf transformation found from Hips to Righthand.");
      printf(
          "\nPlease start rosserial_server node, or Perception Neuron "
          "publisher on Windows.");
      return -1;
    }
    // Calculate direct vector from Hips to RightHand
    transHipsRightHand = transformArray.at(0);
    for(int i = 1; i < transformArray.size(); i++){
        transHipsRightHand = transHipsRightHand.operator*=(transformArray.at(i));
    }
    transformArray.clear();
    // tfBroadcaster.sendTransform(tf::StampedTransform(
    //    transHipsRightHand, ros::Time::now(), "/Hips", "/hipsToHand"));


    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success += pushIntoTransformArray("/base_link", "/world", &transformArray,
                                      &tfListener);
    success += pushIntoTransformArray("/world", "/WorldPerceptionNeuron",
                                      &transformArray, &tfListener);
    success += pushIntoTransformArray("/WorldPerceptionNeuron", "/Hips",
                                      &transformArray, &tfListener);
    /*  // Uncomment if tf /hipsToHand is published externally
        success += pushIntoTransformArray("/Hips", "/hipsToHand",
            &transformArray, &tfListener);
        transBaseLinkRightHand =
            transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(
                transformArray.at(2).operator*=(transformArray.at(3))));
    */
    if (success != 0) {
      printf("\nNo tf Transformation found from base_link to RightHand.");
      printf(
          "\nPlease assure that tf robot model and tf perception neuron model "
          "are being published.");
      return -1;
    }
    // Calculate transformation matrix from base link to RightHand
    transBaseLinkRightHand =
        transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(
            transformArray.at(2).operator*=(transHipsRightHand)));
    transformArray.clear();

    // Rotate goal Transform M_PI around y to turn end effector into the
    // direction of the real hand/palm; only necessary for RightHand
    if(followLeftHand == 0){
        tf::Transform rotationRightHandToEE(
            tf::Quaternion(tf::Vector3(0, 1, 0), M_PI),
            tf::Vector3(0, 0, 0));  // rotates the tf frame of RightHand from
                                    // pointing to the body to poiting outwards
        transBaseLinkRightHand =
            transBaseLinkRightHand.operator*(rotationRightHandToEE);
    }
	// Broadcast the transform from base_link to the RightHand
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    tfBroadcaster.sendTransform(
        tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(),
                             "/base_link", "/baseLinkToHand"));
    return 0;
  }
};


struct ik {
  double* transForIK;  // 4x4 homogeneous transformation matrix in workspace,
                       // input for ur_kinematics IK calculations
  double* qIKSolArr;   // numSols * 6 vector, solution in joint space of IK
                       // calculations
  int numSols;         // number of analytial solutions of IK
  int ikSolNr;         // index of selected solution (elbow up, down...)
                       // TODO: replace ikSolNr by ikSolIndex
  std::vector<double> qIKSolVec;  // Selected solution to get published on robot

  int calcqIKSolVec(const tfPercNeuronInterface& tfPNIntf) {
    // Form goal tf transform into required input matrix double*transForIK for ur_kinematics
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // rotational part of homogeneous transformation matrix
    for (int i = 0; i <= 2; i++) {
      transForIK[i * 4 + 0] =
          tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getX();
      transForIK[i * 4 + 1] =
          tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getY();
      transForIK[i * 4 + 2] =
          tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getZ();
    }
    // translational part
    transForIK[3] = tfPNIntf.transBaseLinkRightHand.getOrigin().getX();
    transForIK[7] = tfPNIntf.transBaseLinkRightHand.getOrigin().getY();
    transForIK[11] = tfPNIntf.transBaseLinkRightHand.getOrigin().getZ();

    transForIK[12] = 0;
    transForIK[13] = 0;
    transForIK[14] = 0;
    transForIK[15] = 1;

    // Calculate Inverse Kinematic Solution
    // Input: 4x4 homogeneous transformation matrix from base to right hand
    // Output: 6*numSols matrix of possible joint positions to fit the end effector to the right hand
    //         Only the joint position vector at ik.ikSolNr will be used.
    // no solution found, if goal transform is out of reach for ur#
    // TODO: was passiert, wenn ich inverse() keine Lösung findet? -> numSols=0 ist?
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    numSols = ur_kinematics::inverse(transForIK, qIKSolArr);

    // Turn values of selected solution over M_PI into negative values (by substraction of 2*M_PI) to
    // use joint limited robot
    // => -M_PI < publishedJointPosition <= M_PI
    for (int j = 0; j < 6; j++) {
      if (M_PI < qIKSolArr[ikSolNr * 6 + j])
        qIKSolArr[ikSolNr * 6 + j] =
            qIKSolArr[ikSolNr * 6 + j] - 2 * M_PI;
    }
    //TODO: evaluate if the above version works. I only turn the values of the selected solution vector
    /*
    for (int i = 0; i < numSols; i++) {
      for (int j = 0; j < 6; j++) {
        if (M_PI < qIKSolArr[i * 6 + j])
          qIKSolArr[i * 6 + j] = qIKSolArr[i * 6 + j] - 2 * M_PI;
      }
    }
    */
    // Copy selected solution into solution vector
    qIKSolVec = {qIKSolArr[ikSolNr * 6 + 0], qIKSolArr[ikSolNr * 6 + 1],
                 qIKSolArr[ikSolNr * 6 + 2], qIKSolArr[ikSolNr * 6 + 3],
                 qIKSolArr[ikSolNr * 6 + 4], qIKSolArr[ikSolNr * 6 + 5]};
    return 0;
  }
};

void printVector(std::vector<double>& vector, bool debugStream) {
  if (debugStream) {
    ROS_DEBUG("\n");
    for (auto& currentElement : vector) {
      ROS_DEBUG("%f ", currentElement * 180 / M_PI);
    }
  } else {
    printf("\n");
    for (auto& currentElement : vector) {
      printf("%f ", currentElement * 180 / M_PI);
    }
  }
}

// Checks the angle restrictions set on the robot joints
// returns 1, if desired angle restricts the robot restrictions
// returns 0, if desired angle is valid
int violatesJointRestriction(const struct robotRestrictions& robRestrRef,
                             double innerAngle) {
  //TODO: CHECK if outcommenting of new double declaration caused any error:
  //double qInnerMin = robRestrRef.innerElbowMin;
  //double qInnerMax = robRestrRef.innerElbowMax;
  if (robRestrRef.innerElbowMin <= innerAngle && innerAngle <= robRestrRef.innerElbowMax)
    return 0;
  else
    return 1;
}
// Interface to robot controller:
// Interpolates last published position with desired goal, checks physical
// constraints on robot and publishes new desired joint values to controller /
// robot
// TODO: output und input beschreiben?
// Input: goalPosition
// Output: publishes position and sets it as lastPubPosition
// Returns 0 if goalPosition got published, -1 if not
int checkAndPublishDesJointValues(ros::NodeHandle& node,
                                  std::vector<double>& goalPosition,
                                  std::vector<double>& lastPubPosition,
                                  pubIntf& pubIntf, double totalTrajTime,
                                  const robotRestrictions& robRestrRef) {

  // Initialize Variables
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  bool goalPossible = false;
  // -2*M_PI < deltaPosition < 2*M_PI
  std::vector<double> deltaPosition = {2 * M_PI, 2 * M_PI, 2 * M_PI,
                                       2 * M_PI, 2 * M_PI, 2 * M_PI};
  std::vector<double> publishPosition = {0, 0, 0, 0, 0, 0};
  //TODO: disable variable and just go into loop below ALWAYS
  bool interpolate =
      1;  // to interpolate betwenn published values from Perception Neuron

  int numberPublishes = floor(
      totalTrajTime /
      pubIntf.jointPubRate);  // round down to never publish faster than 125 Hz
  if(numberPublishes == 0){ // publish at least once, isn't faster than 125 Hz
      numberPublishes = 1;
  }
  double correctedDeltaPos = 0*M_PI;  // corrects the joint distance for the limited
  // joint robot at position jump from -M_PI to M_PI or vice versa by
  // respectively subtracting or adding 2*M_PI to the deltaPosition.

  int elbowJointIndex =
      2;  // Index of elbow joint in goal/solution/delta/lastPubPosition vector

  // position i: 0:qShoulderPan, 1:qShoulderLift, 2:qElbow, 3:qWrist1,
  // 4:qWrist2, 5:qWrist3;

  // Check reachability of goals with elbow joint restriction:
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // default possible default range: 20° - 160° = PI/9 - 8/9*PI
  if (robRestrRef.checkElbowRestriction == 1) {
    double innerAngleElbow = M_PI - abs(goalPosition.at(elbowJointIndex));
    // Works safe for solution 5: elbow up; shoulder right; wrist down / at body
    if (1 == violatesJointRestriction(robRestrRef, innerAngleElbow)) {
      goalPossible = false;
      printf(
          "\n Desired elbow angle: %f° is outside of elbow joint inner angle "
          "boundaries: %f° - %f°, wait for User to come back to robot.",
          innerAngleElbow / M_PI * 180,
          robRestrRef.innerElbowMin / M_PI * 180,
          robRestrRef.innerElbowMax / M_PI * 180);
      return -1;
    } else {
      goalPossible = true;
    }
  }

  // Differenz goalPosition / lastPubPosition bilden
  for (int i = 0; i < goalPosition.size(); i++) {
    deltaPosition.at(i) = goalPosition.at(i) - lastPubPosition.at(i);
  }

  // Check for max velocity regulation
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i < deltaPosition.size(); i++) {
    if (deltaPosition.at(i) < -M_PI) {
      correctedDeltaPos = deltaPosition.at(i) + 2 * M_PI;
      if (correctedDeltaPos / totalTrajTime < robRestrRef.maxVel)  // maxVel excluded
        goalPossible = true;
      else {
        goalPossible = false;
        printf("\n1:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
          "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
          robRestrRef.maxVel / M_PI * 180,
          correctedDeltaPos / M_PI * 180 / totalTrajTime,
          abs(deltaPosition.at(i)) / M_PI * 180,
          totalTrajTime);
        return -1;
      }
    } else if (M_PI < deltaPosition.at(i)) {
      correctedDeltaPos = deltaPosition.at(i) - 2 * M_PI;
      if (-robRestrRef.maxVel < correctedDeltaPos / totalTrajTime)
        goalPossible = true;
      else {
        goalPossible = false;
        printf("\n2:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
          "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
          robRestrRef.maxVel / M_PI * 180,
          correctedDeltaPos / M_PI * 180 / totalTrajTime,
          abs(deltaPosition.at(i)) / M_PI * 180,
          totalTrajTime);
        return -1;
      }
    }
    else if (abs(deltaPosition.at(i)) / totalTrajTime <
             robRestrRef.maxVel)
      goalPossible = true;
    else {
      goalPossible = false;
      printf("\n3:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
        "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
        robRestrRef.maxVel / M_PI * 180,
        deltaPosition.at(i) / M_PI * 180 / totalTrajTime,
        abs(deltaPosition.at(i)) / M_PI * 180,
        totalTrajTime);
      return -1;
    }
  }


  // Interpolate and publish solutions
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  if (interpolate) {
    // all solutions from here on have to be possible solutions and not 
	// damage the robot
    // interpolate in  totalTrajTime ms to send at the rate of robot controller
    ros::Rate rate(1 / pubIntf.jointPubRate);

    for (int step = 1; step <= numberPublishes;
         step++) {  // = 2 at 60Hz main loop
      // Interpolate und set publishPosition
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      for (int i = 0; i < deltaPosition.size(); i++) {
        // Check for jump from positive to negative value or vize versa, caused
        // by negating values after ik calculation
        // So keeps boundaries -M_PI < publishPosition <= M_PI
        if (M_PI < deltaPosition.at(i)) {
          correctedDeltaPos = deltaPosition.at(i) - 2 * M_PI;
          publishPosition.at(i) =
              lastPubPosition.at(i) +
              ((float)step / (float)numberPublishes) * correctedDeltaPos;
          if (publishPosition.at(i) < -M_PI) {
            publishPosition.at(i) += 2 * M_PI;
          }
          printf(
              "\njump from negative to positive, publishPosition: %f, lastPos "
              "%f, deltaPos %f",
              publishPosition.at(i), lastPubPosition.at(i), deltaPosition.at(i));
          ROS_DEBUG_STREAM(
              "Detected M_PI < (jump from negative to positive desired joint "
              "angle)");
        } else if (deltaPosition.at(i) < -M_PI) {
          correctedDeltaPos = deltaPosition.at(i) + 2 * M_PI;
          publishPosition.at(i) =
              lastPubPosition.at(i) +
              ((float)step / (float)numberPublishes) * correctedDeltaPos;
          if (M_PI < publishPosition.at(i)) {
            publishPosition.at(i) -= 2 * M_PI;
          }
          ROS_DEBUG_STREAM(
              "Detected (jump from positive to negative desired joint angle) < "
              "-M_PI");
          printf(
              "\njump from positive to negative, publishPosition: %f, lastPos "
              "%f, deltaPos %f",
              publishPosition.at(i), lastPubPosition.at(i), deltaPosition.at(i));
        } else
          publishPosition.at(i) =
              lastPubPosition.at(i) +
              ((float)step / (float)numberPublishes) * deltaPosition.at(i);
      }
      // Send / Publish Data to Robot Controller
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      pubIntf.jointMsg.header.stamp = ros::Time::now();
      pubIntf.jointMsg.position = {
          publishPosition.at(0), publishPosition.at(1), publishPosition.at(2),
          publishPosition.at(3), publishPosition.at(4), publishPosition.at(5)};

      ROS_INFO_STREAM("Published Position: " <<
                      publishPosition.at(0) << " " << publishPosition.at(1) << " " <<
                      publishPosition.at(2) << " " << publishPosition.at(3) << " " <<
                      publishPosition.at(4) << " " << publishPosition.at(5));
      pubIntf.jointPosPublisher.publish(pubIntf.jointMsg);
      rate.sleep(); 
    }
    // Copy last published Position to serve as updated current position
    std::copy_n(publishPosition.begin(), publishPosition.size(),
                lastPubPosition.begin());
    // ----------------------------

    // Test if value out of boundaries was published
    for (int i = 0; i < lastPubPosition.size(); i++) {
      if (publishPosition.at(i) < -M_PI || M_PI < publishPosition.at(i)) {
        printf(
            "\nInvalid Value published / not in boundaries: -M_PI < "
            "publishPosition.at(%d) < M_PI: %f ",
            i, publishPosition.at(i));
        while (node.ok()) {
          ros::Duration(1).sleep();
        }
      }
    }

  } else {  // Do not interpolate:
    for (int i = 0; i < goalPosition.size(); i++)
      publishPosition.at(i) = goalPosition.at(i);
    pubIntf.jointMsg.header.stamp = ros::Time::now();
    pubIntf.jointMsg.position = {publishPosition.at(0), publishPosition.at(1),
                                 publishPosition.at(2), publishPosition.at(3),
                                 publishPosition.at(4), publishPosition.at(5)};
    pubIntf.jointPosPublisher.publish(pubIntf.jointMsg);
  }

  return 0;
}

void printTransformMatrix(tf::Transform& tfTransform) {
  int i = 0;
  printf("\n%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getX());
  i = 1;
  printf("%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getY());
  i = 2;
  printf("%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getZ());
  printf("%1.6f %1.6f %1.6f %1.6f\n", 0.0, 0.0, 0.0, 1.0);
}

// Drives the robot from its default position to the intial hand position.
// Works over large distance; saves hand position s.t. user can move hand
// during intialization
// Called in the beginning and whenever the robot loses the track of the hand.
// returns -1 if not successful, 0 for success
int initializeRobotToHand(ros::NodeHandle& nodeRef, pubIntf& pubIntf,
                          tfPercNeuronInterface& tfPNIntf, ik& ik,
                          std::vector<double>& lastPubPositionRef,
                          std::vector<double>& defaultRobotPositionRef,
                          const robotRestrictions& robRestrRef) {
  printf(
      "\nPress Enter to drive Robot from Robot Default Position to your "
      "hand.\nPlease hold hand steady during the process.");
  getchar();
  // Get tf transform from base_link of ur# to RightHand of PercNeuron
  int success = -1;
  while (success == -1) {
    success = tfPNIntf.getTransBaseLinkRightHand();  // returns 0 for success 
    // TODO: hier einen Sleep einbauen, damit sich das Programm bei fehlenden tf daten nicht in endlosschleife aufhängt
    // oder return 1 schreiben damit intiializaerobottohand erneut started
  }

  // Get Goal Joint Positions by Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ik.calcqIKSolVec(tfPNIntf);  // sets ik.qIKSolVec


  /* TODO: CHECK IF USE OF calcqIKSolVec() works!!!
   * mögliche Probleme:
   *    ik.qIKSolArr ist so bereits belegt. Nach initialisierung ist er nicht default mäßig belegt und verursacht Probleme
   *        Aber: diese Werte werden sowieso hier innerhalb der structs gelegt

  // Form goal tf transform into required input matrix double*transForIK for ur_kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // rotational part of homogeneous transformation matrix
  for (int i = 0; i <= 2; i++) {
    ik.transForIK[i * 4 + 0] =
        tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getX();
    ik.transForIK[i * 4 + 1] =
        tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getY();
    ik.transForIK[i * 4 + 2] =
        tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getZ();
  }
  // translational part
  ik.transForIK[3] = tfPNIntf.transBaseLinkRightHand.getOrigin().getX();
  ik.transForIK[7] = tfPNIntf.transBaseLinkRightHand.getOrigin().getY();
  ik.transForIK[11] = tfPNIntf.transBaseLinkRightHand.getOrigin().getZ();

  ik.transForIK[12] = 0;
  ik.transForIK[13] = 0;
  ik.transForIK[14] = 0;
  ik.transForIK[15] = 1;

  // Calculate Inverse Kinematic Solution
  // Input: 4x4 homogeneous transformation matrix from base to right hand
  // Output: 6*numSols matrix of possible joint positions to fit the end effector to the right hand
  //         Only the joint position vector at ik.ikSolNr will be used.
  // no solution found, if goal transform is out of reach for ur#
  // TODO: was passiert, wenn ich inverse() keine Lösung findet? -> numSols=0 ist?
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ik.numSols = ur_kinematics::inverse(ik.transForIK, ik.qIKSolArr);

  // Turn values of selected solution over M_PI into negative values (by substraction of 2*M_PI) to
  // use full range of robot, or to use limited robot
  for (int j = 0; j < 6; j++) {
    if (M_PI < ik.qIKSolArr[ik.ikSolNr * 6 + j])
      ik.qIKSolArr[ik.ikSolNr * 6 + j] =
          ik.qIKSolArr[ik.ikSolNr * 6 + j] - 2 * M_PI;
  }
  */

  // Calculate necessary time for reinitialization process
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  double deltaAngle = 0;// difference from current robot joint angle to goal angle
  double maxDeltaAngle = 0;
  double avgRadPSec =
      robRestrRef.maxVel /
      10;  // by setting the avgVel to a 10th of the maxVel, robot won't
  // violate velocity restrictions at its fastest point. As exact function
  // of trajectory calculation isn't known, it will be checked again stepwise.

  // Set maxDeltaAngle
  for (int i = 0; i < lastPubPositionRef.size(); i++) {
    //TODO: EVALUATE IF CHANGE OF FOLLOWING LINE DIDNT CAUSE PROBLEMS
    //deltaAngle = abs(ik.qIKSolArr[ik.ikSolNr * 6 + i] - lastPubPositionRef.at(i));
    deltaAngle = abs(ik.qIKSolVec.at(i) - lastPubPositionRef.at(i));
    if (maxDeltaAngle < deltaAngle) {
      maxDeltaAngle = deltaAngle;
    }
  }
  double minInitDuration = maxDeltaAngle / avgRadPSec;
  printf("\nDuration of (re-)initialization: %f", minInitDuration);

  // Check for Elbow Joint Angle Restriction before driving the robot to the
  // hand position, s.t. user could start to reinitialize from new position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  int elbowJointIndex = 2;
  //TODO: EVALUATE IF CHANGE OF FOLLOWING LINE DIDNT CAUSE PROBLEMS
  double innerAngleElbow =
    M_PI - abs(ik.qIKSolVec.at(elbowJointIndex));
    //M_PI - abs(ik.qIKSolArr[ik.ikSolNr * 6 + elbowJointIndex]);
  if (1 == violatesJointRestriction(robRestrRef, innerAngleElbow)) {
    printf(
        "\nElbow Joint Restriction Violation. Desired inner angle should lie"
        " in between %f° and %f°, but is: %f°",
        robRestrRef.innerElbowMin / M_PI * 180,
        robRestrRef.innerElbowMax / M_PI * 180,
        innerAngleElbow / M_PI * 180);
    return -1; // abort (re-)initialization
  }

  // Set the current robot position and its goal position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Intialize with default position, s.t. an error in the program would
  // publish the current robot position
  lastPubPositionRef = defaultRobotPositionRef;
  std::vector<double> currentGoal = defaultRobotPositionRef;
  std::vector<double>& currentGoalRef = currentGoal;

  // Copy the start and goal position matrices into input for trajectory calculation
  Eigen::Matrix<double, 6, 1> defaultPosInput;  // Matrix< _Scalar, _Rows, _Cols >
  defaultPosInput << defaultRobotPositionRef.at(0), defaultRobotPositionRef.at(1),
      defaultRobotPositionRef.at(2), defaultRobotPositionRef.at(3),
      defaultRobotPositionRef.at(4), defaultRobotPositionRef.at(5);

  Eigen::Matrix<double, 6, 1> goalPosInput;
  //TODO: EVALuate if CHANGE AT THIS PLACE DID NOT CAUSE ANY PROBLEMS:
  goalPosInput << ik.qIKSolVec.at(0),
      ik.qIKSolVec.at(1), ik.qIKSolVec.at(2),
      ik.qIKSolVec.at(3), ik.qIKSolVec.at(4),
      ik.qIKSolVec.at(5);
  //goalPosInput << ik.qIKSolArr[ik.ikSolNr * 6 + 0],
  //    ik.qIKSolArr[ik.ikSolNr * 6 + 1], ik.qIKSolArr[ik.ikSolNr * 6 + 2],
  //    ik.qIKSolArr[ik.ikSolNr * 6 + 3], ik.qIKSolArr[ik.ikSolNr * 6 + 4],
  //    ik.qIKSolArr[ik.ikSolNr * 6 + 5];

  //TODO: Do i need this standard output?
  //TODO: Replace by printing out the goal vector: qIKSolVec[0-5]
  std::cout << "goal: " << goalPosInput << std::endl;
  printf("\nvectordofd initialized");

  QVector<Tum::VectorDOFd> currentGoalOutput;
  // Times used as input for trajectory calculation
  ros::Time initialTime;
  ros::Duration passedTime;
  ros::Time endTime;

  initialTime = ros::Time::now();
  int numberPublishes = (int)floor(minInitDuration / pubIntf.jointPubRate);
  printf("\nminInitDuration in secs : %f", minInitDuration);
  printf("\ninitialTime in secs : %f", initialTime.toSec());
  printf("\nendTime in secs : %f", endTime.toSec());
  printf("\nnumber Publishes : %d", numberPublishes);
  // TODO: is supercounter necessary
  int supercounter = 0;
  // Calculate smooth trajectory from default robot position to hand position
  // and publish it iteratively to robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i <= numberPublishes;
       ++i) {  // publish one more time to publish goalPosition
    if (nodeRef.ok()) {
      passedTime = ros::Time::now() - initialTime;
      // GetJointPVT5 calculates a smooth trajectory and returns it stepwise
      currentGoalOutput = Tum::Tools::MathTools::getJointPVT5(
          defaultPosInput, goalPosInput, passedTime.toSec(), minInitDuration);

      // Type conversion from QVector<Tum::VectorDOFd> to std::vector<double>
      std::copy_n(&(currentGoalOutput[0](0)),
                  currentGoalOutput[0].size(), currentGoal.begin());

      // Publish Data to Robot
      // TODO: delete following two lines, if no error at compilation
      // ros::Time begin = ros::Time::now();
      // ros::Duration elapsedTime = begin - begin;
      // TODO: CHECK IF I SHOULD PUT SUCCESS = CHECK...
      checkAndPublishDesJointValues(nodeRef, currentGoalRef, lastPubPositionRef,
                                    pubIntf, pubIntf.jointPubRate, robRestrRef);
      // check if published goal is already reached. necessary because,
      // checkAndPublishDesJointValues needs more time than pubIntf.jointPubRate
      int equalJoints = 0;
      for (int i = 0; i < lastPubPositionRef.size(); ++i) {
        if (lastPubPositionRef.at(i) == goalPosInput[i]) {
          equalJoints++;
        }
      }
      if (equalJoints == 6) {
        printf(
            "\nDefault position has been reached. You can start moving your "
            "hand slowly now.");
        break;
      };
      supercounter++;
    } else {
      printf("\nAborted initialization process. ");
      ros::shutdown();
    }
  }
  printf("\npublishes: %d times", supercounter);
  passedTime = ros::Time::now() - initialTime;
  printf("\npassedTime in secs : %f", passedTime.toSec());
}

// returns -1 for failure, 0 for success
int getCurrentRobotPosition(std::vector<double>& robotPosition) {
  std::string robotStateTopic = "/ur10_arm_joint_states";

  //TODO: TEST proper functionality of new default message initialization here
  printf("\nGet default robot position from topic %s", robotStateTopic.c_str());

  sensor_msgs::JointState robotPositionMsg;

  // Create shared_ptr to check on received default Position message and not cause an assertion error.
  boost::shared_ptr<sensor_msgs::JointState const> defaultPosMsgPtr;
  defaultPosMsgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>(robotStateTopic,
                                                            ros::Duration(5));

  //sensor_msgs::JointState robotPositionMsg =
  //    *(ros::topic::waitForMessage<sensor_msgs::JointState>(robotStateTopic,
  //                                                          ros::Duration(10)));

  if(defaultPosMsgPtr == NULL){
      //TODO: SAY WHICH MODULE TO START
      printf("\n Default robot position is not published. Start testJointCtrl_ur10. Trying again.");
      // TODO FOR TEST ONLY:
      //return -1;


  }
  else{
      robotPositionMsg = *defaultPosMsgPtr;
  }

  // TODO: FOR TEST ONLY
  // TODO: CHANGE BACK TO robotPositionMsg.position and not testpublishPosition., received from
  // ur10_arm_joint_states topic
  //robotPosition = robotPositionMsg.position;

  std::vector<double> testpublishPosition = {0, 0, 0, 0, 0, 0};
  robotPosition = testpublishPosition;

  printf("\nDefault Position of Roboter: ");
  printVector(robotPosition, 0);
  printf(
      "\nAre you ABSOLUTELY sure that this is the correct default position? y "
      "or n?\n");
  char response;
  cin >> response;
  printf("\nthis  was your response: %c", response);
  if ((response == 'Y' || response == 'y')) {
    printf("\nShown default position is assumed to be correct.\n");
    return 0;
  } else {
    printf("\nTrying again to get updated position.");
    return -1;
  }
}

// Only works for joint_limited version!, if other is desired: change
// calculateTraj.
int main(int argc, char** argv) {

  // Declare general variables
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ros::init(argc, argv, "follow_hand_node");

  ros::NodeHandle node;
  ros::NodeHandle& nodeRef = node;
  int loopCounter = 0;
  int success;  // int to check for successful execution of functions

  // Set FrameRate to scan data; PercNeuron (32 Neuron Version) runs at 60Hz,
  // according to online datasheet
  // If number of neurons of connected suit is less than 19 the acquisition
  // frequency / frequency of callback function is 120Hz
  // This program uses 20 fps, as rosserial_server limits the capacity. A
  // higher fps causes instability in the transmission of Position Data
  // from windows to ROS machine
  // TODO: CHANGE FRAMESPERSECPERCNEURON TO 60!
  double framesPerSecPercNeuron = 20;

  // TODO: DO NOT USE THIS VARIABLE AS it is highly dependant on used computer
  // TODO: First, test by setting it to zero. 0.6ms sind sowieso nicht im Toleranzbereich des Sleeps
  // TODO: Oder, asses dynamically, speicher den Wert und sleep so viel
  double timeForMainLoop = 0.0006;  // max Time for calculations in main loop
                                    // without checkAndPublishDesJointValues()
  double totalTrajPubTime = (1 / framesPerSecPercNeuron) -
                            timeForMainLoop;  // total time of publishing
                                              // interval / calculating /
                                              // interpolating the trajectory
  // TODO: EVALUate if change of position of Perception Neuron Interface caused any errors
  // Initialize Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  // Interface to perc.neuron tf data
                                          // created by perc neuron tf
                                          // broadcaster
  //TODO: follow Right not left hand!
  tfPNIntf.followLeftHand = 1; // default follows Right Hand
  tfPNIntf.transBaseLinkRightHand = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, 0, 0));  // transform from base_link to RightHand directly

  tfPNIntf.tfBroadcaster.sendTransform(
      tf::StampedTransform(tfPNIntf.transBaseLinkRightHand, ros::Time::now(),
                           "/base_link", "/baseLinkToHand"));

  // create string array with the joints in order of the skeleton hierarchy,
  // provided by the Perception Neuron API, to calculate goal transform
  tfPNIntf.bodyJointsRightHand = {
      "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck",
      "RightShoulder", "RightArm", "RightForeArm", "RightHand"};
  tfPNIntf.bodyJointsLeftHand = {
      "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck",
      "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand"};

  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;

  // Declare interface to UR10 / publisher for joint values
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // all ur# values, including DH (Denavit-Hartenberg) values are stored on the
  // parameter server
  // on $ rosparam get /robot_description
  struct pubIntf pubIntf;  // Create interface for publishing joint values to
                           // controller/robot
  pubIntf.jointPubRate = 0.008;  // Rate interval at which joint values are published
  pubIntf.publishToController = true;
  nodeRef.getParam("/follow_hand/publishToController", pubIntf.publishToController);
  if (pubIntf.publishToController){
    printf("Publish data values to robot controller on topic: %s", "/joint_desired_cmd");
    pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
        "/joint_desired_cmd",
        1000);  // publishes to the tum_ics_ur_robot_controller
  }
  else {
    // pubIntf.jointPosPublisher =
    // node.advertise<sensor_msgs::JointState>("/follow_hand/joint_states_ur5",
    // 1000); // publishes to "$roslaunch bringRobot demo.launch". Used when 
	// working without a controller
    printf("Publish data values directly onto robot on topic: %s", "/ur10_arm_joint_states");
    pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
        "/ur10_arm_joint_states",
        1000);  // publishes directly to robot simulation; passes the controller
  }
  // Initialize joint state message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // TODO: jointMsg currently uninitialized!
  //sensor_msgs::JointState* jointStateMsg = (sensor_msgs::JointState*)malloc(sizeof(sensor_msgs::JointState));
  //sensor_msgs::JointState jointStateMsg;
  //pubIntf.jointMsg = &jointStateMsg;
  //sensor_msgs::JointState jointStateMsg(new sensor_msgs::JointState);
  //pubIntf.jointMsg = jointStateMsg;

  pubIntf.jointMsg.header.stamp = ros::Time::now();

  pubIntf.jointMsg.header.frame_id = "";

  std::vector<std::string> jointNames = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  pubIntf.jointMsg.name = jointNames;
  pubIntf.jointMsg.velocity = {};
  pubIntf.jointMsg.effort = {};

  struct pubIntf& pubIntfRef = pubIntf;

  /*TODO:REMOVE
  //TEST MESSAGE PUBLISHER
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //std::vector<double> testpublishPosition = {0, 0, 0, 0, 0, 0};
  //pubIntf.jointMsg.position = testpublishPosition;
  //ros::Duration half_second(0.5);
  int k = 0;
  while(node.ok()){
      std::cout << k << std::endl;
      pubIntf.jointPosPublisher.publish(pubIntfRef.jointMsg);
      ros::Duration(0.5).sleep();
      k++;
  }
  */

  // Set physical restrictions to Robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  struct robotRestrictions robRestr;
  robRestr.checkElbowRestriction;

  double innerElbowMinDeg = 20;  // = M_PI/9;
  nodeRef.getParam("/follow_hand/qInnerElbowMin", innerElbowMinDeg);
  robRestr.innerElbowMin = innerElbowMinDeg / 180 * M_PI;  // convert deg to rad

  double innerElbowMaxDeg = 160;
  nodeRef.getParam("/follow_hand/qInnerElbowMax", innerElbowMaxDeg);
  robRestr.innerElbowMax = innerElbowMaxDeg / 180 * M_PI;

  double maxVelDegPSec = 17;  // default maxVel = 17°/sec = 17/180*M_PI rad/sec
                              // = 0.29670597283 rad/sec
  node.getParam("/follow_hand/maxVel", maxVelDegPSec);
  printf("\nMaximum velocity in degree per seconds: %f", maxVelDegPSec);
  robRestr.maxVel = maxVelDegPSec / 180 * M_PI;

  const struct robotRestrictions& robRestrRef = robRestr;

  // Get the CurrentRobotPosition and set it as defaultRobotPosition
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<double> defaultRobotPosition;
  std::vector<double>& defaultRobotPositionRef = defaultRobotPosition;
  success = -1;
  while (success == -1 && node.ok()) {
    success = getCurrentRobotPosition(defaultRobotPositionRef);
    ros::Duration(1).sleep();
  }

  // Initialize variables for calculation of inverse kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<double> lastPubPosition =
      defaultRobotPositionRef;  // last to robot published joint position
  std::vector<double>& lastPubPositionRef = lastPubPosition;
  tf::Matrix3x3 rotMatrix;

  struct ik ik;
  ik.transForIK = new double[16];  // 4x4 transform matrix in workspace, input
                                   // for ik calculations
  ik.qIKSolArr =
      new double[8 * 6];  // 8 possible ik solutions * 6 joints in joint space
  /**  table for solutions:
       shoulder left = on left side of body (from eyes of perception neuron
     person) and shoulder_pan_joint > 0?
       i = 0 : elbow up;    shoulder left;      wrist up / away from body
       i = 1 : elbow down;  shoulder left;      wrist up / away from body
       i = 2 : elbow up;    shoulder left;      wrist down / at body;
         = 2 : e.g.: -0.713767 -0.823703 1.703598 2.607962 -1.540564 -2.063133
       i = 3 : elbow down;  shoulder left;      wrist down / at body
       i = 4 : elbow down;  shoulder right;     wrist down / at body
       i = 5 : elbow up;    shoulder right;     wrist at down / body;
         = 5 : e.g.:  2.805665 -2.314047 -1.709266 0.515700 1.246372 -1.930651
       i = 6 : elbow down;  shoulder right;     wrist up / away from body
       i = 7 : elbow up;    shoulder right;     wrist up / away from body
  */
  ik.ikSolNr = 5;
  ik.qIKSolVec = defaultRobotPositionRef;
  struct ik& ikRef = ik;

  // Drive Robot from Robot Default Position to Perc. Neuron Default Position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nInitialize Robot for the first time.");
  printf("\nDefault Position: ");
  printVector(defaultRobotPositionRef, 0);
  robRestr.checkElbowRestriction = 0;  // Elbow restrictions are only checked at the beginning of
          // initialization process if it's possible to reach goal position,
          // because robot default position can be out of the restriction
          // boundaries

  success = -1;
  while (success == -1 && node.ok()) {
    printf("\nStarting initialization\n");
    success = initializeRobotToHand(nodeRef, pubIntfRef, tfPNIntfRef, ikRef,
                                    lastPubPositionRef, defaultRobotPositionRef,
                                    robRestrRef);
    if(success == -1){
      printf("\nInitialization process failed. Trying again in 1sec.\n");
      ros::Duration(1).sleep();
    }
  }
  robRestr.checkElbowRestriction = 1;

  // Start the loop to get tf position from Perception Neuron and publish it to the
  // robot / controller
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nYou can start moving slowly now.");
  ros::Time begin = ros::Time::now();
  ros::Duration elapsedTime = begin - begin;
  ros::Rate rate(framesPerSecPercNeuron);
  while (node.ok()) {
    begin = ros::Time::now();

    // Get goal transform for UR10's end effector from UR10's base_link to
    // Perception Neuron's RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success = -1;
    while (success == -1) {
      success = tfPNIntfRef.getTransBaseLinkRightHand();
    }

    // Get Goal Joint Positions from transBaseLinkRightHand by Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ik.calcqIKSolVec(tfPNIntfRef);  // sets ik.qIKSolVec

    // Check physical restrictions, interpolate points from rate framesPerSecPercNeuron
    // to max 125HZ = 1frame / 8ms and publish them to the controller / robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    elapsedTime = ros::Time::now() - begin;

    success = checkAndPublishDesJointValues(nodeRef, ik.qIKSolVec,
                                                lastPubPositionRef, pubIntfRef,
                                                totalTrajPubTime, robRestrRef);

    // Reinitialization if goal position couldn't be published
    // e.g. If human controller moved to fast or out of range
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //TODO: Prüfen, dass das verschieben von der kommenden Zeile um 5 nach unten nicht zu einem Fehler geführt hat.
    char reinitializeResponse;
    if (-1 == success) {  // robot desired value couldn't be published
      printf("\nCurrent Robot Position / lastPubPosition: ");
      printVector(lastPubPositionRef, 0);
      printf(
          "\nDesired goal could not be published.\nDo you want to reinitialize "
          "the robot, so that it drives to your arm?\ny or n?\n");
      cin >> reinitializeResponse;
      printf("\nthis  was your response: %c", reinitializeResponse);
      if ((reinitializeResponse != 'Y' && reinitializeResponse != 'y')) {
        printf("\nAre you sure to terminate the program?\ny or n?\n");
        cin >> reinitializeResponse;
        if ((reinitializeResponse == 'Y' || reinitializeResponse == 'y')) {
          printf("\nTerminate Node.");
          ros::shutdown();
          return 0;
        }
      } else {
        printf("\nStart Reinitialization. Please hold your hand steady.");
        // now defaultRobotPosition is lastPubPosition
        robRestr.checkElbowRestriction = 0;  // Elbow restrictions are 
                // only checked at the beginning ofs
                // initialization process if it's possible to reach goal
                // position, because robot default position can be out of the
                // restriction boundaries
        // TODO: evaluate if success at this point does not lead to an endless loop and can cause that, if the initialization process is aborted, it can be started again.
        success = initializeRobotToHand(nodeRef, pubIntfRef, tfPNIntfRef, ikRef,
                              lastPubPositionRef, lastPubPositionRef, robRestrRef);
        robRestr.checkElbowRestriction = 1;
      }
    }
    loopCounter++;
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
