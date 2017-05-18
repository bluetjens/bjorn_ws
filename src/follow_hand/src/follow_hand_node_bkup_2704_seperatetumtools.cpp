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

#include <ur_kinematics/ur_kin.h>

using namespace std;


// Looks up a Transfrom and stores them in a tf array.
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
  // [0, M_PI], minimum inner angle of elbow of robot in rad
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
  double minPubRate;  // Rate at which joint values are published
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
                                         // directly
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
    // Calculate transformation matrix from world to RightHand
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
  double* transForIK;  // 4x4 transformation matrix in workspace, input for IK
                       // calculations
  double* qIKSolArr;   // numSols * 6 vector, solution in joint space of IK
                       // calculations
  int numSols;         // number of analytial solutions of IK
  int ikSolNr;         // number of selected solution (elbow up, down...)
  std::vector<double> qIKSolVec;  // Selected solution to get published on robot

  int calcqIKSolVec(const tfPercNeuronInterface& tfPNIntf) {
    // Form Goal type from Transform tfPNIntf into transformation matrix T
	// from ur_kinematics
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // rotational part of transf. matrix
    for (int i = 0; i <= 2; i++) {
      // rotational part of transf. matrix
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
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // no solution found, if goal transform is out of reach for ur#
    numSols = ur_kinematics::inverse(transForIK, qIKSolArr);

    // Turn values over M_PI into negative values (by substraction of 2*M_PI) to
    // use joint limited robot
    for (int i = 0; i < numSols; i++) {
      for (int j = 0; j < 6; j++) {
        if (M_PI < qIKSolArr[i * 6 + j])
          qIKSolArr[i * 6 + j] = qIKSolArr[i * 6 + j] - 2 * M_PI;
      }
    }

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
int violatesJointRestriction(const struct robotRestrictions& robRestrRef,
                             double innerAngle) {
  double qInnerMin = robRestrRef.innerElbowMin;
  double qInnerMax = robRestrRef.innerElbowMax;
  if (qInnerMin <= innerAngle && innerAngle <= qInnerMax)
    return 0;
  else
    return 1;
}
// Interface to robot controller:
// Interpolates last published position with desired goal, checks physical
// constraints on robot and publishes new desired joint values to controller /
// robot
// Returns 0 if goalPosition got published, -1 if not
int checkAndPublishDesJointValues(ros::NodeHandle& node,
                                  std::vector<double>& goalPosition,
                                  std::vector<double>& lastPosition,
                                  pubIntf& pubIntf, double totalTrajTime,
                                  const robotRestrictions& robRestrRef) {

  // Initialize Variables
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  bool goalPossible = false;
  // -2*M_PI < deltaPosition < 2*M_PI
  std::vector<double> deltaPosition = {2 * M_PI, 2 * M_PI, 2 * M_PI,
                                       2 * M_PI, 2 * M_PI, 2 * M_PI};
  std::vector<double> publishPosition = {0, 0, 0, 0, 0, 0};
  bool interpolate =
      1;  // to interpolate betwenn published values from Perception Neuron

  int numberPublishes = floor(
      totalTrajTime /
      pubIntf.minPubRate);  // round down to never publish faster than 125 Hz
  if(numberPublishes == 0){ // publish at least once, isn't faster than 125 Hz
      numberPublishes = 1;
  }
  double tempDelta = 0;  // temporary deltaPosition for velocity control

  int elbowJointIndex =
      2;  // Index of elbow joint in goal/solution/delta/lastPosition vector

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

  // Differenz goalPosition / lastPosition bilden
  for (int i = 0; i < goalPosition.size(); i++) {
    deltaPosition.at(i) = goalPosition.at(i) - lastPosition.at(i);
  }

  // Check for max velocity regulation
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i < deltaPosition.size(); i++) {
    if (deltaPosition.at(i) < -M_PI) {
      double checkVel = deltaPosition.at(i) + 2 * M_PI;
      if (checkVel / totalTrajTime < robRestrRef.maxVel)  // maxVel not desired
        goalPossible = true;
      else {
        goalPossible = false;
        printf("\n1:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
          "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
          robRestrRef.maxVel / M_PI * 180,
          checkVel / M_PI * 180 / totalTrajTime,
          abs(deltaPosition.at(i)) / M_PI * 180,
          totalTrajTime);
        return -1;
      }
    } else if (M_PI < deltaPosition.at(i)) {
      double checkVel = deltaPosition.at(i) - 2 * M_PI;
      if (-robRestrRef.maxVel < checkVel / totalTrajTime)  // maxVel not desired
        goalPossible = true;
      else {
        goalPossible = false;
        printf("\n2:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
          "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
          robRestrRef.maxVel / M_PI * 180,
          checkVel / M_PI * 180 / totalTrajTime,
          abs(deltaPosition.at(i)) / M_PI * 180,
          totalTrajTime);
        return -1;
      }
    }
    else if (abs(deltaPosition.at(i)) / totalTrajTime <
             robRestrRef.maxVel)  // maxVel not desired
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
    // interpolate in  totalTrajTime ms to send in a rate of minPubRate
    ros::Rate rate(1 / pubIntf.minPubRate);

    for (int step = 1; step <= numberPublishes;
         step++) {  // numberPublishes = 5 at 20 Hz
      // Interpolate und set publishPosition
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      for (int i = 0; i < deltaPosition.size(); i++) {
        // Check for jump from positive to negative value or vize versa, caused
        // by negating values after ik calculation in main
        // So keeps boundaries -M_PI < publishPosition < M_PI
        if (M_PI < deltaPosition.at(i)) {
          tempDelta = deltaPosition.at(i) - 2 * M_PI;
          publishPosition.at(i) =
              lastPosition.at(i) +
              ((float)step / (float)numberPublishes) * tempDelta;
          if (publishPosition.at(i) < -M_PI) {
            publishPosition.at(i) += 2 * M_PI;
          }
          printf(
              "\njump from negative to positive, publishPosition: %f, lastPos "
              "%f, deltaPos %f",
              publishPosition.at(i), lastPosition.at(i), deltaPosition.at(i));
          ROS_DEBUG_STREAM(
              "Detected M_PI < (jump from negative to positive desired joint "
              "angle)");
        } else if (deltaPosition.at(i) < -M_PI) {
          tempDelta = deltaPosition.at(i) + 2 * M_PI;
          publishPosition.at(i) =
              lastPosition.at(i) +
              ((float)step / (float)numberPublishes) * tempDelta;
          if (M_PI < publishPosition.at(i)) {
            publishPosition.at(i) -= 2 * M_PI;
          }
          ROS_DEBUG_STREAM(
              "Detected (jump from positive to negative desired joint angle) < "
              "-M_PI");
          printf(
              "\njump from positive to negative, publishPosition: %f, lastPos "
              "%f, deltaPos %f",
              publishPosition.at(i), lastPosition.at(i), deltaPosition.at(i));
        } else
          publishPosition.at(i) =
              lastPosition.at(i) +
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
    // Copy last published Position to lastPosition
    std::copy_n(publishPosition.begin(), publishPosition.size(),
                lastPosition.begin());
    // ----------------------------

    // Test if value out of boundaries was published
    for (int i = 0; i < lastPosition.size(); i++) {
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

// Called to drive the robot from the default position to the start position. 
// Called in the beginning and whenever the robot loses the track of the hand.
// returns -1 if not successful, 0 for success
int initializeRobotToHand(ros::NodeHandle& nodeRef, pubIntf& pubIntf,
                          tfPercNeuronInterface& tfPNIntf, ik& ik,
                          std::vector<double>& lastPositionRef,
                          std::vector<double>& defaultRobotPositionRef,
                          const robotRestrictions& robRestrRef) {
  printf(
      "\nPress Enter to drive Robot from Robot Default Position to your "
      "hand.\nPlease hold hand steady during the process.");
  getchar();
  // perc_broadcaster and tf model of perception neuron
  // Get Transform / Vector from shoulder_link of ur# to RightHand of PercNeuron
  int success = -1;
  while (success == -1) {
    success = tfPNIntf.getTransBaseLinkRightHand();  // returns 0 for success 
  }

  // Form Goal Transform into transformation matrix T from ur_kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // rotational part of transf. matrix
  for (int i = 0; i <= 2; i++) {
    // rotational part of transf. matrix
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
  // no solution found, if goal transform is out of reach for ur#
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ik.numSols = ur_kinematics::inverse(ik.transForIK, ik.qIKSolArr);

  // Turn values over M_PI into negative values (by substraction of 2*M_PI) to
  // use full range of robot, or to use limited robot
  for (int j = 0; j < 6; j++) {
    if (M_PI < ik.qIKSolArr[ik.ikSolNr * 6 + j])
      ik.qIKSolArr[ik.ikSolNr * 6 + j] =
          ik.qIKSolArr[ik.ikSolNr * 6 + j] - 2 * M_PI;
  }

  // Calculate time for reinitialization process. Started, when user is too far
  // away from current robot position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  double maxAngle = 0;
  double bufferAngle = 0;
  double avgRadPSec =
      robRestrRef.maxVel /
      10;  // average velocity of initialization is way lower than maxVel
  // Calculate difference of goal to current position
  for (int i = 0; i < lastPositionRef.size(); i++) {
    bufferAngle = abs(ik.qIKSolArr[ik.ikSolNr * 6 + i] - lastPositionRef.at(i));
    if (maxAngle < bufferAngle) {
      maxAngle = bufferAngle;
    }
  }
  double minInitDuration = maxAngle / avgRadPSec;
  printf("\nTime for (re-)initialization: %f", minInitDuration);

  // Check for Elbow Joint Angle Restriction before driving the robot to the
  // default position.
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  int elbowJointIndex = 2;
  double innerAngleElbow =
      M_PI - abs(ik.qIKSolArr[ik.ikSolNr * 6 + elbowJointIndex]);
  if (1 == violatesJointRestriction(robRestrRef, innerAngleElbow)) {
    printf(
        "\nElbow Joint Restriction Violation. Min Value: %f°, MaxValue %f°, "
        "desiredVal: %f°",
        robRestrRef.innerElbowMin / M_PI * 180,
        robRestrRef.innerElbowMax / M_PI * 180,
        innerAngleElbow / M_PI * 180);
    return -1;
  }

  // Set the default/start and goal position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // initialize references
  lastPositionRef = defaultRobotPositionRef;
  std::vector<double> currentGoal = defaultRobotPositionRef;
  std::vector<double>& currentGoalRef = currentGoal;

  // set the default and goal matrices
  Eigen::Matrix<double, 6, 1> defaultPosMat;  // Matrix< _Scalar, _Rows, _Cols >
  defaultPosMat << defaultRobotPositionRef.at(0), defaultRobotPositionRef.at(1),
      defaultRobotPositionRef.at(2), defaultRobotPositionRef.at(3),
      defaultRobotPositionRef.at(4), defaultRobotPositionRef.at(5);

  Eigen::Matrix<double, 6, 1> goalvectordofd;
  goalvectordofd << ik.qIKSolArr[ik.ikSolNr * 6 + 0],
      ik.qIKSolArr[ik.ikSolNr * 6 + 1], ik.qIKSolArr[ik.ikSolNr * 6 + 2],
      ik.qIKSolArr[ik.ikSolNr * 6 + 3], ik.qIKSolArr[ik.ikSolNr * 6 + 4],
      ik.qIKSolArr[ik.ikSolNr * 6 + 5];
  std::cout << "goal: " << goalvectordofd << std::endl;

  printf("\nvectordofd initialized");
  QVector<Tum::VectorDOFd> interpolationvvectordofd;
  ros::Time initialTime;
  ros::Duration passedTime;
  ros::Time endTime;

  initialTime = ros::Time::now();
  int numberPublishes = (int)floor(minInitDuration / pubIntf.minPubRate);
  printf("\nminInitDuration in secs : %f", minInitDuration);
  printf("\ninitialTime in secs : %f", initialTime.toSec());
  printf("\nendTime in secs : %f", endTime.toSec());
  printf("\nnumber Publishes : %d", numberPublishes);
  int supercounter = 0;
  // Calculate smooth trajectory to defaultRobotPositionRef and publish it to
  // robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i <= numberPublishes;
       ++i) {  // publish one more time to publish goalPosition
    if (nodeRef.ok()) {
      passedTime = ros::Time::now() - initialTime;
      interpolationvvectordofd = Tum::Tools::MathTools::getJointPVT5(
          defaultPosMat, goalvectordofd, passedTime.toSec(), minInitDuration);

      std::copy_n(&(interpolationvvectordofd[0](0)),
                  interpolationvvectordofd[0].size(), currentGoal.begin());

      // Publish Data to Robot
      // lastPositionRef gets updated by checkAndPublishDesJointValues()
      ros::Time begin = ros::Time::now();
      ros::Duration elapsedTime = begin - begin;
      checkAndPublishDesJointValues(nodeRef, currentGoalRef, lastPositionRef,
                                    pubIntf, pubIntf.minPubRate, robRestrRef);
      // check if published goal is already reached. necessary because,
      // checkAndPublishDesJointValues needs more time than pubIntf.minPubRate
      int equalJoints = 0;
      for (int i = 0; i < lastPositionRef.size(); ++i) {
        if (lastPositionRef.at(i) == goalvectordofd[i]) {
          equalJoints++;
        }
      }
      if (equalJoints == 6) {
        printf(
            "\nDefault position has been reached. You can start moving your "
            "slowly now.");
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
  sensor_msgs::JointState robotPositionMsg =
      *(ros::topic::waitForMessage<sensor_msgs::JointState>(robotStateTopic,
                                                            ros::Duration(10)));
  robotPosition = robotPositionMsg.position;

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
  double framesPerSecPercNeuron = 20;

  double timeForMainLoop = 0.0006;  // max Time for calculations in main loop
                                    // without checkAndPublishDesJointValues()
  double totalTrajPubTime = (1 / framesPerSecPercNeuron) -
                            timeForMainLoop;  // total time of publishing
                                              // interval / calculating /
                                              // interpolating the trajectory

  // Declare interface to UR# / publisher for joint values
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // all ur# values, including DH (Denavit-Hartenberg) values are stored on the
  // parameter server
  // on $ rosparam get /robot_description
  struct pubIntf pubIntf;  // Create interface for publishing joint values to
                           // controller/robot
  pubIntf.minPubRate = 0.008;  // Rate at which joint values are published
  pubIntf.publishToController = 1;
  nodeRef.getParam("/publishToController", pubIntf.publishToController);
  if (pubIntf.publishToController)
    pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
        "/joint_desired_cmd",
        1000);  // publishes to the tum_ics_ur_robot_controller
  else {
    // pubIntf.jointPosPublisher =
    // node.advertise<sensor_msgs::JointState>("/follow_hand/joint_states_ur5",
    // 1000); // publishes to "$roslaunch bringRobot demo.launch". Used when 
	// working without a controller
    pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
        "/ur10_arm_joint_states",
        1000);  // publishes directly to robot simulation; passes the controller
  }

  // Initialize message to publish joint states
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  pubIntf.jointMsg.header.stamp = ros::Time::now();

  pubIntf.jointMsg.header.frame_id = "";

  std::vector<std::string> jointNames = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  pubIntf.jointMsg.name = jointNames;
  pubIntf.jointMsg.velocity = {};
  pubIntf.jointMsg.effort = {};
  struct pubIntf& pubIntfRef = pubIntf;

  // Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  // Interface to perc.neuron tf data
                                          // created by perc neuron tf
                                          // broadcaster
  tfPNIntf.followLeftHand = 1; // default follows Right Hand
  tfPNIntf.transBaseLinkRightHand = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, 0, 0));  // transform from base_link to RightHand directly

  tfPNIntf.tfBroadcaster.sendTransform(
      tf::StampedTransform(tfPNIntf.transBaseLinkRightHand, ros::Time::now(),
                           "/base_link", "/baseLinkToHand"));

  // create string array with bodyJointsRightHand
  tfPNIntf.bodyJointsRightHand = {
      "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck",
      "RightShoulder", "RightArm", "RightForeArm", "RightHand"};
  tfPNIntf.bodyJointsLeftHand = {
      "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck",
      "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand"};

  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;

  // Set physical restrictions to Robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  struct robotRestrictions robRestr;
  robRestr.checkElbowRestriction;

  double innerElbowMinDeg = 20;  // M_PI/9;
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
  while (success == -1) {
    success = getCurrentRobotPosition(defaultRobotPositionRef);
    ros::Duration(1).sleep();
  }

  // Initialize variables for calculation of inverse kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<double> lastPosition =
      defaultRobotPositionRef;  // last to roboter published joint position
  std::vector<double>& lastPositionRef = lastPosition;
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
                                    lastPositionRef, defaultRobotPositionRef,
                                    robRestrRef);
    if(success == -1){
      printf("\nInitialization process failed. Trying again in 1sec.\n");
      ros::Duration(1).sleep();
    }
  }
  printf("\n checkElbowRestr. for initialization: %d", robRestrRef.checkElbowRestriction);
  robRestr.checkElbowRestriction = 1;

  // Start the loop to get position from Perception Neuron and publish it to the
  // robot / controller
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nYou can start moving slowly now.");
  ros::Time begin = ros::Time::now();
  ros::Duration elapsedTime = begin - begin;
  ros::Rate rate(framesPerSecPercNeuron);
  while (node.ok()) {
    begin = ros::Time::now();

    // Get Transform / Vector from shoulder_link of ur# to RightHand of
    // PercNeuron
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success = -1;
    while (success == -1) {
      success = tfPNIntfRef.getTransBaseLinkRightHand();
    }

    // Get Goal Joint Positions from transBaseLinkRightHand with IK Calculation
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ik.calcqIKSolVec(tfPNIntfRef);  // sets ik.qIKSolVec

    // Interpolate points from rate framesPerSecPercNeuron to max 125HZ = 1
    // frame / 8ms and publish them to the controller / robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    elapsedTime = ros::Time::now() - begin;

    char reinitializeResponse;
    success = checkAndPublishDesJointValues(nodeRef, ik.qIKSolVec,
                                                lastPositionRef, pubIntfRef,
                                                totalTrajPubTime, robRestrRef);

    if (-1 == success) {  // robot desired value couldn't be published
      printf("\nCurrent Robot Position / lastPosition: ");
      printVector(lastPositionRef, 0);
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
        // now defaultRobotPosition is lastPosition
        robRestr.checkElbowRestriction = 0;  // Elbow restrictions are 
				// only checked at the beginning of
                // initialization process if it's possible to reach goal
                // position, because robot default position can be out of the
                // restriction boundaries
        initializeRobotToHand(nodeRef, pubIntfRef, tfPNIntfRef, ikRef,
                              lastPositionRef, lastPositionRef, robRestrRef);
        robRestr.checkElbowRestriction = 1;
      }
    }
    loopCounter++;
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
