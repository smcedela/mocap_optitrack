/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, University of Bonn, Computer Science Institute VI
 *  Author: Kathrin Gräve, 01/2011
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>

#ifndef __MOCAP_DATAPACKETS_H__
#define __MOCAP_DATAPACKETS_H__

#include <sys/types.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

using namespace std;


template <typename T, typename Total, int N>
class Moving_Average
{
  public:
    Moving_Average()
      : num_samples_(0), total_(0)
    { }

    void operator()(T sample)
    {
        if (num_samples_ < N)
        {
            samples_[num_samples_++] = sample;
            total_ += sample;
        }
        else
        {
            T& oldest = samples_[num_samples_++ % N];
            total_ += sample - oldest;
            oldest = sample;
        }
    }

    operator double() const { return total_ / std::min(num_samples_, N); }
    
    double getEstimate() const { return total_ / std::min(num_samples_, N); }

  private:
    T samples_[N];
    int num_samples_;
    Total total_;
};



/// \brief Data object holding the position of a single mocap marker in 3d space
class Marker
{
  public:
    float positionX;
    float positionY;
    float positionZ;
    
    static float norm(const Marker& Vektor)
    {
      return std::sqrt(Vektor.positionX*Vektor.positionX + Vektor.positionY*Vektor.positionY + Vektor.positionZ*Vektor.positionZ);
    } 
};

class Pose
{
  public:
    struct {
      float x;
      float y;
      float z;
    } position;
    struct {
      float x;
      float y;
      float z;
      float w;
    } orientation;
};

class vec4
{
  public:
      float w;
      float x;
      float y;
      float z;
};

class Object
{
  public:
    Object(const std::vector<int>& dists_mm, const std::string& str, const std::vector<geometry_msgs::Point32>& points);

    std::vector<int> distances_mm;
    std::string name;
    std::vector<geometry_msgs::Point32> footprint;
    
};

/// \brief Data object holding information about a single rigid body within a mocap skeleton
class RigidBody
{
  public:
    RigidBody();
    ~RigidBody();

    int ID;
    
    Pose pose;
    Marker cross(Marker u, Marker v);
    Marker translate(Marker a , Marker b);
    float dot(Marker a, Marker b);
    vec4 quat(Marker a, Marker b);
    static vec4 norm(const vec4& a){
	float b=std::sqrt(a.w*a.w + a.x*a.x + a.y*a.y + a.z*a.z);
	vec4 q;
	q.w=a.w/b;
	q.x=a.x/b;
	q.y=a.y/b;
	q.z=a.z/b;
	return q;
    };

    int NumberOfMarkers;
    Marker *marker;

    const geometry_msgs::PoseStamped get_ros_pose();
    bool has_data();

    const geometry_msgs::PointStamped get_ros_marker();
    std::vector<Marker> distance_vectors;
    void pushBackDistanceVector(int a, int b);
    std::vector<float> distances;

    std::string name = "unbekannt";
    void setNameFromKnownObjects(const std::vector< Object >& known_objects);
    const geometry_msgs::PolygonStamped get_ros_footprint();
    vector<geometry_msgs::Point32> footprint;
    

};

class RigidBodyOdomHelper
{
public:
    
    RigidBodyOdomHelper() : rigidbody_name(""), last_time(ros::Time(0)), last_x(0), last_y(0), last_theta(0), first_values(true) 
    {}
   
  
    std::string rigidbody_name;
  
    Moving_Average<double, double, 10> movav_vel;
    Moving_Average<double, double, 10> movav_omega;
    ros::Time last_time;
    double last_x;
    double last_y;
    double last_theta;
    bool first_values;
};

/// \brief Data object describing a single tracked model
class ModelDescription
{
  public:
    ModelDescription();
    ~ModelDescription();

    string name;
    int numMarkers;
    string *markerNames;
};

class MarkerSet
{
  public:
    MarkerSet() : numMarkers(0), markers(0) {}
    ~MarkerSet() { delete[] markers; }
    char name[256];
    int numMarkers;
    Marker *markers;
};

/// \brief Data object holding poses of a tracked model's components
class ModelFrame
{
  public:
    ModelFrame();
    ~ModelFrame();

    MarkerSet *markerSets;
    Marker *otherMarkers;
    RigidBody *rigidBodies;
    RigidBody *Bodies;

    int numMarkerSets;
    int numOtherMarkers;
    int numRigidBodies;
    int numBodies;

    float latency;

    std::vector<Object> known_objects;
};

/// \brief Parser for a NatNet data frame packet
class MoCapDataFormat
{
  public:
    MoCapDataFormat(const char *packet, unsigned short length);
    ~MoCapDataFormat();

    /// \brief Parses a NatNet data frame packet as it is streamed by the Arena software according to the descriptions in the NatNet SDK v1.4
    void parse ();

    const char *packet;
    unsigned short length;

    int frameNumber;
    ModelFrame model;

  private:
    void seek(size_t count);
    template <typename T> void read_and_seek(T& target)
    {
        target = *((T*) packet);
        seek(sizeof(T));
    }
};

#endif  /*__MOCAP_DATAPACKETS_H__*/
