#include "mocap_optitrack/mocap_datapackets.h"

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
using namespace std;
int numberOfBodies;
float distance;
Marker world_frame;

RigidBody::RigidBody() 
  : NumberOfMarkers(0), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

Marker RigidBody::cross(Marker v1, Marker v2)
{
  Marker c;
  c.positionX  = (v1.positionY*v2.positionZ) - (v1.positionZ*v2.positionY);
  c.positionY = -((v1.positionX*v2.positionZ) - (v1.positionZ*v2.positionX));
  c.positionZ = (v1.positionX*v2.positionY) - (v1.positionY*v2.positionX);
  return c;
}

Marker RigidBody::translate(Marker a,Marker b)
{
  Marker c;
  c.positionX=a.positionX - b.positionX;
  c.positionY=a.positionY - b.positionY;
  c.positionZ=a.positionZ - b.positionZ;
  return c;
};

float RigidBody::dot(Marker v1, Marker v2)
{
  return v1.positionX*v2.positionX + v1.positionY*v2.positionY + v1.positionZ*v2.positionZ;
}


vec4 RigidBody::quat(Marker u, Marker v)
{
  Marker x=u;
  Marker y_0=v;
  Marker z= cross(x,y_0);
  Marker y=cross(z,x);
  x=RigidBody::norm3(x);
  y=RigidBody::norm3(y);
  z=RigidBody::norm3(z);

  vec4 q;
  float t = x.positionX + y.positionY + z.positionZ;
  if (t > 0){
    q.w= sqrt((1 + x.positionX + y.positionY + z.positionZ)) /2;
    q.x = (z.positionY - y.positionZ)/( 4 *q.w);
    q.y = (x.positionZ - z.positionX)/( 4 *q.w);
    q.z = (y.positionX - x.positionY)/( 4 *q.w);}
  else{
    float s = 0.5/sqrt(1+x.positionX - y.positionY - z.positionZ);
    q.x = 0.5*s;
    q.y = (x.positionY + y.positionX)*s;
    q.z = (z.positionX - x.positionZ)*s;
  }
  return RigidBody::norm(q);
}

const geometry_msgs::PoseStamped RigidBody::get_ros_pose()
{
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.header.stamp = ros::Time::now();
  // y & z axes are swapped in the Optitrack coordinate system
  ros_pose.pose.position.x = pose.position.x;
  ros_pose.pose.position.y = -pose.position.z;
  ros_pose.pose.position.z = pose.position.y;

  ros_pose.pose.orientation.x = pose.orientation.x;
  ros_pose.pose.orientation.y = -pose.orientation.z;
  ros_pose.pose.orientation.z = pose.orientation.y;
  ros_pose.pose.orientation.w = pose.orientation.w;

  return ros_pose;
}

bool RigidBody::has_data()
{
  static const char zero[sizeof(pose)] = { 0 };
  return memcmp(zero, (char*) &pose, sizeof(pose));
}

const geometry_msgs::PointStamped RigidBody::get_ros_marker()
{
  geometry_msgs::PointStamped ros_marker;
  ros_marker.header.stamp = ros::Time::now();
  /*ros_marker.point.x = marker[0].positionX;
  ros_marker.point.y = marker[0].positionY;
  ros_marker.point.z = marker[0].positionZ;*/

  ros_marker.point.x = distances[0]; // 12
  ros_marker.point.y = distances[1]; // 23
  ros_marker.point.z = distances[2]; // 13

  return ros_marker;
}

Marker ModelFrame::pushBackDistanceVector(int a, int b)
{
  Marker marker;
  marker.positionX = this->otherMarkers[b].positionX - this->otherMarkers[a].positionX;
  marker.positionY = this->otherMarkers[b].positionY - this->otherMarkers[a].positionY;
  marker.positionZ = this->otherMarkers[b].positionZ - this->otherMarkers[a].positionZ;
//  distance_vectors.push_back(marker);
  return marker;
}

void RigidBody::pushBackDistanceVector(int a, int b)
{
  Marker marker;
  marker.positionX = this->marker[b].positionX - this->marker[a].positionX;
  marker.positionY = this->marker[b].positionY - this->marker[a].positionY;
  marker.positionZ = this->marker[b].positionZ - this->marker[a].positionZ;
  distance_vectors.push_back(marker);
}

bool ModelFrame::marker_in_place()
{
  bool temp=false;
  for (int l = 0; l < numOtherMarkers; l++)
  {
    if (otherMarkers[l].positionX <= -1.4 && otherMarkers[l].positionX >= -1.6 && otherMarkers[l].positionZ >=-0.1)
    {
      temp=true;
      for (int j = 0; j < numOtherMarkers; j++)
      {
        if (j == l)
          continue;
        if (float distance = Marker::norm(pushBackDistanceVector(l,j))*1000 <300)
        {
          temp=false;
        }
      }
    }
  }
  return temp;
}

void ModelFrame::findBodies(const std::vector<Object>& known_objects)
{
  output.clear();
  float threshold = 150;
  int i=0;
  float distance = 0;
  distance_sort={};
  distance_unsort={};

  while (marker_index.size()>2)
  { int hits_inside = 0;
    vector<int> index;
    for (int j=i+1; j < marker_index.size(); ++j)
    {
      Marker distance_vector=pushBackDistanceVector(marker_index[i], marker_index[j]);
      distance=Marker::norm(distance_vector) * 1000;
      if (distance < threshold)
      {
        if(hits_inside ==1)
        {
          output.push_back(marker_index[j]);
          distance_unsort.push_back(distance);
          index.push_back(j);
          ++hits_inside;
          break;
        }
        output.push_back(marker_index[i]);
        ++hits_inside;
        output.push_back(marker_index[j]);
        index.push_back(j);
        distance_unsort.push_back(distance);
      }
    }
    if (hits_inside==2)
    {
      distance_unsort.push_back(Marker::norm(pushBackDistanceVector(marker_index[index[0]], marker_index[index[1]])) * 1000);
      for (int k=index.size()-1;k>-1;--k)
      {
        marker_index.erase(marker_index.begin()+index[k]);
      }
    }
    marker_index.erase(marker_index.begin());
  }
}

void ModelFrame::findBodies_c(const std::vector<Object>& known_objects)
{
  output.clear();
  float threshold = 150;
//  int found=0;
  int i=0;
  float min_dist=150;
  float mid_dist=150;
  float mid_dist_temp=0;
  while (marker_index.size()>2)
  { int hits_inside = 0;
    vector<int> index;
    vector <int> temp = {0,0,0};
    for (int j=i+1; j < marker_index.size(); ++j)
    {
      float distance=Marker::norm(pushBackDistanceVector(marker_index[i], marker_index[j])) * 1000;
      if (distance < threshold)
      {//ROS_INFO_STREAM(" " << distance);
        ++hits_inside;
        temp[2]=marker_index[j];
        if (distance < mid_dist)
        {
          mid_dist_temp=distance;
          mid_dist = mid_dist_temp;
          if (mid_dist < min_dist)
          {
            mid_dist=min_dist; //
            min_dist=mid_dist_temp;
            temp={marker_index[i],marker_index[j],temp[1]};
          }
        }
        index.push_back(j);
      }
    }
    if (hits_inside==2)
    {
//      ++found;
      float max_distance_temp=Marker::norm(pushBackDistanceVector(temp[1], temp[2])) * 1000;
      if (max_distance_temp <mid_dist)
        temp ={temp[1],temp[0],temp[2]};
      if (max_distance_temp < min_dist)
        temp ={temp[0],temp[2],temp[1]};
      for (int k=index.size()-1;k>-1;--k)
        marker_index.erase(marker_index.begin()+index[k]);
      for (int l=0; l<temp.size();++l)
        output.push_back(temp[l]);
    }
    min_dist=150; mid_dist=150;
    marker_index.erase(marker_index.begin());
  }
}

void ModelFrame::findOrigin(int m)
{
  distance_sort = distance_unsort;
  rigidBodies[m].marker[0] = otherMarkers[output[0 +3*m]];
  rigidBodies[m].marker[1] = otherMarkers[output[1 +3*m]];
  rigidBodies[m].marker[2] = otherMarkers[output[2 +3*m]];

  if(distance_sort[2 +3*m] < distance_sort[1 +3*m])
  {
    rigidBodies[m].marker[0] = otherMarkers[output[1 +3*m]];
    rigidBodies[m].marker[1] = otherMarkers[output[0 +3*m]];
    rigidBodies[m].marker[2] = otherMarkers[output[2 +3*m]];
    distance_sort[1 + 3*m] = distance_unsort[2 + 3*m];
    distance_sort[2 + 3*m] = distance_unsort[1 + 3*m];
  }
  if(distance_sort[1 +3*m] < distance_sort[0 +3*m])
  {
    rigidBodies[m].marker[0] = otherMarkers[output[0 +3*m]];
    rigidBodies[m].marker[1] = otherMarkers[output[2 +3*m]];
    rigidBodies[m].marker[2] = otherMarkers[output[1 +3*m]];
    distance_sort[0 + 3*m] = distance_unsort[2 + 3*m];
    distance_sort[1 + 3*m] = distance_unsort[0 + 3*m];
  }
  if(distance_sort[2 +3*m] < distance_sort[1 +3*m])
  {
    rigidBodies[m].marker[0] = otherMarkers[output[1 +3*m]];
    rigidBodies[m].marker[1] = otherMarkers[output[0 +3*m]];
    rigidBodies[m].marker[2] = otherMarkers[output[2 +3*m]];
    distance_sort[1 + 3*m] = distance_unsort[1 + 3*m];
    distance_sort[2 + 3*m] = distance_unsort[0 + 3*m];
  }
}

void RigidBody::setNameFromKnownObjects(const std::vector<Object>& known_objects)
{
  float dist_threshold = 5;
  int min_hits = 3;

  std::vector<int> candidate_idx;
  std::vector<double> candidate_meandist;

  for (int i = 0; i < known_objects.size(); i++)
  {
    int hits_inside = 0;

    double distsum = 0;

    std::vector<int> object_distances = known_objects[i].distances_mm; // we make a copy in order to remove found values from the list of known candidates

    for (int j=0; j< distances.size(); ++j)
    {
      // check if "distance" is contained in distances from the known object
      // we want to remove the found distance from the set of reference distances in order to allow the existence of multiple reference distances with the same length.
      // but since we have some threshold that we check (dist_threshold) we want to make sure to delete the correct distance!
      // hence we query all distances and only compare the smallest one with our threshold!
      std::vector<float> dist_cont; // distance, idx
      for (int k=0; k<object_distances.size(); ++k)
        dist_cont.push_back(std::abs( (float)object_distances[k]-distances[j]));

      // get minimum dist
      auto min_dist_it = std::min_element(dist_cont.begin(), dist_cont.end());
      if (min_dist_it == dist_cont.end())
        continue;

      if (*min_dist_it < dist_threshold)
      {
        // hit!!
        ++hits_inside;
        distsum += *min_dist_it;
        // remove candidate from the current reference distances
        object_distances.erase(object_distances.begin()+std::distance(dist_cont.begin(), min_dist_it));
      }
    }

    if (hits_inside >= min_hits && hits_inside >= known_objects[i].distances_mm.size())
    {
      candidate_idx.push_back(i);
      candidate_meandist.push_back( distsum / double(hits_inside) );
    }
  }

  // find object that minimizes mean distance
  std::vector<double>::iterator min_iter = std::min_element(candidate_meandist.begin(), candidate_meandist.end());
  if (min_iter!= candidate_meandist.end())
  {
    int min_idx = std::distance(candidate_meandist.begin(), min_iter);
    candidate_meandist.data();
    int list_idx = candidate_idx[min_idx];
    this->name = known_objects[list_idx].name;
    for (int i=0;i<this->NumberOfMarkers;++i)
    {
      geometry_msgs::Point32 point;
      point.x=this->marker[i].positionX;
      point.y=-this->marker[i].positionZ;
      point.z=this->marker[i].positionY;
      this->footprint.push_back(point);
    }//footprint_init
  }

  //     if (hits_inside > 0 && hits_inside >= Liste[i].Abstand.size())
  //     {
  //       ROS_INFO_STREAM(Liste[i].name << " found.");

  //     }
  
  //   if (this->name == "unbekannt")
  //     ROS_INFO("Rigid Body %i unbekannt", this->ID);
}

const geometry_msgs::PolygonStamped RigidBody::get_ros_footprint()
{
  geometry_msgs::PolygonStamped ros_footprint;
  ros_footprint.header.stamp = ros::Time::now();
  ros_footprint.polygon.points.reserve(footprint.size());

  for (int i = 0; i < footprint.size(); i++)
  {
    ros_footprint.polygon.points.push_back(footprint.at(i));
  }
  
  return ros_footprint;
}

Object::Object(const vector< int >& dists_mm, const string& str, const vector< geometry_msgs::Point32 >& points)
{
  distances_mm = dists_mm;
  name = str;
  footprint = points;
}

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markerSets(0), otherMarkers(0), rigidBodies(0),
    numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0),
    latency(0.0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markerSets;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length) 
  : packet(packet), length(length), frameNumber(0)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse(const vector <int>  numMarkers, const vector<float> dist_sort)
{
  seek(4);
  // parse frame number
  read_and_seek(frameNumber);

  // count number of packetsets
  read_and_seek(model.numMarkerSets);
  model.markerSets = new MarkerSet[model.numMarkerSets];
  ROS_DEBUG("Number of marker sets: %d\n", model.numMarkerSets);
  world_frame.positionX=1;
  world_frame.positionY=0;
  world_frame.positionZ=0;

  for (int i = 0; i < model.numMarkerSets; i++)
  {
    strcpy(model.markerSets[i].name, packet);
    seek(strlen(model.markerSets[i].name) + 1);

    ROS_DEBUG("Parsing marker set named: %s\n", model.markerSets[i].name);

    // read number of markers that belong to the model
    read_and_seek(model.markerSets[i].numMarkers);
    ROS_DEBUG("Number of markers in set: %d\n", model.markerSets[i].numMarkers);

    model.markerSets[i].markers = new Marker[model.markerSets[i].numMarkers];
    for (int k = 0; k < model.markerSets[i].numMarkers; k++)
    {
      // read marker positions
      read_and_seek(model.markerSets[i].markers[k]);
    }
  }

  // read number of 'other' markers (cf. NatNet specs)
  read_and_seek(model.numOtherMarkers);
  model.otherMarkers = new Marker[model.numOtherMarkers];
  ROS_DEBUG("Number of markers not in sets: %d\n", model.numOtherMarkers);
  for (int l = 0; l < model.numOtherMarkers; l++)
  {
    // read positions of 'other' markers
    read_and_seek(model.otherMarkers[l]);
    ROS_INFO_STREAM("marker " << l);
    model.marker_index.push_back(l);
    //if l>0
    //model.pushBackDistanceVector(l, l-1);
    //end
    //model.pushBackDistanceVector(l,0);
    //model.distances.push_back(Marker::norm(model.distance_vectors[i]) * 1000);
  }

  model.output = numMarkers;
  model.distance_sort = dist_sort;

  if (numMarkers.size() != model.numOtherMarkers)
  {
    model.findBodies(model.known_objects);
  }

  numberOfBodies=std::trunc(model.output.size()/3);
  model.numRigidBodies=numberOfBodies;
  model.rigidBodies = new RigidBody[model.numRigidBodies];
  model.marker_in_position=model.marker_in_place();

  for (int m = 0; m < model.numRigidBodies; m++)
  {
    model.rigidBodies[m].NumberOfMarkers=3;
    model.rigidBodies[m].marker = new Marker [model.rigidBodies[m].NumberOfMarkers];
    if(numMarkers.size() != model.numOtherMarkers)
      model.findOrigin(m);
    else
    {
      model.rigidBodies[m].marker[0]=model.otherMarkers[model.output[0 + 3*m]];
      model.rigidBodies[m].marker[1]=model.otherMarkers[model.output[1 + 3*m]];
      model.rigidBodies[m].marker[2]=model.otherMarkers[model.output[2 + 3*m]];
    }

    model.rigidBodies[m].ID=m;
    model.rigidBodies[m].pose.position.x=model.rigidBodies[m].marker[0].positionX;
    model.rigidBodies[m].pose.position.y=model.rigidBodies[m].marker[0].positionY;
    model.rigidBodies[m].pose.position.z=model.rigidBodies[m].marker[0].positionZ;
    vec4 quat = model.rigidBodies[m].quat(model.rigidBodies[m].translate(model.rigidBodies[m].marker[1],model.rigidBodies[m].marker[0]),model.rigidBodies[m].translate(model.rigidBodies[m].marker[2],model.rigidBodies[m].marker[0]));
    model.rigidBodies[m].pose.orientation.w=quat.w;
    model.rigidBodies[m].pose.orientation.x=quat.x;
    model.rigidBodies[m].pose.orientation.y=quat.y;
    model.rigidBodies[m].pose.orientation.z=quat.z;

    if (model.rigidBodies[m].NumberOfMarkers > 0)
    {
     // Verbindungsvektoren
//      model.rigidBodies[m].distance_vectors.clear();
      model.rigidBodies[m].distances.clear();
//      for (int i_marker=0; i_marker < model.rigidBodies[m].NumberOfMarkers; ++i_marker)
//      {
//        for (int j_marker=i_marker+1; j_marker < model.rigidBodies[m].NumberOfMarkers; ++j_marker)
//        {
//          if (i_marker == j_marker)
//            continue;
//          model.rigidBodies[m].pushBackDistanceVector(i_marker, j_marker);
//        }
//      }

//      // AbstÃ¤nde
//      for (int i = 0; i < model.rigidBodies[m].distance_vectors.size(); i++)
//      {
//        model.rigidBodies[m].distances.push_back(Marker::norm(model.rigidBodies[m].distance_vectors[i]) * 1000); // [mm]
//      }
      for (int i = 0; i < model.rigidBodies[m].NumberOfMarkers; i++)
      {
        model.rigidBodies[m].distances.push_back(model.distance_sort[i + 3*m]); // [mm]
      }

      model.rigidBodies[m].setNameFromKnownObjects(model.known_objects);
      ROS_INFO_STREAM("body " << m << ": " << model.rigidBodies[m].name);
    }

    // skip mean marker error
    seek(sizeof(float));

    seek(2); // http://answers.ros.org/question/217341/multiple-rigid-bodies-crash-optitrack-node/
  }

  // TODO: read skeletons
  int numSkeletons = 0;
  read_and_seek(numSkeletons);

  // get latency
  read_and_seek(model.latency);
}
