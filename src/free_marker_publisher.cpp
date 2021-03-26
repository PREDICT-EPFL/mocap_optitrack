/*
 * Copyright (c) 2021, Joseph Lorenzetti
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <mocap_optitrack/free_marker_publisher.h>

#include "mocap_optitrack/PointArrayStamped.h"
#include <geometry_msgs/Point.h>
#include <vector>

namespace mocap_optitrack
{

namespace utilities
{
mocap_optitrack::PointArrayStamped getRosFreeMarkers(
  std::vector<LabeledMarker> const& markers, const Version& coordinatesVersion)
{
  mocap_optitrack::PointArrayStamped msg;
  for (auto& labeledMarker : markers)
  {
    // Only add unlabeled "free" markers
    if (labeledMarker.bUnlabeled)
    {
      // Create a standard point
      geometry_msgs::Point point;

      // TODO: add point.header info?
      if (coordinatesVersion < Version("2.0") && coordinatesVersion >= Version("1.7"))
      {
        // Motive 1.7+ and < Motive 2.0 coordinate system
        point.x = -labeledMarker.marker.x;
        point.y = labeledMarker.marker.z;
        point.z = labeledMarker.marker.y;
      }
      else
      {
        // y & z axes are swapped in the Optitrack coordinate system
        // Also compatible with versions > Motive 2.0
        point.x = labeledMarker.marker.x;
        point.y = -labeledMarker.marker.z;
        point.z = labeledMarker.marker.y;
      }

      // Add the point to the msg points array
      msg.points.push_back(point);
    }
  }
  
  return msg;
}

}  // namespace utilities

FreeMarkerPublisher::FreeMarkerPublisher(ros::NodeHandle &nh,
                                       Version const& natNetVersion)
{
  publisher = nh.advertise<mocap_optitrack::PointArrayStamped>("free_markers", 1000);

  // Motive 1.7+ uses a new coordinate system
  // natNetVersion = (natNetVersion >= Version("1.7"));
  coordinatesVersion = natNetVersion;
}

FreeMarkerPublisher::~FreeMarkerPublisher()
{
}

void FreeMarkerPublisher::publish(
  ros::Time const& time, std::vector<LabeledMarker> const& markers)
{
  // Create message
  mocap_optitrack::PointArrayStamped points = utilities::getRosFreeMarkers(
                                                markers, coordinatesVersion);

  points.header.stamp = time;
  publisher.publish(points);
}

}  // namespace mocap_optitrack
