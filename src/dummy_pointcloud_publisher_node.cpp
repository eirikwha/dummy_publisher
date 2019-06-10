//
// Modified by eirik on 08.06.19.
//

// list pointclouds in folder

// publish by keypress (see zivid hdr republisher)

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**
\author Radu Bogdan Rusu
@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data)
 files from disk and publishes them as ROS messages on the network.
 **/

#include "dirent.h"

// STL
#include <chrono>
#include <thread>

// ROS core
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/publisher.h"

using namespace std;

void wait()
{
    cout << "-------------------------------------------------" << endl;
    cout << "Press ENTER publish a pcd to pointcloud2" << endl;
    cout << "-------------------------------------------------" << endl;
    cin.ignore( numeric_limits <streamsize> ::max(), '\n' );
}

void listFiles(const char *directoryPath, const char *fileType, vector<string> &list) {

    DIR *d;
    struct dirent *dir;
    d = opendir(directoryPath);
    int ret = 0;
    int n = 0;
    char *tok1, *tok2;

    if (d) {
        while ((dir = readdir(d)) != nullptr) {
            tok1 = strtok(dir->d_name, ".");
            tok2 = strtok(nullptr, ".");
                if (tok1 != nullptr) {
                    ret = strcmp(tok2, fileType);
                    if (ret == 0) {
                        string name = string(directoryPath) + dir->d_name + string(".") + fileType;
                        list.emplace_back(name);
                        ++n;
                    }
                }
            }
        closedir(d);
    }
    std::sort(list.begin(), list.end());
}

/* ---[ */
int main (int argc, char** argv)
{
    if (argc < 1)
    {
        std::cerr << "Syntax is: " << argv[0] << "/pcd/file/directory" << std::endl;
        return (-1);
    }

    ros::init (argc, argv, "pcd_to_pointcloud");
    ros::NodeHandle nh_;

    string cloudTopic;

    if (!nh_.getParam("pointcloud2_topic", cloudTopic))
    {
        cloudTopic = "cloud_pcd";
        ROS_ERROR("Dummy PCD to Pointcloud2 publisher: Could not parse topic name. Setting default.");
    }

    vector<string> point_cloud_list;
    listFiles(argv[1], "pcd", point_cloud_list);

    ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(cloudTopic, 1);

    wait();

    while(ros::ok()) {
        for (int i = 0; i < point_cloud_list.size(); i++) {

            cout << "-------------------------------------------------" << endl;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(point_cloud_list[i], *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
            }

            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            pub.publish(output);

            wait();

        }
    }

    return (0);
}
/* ]--- */
