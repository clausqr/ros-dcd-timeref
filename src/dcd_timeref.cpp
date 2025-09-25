/**
 * @file timeref_node.cpp
 * @brief ROS node for publishing time reference from PPS devices
 * 
 * This node reads PPS (Pulse Per Second) signals from a device and publishes
 * sensor_msgs/TimeReference messages for time synchronization across the system.
 * 
 * @author Senior Embedded Developer
 * @date 2024
 */

#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <sys/timepps.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <cassert>

/**
 * @brief Main function for the time reference publisher node
 * 
 * Reads PPS signals from specified device and publishes TimeReference messages.
 * Supports configurable parameters for device path, edge detection, and publishing behavior.
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit status (0 for success, 1 for failure)
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "timeref_node");
    ros::NodeHandle nh("~");
    
    // Parameter declarations with defaults
    std::string pps_device;
    std::string edge;
    std::string source;
    std::string frame_id;
    bool publish_both_edges;
    bool use_header_stamp_now;
    int queue_size;
    double rate_hz;
    double timeout_sec;
    
    // Read parameters from ROS parameter server
    nh.param<std::string>("pps_device", pps_device, "/dev/pps0");
    nh.param<std::string>("edge", edge, "assert");
    nh.param<std::string>("source", source, "PPS");
    nh.param<std::string>("frame_id", frame_id, "");
    nh.param<bool>("publish_both_edges", publish_both_edges, false);
    nh.param<bool>("use_header_stamp_now", use_header_stamp_now, true);
    nh.param<int>("queue_size", queue_size, 10);
    nh.param<double>("rate", rate_hz, 100.0);
    nh.param<double>("timeout_sec", timeout_sec, 1.0);
    
    // Validate parameters
    if (queue_size <= 0)
    {
        ROS_FATAL("queue_size must be positive, got %d", queue_size);
        return 1;
    }
    
    if (rate_hz <= 0.0)
    {
        ROS_FATAL("rate must be positive, got %f", rate_hz);
        return 1;
    }
    
    if (timeout_sec <= 0.0)
    {
        ROS_FATAL("timeout_sec must be positive, got %f", timeout_sec);
        return 1;
    }
    
    // Validate edge parameter
    int pps_mode = 0;
    if (edge == "assert")
    {
        pps_mode = PPS_CAPTUREASSERT;
    }
    else if (edge == "clear")
    {
        pps_mode = PPS_CAPTURECLEAR;
    }
    else if (edge == "both")
    {
        pps_mode = PPS_CAPTUREASSERT | PPS_CAPTURECLEAR;
    }
    else
    {
        ROS_FATAL("Invalid edge parameter: %s. Must be 'assert', 'clear', or 'both'", edge.c_str());
        return 1;
    }
    
    // Log configuration
    ROS_INFO("TimeRef Node Configuration:");
    ROS_INFO("  PPS Device: %s", pps_device.c_str());
    ROS_INFO("  Edge: %s", edge.c_str());
    ROS_INFO("  Source: %s", source.c_str());
    ROS_INFO("  Frame ID: %s", frame_id.empty() ? "(empty)" : frame_id.c_str());
    ROS_INFO("  Publish Both Edges: %s", publish_both_edges ? "true" : "false");
    ROS_INFO("  Use Header Stamp Now: %s", use_header_stamp_now ? "true" : "false");
    ROS_INFO("  Queue Size: %d", queue_size);
    ROS_INFO("  Rate: %.1f Hz", rate_hz);
    ROS_INFO("  Timeout: %.1f sec", timeout_sec);
    
    // Open PPS device
    int fd = open(pps_device.c_str(), O_RDWR);
    if (fd < 0)
    {
        ROS_FATAL("Cannot open PPS device %s: %s", pps_device.c_str(), strerror(errno));
        return 1;
    }
    
    // Create PPS handle
    pps_handle_t handle;
    if (time_pps_create(fd, &handle) < 0)
    {
        ROS_FATAL("time_pps_create failed: %s", strerror(errno));
        close(fd);
        return 1;
    }
    
    // Configure PPS parameters
    pps_params_t params;
    if (time_pps_getparams(handle, &params) < 0)
    {
        ROS_FATAL("time_pps_getparams failed: %s", strerror(errno));
        time_pps_destroy(handle);
        close(fd);
        return 1;
    }
    
    params.mode = pps_mode | PPS_TSFMT_TSPEC;
    if (time_pps_setparams(handle, &params) < 0)
    {
        ROS_FATAL("time_pps_setparams failed: %s", strerror(errno));
        time_pps_destroy(handle);
        close(fd);
        return 1;
    }
    
    // Create publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", queue_size);
    
    // Wait for subscribers (optional)
    ROS_INFO("Waiting for subscribers...");
    while (pub.getNumSubscribers() == 0 && ros::ok())
    {
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Subscriber(s) connected, starting PPS monitoring");
    
    // Main loop variables
    ros::Rate rate(rate_hz);
    pps_seq_t last_assert_seq = 0;
    pps_seq_t last_clear_seq = 0;
    struct timespec timeout = {static_cast<time_t>(timeout_sec), 0};
    
    ROS_INFO("Starting PPS monitoring loop");
    
    while (ros::ok())
    {
        pps_info_t info;
        int ret = time_pps_fetch(handle, PPS_TSFMT_TSPEC, &info, &timeout);
        
        if (ret < 0)
        {
            if (errno == ETIMEDOUT)
            {
                // Timeout is normal, continue
                ros::spinOnce();
                rate.sleep();
                continue;
            }
            else
            {
                ROS_WARN("time_pps_fetch failed: %s", strerror(errno));
                ros::spinOnce();
                rate.sleep();
                continue;
            }
        }
        
        bool should_publish = false;
        sensor_msgs::TimeReference msg;
        
        // Check for assert edge
        if ((pps_mode & PPS_CAPTUREASSERT) && 
            (info.assert_sequence != last_assert_seq))
        {
            last_assert_seq = info.assert_sequence;
            should_publish = true;
            
            msg.time_ref = ros::Time(info.assert_timestamp.tv_sec, 
                                     info.assert_timestamp.tv_nsec);
        }
        
        // Check for clear edge
        if ((pps_mode & PPS_CAPTURECLEAR) && 
            (info.clear_sequence != last_clear_seq))
        {
            last_clear_seq = info.clear_sequence;
            
            if (publish_both_edges || !should_publish)
            {
                should_publish = true;
                msg.time_ref = ros::Time(info.clear_timestamp.tv_sec, 
                                        info.clear_timestamp.tv_nsec);
            }
        }
        
        // Publish message if needed
        if (should_publish)
        {
            // Set header
            if (use_header_stamp_now)
            {
                msg.header.stamp = ros::Time::now();
            }
            else
            {
                msg.header.stamp = ros::Time(0);
            }
            
            msg.header.frame_id = frame_id;
            msg.source = source;
            
            pub.publish(msg);
            
            ROS_DEBUG("Published TimeReference: time_ref=%.6f, source=%s", 
                     msg.time_ref.toSec(), msg.source.c_str());
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    ROS_INFO("Shutting down timeref_node");
    time_pps_destroy(handle);
    close(fd);
    
    return 0;
}
