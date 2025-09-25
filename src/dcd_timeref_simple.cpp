/**
 * @file dcd_timeref_simple.cpp
 * @brief Simplified ROS node for time reference (without PPS dependencies)
 * 
 * This is a simplified version for testing compilation without PPS libraries.
 * The full version requires libpps-dev which may not be available in all environments.
 * 
 * @author Senior Embedded Developer
 * @date 2024
 */

#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <chrono>
#include <thread>

/**
 * @brief Simplified time reference publisher node
 * 
 * This version simulates PPS signals for testing purposes.
 * In production, replace with actual PPS device reading.
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit status (0 for success, 1 for failure)
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dcd_timeref");
    ros::NodeHandle nh("~");
    
    // Parameter declarations with defaults
    std::string source;
    std::string frame_id;
    bool use_header_stamp_now;
    int queue_size;
    double rate_hz;
    
    // Read parameters from ROS parameter server
    nh.param<std::string>("source", source, "SIMULATED_PPS");
    nh.param<std::string>("frame_id", frame_id, "");
    nh.param<bool>("use_header_stamp_now", use_header_stamp_now, true);
    nh.param<int>("queue_size", queue_size, 10);
    nh.param<double>("rate", rate_hz, 1.0);  // 1 Hz for simulation
    
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
    
    // Log configuration
    ROS_INFO("Simplified TimeRef Node Configuration:");
    ROS_INFO("  Source: %s", source.c_str());
    ROS_INFO("  Frame ID: %s", frame_id.empty() ? "(empty)" : frame_id.c_str());
    ROS_INFO("  Use Header Stamp Now: %s", use_header_stamp_now ? "true" : "false");
    ROS_INFO("  Queue Size: %d", queue_size);
    ROS_INFO("  Rate: %.1f Hz", rate_hz);
    
    // Create publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", queue_size);
    
    // Wait for subscribers (optional)
    ROS_INFO("Waiting for subscribers...");
    while (pub.getNumSubscribers() == 0 && ros::ok())
    {
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Subscriber(s) connected, starting time reference simulation");
    
    // Main loop variables
    ros::Rate rate(rate_hz);
    auto start_time = std::chrono::steady_clock::now();
    
    ROS_INFO("Starting time reference simulation loop");
    
    while (ros::ok())
    {
        // Simulate PPS signal (every second)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        
        if (elapsed.count() >= 1)
        {
            // Create and publish message
            sensor_msgs::TimeReference msg;
            
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
            msg.time_ref = ros::Time::now();  // Simulated PPS timestamp
            msg.source = source;
            
            pub.publish(msg);
            
            ROS_DEBUG("Published TimeReference: time_ref=%.6f, source=%s", 
                     msg.time_ref.toSec(), msg.source.c_str());
            
            // Reset timer
            start_time = now;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Shutting down dcd_timeref");
    return 0;
}
