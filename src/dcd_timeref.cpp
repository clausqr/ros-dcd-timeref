/**
 * @file timeref_node.cpp
 * @brief ROS node for publishing time reference from PPS devices
 * 
 * This node reads PPS (Pulse Per Second) signals from a device and publishes
 * sensor_msgs/TimeReference messages for time synchronization across the system.
 * 
 * @author clausqr
 * @date 2025
 */

#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <sys/timepps.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <cassert>
#include <signal.h>
#include <limits>

// Global variables for signal handling and cleanup
static int g_fd = -1;
static pps_handle_t g_handle = -1;  // Invalid handle value
static bool g_shutdown_requested = false;

/**
 * @brief Signal handler for graceful shutdown
 * 
 * Handles SIGINT and SIGTERM signals to ensure proper resource cleanup
 * 
 * @param sig Signal number
 */
void signal_handler(int sig)
{
    ROS_INFO("Received signal %d, shutting down gracefully...", sig);
    g_shutdown_requested = true;
    ros::shutdown();
}

/**
 * @brief Cleanup function for resources
 * 
 * Ensures all resources are properly released in the correct order
 */
void cleanup_resources()
{
    if (g_handle >= 0)
    {
        time_pps_destroy(g_handle);
        g_handle = -1;
        ROS_DEBUG("PPS handle destroyed");
    }
    
    if (g_fd >= 0)
    {
        close(g_fd);
        g_fd = -1;
        ROS_DEBUG("File descriptor closed");
    }
}

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
    
    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
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
    
    // Initialize resources to invalid values for proper cleanup tracking
    g_fd = -1;
    g_handle = -1;
    
    // Open PPS device
    g_fd = open(pps_device.c_str(), O_RDWR);
    if (g_fd < 0)
    {
        ROS_FATAL("Cannot open PPS device %s: %s", pps_device.c_str(), strerror(errno));
        return 1;
    }
    
    // Create PPS handle
    if (time_pps_create(g_fd, &g_handle) < 0)
    {
        ROS_FATAL("time_pps_create failed: %s", strerror(errno));
        close(g_fd);
        g_fd = -1;  // Mark as cleaned up
        return 1;
    }
    
    // Configure PPS parameters
    pps_params_t params;
    if (time_pps_getparams(g_handle, &params) < 0)
    {
        ROS_FATAL("time_pps_getparams failed: %s", strerror(errno));
        time_pps_destroy(g_handle);
        g_handle = -1;  // Mark as cleaned up
        close(g_fd);
        g_fd = -1;  // Mark as cleaned up
        return 1;
    }
    
    params.mode = pps_mode | PPS_TSFMT_TSPEC;
    if (time_pps_setparams(g_handle, &params) < 0)
    {
        ROS_FATAL("time_pps_setparams failed: %s", strerror(errno));
        time_pps_destroy(g_handle);
        g_handle = -1;  // Mark as cleaned up
        close(g_fd);
        g_fd = -1;  // Mark as cleaned up
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
    
    // Validate timeout bounds to prevent integer overflow
    if (timeout_sec <= 0.0)
    {
        ROS_FATAL("timeout_sec must be positive, got %f", timeout_sec);
        cleanup_resources();
        return 1;
    }
    
    // Use a practical limit for timeouts (1 hour = 3600 seconds)
    const double MAX_TIMEOUT_SEC = 3600.0;
    if (timeout_sec > MAX_TIMEOUT_SEC)
    {
        ROS_FATAL("timeout_sec too large: %f (max: %.1f seconds)", timeout_sec, MAX_TIMEOUT_SEC);
        cleanup_resources();
        return 1;
    }
    
    // Main loop variables
    ros::Rate rate(rate_hz);
    pps_seq_t last_assert_seq = 0;
    pps_seq_t last_clear_seq = 0;
    
    // Safe conversion with bounds checking
    time_t timeout_sec_int = static_cast<time_t>(timeout_sec);
    long timeout_nsec = static_cast<long>((timeout_sec - timeout_sec_int) * 1000000000L);
    
    // Ensure nanoseconds are within valid range [0, 999999999]
    if (timeout_nsec < 0) timeout_nsec = 0;
    if (timeout_nsec > 999999999L) timeout_nsec = 999999999L;
    
    struct timespec timeout = {timeout_sec_int, timeout_nsec};
    
    ROS_INFO("Starting PPS monitoring loop");
    
    while (ros::ok() && !g_shutdown_requested)
    {
        pps_info_t info;
        int ret = time_pps_fetch(g_handle, PPS_TSFMT_TSPEC, &info, &timeout);
        
        if (ret < 0)
        {
            if (errno == ETIMEDOUT)
            {
                // Timeout is normal, continue
                ros::spinOnce();
                rate.sleep();
                continue;
            }
            else if (errno == EACCES || errno == EPERM)
            {
                ROS_FATAL("Permission denied accessing PPS device: %s", strerror(errno));
                cleanup_resources();
                return 1;
            }
            else if (errno == ENODEV || errno == ENOENT)
            {
                ROS_FATAL("PPS device no longer available: %s", strerror(errno));
                cleanup_resources();
                return 1;
            }
            else if (errno == EINVAL)
            {
                ROS_FATAL("Invalid PPS operation: %s", strerror(errno));
                cleanup_resources();
                return 1;
            }
            else if (errno == EIO)
            {
                ROS_FATAL("I/O error on PPS device: %s", strerror(errno));
                cleanup_resources();
                return 1;
            }
            else
            {
                ROS_ERROR("Critical PPS operation failed: %s", strerror(errno));
                cleanup_resources();
                return 1;
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
    
    // Cleanup - ensure all resources are properly released
    ROS_INFO("Shutting down timeref_node");
    cleanup_resources();
    
    return 0;
}
