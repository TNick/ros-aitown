/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/*!
  \file			ros_aitown_dstorage_node.cc
  \date			September 2013
  \author		TNick

*//*


 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Please read COPYING and README files in root folder
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* ------------------------------------------------------------------------- */
/* ========================================================================= */
//
//
//
//
/*  INCLUDES    ------------------------------------------------------------ */

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <stdio.h>
#include <signal.h>
#include <inttypes.h>

#include <ros_aitown_dstorage/NewID.h>
#include <ros_aitown_dstorage/GetID.h>
#include <ros_aitown_dstorage/SetID.h>

#include <aitown/aitown-dstorage.h>


/*  INCLUDES    ============================================================ */
//
//
//
//
/*  DEFINITIONS    --------------------------------------------------------- */


namespace ros_aitown_dstorage {

}

using namespace ros_aitown_dstorage;

/*  DEFINITIONS    ========================================================= */
//
//
//
//
/*  DATA    ---------------------------------------------------------------- */

//! our node
ros::NodeHandle * aitown_node = NULL;

//! the instance
dstorage_t storage;

/*  DATA    ================================================================ */
//
//
//
//
/*  FUNCTIONS    ----------------------------------------------------------- */


static void intHandler(int ) {
    ROS_INFO("...Ending");

    // stop the database
    dstorage_end (&storage);

    // our node
    if (aitown_node != NULL) {
        delete aitown_node;
        aitown_node = NULL;
    }

    ros::shutdown();
}


//! we're creating a new id here
static bool kb_new_id(NewID::Request &req,
                   NewID::Response &res)
{


    return true;
}

//! we're retrieving the content for an id
static bool kb_get_id(GetID::Request &req,
                   GetID::Response &res)
{


    return true;
}

//! we're removing a sensor by name; this is definitive
static void kb_set_id (const SetID::ConstPtr& msg)
{

}


int main(int argc, char **argv)
{

    ROS_INFO("Starting...");
    int ret = 1;

    // parse arguments ROS style
    ros::init(argc, argv, "ros_aitown_brain");

    // prepare the one and only node
    aitown_node = new ros::NodeHandle();

    for (;;) {

        // start the database
        dstorage_init (&storage);

        // we will catch CTRL+C (not happening right now)
        signal(SIGINT, intHandler);

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // ...

            ros::ServiceServer service_new_id =
                aitown_node->advertiseService("new_id", kb_new_id);
            if (!service_new_id) {
                ROS_ERROR("Failed to advertise service <new_id>");
                break;
            }

            ros::ServiceServer service_get_id =
                aitown_node->advertiseService("get_id", kb_get_id);
            if (!service_get_id) {
                ROS_ERROR("Failed to advertise service <get_id>");
                break;
            }

            ros::Subscriber subscribe_set_id =
                aitown_node->subscribe("set_id", 10, kb_set_id);
            if (!subscribe_set_id) {
                ROS_ERROR("Failed to subscribe to <set_id>");
                break;
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

        // exit ok
        ret = 0;
        break;
    }


    ROS_INFO("Node never reaches this point on normal execution");
    return ret;
}
