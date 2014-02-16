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

/*  INCLUDES    ============================================================ */
//
//
//
//
/*  DEFINITIONS    --------------------------------------------------------- */


namespace ros_aitown_brain {

}

using namespace ros_aitown_brain;

/*  DEFINITIONS    ========================================================= */
//
//
//
//
/*  DATA    ---------------------------------------------------------------- */

//! our node
ros::NodeHandle * aitown_node = NULL;

/*  DATA    ================================================================ */
//
//
//
//
/*  FUNCTIONS    ----------------------------------------------------------- */


static void intHandler(int ) {
    ROS_INFO("...Ending");


    // our node
    if (aitown_node != NULL) {
        delete aitown_node;
        aitown_node = NULL;
    }

    ros::shutdown();
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

        // ...

        // we will catch CTRL+C (not happening right now)
        signal(SIGINT, intHandler);

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // ...

            ros::spinOnce();
            loop_rate.sleep();
        }

        // exit ok
        ret = 0;
        break;
    }


    ROS_INFO("Node never reaches thiss point");
    return ret;
}
