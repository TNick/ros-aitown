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

//! time to wait before declaring a request lost
#define TIME_OUT_MILISECONDS    1000

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
    // get handle with new id and storage
    dstorage_handle_t * h = dstorage_new (
            &storage, req.initial_data.size());
    if (h == NULL) {
        ROS_ERROR("Failed to allocate a new ID");
        return false;
    }

    // copy data from input and release the handle
    memcpy (h->p_data, &req.initial_data[0], req.initial_data.size());
    dstorage_handle_done (&storage, &h, h);

    // respond with new id
    res.id = h->id;

    return true;
}

typedef struct _kb_get_id_final_t {
    int lock;
    dstorage_ctrl_sts_t sts;
    int dismissed;
} kb_get_id_final_t;

void kb_get_id_final(dstorage_ctrl_sts_t sts, dstorage_ctrl_param_t* req)
{
    kb_get_id_final_t * payload = (kb_get_id_final_t*)req->user;
    payload->sts = sts;
    payload->lock = 1;
}

//! we're retrieving the content for an id
static bool kb_get_id(GetID::Request &req,
                   GetID::Response &res)
{

    // get a handle for this id
    dstorage_handle_t * h = dstorage_handle (
                &storage, req.id, &storage);
    if (h == NULL) {
        ROS_ERROR("Failed to get a handle for ID");
        return false;
    }

    // attempt to resolve
    kb_get_id_final_t payload;
    payload.sts = DSTORAGE_CTRL_OK;
    payload.lock = 0;
    payload.dismissed = 0;
    dstorage_handle_resolve (&storage, h, &payload, kb_get_id_final);

    // wait for the result
    int i;
    for (i=0;i<TIME_OUT_MILISECONDS;i++) {
        if (payload.lock != 0)
            break;
        usleep(1000);
    }

    // see if we've got it
    bool result;
    if (payload.lock == 0) {
        result = (payload.sts == DSTORAGE_CTRL_OK);
        if (result) {
            res.data.reserve (h->p_data->user_sz);
            res.data.assign (
                        dstorage_chunk_user (h->p_data),
                        dstorage_chunk_user (h->p_data)+h->p_data->user_sz);
        } else {
            ROS_ERROR ("ID data request failed with code %d", payload.sts);
        }
    } else {
        ROS_ERROR ("Timed out waiting for ID data");
        payload.dismissed = 1;
        result = false;
    }

    // release the handle in every case
    dstorage_handle_done (&storage, &h, &storage);
    return result;

}

//! we're removing a sensor by name; this is definitive
static void kb_set_id (const SetID::ConstPtr& msg)
{

    // get a handle for this id
    dstorage_handle_t * h = dstorage_handle (
                &storage, msg->id, &storage);
    if (h == NULL) {
        ROS_ERROR("Failed to get a handle for ID");
        return;
    }

    // allocate a chunk large enough
    h->p_data = dstorage_alloc_chunk (&storage, msg->data.size());
    memcpy (h->p_data, &msg->data[0], msg->data.size());
    dstorage_handle_mark_resolved (h);
    dstorage_handle_mark_dirty (h);

    // release the handle that will trigger a save
    dstorage_handle_done (&storage, &h, &storage);

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
        dstorage_init (&storage, "ros_aitown_dstorage", NULL);

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
