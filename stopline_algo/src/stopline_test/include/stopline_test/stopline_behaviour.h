#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>

#include <aux_lib/struct_defs.h>
#include <aux_lib/aux_functions.h>
#include <aux_lib/qp_problem.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <traffic_msgs/CenterLanes.h>
#include <traffic_msgs/Decision.h>
#include <traffic_msgs/DecisionTrajectory.h>

#include <eigen3/Eigen/Dense>

#include "matplotlibcpp.h"

struct QPProblemUpdatedStruct {
  bool updated = false; /**< Flag to check if the problem has been updated */
  std::unique_ptr<OsqpEigen::Solver> qp_problem; /**< OSQP QP problem */
};

/**
 * @brief Data structure with parameters for Stopline Breaking algo 
 * - added by shrita
 */
struct StoplineBreak {
    bool NearStopline; /**<Bool for whether car is done waiting as stopline (0 for done)*/
    double stopline_dist; /**<Distance between car and stopline*/
    int stopline_timer; /**<Timer for stopline wait*/
    Eigen::VectorXd p_traj; /**<Upper bound on velocity for stop line*/
    Eigen::VectorXd v_traj; /**<Upper bound on velocity for stop line*/
    Eigen::VectorXd a_traj; /**<Upper bound on velocity for stop line*/
    Eigen::VectorXd j_traj; /**<Upper bound on velocity for stop line*/
};

//class for stopline algo breaking

class StoplineBehaviour{
    public:
        //initializing function
        StoplineBehaviour();

    private:
        //AD vehicle state
        bool valid_ad_status_m = false; /**< Flag to verify we got a valid AD state */
        ai4ad::AgentStateSimple ad_cur_state_m; /**< AD current simple state */
        geometry::Point_Frenet ad_frenet_m; /**< AD current state in frenet frame */
        geometry::Point2D ad_xy_m; /**< AD current [x,y] in global space */

        double ad_prev_state_m;    /**< Previous AD state */
        double prev_accel_m;       /**< AD estimated trajectory acceleration */
        
        // Warning and times for each message
        ros::Time prev_t_ad_status_m; /**< Stores previous AD state message timestamp */
        ros::Time prev_t_decision_m; /**< Stores previous decision message timestamp */
        
        // Scenario variables
        traffic_msgs::Decision decision_m; /**< Stores the decision sent from Decision Governor */
        std::vector<geometry::Point2D> global_path_m; /**< Stores global path to follow*/

        //Lane info, Path, Stopline pos variables

        // Simulation variables
        ai4ad::struct_params_alg params_alg_m; /**< Stores algorithm parameters */

        // ROS variables
        ros::NodeHandle nh_m;               /**< ROS node handle */
        ros::Subscriber center_lanes_sub_m; /**< ROS subscriber for center lanes */
        ros::Subscriber ad_state_sub_m;     /**< ROS subscriber for AD state */
        ros::Subscriber decision_sub_m;    /**< ROS subscriber decision */
        ros::Subscriber global_path_sub_m;    /**< ROS subscriber for global path */
        ros::Subscriber stopline_marker_sub_m;    /**< ROS subscriber for global path */
        ros::Publisher stopline_traj_pub_m; /**< ROS publisher for stopline trajectory */

        // Optimization variables
        QPProblem* qp_problem_m; /**< Pointer to a QPProblem */

        //METHODS
        void ADStateCallback(const traffic_msgs::VehicleState::ConstPtr& msg);
        void LanesCenterCallback(const traffic_msgs::CenterLanes::ConstPtr& msg);
        void DecisionCallback(const traffic_msgs::Decision::ConstPtr& msg) {
            decision_m = *msg;
            prev_t_decision_m = ros::Time::now();
        }  
        void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg);
        void TimerCallback(const ros::TimerEvent&);
        void StoplineInfoCallback()


}