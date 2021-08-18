#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>

namespace gsplines_follow_trajectory {

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    const std::string &_name)
    : nh_(), nh_prv_("~"), name_(_name),
      action_server_(
          nh_, name_,
          boost::bind(&FollowJointTrajectoryActionWrapper::action_callback,
                      this, _1),
          false) {}

} // namespace gsplines_follow_trajectory_control
