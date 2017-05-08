/**
   Software License Agreement (BSD)

   \file      channel.h
   \authors   Mike Purvis <mpurvis@clearpathrobotics.com>
   \copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
   following conditions are met:
   * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ROBOTEQ_CHANNEL
#define ROBOTEQ_CHANNEL

#include "ros/ros.h"

/*
 * This will depend on how you set up the RoboteQ to get feedback
 * The .mbs script needs to be set up correctly
 * Hall sensors = 6.0 * PolePairs 'lines' per rev
 * Quadrature Encoder = 4.0 * N lines per rev
 */
/* #define ENC_LINES (4.0 * 4096.0) */
/* #define ENC_LINES_REMOVE_ME_WHEN_NO_DEPENDENCY_IN_ANT_MOTOR_FEEDBACK 512 */
/* const double max_radians = (((uint64_t)INT_MAX - INT_MIN) + 1) * (2 * M_PI) / ENC_LINES_REMOVE_ME_WHEN_NO_DEPENDENCY_IN_ANT_MOTOR_FEEDBACK; */

#define MAX_POSITION_REGISTER_COUNTS 0xFFFFFFFF

namespace roboteq_msgs {
  ROS_DECLARE_MESSAGE(Command);
  ROS_DECLARE_MESSAGE(Feedback);
}

namespace roboteq {

  class Controller;

  class Channel {
  public:
    Channel(int channel_num, std::string ns, Controller* controller);
    void feedbackCallback(std::vector<std::string>);
  
  private:
    uint32_t first_time;
    uint32_t previous_ticks;
  
  protected:
    /**
     * @param x Angular velocity in radians/s.
     * @return Angular velocity in RPM.
     */
    static double to_rpm(double x)
    {
      return x * 60 / (2 * M_PI);
    }

    /**
     * @param x Angular velocity in RPM.
     * @return Angular velocity in rad/s.
     */
    static double from_rpm(double x)
    {
      return x * (2 * M_PI) / 60;
    }

    /**
     * Conversion of radians to encoder ticks. Note that this assumes a
     * 1024-line quadrature encoder (hence 4096).
     *
     * @param x Angular position in radians.
     * @return Angular position in encoder ticks.
     */
    double to_encoder_ticks(double x)
    {
      //return x * ENC_LINES / (2 * M_PI);
      return x * enc_quad_lines_ / (2 * M_PI);
    }

    /**
     * Conversion of encoder ticks to radians. Note that this assumes a
     * 1024-line quadrature encoder (hence 4096).
     *
     * @param x Angular position in encoder ticks.
     * @return Angular position in radians.
     */
    double from_encoder_ticks(double x)
    {
      double rad;
      //return x * (2 * M_PI) / ENC_LINES;
      rad = x * (2 * M_PI) / enc_quad_lines_;
      
      /* ROS_WARN("ticks = %lf, rad = %lf", x, rad); */

      return rad;
    }
  
    void cmdCallback(const roboteq_msgs::Command&);
    void timeoutCallback(const ros::TimerEvent&);

    ros::NodeHandle nh_;
    boost::shared_ptr<Controller> controller_;
    int channel_num_;
    float max_rpm_;

    ros::Subscriber sub_cmd_;
    ros::Publisher pub_feedback_;
    ros::Timer timeout_timer_;

    ros::Time last_feedback_time_;
    uint8_t last_mode_;

    /* Position functions will report wrong until the Roboteq sends back the number
       of lines per rev in a feedback message.
    */
    double enc_quad_lines_;
    /* double max_radians_; */
    double previous_position_;
  };

}

#endif
