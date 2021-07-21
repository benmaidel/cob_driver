
// standard includes
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <signal.h>

// ros includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

// ros message includes
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cob_light/action/set_light_mode.hpp>

// serial connection includes
#include <serialIO.h>

// additional includes
#include <colorUtils.h>
#include <modeExecutor.h>
#include <colorO.h>
#include <colorOSim.h>
#include <ms35.h>
#include <stageprofi.h>

class LightNode : public rclcpp::Node
{
public:
  LightNode() :
    Node("light_node"),
    _baudrate(0), _invertMask(0), _num_leds(0), _topic_priority(0), _bPubMarker(false), _bSimEnabled(true)
  {
  }
  bool init()
  {

    return true;
  }

  ~LightNode()
  {

  }



private:
  std::string _deviceDriver;
  std::string _deviceString;
  std::string _sMarkerFrame;

  int   _baudrate;
  int   _invertMask;
  int   _num_leds;
  int   _topic_priority;
  bool  _bPubMarker;
  bool  _bSimEnabled;

  rclcpp_action::Server<cob_light::action::SetLightMode>::SharedPtr _as;

  color::rgba _color;

  IColorO* p_colorO;
  SerialIO _serialIO;
  ModeExecutor* p_modeExecutor;

  boost::mutex _mutex;
};

int main(int argc, char** argv)
{


  return 0;
}
