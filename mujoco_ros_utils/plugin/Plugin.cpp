#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"
#include "WrenchToActuators.h"
#include "OdometryPublisher.h"
#include "ImuPublisher.h"
#include "AcroMode.h"

namespace MujocoRosUtils
{

mjPLUGIN_LIB_INIT
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
  ImagePublisher::RegisterPlugin();
  ActuatorCommand::RegisterPlugin();
  ExternalForce::RegisterPlugin();
  SensorPublisher::RegisterPlugin();
  WrenchToActuators::RegisterPlugin();
  OdometryPublisher::RegisterPlugin();
  ImuPublisher::RegisterPlugin();
  AcroMode::RegisterPlugin();
}

} // namespace MujocoRosUtils
