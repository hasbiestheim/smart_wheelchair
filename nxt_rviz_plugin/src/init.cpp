#include "rviz/plugin/type_registry.h"

#include "nxt_ultrasonic_display.h"

using namespace nxt_rviz_plugin;

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<NXTUltrasonicDisplay>("nxt_rviz_plugin::NXTUltrasonicDisplay");

}

