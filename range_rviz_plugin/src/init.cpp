#include "rviz/plugin/type_registry.h"

#include "range_display.h"

using namespace range_rviz_plugin;

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<RangeDisplay>("range_rviz_plugin::RangeDisplay");

}

