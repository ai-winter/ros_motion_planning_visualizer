#include "include/path_visual_plugin.h"
#include "include/ui_path_visual_plugin.h"

namespace path_visual_plugin{

  PathVisualPlugin::PathVisualPlugin(QWidget *parent)
      : rviz::Panel(parent)
      , ui(new Ui::PathVisualPlugin)
  {
      ui->setupUi(this);
  }

  PathVisualPlugin::~PathVisualPlugin()
  {
      delete ui;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_visual_plugin::PathVisualPlugin, rviz::Panel)