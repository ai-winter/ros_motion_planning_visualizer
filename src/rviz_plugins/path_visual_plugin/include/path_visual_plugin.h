#ifndef PATHVISUALPLUGIN_H
#define PATHVISUALPLUGIN_H

#include <QWidget>

#include <ros/ros.h>
#include <rviz/panel.h>

QT_BEGIN_NAMESPACE
namespace Ui { class PathVisualPlugin; }
QT_END_NAMESPACE

namespace path_visual_plugin{

  class PathVisualPlugin : public rviz::Panel
  {
      Q_OBJECT

  public:
      PathVisualPlugin(QWidget *parent = nullptr);
      ~PathVisualPlugin();

  private:
      Ui::PathVisualPlugin *ui;
  };

}
#endif // PATHVISUALPLUGIN_H
