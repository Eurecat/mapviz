#ifndef LOCAL_XY_DIALOG_H
#define LOCAL_XY_DIALOG_H

#include <QDialog>
#include <geometry_msgs/PoseStamped.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transform_manager.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include "ui_local_xy_dialog.h"

class LocalXYDialog : public QDialog
{
  Q_OBJECT

public:
  explicit LocalXYDialog(boost::shared_ptr<tf::TransformListener> tf,
                         const geometry_msgs::PoseStamped &default_value,
                         QWidget *parent = 0);

  geometry_msgs::PoseStamped getMsg() const;

  ~LocalXYDialog();

private Q_SLOTS:
  void OnAccepted();
  void SelectFixedFramePushed();
  void SelectGPSTopicPushed();
  void CheckBoxToggled(bool);

private:
  Ui::LocalXYDialog ui_;
  boost::shared_ptr<tf::TransformListener> tf_;

  ros::Subscriber navsat_subscriber_;
  void NavSatCallback(const sensor_msgs::NavSatFixConstPtr navsat);
};

#endif // LOCAL_XY_DIALOG_H
