#include "mapviz/local_xy_dialog.h"
#include"mapviz/select_frame_dialog.h"
#include"mapviz/select_topic_dialog.h"

#include <QSettings>
#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_transform_util/earth_constants.h>
#include <swri_transform_util/transform_util.h>



LocalXYDialog::LocalXYDialog(boost::shared_ptr<tf::TransformListener> tf,
                             const geometry_msgs::PoseStamped& default_value,
                             QWidget *parent) :
  QDialog(parent),
  tf_(tf)
{
  ui_.setupUi(this);
  ui_.latitude->setValue( default_value.pose.position.y );
  ui_.longitude->setValue( default_value.pose.position.x );
  ui_.altitude->setValue( default_value.pose.position.z );
  ui_.frame->setText( default_value.header.frame_id.c_str() );

  QSettings settings("Eurecat","Mapviz2");
  ui_.checkBox->setChecked( settings.value("LocalXYDialog/checkbox", false).toBool() );
  ui_.navsat_topic->setText( settings.value("LocalXYDialog/navsat_topic").toString() );
  ui_.angleOffset->setValue( settings.value("LocalXYDialog/angle_offset").toDouble());

  CheckBoxToggled(ui_.checkBox->isChecked());

  connect(ui_.select_fixed_frame, SIGNAL(pressed()), this, SLOT(SelectFixedFramePushed()));
  connect(ui_.select_topic, SIGNAL(pressed()), this, SLOT(SelectGPSTopicPushed()));
  connect(ui_.checkBox, SIGNAL(toggled(bool)), this, SLOT(CheckBoxToggled(bool)));
  connect(this, SIGNAL(accepted()), this, SLOT(OnAccepted()));

}

geometry_msgs::PoseStamped LocalXYDialog::getMsg() const
{
  geometry_msgs::PoseStamped origin;
  origin.header.frame_id = ui_.frame->text().toStdString();
  origin.pose.position.y = ui_.latitude->value();
  origin.pose.position.x = ui_.longitude->value();
  origin.pose.position.z = ui_.altitude->value();

  tf::Quaternion quat;
  quat.setEuler( ui_.angleOffset->value()*M_PI/180.0, 0.0, 0.0);
 // quat.setEuler( 0.0, 0.0, 0.0);
  origin.pose.orientation.x = quat.x();
  origin.pose.orientation.y = quat.y();
  origin.pose.orientation.z = quat.z();
  origin.pose.orientation.w = quat.w();
  return origin;
}

LocalXYDialog::~LocalXYDialog()
{}

void LocalXYDialog::OnAccepted()
{
  QSettings settings("Eurecat","Mapviz2");
  settings.setValue("LocalXYDialog/checkbox", ui_.checkBox->isChecked());
  settings.setValue(("LocalXYDialog/navsat_topic"),  ui_.navsat_topic->text());
  settings.setValue(("LocalXYDialog/angle_offset"),  ui_.angleOffset->value());
}

void LocalXYDialog::SelectGPSTopicPushed()
{
  ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic("sensor_msgs/NavSatFix");
  if (!topic.name.empty())
  {
    ui_.navsat_topic->setText(QString::fromStdString(topic.name));
    ros::NodeHandle node;
    navsat_subscriber_ = node.subscribe(topic.name,1, &LocalXYDialog::NavSatCallback, this);
  }
}


void LocalXYDialog::CheckBoxToggled(bool toggled)
{
  ui_.frame_bottom->setVisible(toggled);
  ui_.textinfo->setVisible(toggled);

  if( toggled )
  {
    ros::NodeHandle node;
    navsat_subscriber_ = node.subscribe(ui_.navsat_topic->text().toStdString(),
                                        1, &LocalXYDialog::NavSatCallback, this);
  }
  else{
    navsat_subscriber_.shutdown();
  }
}

void LocalXYDialog::NavSatCallback(const sensor_msgs::NavSatFixConstPtr navsat)
{
  using namespace swri_transform_util;

  std::string fixed_frame = ui_.frame->text().toStdString();
  std::string gps_frame =  navsat->header.frame_id;
  tf::StampedTransform transform;
  transform.setOrigin( {0,0,0 } );

  try{
    tf_->lookupTransform(fixed_frame, gps_frame, ros::Time(), transform);
    ui_.status->setText("");
  }
  catch(std::runtime_error& e)
  {
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText( QString("Error transforming from [%1] to [%2]").arg(fixed_frame.c_str()).arg(gps_frame.c_str()) );
    return;
  }

  double x = -transform.getOrigin().x();
  double y = -transform.getOrigin().y();

  const double TO_RAD = M_PI / 180.0;
  const double TO_GED = 180.0 / M_PI;

  double reference_latitude = navsat->latitude;
  double reference_longitude = navsat->longitude;
  double depth = navsat->altitude;

  double p = _earth_eccentricity * std::sin(reference_latitude*TO_RAD);
  p = 1.0 - p * p;

  double rho_e = _earth_equator_radius *
      (1.0 - _earth_eccentricity * _earth_eccentricity) / (std::sqrt(p) * p);
  double rho_n = _earth_equator_radius / std::sqrt(p);

  double rho_lat = rho_e - depth;
  double rho_lon = (rho_n - depth) * std::cos(reference_latitude*TO_RAD);
//  double rlat = (y / rho_lat) * TO_GED ;
//  double rlon = (x / rho_lon) * TO_GED ;

  // KEEP for future improvements
  double reference_angle =  ui_.angleOffset->value()*TO_RAD; // keep for future improvements
  double cos_angle = std::cos(reference_angle);
  double sin_angle = std::sin(reference_angle);
  double dX = cos_angle * x - sin_angle * y;
  double dY = sin_angle * x + cos_angle * y;
  double rlat = (dY / rho_lat)*TO_GED ;
  double rlon = (dX / rho_lon)*TO_GED ;

  ui_.status->setText( QString("X: %1  Y: %2").arg(dX).arg(dY) );

  ui_.latitude->setValue( rlat + reference_latitude );
  ui_.longitude->setValue( rlon + reference_longitude );
  ui_.altitude->setValue( depth );
}

void LocalXYDialog::SelectFixedFramePushed()
{
  std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_);
  if (!frame.empty())
  {
    ui_.frame->setText(QString::fromStdString(frame));
  }
}
