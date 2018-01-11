// *****************************************************************************
//
// Copyright (c) 2018, Eurecat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Eurecat nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <GL/glew.h>

#include <mapviz_plugins/occupancy_grid_plugin.h>
#include <GL/glut.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::OccupancyGridPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  OccupancyGridPlugin::OccupancyGridPlugin() :
    config_widget_(new QWidget()),
    transformed_(false),
    texture_id_(0)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.select_topic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(textEdited(const QString&)), this, SLOT(TopicEdited()));
    QObject::connect(this, SIGNAL(TargetFrameChanged(std::string)), this, SLOT(FrameChanged(std::string)));
  }

  OccupancyGridPlugin::~OccupancyGridPlugin()
  {
    Shutdown();
  }

  void OccupancyGridPlugin::Shutdown()
  {
  }

  void OccupancyGridPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);
      
      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);
      
      QPen pen(Qt::black);
      
      pen.setWidth(2);
      pen.setCapStyle(Qt::SquareCap);
      painter.setPen(pen);

      painter.drawLine(2, 2, 14, 2);
      painter.drawLine(2, 2, 2, 14);
      painter.drawLine(14, 2, 14, 14);
      painter.drawLine(2, 14, 14, 14);
      painter.drawLine(8, 2, 8, 14);
      painter.drawLine(2, 8, 14, 8);
      
      icon_->SetPixmap(icon);
    }
  }

  void OccupancyGridPlugin::FrameChanged(std::string)
  {
    transformed_ = false;
  }

  void OccupancyGridPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic("nav_msgs/OccupancyGrid");
    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void OccupancyGridPlugin::TopicEdited()
  {
    const std::string topic = ui_.topic->text().trimmed().toStdString();

    initialized_ = false;
    grid_.reset();
    PrintWarning("No messages received.");

    grid_sub_.shutdown();

    topic_ = topic;
    if (!topic.empty())
    {
      grid_sub_ = node_.subscribe(topic_, 1, &OccupancyGridPlugin::Callback, this);
      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void OccupancyGridPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void OccupancyGridPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void OccupancyGridPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* OccupancyGridPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool OccupancyGridPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    DrawIcon();
    return true;
  }

  void OccupancyGridPlugin::Callback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    const int channels   = 2;
    initialized_ = true;
    grid_ = msg;
    source_frame_ = msg->header.frame_id;
    transformed_ = GetTransform( source_frame_, msg->header.stamp, transform_);
    if ( !transformed_ )
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }

    int32_t max_dimension = std::max(msg->info.height, msg->info.width);
    int32_t texture_size = 2;
    while (texture_size < max_dimension){
      texture_size = texture_size << 1;
    }

    std::vector<uchar> buffer(texture_size*texture_size*channels, 0);

    for (size_t row = 0; row < msg->info.height; row++)
    {
      for (size_t col = 0; col < msg->info.width; col++)
      {
        size_t index_src = (col + row*msg->info.width);
        double color = msg->data[ index_src ];
        size_t index_dst = (col + row*texture_size)*channels;

        if( color < 0)
        {
          buffer[index_dst]   = 150;
          buffer[index_dst+1] = 150;
        }
        else{
          buffer[index_dst]   = static_cast<uint8_t>((100 - color)*2.55);
          buffer[index_dst+1] = 255;
        }
      }
    }
    if (texture_id_ != -1)
    {
      glDeleteTextures(1, &texture_id_);
    }

    // Get a new texture id.
    glGenTextures(1, &texture_id_);

    // Bind the texture with the correct size and null memory.
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_LUMINANCE_ALPHA,
        texture_size,
        texture_size,
        0,
        GL_LUMINANCE_ALPHA,
        GL_UNSIGNED_BYTE,
        buffer.data());

    texture_x_ = static_cast<float>(msg->info.width) / static_cast<float>(texture_size);
    texture_y_ = static_cast<float>(msg->info.height) / static_cast<float>(texture_size);

    glBindTexture(GL_TEXTURE_2D, 0);
  }

  void OccupancyGridPlugin::Draw(double x, double y, double scale)
  {
    glPushMatrix();

    if( grid_ && transformed_)
    {
      double resolution = grid_->info.resolution;
      glTranslatef( transform_.GetOrigin().getX(),
                    transform_.GetOrigin().getY(),
                    0.0);

      tfScalar yaw, pitch, roll;
      tf::Matrix3x3 mat( transform_.GetOrientation() );
      mat.getEulerYPR(yaw, pitch, roll);

      glRotatef(pitch * 180.0 / M_PI, 0, 1, 0);
      glRotatef(roll  * 180.0 / M_PI, 1, 0, 0);
      glRotatef(yaw   * 180.0 / M_PI + ui_.angleOffset->value(), 0, 0, 1);

      glTranslatef( grid_->info.origin.position.x,
                    grid_->info.origin.position.y,
                    0.0);

      glScalef( resolution, resolution, 1.0);

      float width  = static_cast<float>(grid_->info.width);
      float height = static_cast<float>(grid_->info.height);

      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glBegin(GL_TRIANGLES);

      glColor4f(1.0f, 1.0f, 1.0f, ui_.alpha->value() );

      glTexCoord2d(0, 0);
      glVertex2d(0, 0);
      glTexCoord2d(texture_x_, 0);
      glVertex2d(width, 0);
      glTexCoord2d(texture_x_, texture_y_);
      glVertex2d(width, height);

      glTexCoord2d(0, 0);
      glVertex2d(0, 0);
      glTexCoord2d(texture_x_, texture_y_);
      glVertex2d(width, height);
      glTexCoord2d(0, texture_y_);
      glVertex2d(0, height);

      glEnd();

      glBindTexture(GL_TEXTURE_2D, 0);
      glDisable(GL_TEXTURE_2D);
      PrintInfo("OK");
    }
    glPopMatrix();
  }

  void OccupancyGridPlugin::Transform()
  {
    swri_transform_util::Transform transform;
    if ( grid_ )
    {
      if( GetTransform( source_frame_, ros::Time(0), transform) )
      {
          transformed_ = true;
          transform_ = transform;
      }
    }
    if ( !transformed_ )
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void OccupancyGridPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(QString::fromStdString(topic));
    }

    if (node["alpha"])
    {
      double alpha;
      node["alpha"] >> alpha;
      ui_.alpha->setValue(alpha);
    }

    TopicEdited();
  }

  void OccupancyGridPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "alpha" << YAML::Value << ui_.alpha->value();

    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;
  }
}

