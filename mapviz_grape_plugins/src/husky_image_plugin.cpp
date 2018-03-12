// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
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

#include <grape_plugins/husky_image_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>
#include <QImage>
#include <QFileDialog>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_frame_dialog.h>
#include <swri_transform_util/transform_manager.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grape_plugins::HuskyImagePlugin, mapviz::MapvizPlugin)

namespace grape_plugins
{
  HuskyImagePlugin::HuskyImagePlugin() :
    config_widget_(new QWidget()),
    original_width_(1),
    original_height_(1),
    size_(1.0),
    texture_loaded_(false),
    transformed_(false)
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

    source_frame_ = "base_link";

    LoadImage();

    QObject::connect(ui_.spinBoxSize, SIGNAL(valueChanged(double)), this, SLOT(SizeChanged(double)));
    QObject::connect(ui_.checkBoxFixed, SIGNAL(toggled(bool)), this, SLOT(FixedChanged(bool)));

  }

  HuskyImagePlugin::~HuskyImagePlugin()
  {
  }

  void HuskyImagePlugin::SizeChanged(double value)
  {
    size_ = value;
  }

  void HuskyImagePlugin::FixedChanged(bool value)
  {
     fixed_ = value;
  }

  void HuskyImagePlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void HuskyImagePlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void HuskyImagePlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* HuskyImagePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool HuskyImagePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    initialized_ = true;
    return true;
  }

  void HuskyImagePlugin::Draw(double x, double y, double scale)
  {
      if (texture_loaded_ && transformed_)
      {
          double half_width  = size_ * 0.5;
          double half_height = half_width * original_height_ / original_width_;
          if( fixed_ )
          {
              half_width  *= scale * 100;
              half_height *= scale * 100;
          }
          tf::Point top_left = tf::Point(-half_width, half_height, 0);
          tf::Point top_right = tf::Point(half_width, half_height, 0);
          tf::Point bottom_left = tf::Point(-half_width, -half_height, 0);
          tf::Point bottom_right = tf::Point(half_width, -half_height, 0);

          top_left = transform_ * top_left;
          top_right = transform_ * top_right;
          bottom_left = transform_ * bottom_left;
          bottom_right = transform_ * bottom_right;

          glColor3f(1.0f, 1.0f, 1.0f);
          glEnable(GL_TEXTURE_2D);
          glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_id_));

          glBegin(GL_QUADS);

          glTexCoord2f(0, 1); glVertex2d(top_left.x(), top_left.y());
          glTexCoord2f(1, 1); glVertex2d(top_right.x(), top_right.y());
          glTexCoord2f(1, 0); glVertex2d(bottom_right.x(), bottom_right.y());
          glTexCoord2f(0, 0); glVertex2d(bottom_left.x(), bottom_left.y());

          glEnd();

          glDisable(GL_TEXTURE_2D);
          PrintInfo("OK");
      }
  }

  void HuskyImagePlugin::Transform()
  {
    transformed_ = false;

    swri_transform_util::Transform transform;
    if (GetTransform(ros::Time(), transform))
    {
      transform_ = transform;
      transformed_ = true;
    }
    else
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void HuskyImagePlugin::LoadImage()
  {
    try
    {
      image_ = QImage(":/grape/images/husky.png");

      if (texture_loaded_)
      {
        GLuint ids[1];
        ids[0] = static_cast<GLuint>(texture_id_);
        glDeleteTextures(1, &ids[0]);
        texture_loaded_ = false;
      }

      original_width_  = image_.width();
      original_height_ = image_.height();

      float max_dim = std::max(image_.width(), image_.height());
      dimension_ = static_cast<int>(std::pow(2, std::ceil(std::log(max_dim) / std::log(2.0f))));

      if (original_width_ != dimension_ || original_height_ != dimension_)
      {
          image_ = image_.scaled(dimension_, dimension_, Qt::IgnoreAspectRatio, Qt::FastTransformation);
      }

      image_ = QGLWidget::convertToGLFormat(image_);

      GLuint ids[1];
      glGenTextures(1, &ids[0]);
      texture_id_ = ids[0];

      glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_id_));
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dimension_, dimension_, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.bits());

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

      texture_loaded_ = true;
    }
    catch (const std::exception& e)
    {
      PrintError("Failed to load image.  Exception occured.");
    }
  }

  void HuskyImagePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["size"])
    {
      node["size"] >> size_;
      ui_.spinBoxSize->setValue(size_);
    }
    if (node["fixed"])
    {
      node["fixed"] >> fixed_;
      ui_.checkBoxFixed->setChecked(fixed_);
    }
  }

  void HuskyImagePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "size" << YAML::Value << size_;
    emitter << YAML::Key << "fixed" << YAML::Value << fixed_;
  }
}

