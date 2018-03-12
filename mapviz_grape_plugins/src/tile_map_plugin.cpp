// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#include <grape_tile_map/tile_map_plugin.h>
#include <grape_tile_map/tile_source.h>

// QT libraries
#include <QGLWidget>
#include <QInputDialog>
#include <QMessageBox>
#include <QPalette>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <swri_transform_util/frames.h>
#include <swri_yaml_util/yaml_util.h>

#include <mapviz/select_frame_dialog.h>
#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins,
                        tile_map_grape,
                        grape_plugins::TileMapPlugin,
                        mapviz::MapvizPlugin)

namespace grape_plugins
{
    std::string TileMapPlugin::BASE_URL_KEY = "base_url";
    std::string TileMapPlugin::CUSTOM_SOURCES_KEY = "custom_sources";
    std::string TileMapPlugin::MAX_ZOOM_KEY = "max_zoom";
    std::string TileMapPlugin::NAME_KEY = "name";
    std::string TileMapPlugin::SOURCE_KEY = "source";
    std::string TileMapPlugin::TYPE_KEY = "type";

    TileMapPlugin::TileMapPlugin() :
        config_widget_(new QWidget()),
        transformed_(false),
        last_center_x_(0.0),
        last_center_y_(0.0),
        last_scale_(0.0),
        last_height_(0),
        last_width_(0),
        tile_source_("MapProxy", "http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png", true, 19),
        tile_map_(tile_source_)
    {
        ui_.setupUi(config_widget_);

        QPalette p(config_widget_->palette());
        p.setColor(QPalette::Background, Qt::white);
        config_widget_->setPalette(p);

        QPalette p2(ui_.status->palette());
        p2.setColor(QPalette::Text, Qt::red);
        ui_.status->setPalette(p2);

        source_frame_ = swri_transform_util::_wgs84_frame;

        QObject::connect(ui_.reset_cache_button, SIGNAL(clicked()),
                         this, SLOT(ResetTileCache()));
    }

    TileMapPlugin::~TileMapPlugin()
    {
    }

    void TileMapPlugin::ResetTileCache()
    {
        tile_map_.ResetCache();
    }

    void TileMapPlugin::PrintError(const std::string& message)
    {
        if (message == ui_.status->text().toStdString())
            return;

        ROS_ERROR_THROTTLE(1.0, "Error: %s", message.c_str());
        QPalette p(ui_.status->palette());
        p.setColor(QPalette::Text, Qt::red);
        ui_.status->setPalette(p);
        ui_.status->setText(message.c_str());
    }

    void TileMapPlugin::PrintInfo(const std::string& message)
    {
        if (message == ui_.status->text().toStdString())
            return;

        ROS_INFO_THROTTLE(1.0, "%s", message.c_str());
        QPalette p(ui_.status->palette());
        p.setColor(QPalette::Text, Qt::green);
        ui_.status->setPalette(p);
        ui_.status->setText(message.c_str());
    }

    void TileMapPlugin::PrintWarning(const std::string& message)
    {
        if (message == ui_.status->text().toStdString())
            return;

        ROS_WARN_THROTTLE(1.0, "%s", message.c_str());
        QPalette p(ui_.status->palette());
        p.setColor(QPalette::Text, Qt::darkYellow);
        ui_.status->setPalette(p);
        ui_.status->setText(message.c_str());
    }

    QWidget* TileMapPlugin::GetConfigWidget(QWidget* parent)
    {
        config_widget_->setParent(parent);
        return config_widget_;
    }

    bool TileMapPlugin::Initialize(QGLWidget* canvas)
    {
        canvas_ = canvas;

        ui_.url_label->setText("Base URL:");
        last_height_ = 0; // This will force us to recalculate our view
        ui_.base_url_text->setText(tile_source_.GetBaseUrl());
        initialized_ = true;

        return true;
    }

    void TileMapPlugin::Draw(double x, double y, double scale)
    {
        swri_transform_util::Transform to_wgs84;
        if (tf_manager_->GetTransform(source_frame_, target_frame_, to_wgs84))
        {
            tf::Vector3 center(x, y, 0);
            center = to_wgs84 * center;
            if (center.y() != last_center_y_ ||
                    center.x() != last_center_x_ ||
                    scale != last_scale_ ||
                    canvas_->width() != last_width_ ||
                    canvas_->height() != last_height_)
            {
                // Draw() is called very frequently, and SetView is a fairly expensive operation, so we
                // can save some CPU time by only calling it when the relevant parameters have changed.
                last_center_y_ = center.y();
                last_center_x_ = center.x();
                last_scale_ = scale;
                last_width_ = canvas_->width();
                last_height_ = canvas_->height();
                tile_map_.SetView(center.y(), center.x(), scale, canvas_->width(), canvas_->height());
            }
            tile_map_.Draw();
        }
    }

    void TileMapPlugin::Transform()
    {
        swri_transform_util::Transform to_target;
        if (tf_manager_->GetTransform(target_frame_, source_frame_, to_target))
        {
            tile_map_.SetTransform(to_target);
            PrintInfo("OK");
        }
        else
        {
            PrintError("No transform between " + source_frame_ + " and " + target_frame_);
        }
    }

    void TileMapPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
    {

    }

    void TileMapPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
    {
        emitter << YAML::Key << CUSTOM_SOURCES_KEY << YAML::Value;
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << BASE_URL_KEY << YAML::Value << tile_source_.GetBaseUrl().toStdString();
        emitter << YAML::Key << MAX_ZOOM_KEY << YAML::Value << tile_source_.GetMaxZoom();
        emitter << YAML::Key << NAME_KEY << YAML::Value << tile_source_.GetName().toStdString();
        emitter << YAML::Key << TYPE_KEY << YAML::Value << tile_source_.GetType().toStdString();
        emitter << YAML::EndMap;

        emitter << YAML::EndSeq;

        //    emitter << YAML::Key << SOURCE_KEY <<
        //               YAML::Value << boost::trim_copy(ui_.source_combo->currentText().toStdString());
    }
}

