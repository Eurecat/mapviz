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

#include <grape_plugins/plan_route_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>
#include <QDebug>
#include <QSettings>
#include <QFileDialog>
#include <fstream>

// ROS libraries
#include <ros/master.h>
#include <swri_transform_util/frames.h>
#include <grape_msgs/PlanRoute.h>
#include <grape_msgs/VirtualFences.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins,
                        plan_route_grape,
                        grape_plugins::PlanRoutePlugin,
                        mapviz::MapvizPlugin)


namespace stu = swri_transform_util;
using grape_msgs::RoutePoint;

namespace grape_plugins
{

PlanRoutePlugin::PlanRoutePlugin() :
    config_widget_(new QWidget()),
    map_canvas_(NULL),
    selected_point_(nullptr),
    is_mouse_down_(false),
    max_ms_(Q_INT64_C(500)),
    max_distance_(2.0),
    CTRL_pressed_(false)
{
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);
    // Set status text red

    ui_.status->setText("OK");
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));

    QObject::connect(ui_.pushButtonUndo, SIGNAL(clicked()), this,
                     SLOT(on_pushButtonUndo_clicked()));
    QObject::connect(ui_.pushButtonRedo, SIGNAL(clicked()), this,
                     SLOT(on_pushButtonRedo_clicked()));

    QObject::connect(ui_.pushButtonSave, SIGNAL(clicked()), this,
                     SLOT(on_pushButtonSave_clicked()));
    QObject::connect(ui_.pushButtonLoad, SIGNAL(clicked()), this,
                     SLOT(on_pushButtonLoad_clicked()));
    QObject::connect(ui_.pushButtonSend, SIGNAL(clicked()), this,
                     SLOT(sendButtonPressed()));

    QObject::connect(ui_.radioDisabled, SIGNAL(toggled(bool)), this,
                     SLOT(on_radio_toggled(bool)));
    QObject::connect(ui_.radioRoute, SIGNAL(toggled(bool)), this,
                     SLOT(on_radio_toggled(bool)));
    QObject::connect(ui_.radioFences, SIGNAL(toggled(bool)), this,
                     SLOT(on_radio_toggled(bool)));

    ros::NodeHandle nh("~");
    fence_publisher_ = nh.advertise<grape_msgs::VirtualFences>("/prohibition_areas_update",10);
}

PlanRoutePlugin::~PlanRoutePlugin()
{
    if (map_canvas_)
    {
        map_canvas_->removeEventFilter(this);
    }
}


void PlanRoutePlugin::Clear()
{
    if( ui_.radioRoute->isChecked()){
        waypoints_.clear();
    }
    else if( ui_.radioFences->isChecked()){
        fences_.clear();
    }
}

void PlanRoutePlugin::sendPlannedRoute()
{
  grape_msgs::PlanRoute msg;
  msg.request.header.frame_id = target_frame_;
  msg.request.header.stamp = ros::Time::now();
  msg.request.route_points.resize(  waypoints_.size() );

  for (size_t i = 0; i < waypoints_.size(); i++)
  {
      auto& route_point = msg.request.route_points[i];
      route_point.pos_x = waypoints_[i].pos_x;
      route_point.pos_y = waypoints_[i].pos_y;
      if( waypoints_[i].type == DEPLOYMENT) {
          route_point.type = grape_msgs::RoutePoint::DEPLOYMENT_POINT;
      }
      if( waypoints_[i].type == WAYPOINT) {
          route_point.type = grape_msgs::RoutePoint::WAYPOINT;
      }
  }

  bool server_up = planned_route_client_.waitForExistence( ros::Duration(1.0));

  if( !server_up )
  {
      PrintError("Service [plan_route] is down");
  }
  else{
      if (planned_route_client_.call(msg)) {
          PrintInfo("OK");
      }
      else {
          PrintError("Failed to call service [plan_route]");
      }
  }
}

void PlanRoutePlugin::sendProhibitedLines()
{
    grape_msgs::VirtualFences msg;
    grape_msgs::VirtualFence fence;

    stu::Transform transform;
    if( tf_manager_->GetTransform( "/map", target_frame_, transform) == false )
    {
        PrintError("Failed to transform from /map");
        return;
    }

    for (const Point2D& p: fences_)
    {
        tf::Vector3 tf_point(p.pos_x, p.pos_y, 0.0);
        tf_point = transform * tf_point;
        geometry_msgs::Point point;
        point.x = tf_point.x();
        point.y = tf_point.y();
        point.z = 0.0;

        if( p.type == FENCE_BEGIN)
        {
           fence.points.clear();
           fence.points.push_back(point);
        }
        else if( p.type == FENCE)
        {
            fence.points.push_back(point);
        }
        else if( p.type == FENCE_END)
        {
            fence.points.push_back(point);
            msg.fences.push_back(fence);
        }
    }
    fence_publisher_.publish(msg);
}

void PlanRoutePlugin::sendButtonPressed()
{
    if( ui_.radioDisabled->isChecked() ) return;

    if( ui_.radioRoute->isChecked() ){
        sendPlannedRoute();
    }
    else{
        sendProhibitedLines();
    }
}


void PlanRoutePlugin::PrintError(const std::string& message)
{
    PrintErrorHelper( ui_.status, message, 1.0);
}

void PlanRoutePlugin::PrintInfo(const std::string& message)
{
     PrintInfoHelper( ui_.status, message, 1.0);
}

void PlanRoutePlugin::PrintWarning(const std::string& message)
{
     PrintWarningHelper( ui_.status, message, 1.0);
}

QWidget* PlanRoutePlugin::GetConfigWidget(QWidget* parent)
{
    config_widget_->setParent(parent);
    return config_widget_;
}

bool PlanRoutePlugin::Initialize(QGLWidget* canvas)
{
    map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    planned_route_client_ = node_.serviceClient<grape_msgs::PlanRoute>("plan_route");

    return true;
}

bool PlanRoutePlugin::eventFilter(QObject *object, QEvent* event)
{
    if( this->Visible() == false)
    {
        return false;
    }
    if( ui_.radioRoute->isChecked())
    {
        switch (event->type())
        {
        case QEvent::MouseButtonPress:
            return handleMousePressRoute(static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonRelease:
            return handleMouseReleaseRoute(static_cast<QMouseEvent*>(event));
        case QEvent::MouseMove:
            return handleMouseMoveRoute(static_cast<QMouseEvent*>(event));
        case QEvent::KeyPress:
            return handleKeyPress(static_cast<QKeyEvent*>(event));
        case QEvent::KeyRelease:
            return handleKeyRelease(static_cast<QKeyEvent*>(event));
        default:
            return false;
        }
    }
    else if( ui_.radioFences->isChecked()) {
        switch (event->type())
        {
        case QEvent::MouseButtonPress:
            return handleMousePressFence(static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonRelease:
            return handleMouseReleaseFence(static_cast<QMouseEvent*>(event));
        case QEvent::MouseMove:
            return handleMouseMoveFence(static_cast<QMouseEvent*>(event));
        default:
            return false;
        }
    }
}

double minimum_distance(const tf::Vector3& v, const tf::Vector3& w, const tf::Vector3& p, double min_dist) {
    // Return minimum distance between line segment vw and point p
    const double l2 = tf::tfDistance2(v,w);  // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 <= min_dist*min_dist) return tf::tfDistance(p,v);   // v == w case
    double t = tf::tfDot(p - v, w - v) / l2;
    t = std::max( 0.0, std::min( 1.0, t ));
    const tf::Vector3 projection = v + t * (w - v);  // Projection falls on the segment
    return tf::tfDistance(p,projection);
}

QPointF PlanRoutePlugin::toGLCanvas(const Point2D& point)
{
    return  map_canvas_->FixedFrameToMapGlCoord(QPointF(point.pos_x, point.pos_y));
}

int PlanRoutePlugin::getPointSelected(QPointF gl_point,const std::vector<Point2D>& points)
{
    int closest_point = -1;

    double closest_distance = std::numeric_limits<double>::max();
    const double MIN_DISTANCE = 15;

    for (size_t i = 0; i < points.size(); i++)
    {
        QPointF transformed = toGLCanvas(points[i]);

        const double distance = QLineF(transformed, gl_point).length();

        if (distance < closest_distance && distance < MIN_DISTANCE)
        {
            closest_distance = distance;
            closest_point = static_cast<int>(i);
        }
    }
    return closest_point;
}


bool PlanRoutePlugin::handleMousePressRoute(QMouseEvent* event)
{
    // null if no point selected
    const QPointF gl_point = event->localPos();
    int closest_point = getPointSelected(gl_point, waypoints_);

    bool is_split_point = false;

    const double MIN_DISTANCE = 15;

    // check if you are inserting a point on a segment
    if( closest_point < 0 )
    {
        for (size_t i = 1; i < waypoints_.size(); i++)
        {
            QPointF A = toGLCanvas(waypoints_[i-1]);
            QPointF B = toGLCanvas(waypoints_[i]);

            tf::Vector3 v( A.x(), A.y(),  0.0);
            tf::Vector3 w( B.x(), B.y(),  0.0);
            tf::Vector3 p( gl_point.x(), gl_point.y(),  0.0);

            if( minimum_distance( v,w,p, MIN_DISTANCE ) < MIN_DISTANCE)
            {
                is_split_point = true;
                saveToUndoStack();

                QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(gl_point);
                tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
                Point2D wp;
                wp.pos_x = position.x();
                wp.pos_y = position.y();
                wp.type = (CTRL_pressed_) ? DEPLOYMENT : WAYPOINT;
                closest_point = i;
                waypoints_.insert( waypoints_.begin() + closest_point, wp ) ;
                break;
            }
        }
    }

    if (event->button() == Qt::LeftButton)
    {
        is_mouse_down_ = true;
        if ( closest_point >= 0 )
        {  
            selected_point_ = &waypoints_[closest_point];
            if( !is_split_point)
            {
                saveToUndoStack();
                if( CTRL_pressed_ )
                {
                    auto& type = selected_point_->type;
                    if( type == DEPLOYMENT )
                    {
                        type = WAYPOINT;
                    }
                    else if( type == WAYPOINT )
                    {
                        type = DEPLOYMENT;
                    }
                    selected_point_ = nullptr;
                }
            }
            return true;
        }
        else{
            saveToUndoStack();
            mouse_down_pos_ = event->localPos();
            mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
            selected_point_ = nullptr;
            return false;
        }
    }
    else if (event->button() == Qt::RightButton)
    {
        if (closest_point >= 0)
        {
            saveToUndoStack();

            waypoints_.erase(waypoints_.begin() + closest_point );
            return true;
        }
    }
    return false;
}

bool PlanRoutePlugin::handleMouseMoveRoute(QMouseEvent* event)
{
    if (selected_point_)
    {
        QPointF point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
        selected_point_->pos_x = point.x();
        selected_point_->pos_y = point.y();
        return true;
    }
    return false;
}

bool PlanRoutePlugin::handleMouseReleaseRoute(QMouseEvent* event)
{
    if (selected_point_)
    {
        QPointF point = map_canvas_->MapGlCoordToFixedFrame( event->localPos() );
        selected_point_->pos_x = point.x();
        selected_point_->pos_y = point.y();
        selected_point_ = nullptr;
        return true;
    }
    else if (is_mouse_down_)
    {
        qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
        qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

        // Only fire the event if the mouse has moved less than the maximum distance
        // and was held for shorter than the maximum time..  This prevents click
        // events from being fired if the user is dragging the mouse across the map
        // or just holding the cursor in place.
        if (msecsDiff < max_ms_ && distance <= max_distance_)
        {
            QPointF point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
            Point2D wp;
            wp.pos_x = point.x();
            wp.pos_y = point.y();
            wp.type = CTRL_pressed_ ? DEPLOYMENT : WAYPOINT;
            waypoints_.push_back(wp);
        }
    }
    is_mouse_down_ = false;

    return false;
}

bool PlanRoutePlugin::handleMousePressFence(QMouseEvent *event)
{
    selected_point_ = nullptr;
    const QPointF gl_point = event->localPos();
    const int closest_point = getPointSelected(gl_point, fences_);

    if( event->button() == Qt::LeftButton )
    {
        is_mouse_down_ = true;
        mouse_down_pos_ = event->localPos();
        mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

        if( fences_.empty() == false &&
            closest_point == fences_.size() -1 &&
            fences_.back().type == FENCE )
        {
            fences_.back().type = FENCE_END;
            is_mouse_down_ = false;
            selected_point_ = nullptr;
            building_fence_ = false;
            return true;
        }
        else if( closest_point >= 0)
        {
            selected_point_ = &fences_[closest_point];
            return true;
        }
    }
    else if( event->button() == Qt::RightButton)
    {
        if( building_fence_)
        {
            building_fence_ = false;
            if( fences_.back().type == FENCE_BEGIN )
            {
                fences_.pop_back();
            }
            else if( fences_.back().type == FENCE )
            {
                fences_.back().type = FENCE_END;
            }
            return true;
        }
        else if( closest_point >= 0 ) //delete existing point
        {
            saveToUndoStack();
            PointType type = fences_[closest_point].type;
            fences_.erase( fences_.begin() + closest_point );

            if( type == FENCE_BEGIN )
            {
                if( fences_[closest_point].type == FENCE )
                {
                    fences_[closest_point].type = FENCE_BEGIN;
                }
                else if(  fences_[closest_point].type == FENCE_END)
                {
                    fences_.erase( fences_.begin() + closest_point );
                }
            }
            else if( type == FENCE_END )
            {
                if( fences_[closest_point-1].type == FENCE_BEGIN)
                {
                    fences_.erase( fences_.begin() + closest_point -1 );
                }
                else{
                    fences_[closest_point-1].type = FENCE_END;
                }
            }
            return true;
        }
    }
    return false;
}

bool PlanRoutePlugin::handleMouseMoveFence(QMouseEvent *event)
{
    if( building_fence_ )
    {
        sticky_fence_end_ = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
    }
    else{
        if (selected_point_)
        {
            QPointF point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
            selected_point_->pos_x = point.x();
            selected_point_->pos_y = point.y();
            return true;
        }
    }
    return false;
}

bool PlanRoutePlugin::handleMouseReleaseFence(QMouseEvent *event)
{
    if (selected_point_)
    {
        QPointF point = map_canvas_->MapGlCoordToFixedFrame( event->localPos() );
        selected_point_->pos_x = point.x();
        selected_point_->pos_y = point.y();
        selected_point_ = nullptr;
        return true;
    }
    else if (is_mouse_down_)
    {
        qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
        qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;
        if (msecsDiff < max_ms_ && distance <= max_distance_)
        {
            QPointF point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
            Point2D wp;
            wp.pos_x = point.x();
            wp.pos_y = point.y();

            if( fences_.empty() || fences_.back().type == FENCE_END)
            {
                wp.type = FENCE_BEGIN;
                building_fence_ = true;
                saveToUndoStack();
            }
            else{
                wp.type = FENCE;
            }
            fences_.push_back(wp);
            sticky_fence_end_ = point;
        }
    }
    is_mouse_down_ = false;

    return false;
}

void PlanRoutePlugin::saveToUndoStack()
{
    RouteState state;
    state.wp = waypoints_;
    state.fn = fences_;
    undo_stack_.push_back( std::move(state) );
    while( undo_stack_.size()>100) undo_stack_.pop_front();
    redo_stack_.clear();
}


bool PlanRoutePlugin::handleKeyPress(QKeyEvent *event)
{
    if( event->key() == Qt::Key_Control)
    {
        CTRL_pressed_ = true;
    }
}

bool PlanRoutePlugin::handleKeyRelease(QKeyEvent *event)
{
    if( event->key() == Qt::Key_Control)
    {
        CTRL_pressed_ = false;
    }
}

bool doIntersect(QLineF l1, QLineF l2)
{
    // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

    // 0 --> p, q and r are colinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    auto orientation = [](QPointF p, QPointF q, QPointF r)
    {
        int val = (q.y() - p.y()) * (r.x() - q.x()) -
                  (q.x() - p.x()) * (r.y() - q.y());

        if (val == 0) return 0;
        return (val > 0)? 1: 2;
    };

    auto onSegment = [](QPointF p, QPointF q, QPointF r)
    {
        return (q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
                q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y()));
    };

    const QPointF& p1 = l1.p1();
    const QPointF& q1 = l1.p2();
    const QPointF& p2 = l2.p1();
    const QPointF& q2 = l2.p2();

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}


void PlanRoutePlugin::Draw(double x, double y, double scale)
{
    glLineWidth(2);
    glBegin(GL_LINES);
    for (size_t i = 1; i < waypoints_.size(); i++)
    {      
        const QPointF p1(waypoints_[i-1].pos_x, waypoints_[i-1].pos_y);
        const QPointF p2(waypoints_[i].pos_x,   waypoints_[i].pos_y);
        const QLineF segment(p1, p2);

        bool cross_fence = false;

        for (int f=1; f < fences_.size() && !cross_fence ; f++)
        {
            if( fences_[f-1].type != FENCE_END)
            {
                const QPointF f1(fences_[f-1].pos_x, fences_[f-1].pos_y);
                const QPointF f2(fences_[f].pos_x,   fences_[f].pos_y);
                const QLineF fence(f1, f2);
                cross_fence = doIntersect(fence, segment);
            }
        }
        if( !cross_fence ) glColor4d( 0.0, 1.0, 0.0, 1.0);
        else               glColor4d( 1.0, 0.6, 0.0, 1.0);

        glVertex2d(p1.x(), p1.y());
        glVertex2d(p2.x(), p2.y());
    }
    glEnd();

    // Draw waypoints
    glPointSize(20);
    glBegin(GL_POINTS);

    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        QPointF point(waypoints_[i].pos_x, waypoints_[i].pos_y);
        if( waypoints_[i].type == WAYPOINT ){
            glColor4f(0.0, 1.0, 1.0, 1.0);
        }
        else if( waypoints_[i].type == DEPLOYMENT ){
            glColor4f(1.0, 1.0, 0.0, 1.0);
        }
        glVertex2d(point.x(), point.y());
    }
    glEnd();
    //--------------------------
    glLineWidth(2);
    glColor4d( 1.0, 0.0, 0.0, 1.0);

    for (const auto fence_point: fences_)
    {
        if(fence_point.type == FENCE_BEGIN)
        {
            glBegin(GL_LINE_STRIP);
        }

        glVertex2d( fence_point.pos_x, fence_point.pos_y );

        if(fence_point.type == FENCE_END)
        {
            glEnd();
        }
    }
    if( fences_.empty() == false && fences_.back().type != FENCE_END)
    {
        glVertex2d( sticky_fence_end_.x(), sticky_fence_end_.y() );
        glEnd();
    }

    glPointSize(5);
    glBegin(GL_POINTS);
    glColor4d( 1.0, 0.0, 0.0, 1.0);

    for (const auto fence_point: fences_)
    {
//        if( fence_point.type == FENCE)        glColor4d( 0.0, 1.0, 0.0, 1.0);
//        if( fence_point.type == FENCE_END)    glColor4d( 0.0, 0.0, 1.0, 1.0);

        glVertex2d( fence_point.pos_x, fence_point.pos_y );
    }
    glEnd();
}

void PlanRoutePlugin::Paint(QPainter* painter, double x, double y, double scale)
{
    painter->save();
    painter->resetTransform();

    QPen pen(QBrush(QColor(Qt::darkCyan).darker()), 1);
    painter->setPen(pen);
    painter->setFont(QFont("DejaVu Sans Mono", 10));

    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        QPointF gl_point = toGLCanvas(waypoints_[i]);
        QPointF corner(gl_point.x() - 20, gl_point.y() - 20);
        QRectF rect(corner, QSizeF(40, 40));
        painter->drawText(rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number( i + 1 ));
    }
    painter->restore();
}

void PlanRoutePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
//    if (node["color"])
//    {
//        std::string color;
//        node["color"] >> color;
//        ui_.color->setColor(QColor(color.c_str()));
//    }
}

void PlanRoutePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
//    std::string color = ui_.color->color().name().toStdString();
//    emitter << YAML::Key << "color" << YAML::Value << color;
}

void PlanRoutePlugin::on_pushButtonRedo_clicked()
{
    if( redo_stack_.size() >= 1)
    {
        RouteState state;
        state.wp = waypoints_;
        state.fn = fences_;
        undo_stack_.push_back( std::move(state) );

        while( undo_stack_.size()>100) undo_stack_.pop_front();

        waypoints_ = redo_stack_.back().wp;
        fences_ = redo_stack_.back().fn;
        redo_stack_.pop_back();
    }
}

void PlanRoutePlugin::on_pushButtonUndo_clicked()
{
    if( undo_stack_.size() >= 1)
    {
        RouteState state;
        state.wp = waypoints_;
        state.fn = fences_;
        redo_stack_.push_back( std::move(state) );
        while( redo_stack_.size()>100) redo_stack_.pop_front();

        waypoints_ = undo_stack_.back().wp;
        fences_ = undo_stack_.back().fn;
        undo_stack_.pop_back();
    }
}

void PlanRoutePlugin::on_pushButtonLoad_clicked()
{
    if( ui_.radioDisabled->isChecked() ) return;

    FileType file_type = ROUTE_FILE;
    if( ui_.radioFences->isChecked() ) file_type = FENCE_FILE;

    stu::Transform transform;
    if( tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform) == false )
    {
        PrintError("Failed to transform to wgs84");
        return;
    }

    QSettings settings( "Eurecat", "MapvizGrape");
    // use previous directory or the current one
    QString directory_path  = settings.value("PlanRoutePlugin.on_pushButtonLoad_clicked",
                                             QDir::currentPath() ). toString();

    QString filter("*.route.yaml");
    if( file_type == FENCE_FILE){
        filter = "*.fence.yaml";
    }
    QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Route File"),
                                                    directory_path,
                                                    filter );
    if (filename.isEmpty()) {
        return;
    }
    //save the directory for the next time
    directory_path = QFileInfo(filename).absolutePath();
    settings.setValue("PlanRoutePlugin.on_pushButtonLoad_clicked", directory_path);
    //---------------------------------------------

    YAML::Node node = YAML::LoadFile(filename.toStdString().c_str());

    if( file_type == ROUTE_FILE )
    {
        waypoints_.clear();
        YAML::Node wp_node = node["waypoints"];

        for(std::size_t i=0; i<wp_node.size(); i++)
        {
          double lat, lon;
          lat = wp_node[i][0].as<double>();
          lon = wp_node[i][1].as<double>();

          Point2D wp;
          tf::Vector3 point(lat, lon, 0.0);
          point = transform * point;
          wp.pos_x = point.x();
          wp.pos_y = point.y();

          if( wp_node[i][2].Scalar() == "deployment" ){
             wp.type = DEPLOYMENT;
          }
          else{
             wp.type = WAYPOINT;
          }
          waypoints_.push_back(wp);
        }
    }
    else{
        fences_.clear();
        YAML::Node wp_node = node["fences"];

        for(std::size_t i=0; i<wp_node.size(); i++)
        {
          double lat, lon;
          lat = wp_node[i][0].as<double>();
          lon = wp_node[i][1].as<double>();

          Point2D wp;
          tf::Vector3 point(lat, lon, 0.0);
          point = transform * point;
          wp.pos_x = point.x();
          wp.pos_y = point.y();

          if( wp_node[i][2].Scalar() == "fence_begin" ){
             wp.type = FENCE_BEGIN;
          }
          else if( wp_node[i][2].Scalar() == "fence_end" ){
             wp.type = FENCE_END;
          }
          else{
             wp.type = FENCE;
          }
          fences_.push_back(wp);
        }
    }
}

void PlanRoutePlugin::on_pushButtonSave_clicked()
{
    if( ui_.radioDisabled->isChecked() ) return;

    FileType file_type = ROUTE_FILE;
    if( ui_.radioFences->isChecked() ) file_type = FENCE_FILE;

    stu::Transform transform;
    if( ROUTE_FILE )
    {
        if( tf_manager_->GetTransform( stu::_wgs84_frame, target_frame_, transform) == false )
        {
            PrintError("Failed to transform from wgs84");
            return;
        }
    }
    else{
        if( tf_manager_->GetTransform( stu::_wgs84_frame, target_frame_, transform) == false )
        {
            PrintError("Failed to transform from wgs84");
            return;
        }
    }

    QSettings settings( "Eurecat", "MapvizGrape");

    // use previous directory or the current one
    QString directory_path  = settings.value("PlanRoutePlugin.on_pushButtonSave_clicked",
                                             QDir::currentPath() ). toString();

    QFileDialog saveDialog;
    saveDialog.setAcceptMode(QFileDialog::AcceptSave);

    if( file_type == ROUTE_FILE){
        saveDialog.setDefaultSuffix("route.yaml");
        saveDialog.setNameFilter("YAML (*.route.yaml)");
    }
    else{
        saveDialog.setDefaultSuffix("fence.yaml");
        saveDialog.setNameFilter("YAML (*.fence.yaml)");
    }
    saveDialog.setDirectory(directory_path);
    saveDialog.exec();

    if(saveDialog.result() != QDialog::Accepted || saveDialog.selectedFiles().empty()) {
        return;
    }

    QString fileName = saveDialog.selectedFiles().first();

    if (fileName.isEmpty()){
        return;
    }

    std::ofstream fout(fileName.toStdString().c_str() );
    if (fout.fail())
    {
      ROS_ERROR("Failed to open file: %s", fileName.toStdString().c_str() );
      return;
    }
    //save the directory for the next time
    directory_path = QFileInfo(fileName).absolutePath();
    settings.setValue("PlanRoutePlugin.on_pushButtonSave_clicked", directory_path);
    //----------------------------------------

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "frame" << YAML::Value << "wgs84";
    if( file_type == ROUTE_FILE )
    {
        out << YAML::Key << "waypoints";
        out << YAML::BeginSeq;
        for (int i=0; i< waypoints_.size(); i++)
        {
            out << YAML::Flow;
            out << YAML::BeginSeq;

            tf::Vector3 point(waypoints_[i].pos_x, waypoints_[i].pos_y, 0.0);
            point = transform * point;

            out <<  point.x();
            out <<  point.y();
            if(waypoints_[i].type == DEPLOYMENT){
                out << "deployment";
            }
            else{
                out << "waypoint";
            }
            out << YAML::EndSeq;
        }
    }
    else{
        out << YAML::Key << "fences";
        out << YAML::BeginSeq;
        for (int i=0; i< fences_.size(); i++)
        {
            out << YAML::Flow;
            out << YAML::BeginSeq;

            tf::Vector3 point(fences_[i].pos_x, fences_[i].pos_y, 0.0);
            point = transform * point;

            out <<  point.x();
            out <<  point.y();
            if(fences_[i].type == FENCE_BEGIN){
                out << "fence_begin";
            }
            else if(fences_[i].type == FENCE_END){
                out << "fence_end";
            }
            else{
                out << "fence";
            }
            out << YAML::EndSeq;
        }
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    out.c_str();

    fout << out.c_str();
    fout.close();
}

void PlanRoutePlugin::on_radio_toggled(bool checked)
{
    if( ui_.radioDisabled->isChecked() )
    {
        ui_.groupCommands->setEnabled( false );
        ui_.groupCommands->setTitle("Commands (disabled)");
    }
    else{
        ui_.groupCommands->setEnabled( true );
        if( ui_.radioRoute->isChecked() )
        {
            ui_.groupCommands->setTitle("Route Commands");
        }
        else if( ui_.radioFences->isChecked() )
        {
            ui_.groupCommands->setTitle("Fence Commands");
        }
    }
}

}
