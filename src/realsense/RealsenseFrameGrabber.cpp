/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Synchronous capture of camera images using Intel's Realsense library.
 *
 * @author Ulrich Eck <ulrich.eck@tum.de>
 *
 */

#include "RealsenseFrameGrabber.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <utDataflow/ComponentFactory.h>
#include <utUtil/OS.h>
#include <boost/array.hpp>

#include <opencv2/opencv.hpp>

#include <log4cpp/Category.hh>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.device_camera_realsense.RealsenseFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Drivers;


namespace Ubitrack { namespace Drivers {

    void RealsenseModule::setupDevice()
    {
        auto devices = m_ctx.query_devices();
        size_t device_count = devices.size();
        if (!device_count) {
            UBITRACK_THROW("No Realsense camera connected.");
        }

        bool found_device = false;

        // if serialnumber == 0 then use first device
        if (m_serialNumber == 0) {
            m_dev = std::make_shared<rs2::device>(devices.front());
            found_device = true;
        } else {
            for (auto i = 0; i < device_count; ++i)
            {
                auto dev = devices[i];
                // do we need to catch an exception here ?
                int serial_number = std::stoi(devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
                if (m_serialNumber == serial_number) {
                    m_dev = std::make_shared<rs2::device>(devices[i]);
                    found_device = true;
                }
            }
        }

        if (!found_device) {
            UBITRACK_THROW("No Realsense camera with given serial number found");
        }

        // initialize components
        ComponentList allComponents = getAllComponents();

        // get stream requests
        m_stream_requests.clear();

        for (auto i = allComponents.begin(); i != allComponents.end(); ++i) {
            switch((*i)->getSensorType()) {
                case REALSENSE_SENSOR_CONFIG:
                {
                    break;
                }
                case REALSENSE_SENSOR_VIDEO:
                {
                    auto vc = boost::static_pointer_cast<RealsenseVideoComponent>(*i);
                    m_stream_requests.push_back({ vc->getSensorType(),
                                                  vc->getStreamType(),
                                                  vc->getStreamFormat(),
                                                  vc->getImageWidth(),
                                                  vc->getImageHeight(),
                                                  vc->getFrameRate(),
                                                  vc->getStreamIndex()
                                                });
                    break;
                }
                case REALSENSE_SENSOR_DEPTH:
                {
                    auto pc = boost::static_pointer_cast<RealsensePointCloudComponent>(*i);
                    m_stream_requests.push_back({ pc->getSensorType(),
                                                  pc->getStreamType(),
                                                  pc->getStreamFormat(),
                                                  pc->getImageWidth(),
                                                  pc->getImageHeight(),
                                                  pc->getFrameRate(),
                                                  pc->getStreamIndex()
                                                });
                    break;
                }
            }
        }

        if (!m_stream_requests.empty()) {
            std::sort(m_stream_requests.begin(), m_stream_requests.end(),
                      [](const stream_request &l, const stream_request &r) {
                          return l._stream_type < r._stream_type;
                      });

            for (auto i = 0; i < m_stream_requests.size() - 1; i++) {
                if ((m_stream_requests[i]._stream_type == m_stream_requests[i + 1]._stream_type) &&
                    ((m_stream_requests[i]._stream_idx == m_stream_requests[i + 1]._stream_idx)))
                    UBITRACK_THROW("Invalid configuration - multiple requests for the same sensor");
            }
        }

        // configure sensors
        bool succeed = false;
        std::vector<rs2::stream_profile> matches;
        size_t expected_number_of_streams = m_stream_requests.size();

        // Configure and starts streaming
        for (auto&& sensor : m_dev->query_sensors())
        {
            for (auto& profile : sensor.get_stream_profiles())
            {
                // All requests have been resolved
                if (m_stream_requests.empty())
                    break;

                // Find profile matches
                auto fulfilled_request = std::find_if(m_stream_requests.begin(), m_stream_requests.end(),
                        [&matches, profile](const stream_request& req)
                    {
                        bool res = false;
                        if ((profile.stream_type() == req._stream_type) &&
                            (profile.format() == req._stream_format) &&
                            (profile.stream_index() == req._stream_idx) &&
                            (profile.fps() == req._fps))
                        {
                            if (auto vp = profile.as<rs2::video_stream_profile>())
                            {
                                if ((vp.width() != req._width) || (vp.height() != req._height))
                                    return false;
                            }
                            res = true;
                            matches.emplace_back(profile);
                        }

                        return res;
                    });

                // Remove the request once resolved
                if (fulfilled_request != m_stream_requests.end())
                    m_stream_requests.erase(fulfilled_request);
            }

            // Aggregate resolved requests
            if (!matches.empty())
            {
                std::copy(matches.begin(), matches.end(), std::back_inserter(m_selected_stream_profiles));
                sensor.open(matches);
                m_active_sensors.emplace_back(sensor);
                matches.clear();
            }

            if (m_selected_stream_profiles.size() == expected_number_of_streams)
                succeed = true;
        }

        if (!succeed) {
            UBITRACK_THROW("Could not find matching stream profiles for all components.");
        }
    }

    void RealsenseModule::startModule()
    {
        if ( !m_running )
        {
            // check if oclmanager is active
            Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
            if ((oclManager.isEnabled()) && (oclManager.isActive()) && (!oclManager.isInitialized())) {
                LOG4CPP_INFO(logger, "Waiting for OpenCLManager Initialization callback.");
                oclManager.registerInitCallback(boost::bind(&RealsenseModule::startCapturing, this));
            } else {
                startCapturing();
            }
            m_running = true;
        }
    }

    void RealsenseModule::startCapturing() {

        setupDevice();

        /** D435 Options
            setting options works on sensors not on streams.
            not sure how to implement this in a useful way

            Options for Stereo Module
             Supported options:                                    min        max       step  default
                Exposure                                           : 20   ... 166000      20    8500
                Gain                                               : 16   ... 248         1     16
                Enable Auto Exposure                               : 0    ... 1           1     1
                Visual Preset                                      : 0    ... 6           1     0
                Laser Power                                        : 0    ... 360         30    150
                Emitter Enabled                                    : 0    ... 2           1     1
                Frames Queue Size                                  : 0    ... 32          1     16
                Error Polling Enabled                              : 0    ... 1           1     0
                Output Trigger Enabled                             : 0    ... 1           1     0
                Depth Units                                        : 0.0001... 0.01        1e-06 0.001
                Stereo Baseline                                    : 49.9954... 49.9954     0     49.9954

            Options for RGB Camera
             Supported options:                                    min        max       step  default
                Backlight Compensation                             : 0    ... 1           1     0
                Brightness                                         : -64  ... 64          1     0
                Contrast                                           : 0    ... 100         1     50
                Exposure                                           : 41   ... 10000       1     166
                Gain                                               : 0    ... 128         1     64
                Gamma                                              : 100  ... 500         1     300
                Hue                                                : -180 ... 180         1     0
                Saturation                                         : 0    ... 100         1     64
                Sharpness                                          : 0    ... 100         1     50
                White Balance                                      : 2800 ... 6500        10    4600
                Enable Auto Exposure                               : 0    ... 1           1     1
                Enable Auto White Balance                          : 0    ... 1           1     1
                Frames Queue Size                                  : 0    ... 32          1     16
                Power Line Frequency                               : 0    ... 2           1     3
                Auto Exposure Priority                             : 0    ... 1           1     0
         */


        // Start streaming
        for (auto&& sensor : m_active_sensors) {
            sensor.start([this](rs2::frame f)
                         {
                             handleFrame(f);
                         });
        }

    }

    void RealsenseModule::handleFrame(rs2::frame f) {

        // convert from frame timestamp (milliseconds, double) to Measurement::Timestamp (nanoseconds, unsigned long long)
        auto ts = (Measurement::Timestamp)(f.get_timestamp() * 1000000);

        rs2_stream stream_type = f.get_profile().stream_type();
        RealsenseSensorType sensor_type;
        switch(stream_type) {
            case rs2_stream::RS2_STREAM_COLOR:
            case rs2_stream::RS2_STREAM_INFRARED:
                sensor_type = REALSENSE_SENSOR_VIDEO;
                break;

            case rs2_stream::RS2_STREAM_DEPTH:
                sensor_type = REALSENSE_SENSOR_DEPTH;
                break;

            default:
                LOG4CPP_WARN(logger, "RealsenseModule received frame with unhandled stream_type: " << stream_type);
                return;
        }

        RealsenseComponentKey key(sensor_type, stream_type, (unsigned int)(f.get_profile().stream_index()));
        if ( hasComponent( key ) ) {
            switch( sensor_type ) {
                case REALSENSE_SENSOR_VIDEO:
                {
                    auto vc = boost::static_pointer_cast<RealsenseVideoComponent>(getComponent( key ));
                    vc->handleFrame(ts, f);
                    break;
                }

                case REALSENSE_SENSOR_DEPTH:
                {
                    auto pc = boost::static_pointer_cast<RealsensePointCloudComponent>(getComponent( key ));
                    pc->handleFrame(ts, f);
                    break;
                }
            }
        }
    }

    void RealsenseModule::stopModule()
    {
        if ( m_running )
        {
            m_running = false;
            LOG4CPP_INFO( logger, "Trying to stop Realsense module");

            for (auto&& sensor : m_active_sensors) {
                sensor.stop();
                sensor.close();
            }

            teardownDevice();
        }
    }

    void RealsenseModule::teardownDevice()
    {
        m_dev.reset();
    }


    boost::shared_ptr< RealsenseComponent > RealsenseModule::createComponent( const std::string& type,
                                                                  const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
                                                                  const ComponentKey& key, ModuleClass* pModule )
    {

        if ( type == "RealsenseVideoStream" )
            return boost::shared_ptr< RealsenseComponent >( new RealsenseVideoComponent( name, subgraph, key, pModule ) );
//            else if ( type == "RealsenseCameraCalibration" )
//                return boost::shared_ptr< RealsenseComponent >( new RealsenseCameraCalibrationComponent( name, subgraph, key, pModule ) );
        else if ( type == "RealsensePointCloud" )
            return boost::shared_ptr< RealsenseComponent >( new RealsensePointCloudComponent( name, subgraph, key, pModule ) );

        UBITRACK_THROW( "Class " + type + " not supported by Realsense module" );
    }

    void RealsenseVideoComponent::handleFrame(Measurement::Timestamp ts, rs2::frame f) {
        if (auto vf = f.as<rs2::video_frame>())
        {
            auto imageFormatProperties = Vision::Image::ImageFormatProperties();
            switch (f.get_profile().format()) {
                case RS2_FORMAT_BGRA8:
                    imageFormatProperties.depth = CV_8U;
                    imageFormatProperties.channels = 4;
                    imageFormatProperties.matType = CV_8UC4;
                    imageFormatProperties.bitsPerPixel = 32;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::BGRA;
                    break;

                case RS2_FORMAT_BGR8:
                    imageFormatProperties.depth = CV_8U;
                    imageFormatProperties.channels = 3;
                    imageFormatProperties.matType = CV_8UC3;
                    imageFormatProperties.bitsPerPixel = 24;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::BGR;
                    break;

                case RS2_FORMAT_RGBA8:
                    imageFormatProperties.depth = CV_8U;
                    imageFormatProperties.channels = 4;
                    imageFormatProperties.matType = CV_8UC4;
                    imageFormatProperties.bitsPerPixel = 32;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::RGBA;
                    break;

                case RS2_FORMAT_RGB8:
                    imageFormatProperties.depth = CV_8U;
                    imageFormatProperties.channels = 3;
                    imageFormatProperties.matType = CV_8UC3;
                    imageFormatProperties.bitsPerPixel = 24;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::RGB;
                    break;

                case RS2_FORMAT_Y16:
                    imageFormatProperties.depth = CV_16U;
                    imageFormatProperties.channels = 1;
                    imageFormatProperties.matType = CV_16UC1;
                    imageFormatProperties.bitsPerPixel = 16;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
                    break;

                case RS2_FORMAT_Y8:
                    imageFormatProperties.depth = CV_8U;
                    imageFormatProperties.channels = 1;
                    imageFormatProperties.matType = CV_8UC1;
                    imageFormatProperties.bitsPerPixel = 8;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
                    break;

                case RS2_FORMAT_Z16:
                    imageFormatProperties.depth = CV_16U;
                    imageFormatProperties.channels = 1;
                    imageFormatProperties.matType = CV_16UC1;
                    imageFormatProperties.bitsPerPixel = 16;
                    imageFormatProperties.origin = 0;
                    imageFormatProperties.imageFormat = Vision::Image::DEPTH;
                    break;

                default:
                    UBITRACK_THROW("Realsense frame format is not supported!");
            }

            int w = vf.get_width();
            int h = vf.get_height();

            // need to copy image here.
            auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void*)f.get_data(), cv::Mat::AUTO_STEP).clone();

            boost::shared_ptr< Vision::Image > pColorImage(new Vision::Image(image));
            pColorImage->set_pixelFormat(imageFormatProperties.imageFormat);
            pColorImage->set_origin(imageFormatProperties.origin);

            if (m_autoGPUUpload) {
                Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
                if (oclManager.isInitialized()) {
                    //force upload to the GPU
                    pColorImage->uMat();
                }
            }

            m_outputPort.send(Measurement::ImageMeasurement(ts, pColorImage));

            // should we add a grayImageOutputPort here ?

        } else {
            LOG4CPP_WARN(logger, "ignoring non-video frame received in RealsenseVideoComponent.");
        }
    }

    void RealsensePointCloudComponent::handleFrame(Measurement::Timestamp ts, rs2::frame f) {
        if (auto df = f.as<rs2::depth_frame>())
        {
            if (m_outputPort.isConnected()) {
                // Declare pointcloud object, for calculating pointclouds and texture mappings
                rs2::pointcloud pc;

                // Generate the pointcloud and texture mappings
                rs2::points points = pc.calculate(df);

                // Tell pointcloud object to map to this color frame
                // @todo: currently no access to the color image .. now sure how to achieve this with the current structure ..
                // pc.map_to(color);


                auto vertices = points.get_vertices();

                Math::Vector3d init_pos(0, 0, 0);
                boost::shared_ptr < std::vector<Math::Vector3d> > pPointCloud = boost::make_shared< std::vector<Math::Vector3d> >(points.size(), init_pos);

                for (size_t i = 0; i < points.size(); i++) {
                    Math::Vector3d& p = pPointCloud->at(i);

                    if (vertices[i].z != 0.)
                    {
                        p[0] = vertices[i].x;
                        p[1] = vertices[i].y;
                        p[2] = vertices[i].z;
                    } else {
                        p[0] = p[1] = p[2] = 0.;
                    }
                }

                m_outputPort.send(Measurement::PositionList(ts, pPointCloud));
            }

            if (m_outputDepthmapPort.isConnected()) {

                auto imageFormatProperties = Vision::Image::ImageFormatProperties();
                switch (f.get_profile().format()) {
                    case RS2_FORMAT_Z16:
                        imageFormatProperties.depth = CV_16U;
                        imageFormatProperties.channels = 1;
                        imageFormatProperties.matType = CV_16UC1;
                        imageFormatProperties.bitsPerPixel = 16;
                        imageFormatProperties.origin = 0;
                        imageFormatProperties.imageFormat = Vision::Image::DEPTH;
                        break;

                    default:
                        UBITRACK_THROW("Realsense frame format is not supported!");
                }

                int w = df.get_width();
                int h = df.get_height();

                // need to copy image here.
                auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void*)f.get_data(), cv::Mat::AUTO_STEP).clone();

                boost::shared_ptr< Vision::Image > pDepthImage(new Vision::Image(image));
                pDepthImage->set_pixelFormat(imageFormatProperties.imageFormat);
                pDepthImage->set_origin(imageFormatProperties.origin);

                if (m_autoGPUUpload) {
                    Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
                    if (oclManager.isInitialized()) {
                        //force upload to the GPU
                        pDepthImage->uMat();
                    }
                }

                m_outputDepthmapPort.send(Measurement::ImageMeasurement(ts, pDepthImage));

            }
        }
    }


} } // namespace Ubitrack::Drivers
