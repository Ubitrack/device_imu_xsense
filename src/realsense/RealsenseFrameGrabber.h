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
 * @ingroup driver_components
 * @file
 * Realsense driver
 * This file contains the driver component to
 * synchronously capture camera images using realsense.
 *
 * The received data is sent via a push interface.
 *
 * @author Ulrich Eck <ulrick.eck@tum.de>
 */

#ifndef __RealsenseFramegrabber_h_INCLUDED__
#define __RealsenseFramegrabber_h_INCLUDED__


#include <string>
#include <cstdlib>

#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utVision/OpenCLManager.h>
#include <utUtil/TracingProvider.h>

#include <utVision/Image.h>
#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>

namespace Ubitrack { namespace Drivers {
        enum RealsenseSensorType {
            REALSENSE_SENSOR_CONFIG = 0,
            REALSENSE_SENSOR_VIDEO,
            REALSENSE_SENSOR_DEPTH
        };
}}

namespace {

	class RealsenseSensorTypeMap : public std::map< std::string, Ubitrack::Drivers::RealsenseSensorType > {
    public:
        RealsenseSensorTypeMap() {
            (*this)["CONFIG"] = Ubitrack::Drivers::REALSENSE_SENSOR_CONFIG;
            (*this)["VIDEO"] = Ubitrack::Drivers::REALSENSE_SENSOR_VIDEO;
            (*this)["DEPTH"] = Ubitrack::Drivers::REALSENSE_SENSOR_DEPTH;
        }
    };
    static RealsenseSensorTypeMap realsenseSensorTypeMap;

    class RealsenseStreamTypeMap : public std::map< std::string, rs2_stream > {
    public:
        RealsenseStreamTypeMap() {
            (*this)["COLOR"] = rs2_stream::RS2_STREAM_COLOR;
            (*this)["INFRARED"] = rs2_stream::RS2_STREAM_INFRARED;
            (*this)["DEPTH"] = rs2_stream::RS2_STREAM_DEPTH;
            (*this)["CONFIDENCE"] = rs2_stream::RS2_STREAM_CONFIDENCE;
            (*this)["ACCEL"] = rs2_stream::RS2_STREAM_ACCEL;
            (*this)["GYRO"] = rs2_stream::RS2_STREAM_GYRO;
            (*this)["POSE"] = rs2_stream::RS2_STREAM_POSE;
            (*this)["FISHEYE"] = rs2_stream::RS2_STREAM_FISHEYE;
            (*this)["GPIO"] = rs2_stream::RS2_STREAM_GPIO;
        }
    };
    static RealsenseStreamTypeMap realsenseStreamTypeMap;

    class RealsenseStreamFormatMap : public std::map< std::string, rs2_format > {
    public:
        RealsenseStreamFormatMap() {
            (*this)["Z16"] = rs2_format::RS2_FORMAT_Z16;
            (*this)["DISPARITY16"] = rs2_format::RS2_FORMAT_DISPARITY16;
            (*this)["XYZ32F"] = rs2_format::RS2_FORMAT_XYZ32F;
            (*this)["YUYV"] = rs2_format::RS2_FORMAT_YUYV;
            (*this)["RGB8"] = rs2_format::RS2_FORMAT_RGB8;
            (*this)["BGR8"] = rs2_format::RS2_FORMAT_BGR8;
            (*this)["RGBA8"] = rs2_format::RS2_FORMAT_RGBA8;
            (*this)["BGRA8"] = rs2_format::RS2_FORMAT_BGRA8;
            (*this)["Y8"] = rs2_format::RS2_FORMAT_Y8;
            (*this)["Y16"] = rs2_format::RS2_FORMAT_Y16;
            (*this)["RAW10"] = rs2_format::RS2_FORMAT_RAW10;
            (*this)["RAW16"] = rs2_format::RS2_FORMAT_RAW16;
            (*this)["RAW8"] = rs2_format::RS2_FORMAT_RAW8;
        }
    };
    static RealsenseStreamFormatMap realsenseStreamFormatMap;

    class RealsenseStreamResolutionMap : public std::map< std::string, std::tuple< unsigned int, unsigned int> > {
    public:
        RealsenseStreamResolutionMap() {
            (*this)["1920x1080"] = std::make_tuple(1920, 1080);
            (*this)["1280x800"] = std::make_tuple(1280, 800);
            (*this)["1280x720"] = std::make_tuple(1280, 720);
            (*this)["960x540"] = std::make_tuple(960, 540);
            (*this)["848x480"] = std::make_tuple(848, 480);
            (*this)["640x480"] = std::make_tuple(640, 480);
            (*this)["640x400"] = std::make_tuple(640, 400);
            (*this)["640x360"] = std::make_tuple(640, 360);
            (*this)["480x270"] = std::make_tuple(480, 270);
            (*this)["424x240"] = std::make_tuple(424, 240);
            (*this)["320x240"] = std::make_tuple(320, 240);
            (*this)["320x180"] = std::make_tuple(320, 180);
        }
    };
    static RealsenseStreamResolutionMap realsenseStreamResolutionMap;


} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

    struct stream_request
    {
        Ubitrack::Drivers::RealsenseSensorType _sensor_type;
        rs2_stream   _stream_type;
        rs2_format   _stream_format;
        unsigned int _width;
        unsigned int _height;
        unsigned int _fps;
        unsigned int _stream_idx;
    };


        // forward declaration
    class RealsenseComponent;

/**
 * Module key for art.
 * Represents the port number on which to listen.
 */
    MAKE_DATAFLOWCONFIGURATIONATTRIBUTEKEY_DEFAULT( RealsenseModuleKey, int, "rsSerialNumber", 0 );


/**
 * Component key for realsense camera.
 * Represents the camera
 */
		class RealsenseComponentKey
		{
		public:
			RealsenseComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
					: m_sensor_type( REALSENSE_SENSOR_VIDEO )
					, m_stream_type( rs2_stream::RS2_STREAM_COLOR )
					, m_stream_index ( 0 ) {

				if (subgraph->m_DataflowClass == "RealsenseCameraCalibration") {
					m_sensor_type = REALSENSE_SENSOR_CONFIG;
					// there is only one calibration component per device, therefore we're assuming default values for stream_type and index
					return;
				} else if (subgraph->m_DataflowClass == "RealsenseVideoStream") {
					m_sensor_type = REALSENSE_SENSOR_VIDEO;
				} else if (subgraph->m_DataflowClass == "RealsensePointCloud") {
					m_sensor_type = REALSENSE_SENSOR_DEPTH;
				} else   {
					UBITRACK_THROW("Realsense Camera: Invalid Component Sensor Type.");
				}

				if (subgraph->m_DataflowAttributes.hasAttribute("rsSensorType")) {
					std::string sSensorType = subgraph->m_DataflowAttributes.getAttributeString("rsSensorType");
					if (realsenseSensorTypeMap.find(sSensorType) == realsenseSensorTypeMap.end())
						UBITRACK_THROW("unknown sensor type: \"" + sSensorType + "\"");
					m_sensor_type = realsenseSensorTypeMap[sSensorType];
				}

				if (subgraph->m_DataflowAttributes.hasAttribute("rsStreamType")) {
					std::string sStreamType = subgraph->m_DataflowAttributes.getAttributeString("rsStreamFormat");
					if (realsenseStreamTypeMap.find(sStreamType) == realsenseStreamTypeMap.end())
						UBITRACK_THROW("unknown stream type: \"" + sStreamType + "\"");
					m_stream_type = realsenseStreamTypeMap[sStreamType];
				}

				if (subgraph->m_DataflowAttributes.hasAttribute("rsStreamIndex")) {
                    subgraph->m_DataflowAttributes.getAttributeData("rsStreamIndex", m_stream_index);
				}
			}


			// construct from given values
			RealsenseComponentKey( RealsenseSensorType  t, rs2_stream v, unsigned int i)
					: m_sensor_type( t )
					, m_stream_type( v )
					, m_stream_index( i )
			{}

            RealsenseSensorType getSensorType() const
			{
				return m_sensor_type;
			}

            rs2_stream getStreamType() const
			{
				return m_stream_type;
			}

            unsigned int getStreamIndex() const
            {
                return m_stream_index;
            }

            // less than operator for map
			bool operator<( const RealsenseComponentKey& b ) const
			{
				if ( m_sensor_type == b.m_sensor_type )
                    if ( m_stream_type == b.m_stream_type )
                        return m_stream_index < b.m_stream_index;
                    else
                        return m_stream_type < b.m_stream_type;
				else
					return m_sensor_type < b.m_sensor_type;
			}

		protected:
			RealsenseSensorType m_sensor_type;
			rs2_stream m_stream_type;
			unsigned int m_stream_index;
		};


		std::ostream& operator<<( std::ostream& s, const RealsenseComponentKey& k );



        /**
 * Module for Realsense camera.
 */
        class RealsenseModule
                : public Module< RealsenseModuleKey, RealsenseComponentKey, RealsenseModule, RealsenseComponent >
        {
        public:
            /** UTQL constructor */
            RealsenseModule( const RealsenseModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
                    : Module< RealsenseModuleKey, RealsenseComponentKey, RealsenseModule, RealsenseComponent >(key, pFactory)
                    , m_serialNumber(0)
            {
                if (subgraph->m_DataflowAttributes.hasAttribute("rsSerialNumber")) {
                    subgraph->m_DataflowAttributes.getAttributeData("rsSerialNumber", m_serialNumber);
                }
            }

            /** destructor */
            ~RealsenseModule() override = default;

            virtual void setupDevice();

            void startModule() override;

            void startCapturing();

            void handleFrame(rs2::frame f);

            void stopModule() override;

            virtual void teardownDevice();

        protected:

            /** librealsense context for managing devices **/
            rs2::context m_ctx;

            /** the associated realsense device **/
            std::shared_ptr<rs2::device> m_dev;

            std::vector<stream_request> m_stream_requests;
            std::vector<rs2::stream_profile> m_selected_stream_profiles;

            std::vector<rs2::sensor> m_active_sensors;

            /** camera serial **/
            unsigned int m_serialNumber;

            /** create the components **/
            boost::shared_ptr< RealsenseComponent > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
                                                                     const ComponentKey& key, ModuleClass* pModule );

        };

/**
 * Component for Realsense tracker.
 */
    class RealsenseComponent : public RealsenseModule::Component {
    public:
        /** constructor */
        RealsenseComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                const RealsenseComponentKey& componentKey, RealsenseModule* pModule )
                : RealsenseModule::Component(name, componentKey, pModule)
        {

        }

        RealsenseSensorType getSensorType() {
            return m_componentKey.getSensorType();
        }

        rs2_stream getStreamType() {
            return m_componentKey.getStreamType();
        }

        unsigned int getStreamIndex() {
            return m_componentKey.getStreamIndex();
        }

        /** destructor */
        ~RealsenseComponent() override = default;

    };


    class RealsenseVideoComponent : public RealsenseComponent {
    public:
        /** constructor */
        RealsenseVideoComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                const RealsenseComponentKey& componentKey, RealsenseModule* pModule )
                : RealsenseComponent(name, subgraph, componentKey, pModule)
                , m_outputPort("ImageOutput", *this)
                , m_streamFormat( rs2_format::RS2_FORMAT_Y8 )
                , m_imageWidth(0)
                , m_imageHeight(0)
                , m_frameRate(0)
                , m_autoGPUUpload( false )
        {

            // we have different enums/attribute names for color and infrared streams to improve usability of trackman
            if (m_componentKey.getStreamType() == rs2_stream::RS2_STREAM_COLOR) {
                if ( subgraph->m_DataflowAttributes.hasAttribute( "rsColorVideoResolution" ) )
                {
                    std::string sResolution = subgraph->m_DataflowAttributes.getAttributeString( "rsColorVideoResolution" );
                    if ( realsenseStreamResolutionMap.find( sResolution ) == realsenseStreamResolutionMap.end() )
                        UBITRACK_THROW( "unknown stream resolution: \"" + sResolution + "\"" );
                    std::tuple<unsigned int, unsigned int> resolution = realsenseStreamResolutionMap[ sResolution ];
                    m_imageWidth = std::get<0>(resolution);
                    m_imageHeight = std::get<1>(resolution);
                }
                if ( subgraph->m_DataflowAttributes.hasAttribute( "rsColorVideoStreamFormat" ) )
                {
                    std::string sStreamFormat = subgraph->m_DataflowAttributes.getAttributeString( "rsColorVideoStreamFormat" );
                    if ( realsenseStreamFormatMap.find( sStreamFormat ) == realsenseStreamFormatMap.end() )
                        UBITRACK_THROW( "unknown stream type: \"" + sStreamFormat + "\"" );
                    m_streamFormat = realsenseStreamFormatMap[ sStreamFormat ];
                }

            } else if (m_componentKey.getStreamType() == rs2_stream::RS2_STREAM_INFRARED) {
                if ( subgraph->m_DataflowAttributes.hasAttribute( "rsInfraredVideoResolution" ) )
                {
                    std::string sResolution = subgraph->m_DataflowAttributes.getAttributeString( "rsInfraredVideoResolution" );
                    if ( realsenseStreamResolutionMap.find( sResolution ) == realsenseStreamResolutionMap.end() )
                        UBITRACK_THROW( "unknown stream resolution: \"" + sResolution + "\"" );
                    std::tuple<unsigned int, unsigned int> resolution = realsenseStreamResolutionMap[ sResolution ];
                    m_imageWidth = std::get<0>(resolution);
                    m_imageHeight = std::get<1>(resolution);
                }
                if ( subgraph->m_DataflowAttributes.hasAttribute( "rsInfraredVideoStreamFormat" ) )
                {
                    std::string sStreamFormat = subgraph->m_DataflowAttributes.getAttributeString( "rsInfraredVideoStreamFormat" );
                    if ( realsenseStreamFormatMap.find( sStreamFormat ) == realsenseStreamFormatMap.end() )
                        UBITRACK_THROW( "unknown stream type: \"" + sStreamFormat + "\"" );
                    m_streamFormat = realsenseStreamFormatMap[ sStreamFormat ];
                }
            }

            subgraph->m_DataflowAttributes.getAttributeData( "rsFrameRate", m_frameRate );


            Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
            if (oclManager.isEnabled()) {
                if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
                    m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
                }
                if (m_autoGPUUpload){
                    oclManager.activate();
                }
            }

        }

        /** handle the frame **/
        void handleFrame(Measurement::Timestamp ts, rs2::frame f);

        rs2_format getStreamFormat() {
            return m_streamFormat;
        }

        unsigned int getImageWidth() {
            return m_imageWidth;
        }

        unsigned int getImageHeight() {
            return m_imageHeight;
        }

        unsigned int getFrameRate() {
            return m_frameRate;
        }

        /** destructor */
        ~RealsenseVideoComponent() {};

    protected:
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputPort;
        unsigned int m_imageWidth;
        unsigned int m_imageHeight;
        unsigned int m_frameRate;

        rs2_format m_streamFormat;

        bool m_autoGPUUpload;

    };


    class RealsensePointCloudComponent : public RealsenseComponent {
    public:
        /** constructor */
        RealsensePointCloudComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                                 const RealsenseComponentKey& componentKey, RealsenseModule* pModule )
                : RealsenseComponent(name, subgraph, componentKey, pModule)
                , m_outputPort("PointCloudOutput", *this)
                , m_outputDepthmapPort("DepthImageOutput", *this)
                , m_streamFormat( rs2_format::RS2_FORMAT_Z16 )
                , m_imageWidth(0)
                , m_imageHeight(0)
                , m_frameRate(0)
                , m_autoGPUUpload( false )
        {

            if ( subgraph->m_DataflowAttributes.hasAttribute( "rsDepthResolution" ) )
            {
                std::string sResolution = subgraph->m_DataflowAttributes.getAttributeString( "rsDepthResolution" );
                if ( realsenseStreamResolutionMap.find( sResolution ) == realsenseStreamResolutionMap.end() )
                    UBITRACK_THROW( "unknown stream type: \"" + sResolution + "\"" );
                std::tuple<unsigned int, unsigned int> resolution = realsenseStreamResolutionMap[ sResolution ];
                m_imageWidth = std::get<0>(resolution);
                m_imageHeight = std::get<1>(resolution);
            }

            if ( subgraph->m_DataflowAttributes.hasAttribute( "rsDepthStreamFormat" ) )
            {
                std::string sStreamFormat = subgraph->m_DataflowAttributes.getAttributeString( "rsDepthStreamFormat" );
                if ( realsenseStreamFormatMap.find( sStreamFormat ) == realsenseStreamFormatMap.end() )
                    UBITRACK_THROW( "unknown stream type: \"" + sStreamFormat + "\"" );
                m_streamFormat = realsenseStreamFormatMap[ sStreamFormat ];
            }

            subgraph->m_DataflowAttributes.getAttributeData( "rsFrameRate", m_frameRate );

            Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
            if (oclManager.isEnabled()) {
                if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
                    m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
                }
                if (m_autoGPUUpload){
                    oclManager.activate();
                }
            }

        }

        /** handle the frame **/
        void handleFrame(Measurement::Timestamp ts, rs2::frame f);

        rs2_format getStreamFormat() {
            return m_streamFormat;
        }

        unsigned int getImageWidth() {
            return m_imageWidth;
        }

        unsigned int getImageHeight() {
            return m_imageHeight;
        }

        unsigned int getFrameRate() {
            return m_frameRate;
        }

        /** destructor */
        ~RealsensePointCloudComponent() {};

    protected:
        Dataflow::PushSupplier <Measurement::PositionList> m_outputPort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputDepthmapPort;
        unsigned int m_imageWidth;
        unsigned int m_imageHeight;
        unsigned int m_frameRate;
        rs2_format m_streamFormat;

        bool m_autoGPUUpload;
    };

} } // namespace Ubitrack::Drivers

#endif // __RealsenseFramegrabber_h_INCLUDED__
