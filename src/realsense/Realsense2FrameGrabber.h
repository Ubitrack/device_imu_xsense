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
 * Realsense2 driver
 * This file contains the driver component to
 * synchronously capture camera images using realsense.
 *
 * The received data is sent via a push interface.
 *
 * @author Ulrich Eck <ulrick.eck@tum.de>
 */

#ifndef __Realsense2Framegrabber_h_INCLUDED__
#define __Realsense2Framegrabber_h_INCLUDED__


#include <string>
#include <cstdlib>

#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>

#include <utVision/Image.h>
#include <opencv/cv.h>

#include <librealsense2/rs.hpp>


namespace {

    class RealsenseStreamTypeMap : public std::map< std::string, rs2_stream > {
    public:
        RealsenseStreamTypeMap() {
            (*this)["DEPTH"] = rs2_stream::RS2_STREAM_DEPTH;
            (*this)["COLOR"] = rs2_stream::RS2_STREAM_COLOR;
            (*this)["INFRARED"] = rs2_stream::RS2_STREAM_INFRARED;
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

	// class RealsenseModeMap 
	// 	: public std::map< std::string, Realsense2::VideoMode >
	// {
	// public:
	// 	RealsenseModeMap()
	// 	{
	// 		using namespace Realsense2;
			
	// 		(*this)[ "640x480RGB" ] = VIDEOMODE_640x480RGB;
	// 		(*this)[ "640x480Y8" ] = VIDEOMODE_640x480Y8;
	// 		(*this)[ "800x600RGB" ] = VIDEOMODE_800x600RGB;
	// 		(*this)[ "800x600Y8" ] = VIDEOMODE_800x600Y8;
	// 		(*this)[ "1024x768RGB" ] = VIDEOMODE_1024x768RGB;
	// 		(*this)[ "1024x768Y8" ] = VIDEOMODE_1024x768Y8;
	// 		(*this)[ "1280x960RGB" ] = VIDEOMODE_1280x960RGB;
	// 		(*this)[ "1280x960Y8" ] = VIDEOMODE_1280x960Y8;
	// 		(*this)[ "1600x1200RGB" ] = VIDEOMODE_1600x1200RGB;
	// 		(*this)[ "1600x1200Y8" ] = VIDEOMODE_1600x1200Y8;
	// 	}
	// };
	// static RealsenseModeMap flyCaptureModeMap;

	// class RealsensePixelFormatMap 
	// 	: public std::map< std::string, Realsense2::PixelFormat >
	// {
	// public:
	// 	RealsensePixelFormatMap()
	// 	{
	// 		using namespace Realsense2;
			
	// 		(*this)[ "MONO8" ] = PIXEL_FORMAT_MONO8;
	// 		(*this)[ "411YUV8" ] = PIXEL_FORMAT_411YUV8;
	// 		(*this)[ "422YUV8" ] = PIXEL_FORMAT_422YUV8;
	// 		(*this)[ "444YUV8" ] = PIXEL_FORMAT_444YUV8;
	// 		(*this)[ "RGB8" ] = PIXEL_FORMAT_RGB8;
	// 		(*this)[ "MONO16" ] = PIXEL_FORMAT_MONO16;
	// 		(*this)[ "RGB16" ] = PIXEL_FORMAT_RGB16;
	// 		(*this)[ "S_MONO16" ] = PIXEL_FORMAT_S_MONO16;
	// 		(*this)[ "S_RGB16" ] = PIXEL_FORMAT_S_RGB16;
	// 		(*this)[ "RAW8" ] = PIXEL_FORMAT_RAW8;
	// 		(*this)[ "RAW16" ] = PIXEL_FORMAT_RAW16;
	// 		(*this)[ "MONO12" ] = PIXEL_FORMAT_MONO12;
	// 		(*this)[ "RAW12" ] = PIXEL_FORMAT_RAW12;
	// 		(*this)[ "BGR" ] = PIXEL_FORMAT_BGR;
	// 		(*this)[ "BGRU" ] = PIXEL_FORMAT_BGRU;
	// 		(*this)[ "RGB" ] = PIXEL_FORMAT_RGB;
	// 		(*this)[ "RGBU" ] = PIXEL_FORMAT_RGBU;
	// 	}
	// };
	// static RealsensePixelFormatMap flyCapturePixelFormatMap;

	// class RealsenseFrameRateMap 
	// 	: public std::map< std::string, Realsense2::FrameRate >
	// {
	// public:
	// 	RealsenseFrameRateMap()
	// 	{
	// 		using namespace Realsense2;
			
	// 		(*this)[ "1.875" ] = FRAMERATE_1_875;
	// 		(*this)[ "3.75" ] = FRAMERATE_3_75;
	// 		(*this)[ "7.5" ] = FRAMERATE_7_5;
	// 		(*this)[ "15" ] = FRAMERATE_15;
	// 		(*this)[ "30" ] = FRAMERATE_30;
	// 		(*this)[ "60" ] = FRAMERATE_60;
	// 		(*this)[ "120" ] = FRAMERATE_120;
	// 		(*this)[ "240" ] = FRAMERATE_240;
	// 	}
	// };
	// static RealsenseFrameRateMap flyCaptureFrameRateMap;

} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

// forward declaration
class Realsense2Component;

/**
 * Component key for flycapture.
 * Represents the camera
 */
class Realsense2ComponentKey
{
public:

	explicit Realsense2ComponentKey( const boost::shared_ptr< Graph::UTQLSubgraph > &subgraph )
	: m_cameraSerialNumber( "" )
	{
	  subgraph->m_DataflowAttributes.getAttributeData( "cameraSerialNumber", m_cameraSerialNumber );
	  if (( m_cameraSerialNumber.length() == 0 ))
            UBITRACK_THROW( "Invalid camera serial number" );

	}

	// construct from body number
	explicit Realsense2ComponentKey( std::string a )
		: m_cameraSerialNumber( std::move(a) )
 	{}
	
	std::string  getCameraSerialNumber() const {
		return m_cameraSerialNumber;
	}

	// less than operator for map
	bool operator<( const Realsense2ComponentKey& b ) const
    {
		return m_cameraSerialNumber.compare(b.m_cameraSerialNumber) < 0;
    }

protected:
	std::string m_cameraSerialNumber;
};



/**
 * Module for Realsense2 tracker.
 * Does all the work
 */
class Realsense2Module
	: public Module< SingleModuleKey, Realsense2ComponentKey, Realsense2Module, Realsense2Component >
{
public:
	/** UTQL constructor */
	Realsense2Module( const SingleModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* pFactory );

	/** destructor */
	override ~Realsense2Module();

protected:

	/** librealsense context for managing devices **/
	rs2::context ctx;

	/** the associated realsense device **/
	rs2::device dev;


	/** create the components **/
	boost::shared_ptr< ComponentClass > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
														 const ComponentKey& key, ModuleClass* pModule );

};

std::ostream& operator<<( std::ostream& s, const Realsense2ComponentKey& k );

/**
 * Component for Realsense2 tracker.
 */
class Realsense2Component : public Realsense2Module::Component {
public:
	/** constructor */
	Realsense2Component( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const Realsense2ComponentKey& componentKey, Realsense2Module* pModule );

	void setupCamera( int index, rs2::sensor& sensor );

	inline std::string getCameraIndex() {
		return m_cameraSerialNumber;
	}

	/** start async capturing **/
	void startCapturing();

	/** received captured image **/
	void onImageGrabbed(rs2::frame f);

	/** stop async capturing **/
	void stopCapturing();


	/** destructor */
	~Realsense2Component() {};

protected:

	rs2::config cfg;
	rs2::sensor sensor;

    // DEPTH/COLOR/INFRARED
    rs2_stream m_stream_type;

    // the actual pixel format RGB8/Z16/...
    rs2_format m_stream_format;

    // image width
	unsigned int m_image_width;

	//image height
	unsigned int m_image_height;

	// frame rate
	unsigned int m_framerate;

	// index in the camera struct
	int m_index;

	// the serial number
	std::string m_cameraSerialNumber;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

};

} } // namespace Ubitrack::Drivers

#endif // __Realsense2Framegrabber_h_INCLUDED__
