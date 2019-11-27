/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 3* contributors as indicated by the @authors tag. See the
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
 * @ingroup Inertial_components
 * @file
 * Reads IMU data
 *
 * @author Adnane Jadid <jadid@in.tum.de>
 */


//--------------------------------------------------------------------------------
// Xsens device API example for an MTi / MTx / Mtmk4 device using the C++ API
//
//--------------------------------------------------------------------------------
#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>


#include <xsensdeviceapi.h> // The Xsens device API header
#include <stdexcept>
#include <xsens/xstime.h>
#include <xsens/xsmutex.h>

#include <string>
#include <list>
#include <iostream>
#include <iomanip>
#include <strstream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>



// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.XsensSensor" ) );

using namespace Ubitrack;


namespace Ubitrack { namespace Drivers {




/**
 * @ingroup components
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Measurement::IMUMeasurement.
 *
 * @par Configuration
 * The configuration tag contains a \c <dsvl_input> configuration.
 * For details, see the InertialSense Docs
 *
 */
 class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5) :
		m_maxNumberOfPacketsInBuffer(maxBufferSize),
		m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		XsMutexLocker lock(m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
		{
			(void)getNextPacket();
		}
		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable XsMutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};

class XsensSensor
	: public Dataflow::Component
{
public:

	/** constructor */
	XsensSensor( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~XsensSensor();

	/** starts the IMU */
	void start();

	/** starts the capturing */
	void startCapturing();

	/** stops the IMU */
	void stop();

protected:


	
	void handleDataMessage(XsDataPacket packet);

	// shift timestamps (ms)
	int m_timeOffset;
	
	//Device 
	XsDevice* device;
	XsControl* control;
	CallbackHandler callback;
	//serialPort communicatiom configuration
	XsPortInfoArray::const_iterator mtPort;
	//params
	int freq;
	std::string portName;
	int baudRate;
	
	//message
	XsByteArray data;
	XsMessageArray msgs;

	/** thread is running?*/
	bool m_bStop;

	// pointer to the thread
	boost::shared_ptr< boost::thread > m_pThread;

	int count;

	uint8_t inByte;

	int messageSize;
	uint8_t buffer[2048];

	/** timestamp of last frame */
	double m_lastTime;


	// the ports
	Dataflow::PushSupplier< Measurement::Vector3D > m_acc_OutPort;
	Dataflow::PushSupplier< Measurement::RotationVelocity > m_gyro_OutPort;
	Dataflow::PushSupplier< Measurement::Vector3D > m_mag_OutPort;

	Dataflow::PushSupplier< Measurement::Rotation > m_orientation_outPort;
  
};



XsensSensor::XsensSensor(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
	: Dataflow::Component(sName)
	, m_timeOffset(0)
	, m_lastTime(-1e10)
	, m_bStop(true)
	, m_acc_OutPort("acc_OutPort", *this)
	, m_gyro_OutPort("gyro_OutPort", *this)
	, m_mag_OutPort("mag_OutPort", *this)
	, m_orientation_outPort("gyro_outPortRAW", *this)
{
	subgraph->m_DataflowAttributes.getAttributeData("timeOffset", m_timeOffset);
	
	
		 	subgraph->m_DataflowAttributes.getAttributeData( "frequency", freq );
			subgraph->m_DataflowAttributes.getAttributeData( "baudrate", baudRate );
			portName = subgraph->m_DataflowAttributes.getAttributeString( "port" );

		XsPortInfo mtPort(portName, XsBaud::numericToRate(baudRate));
				
}



XsensSensor::~XsensSensor()
{
	
}


void XsensSensor::start()
{
	
	if ( !m_running ) {
		
		control = XsControl::construct();
		//assert(control != 0);
			
			// Scan for connected devices
		XsPortInfoArray portInfoArray = XsScanner::scanPorts();

		// Find an MTi / MTx / MTmk4 device
		 mtPort = portInfoArray.begin();
		while (mtPort != portInfoArray.end() && !mtPort->deviceId().isMt9c() && !mtPort->deviceId().isLegacyMtig() && !mtPort->deviceId().isMtMk4() && !mtPort->deviceId().isFmt_X000()) {++mtPort;}
		if (mtPort == portInfoArray.end())
		{
			LOG4CPP_ERROR(logger,"No MTi / MTx / MTmk4 device found. Aborting.");
		}
	
	
		if (!control->openPort(mtPort->portName().toStdString(), mtPort->baudrate()))
		{
			LOG4CPP_ERROR(logger,"Could not open port. Aborting.");
		}
	
			// Get the device object
			device = control->device(mtPort->deviceId());
			//assert(device != 0);

			

			// Create and attach callback handler to device
			
			device->addCallbackHandler(&callback);

		
			if (!device->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("Could not put device into configuration mode. Aborting.");
			}

			// Configure the device. Note the differences between MTix and MTmk4
			
			if (device->deviceId().isMtMk4() || mtPort->deviceId().isFmt_X000())
			{
				XsOutputConfiguration avel(XDI_RateOfTurn, freq);
				//XsOutputConfiguration quat(XDI_Quaternion, freq);
				XsOutputConfiguration acc(XDI_Acceleration, freq);
				XsOutputConfiguration mag(XDI_MagneticField, freq);

				XsOutputConfigurationArray configArray;

				configArray.push_back(avel);
				configArray.push_back(mag);
				//configArray.push_back(quat);
				configArray.push_back(acc);
				if (!device->setOutputConfiguration(configArray))
				{

					LOG4CPP_ERROR(logger,"Could not configure MTmk4 device. Aborting.");
				}
			}
			else
			{
				LOG4CPP_ERROR(logger,"Unknown device while configuring. Aborting.");
			}

		
			if (!device->gotoMeasurement())
			{
				LOG4CPP_ERROR(logger,"Could not put device into measurement mode. Aborting.");
			}
		m_running = true;
		m_bStop = false;
		m_pThread.reset(new boost::thread(boost::bind(&XsensSensor::startCapturing, this)));
	}
	Component::start();
}

void XsensSensor::startCapturing()
{
	while (m_running)
	{
		if (callback.packetAvailable())
				{
					// Retrieve a packet
					XsDataPacket packet = callback.getNextPacket();
				
					
					
					handleDataMessage(packet);
				}
	}
}

void XsensSensor::stop()
{
	if (m_running)
	{
		m_running = false;
		m_bStop = true;
		control->closePort(mtPort->portName().toStdString());
		control->destruct();
		if (m_pThread)
		{
			m_pThread->join();
		}
	}
	Component::stop();
}

void XsensSensor::handleDataMessage(XsDataPacket packet){
	Measurement::Timestamp ts = packet.timeOfArrival().msTime();//Measurement::now();// ins->timeOfWeek;
	// Get the  data
	XsVector acc = packet.calibratedAcceleration();
	XsVector mag = packet.calibratedMagneticField();
	XsVector gyro = packet.calibratedGyroscopeData();
	//XsQuaternion quaternion = packet.orientationQuaternion();

	Measurement::Vector3D macc(ts, Math::Vector3d(acc.at(0), acc.at(1), acc.at(2)));
	m_acc_OutPort.send(macc);
	Measurement::RotationVelocity mgyro(ts, Math::RotationVelocity(gyro.at(0), gyro.at(1), gyro.at(2)));
	m_gyro_OutPort.send(mgyro);
	Measurement::Vector3D mmag(ts, Math::Vector3d(mag.at(0), mag.at(1), mag.at(2)));
	m_mag_OutPort.send(mmag);
	
	
					/*
					std::cout << "\r"
							  << "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.w()
							  << ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.x()
							  << ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.y()
							  << ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.z()
					;
					*/

}



} } // namespace Ubitrack::Components

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::XsensSensor > ( "XSenseSensor" );
}