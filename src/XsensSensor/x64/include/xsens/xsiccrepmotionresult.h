/*	WARNING: COPYRIGHT (C) 2017 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifndef XSICCREPMOTIONRESULT_H
#define XSICCREPMOTIONRESULT_H

#include <xsens/pstdint.h>

/*! \brief Contains the result of the representative motion processed by ICC
*/
struct XsIccRepMotionResult
{
	float m_ddtAccuracy;	//!< The ddtAccuracy of the In-Run Compass Calibration
	uint8_t m_dimension;	//!< The dimension of the In-Run Compass Calibration
	uint8_t m_status;		//!< The status of the In-Run Compass Calibration

#ifdef __cplusplus
	XsIccRepMotionResult() : m_ddtAccuracy(0.0), m_dimension(0), m_status(0)
	{

	}

	/*! \brief Copy constructor for a filter profile object
		\param other the filter profile object to construct a copy of
	*/
	XsIccRepMotionResult(const XsIccRepMotionResult& other)
		: m_ddtAccuracy(other.m_ddtAccuracy)
		, m_dimension(other.m_dimension)
		, m_status(other.m_status)
	{
	}

	//! \returns the ddtAccuracy
	inline float ddtAccuracy() const
	{
		return m_ddtAccuracy;
	}

	//! \returns the dimension
	inline uint8_t dimension() const
	{
		return m_dimension;
	}

	//! \returns the status
	inline uint8_t status() const
	{
		return m_status;
	}
#endif
};

typedef struct XsIccRepMotionResult XsIccRepMotionResult;

#endif
