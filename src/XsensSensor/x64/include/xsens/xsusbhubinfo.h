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

#ifndef XSUSBHUBINFO_H
#define XSUSBHUBINFO_H

#include "xdaconfig.h"

#ifdef _WIN32
	typedef int XsHubIdentifier;
#else
	typedef const char* XsHubIdentifier;
#endif

struct XsUsbHubInfo;
#ifndef __cplusplus
#define XSUSBHUBINFO_INITIALIZER { 0 }
typedef struct XsUsbHubInfo XsUsbHubInfo;
#else
extern "C" {
#endif

XDA_DLL_API void XsUsbHubInfo_assign(XsUsbHubInfo* thisPtr, XsHubIdentifier hub);
XDA_DLL_API void XsUsbHubInfo_construct(XsUsbHubInfo* thisPtr, XsHubIdentifier hub);
XDA_DLL_API void XsUsbHubInfo_destruct(XsUsbHubInfo* thisPtr);
XDA_DLL_API void XsUsbHubInfo_copy(XsUsbHubInfo* copy, XsUsbHubInfo const* src);
XDA_DLL_API void XsUsbHubInfo_swap(XsUsbHubInfo* thisPtr, XsUsbHubInfo* thatPtr);
XDA_DLL_API int XsUsbHubInfo_parentPathMatches(const XsUsbHubInfo* thisPtr, const XsUsbHubInfo* other);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \struct XsUsbHubInfo
	\brief A structure that wraps USB hub information
*/
struct XsUsbHubInfo {
#ifdef __cplusplus
	/*! \brief Default constructor

	  \param hubid an optional hub identifier to initialize with
	  \sa XsUsbHubInfo_construct
	 */
	explicit XsUsbHubInfo(XsHubIdentifier hubid = 0)
		: m_hub(0)
	{
		if (hubid)
			XsUsbHubInfo_construct(this, hubid);
	}

	/*! \brief Destructor \sa XsUsbHubInfo_destruct */
	~XsUsbHubInfo()
	{
		XsUsbHubInfo_destruct(this);	//lint !e1551
	}

	/*! \brief Copy constructor
	  \param other the object to copy
	  \sa XsUsbHubInfo_copy \sa XsUsbHubInfo_construct
	 */
	XsUsbHubInfo(const XsUsbHubInfo &other)
		: m_hub(0)
	{
		if (other.m_hub)
			XsUsbHubInfo_construct(this, other.m_hub);
	}

	/*! \brief Assigns \a other to this XsUsbHubInfo
	   \param other the object to copy
	   \returns a const reference to this info object
	   \sa XsUsbHubInfo_copy
	 */
	const XsUsbHubInfo& operator=(const XsUsbHubInfo &other)
	{
		if (this != &other)
			XsUsbHubInfo_copy(this, &other);
		return *this;
	}

	/*! \brief \copybrief XsUsbHubInfo_parentPathMatches
	 * \param other the object to compare to
	 * \returns true if the two objects share the same immediate parent hub, false otherwise
	 * \sa XsUsbHubInfo_parentPathMatches
	 */
	bool parentPathMatches(const XsUsbHubInfo &other) const
	{
		return 0 != XsUsbHubInfo_parentPathMatches(this, &other);
	}

	/*! \brief Returns true if a valid hub is set
	*/
	bool isValid() const
	{
		return m_hub != 0;
	}

	/*! \brief Return the hub identifier
	*/
	inline XsHubIdentifier hub() const
	{
		return m_hub;
	}

private:
//! \protectedsection
#endif
	XsHubIdentifier m_hub;		//!< The identifier of the USB hub
};
typedef struct XsUsbHubInfo XsUsbHubInfo;

#endif	// file guard
