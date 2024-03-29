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

#ifndef XSDEVICEPTRARRAY_H
#define XSDEVICEPTRARRAY_H

#include <xsens/xsarray.h>
#include "xsdeviceptr.h"
#include "xdaconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

extern XsArrayDescriptor const XDA_DLL_API g_xsDevicePtrArrayDescriptor;

#ifndef __cplusplus
#define XSDEVICEPTRARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsDevicePtrArrayDescriptor)

XSARRAY_STRUCT(XsDevicePtrArray, XsDevicePtr);
typedef struct XsDevicePtrArray XsDevicePtrArray;

XDA_DLL_API void XsDevicePtrArray_construct(XsDevicePtrArray* thisPtr, XsSize count, XsDevicePtr const* src);
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsDevicePtrArray : public XsArrayImpl<XsDevicePtr, g_xsDevicePtrArrayDescriptor, XsDevicePtrArray> {
	//! \brief Constructs an XsDevicePtrArray
	inline explicit XsDevicePtrArray(XsSize sz = 0, XsDevicePtr const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsDevicePtrArray as a copy of \a other
	inline XsDevicePtrArray(XsDevicePtrArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsDevicePtrArray that references the data supplied in \a ref
	inline explicit XsDevicePtrArray(XsDevicePtr* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsDevicePtrArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsDevicePtrArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
}; //lint !e1509 base class destructor for class 'XsArrayImpl' is not virtual. Cannot be virtual because of needed C compatibility
#endif
#endif
