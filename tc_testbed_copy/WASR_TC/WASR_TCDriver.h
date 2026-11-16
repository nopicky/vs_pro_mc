///////////////////////////////////////////////////////////////////////////////
// WASR_TCDriver.h

#ifndef __WASR_TCDRIVER_H__
#define __WASR_TCDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define WASR_TCDRV_NAME        "WASR_TC"
#define WASR_TCDRV_Major       1
#define WASR_TCDRV_Minor       0

#define DEVICE_CLASS CWASR_TCDriver

#include "ObjDriver.h"

class CWASR_TCDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl WASR_TCDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(WASR_TCDRV)
	VxD_Service( WASR_TCDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __WASR_TCDRIVER_H__