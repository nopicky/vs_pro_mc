///////////////////////////////////////////////////////////////////////////////
// WASR_TCDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "WASR_TCDriver.h"
#include "WASR_TCClassFactory.h"

DECLARE_GENERIC_DEVICE(WASR_TCDRV)

IOSTATUS CWASR_TCDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CWASR_TCClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CWASR_TCDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CWASR_TCDriver::WASR_TCDRV_GetVersion( )
{
	return( (WASR_TCDRV_Major << 8) | WASR_TCDRV_Minor );
}

