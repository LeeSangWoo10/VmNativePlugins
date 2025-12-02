#include "DXUTComponents.h"

//ID3D10Device* g_pd3dDevice = NULL;

using namespace std;
using namespace vmmath;

bool dxInitialize()
{
	//if (g_pd3dDevice != NULL)
	//	return false;

	//if(D3D10CreateDevice(NULL, D3D10_DRIVER_TYPE_HARDWARE, (HMODULE)0, 0, D3D10_SDK_VERSION, &g_pd3dDevice) == E_FAIL)
	//	return false;

	return true;
}

bool dxDeinitialize()
{
	//SAFE_RELEASE(g_pd3dDevice);
	return true;
}
