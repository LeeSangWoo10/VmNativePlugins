/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//------------------------------------------------------------------------

#ifndef FW_DLL_IMPORT_RETV
#   define FW_DLL_IMPORT_RETV(RET, CALL, NAME, PARAMS, PASS)
#   define FW_DLL_IMPORT_VOID(RET, CALL, NAME, PARAMS, PASS)
#   define FW_DLL_DECLARE_RETV(RET, CALL, NAME, PARAMS, PASS)
#   define FW_DLL_DECLARE_VOID(RET, CALL, NAME, PARAMS, PASS)
#   define FW_DLL_IMPORT_CUDA(RET, CALL, NAME, PARAMS, PASS)
#   define FW_DLL_IMPORT_CUV2(RET, CALL, NAME, PARAMS, PASS)
#endif


//------------------------------------------------------------------------
// WinBase
//------------------------------------------------------------------------

FW_DLL_IMPORT_VOID(void,        WINAPI,     InitializeConditionVariable,            (PCONDITION_VARIABLE ConditionVariable), (ConditionVariable))
FW_DLL_IMPORT_RETV(BOOL,        WINAPI,     SleepConditionVariableCS,               (PCONDITION_VARIABLE ConditionVariable, PCRITICAL_SECTION CriticalSection, DWORD dwMilliseconds), (ConditionVariable, CriticalSection, dwMilliseconds))
//FW_DLL_IMPORT_VOID(void,        WINAPI,     WakeAllConditionVariable,               (PCONDITION_VARIABLE ConditionVariable), (ConditionVariable))
FW_DLL_IMPORT_VOID(void,        WINAPI,     WakeConditionVariable,                  (PCONDITION_VARIABLE ConditionVariable), (ConditionVariable))

//------------------------------------------------------------------------
// WinMM
//------------------------------------------------------------------------

FW_DLL_IMPORT_RETV( MMRESULT,   WINAPI,     waveOutOpen,                            (OUT LPHWAVEOUT phwo, IN UINT uDeviceID, IN LPCWAVEFORMATEX pwfx, IN DWORD_PTR dwCallback, IN DWORD_PTR dwInstance, IN DWORD fdwOpen), (phwo, uDeviceID, pwfx, dwCallback, dwInstance, fdwOpen))
FW_DLL_IMPORT_RETV( MMRESULT,   WINAPI,     waveOutClose,                           (IN OUT HWAVEOUT hwo), (hwo))
FW_DLL_IMPORT_RETV( MMRESULT,   WINAPI,     waveOutPrepareHeader,                   (IN HWAVEOUT hwo, IN OUT LPWAVEHDR pwh, IN UINT cbwh), (hwo, pwh, cbwh))
FW_DLL_IMPORT_RETV( MMRESULT,   WINAPI,     waveOutUnprepareHeader,                 (IN HWAVEOUT hwo, IN OUT LPWAVEHDR pwh, IN UINT cbwh), (hwo, pwh, cbwh))
FW_DLL_IMPORT_RETV( MMRESULT,   WINAPI,     waveOutWrite,                           (IN HWAVEOUT hwo, IN OUT LPWAVEHDR pwh, IN UINT cbwh), (hwo, pwh, cbwh))
FW_DLL_IMPORT_RETV( MMRESULT,   WINAPI,     waveOutReset,                           (IN HWAVEOUT hwo), (hwo))

//------------------------------------------------------------------------
// ShLwAPI
//------------------------------------------------------------------------

FW_DLL_IMPORT_RETV( BOOL,   STDAPICALLTYPE, PathRelativePathToA,                    (LPSTR pszPath, LPCSTR pszFrom, DWORD dwAttrFrom, LPCSTR pszTo, DWORD dwAttrTo), (pszPath, pszFrom, dwAttrFrom, pszTo, dwAttrTo))

//------------------------------------------------------------------------
