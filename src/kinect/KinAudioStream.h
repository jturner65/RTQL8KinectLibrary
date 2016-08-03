/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): John Turner <jturner65@gatech.edu>
 * Georgia Tech Graphics Lab
 * 
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * This code incorporates portions of Open Dynamics Engine 
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights 
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow 
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __KINAUDIOSTREAM_H__
#define __KINAUDIOSTREAM_H__

namespace rtql8{
	namespace kinect{
		class KinAudioHandler;
		class CStaticMediaBuffer : public IMediaBuffer{
		public :
			// Constructor
			CStaticMediaBuffer() : dataBuffLen(0) {}

			// IUnknown methods
			STDMETHODIMP_(unsigned long) AddRef() { return 2; }
			STDMETHODIMP_(unsigned long) Release() { return 1; }
			STDMETHODIMP QueryInterface(REFIID riid, void **ppv){
				if (riid == IID_IUnknown){
					AddRef();
					*ppv = (IUnknown*)this;
					return NOERROR;
				} else if (riid == IID_IMediaBuffer){
					AddRef();
					*ppv = (IMediaBuffer*)this;
					return NOERROR;
				} else {				return E_NOINTERFACE;}
			}//QueryInterface

			// IMediaBuffer methods
			STDMETHODIMP SetLength(DWORD length) {dataBuffLen = length; return NOERROR;}
			STDMETHODIMP GetMaxLength(DWORD *pMaxLength) {*pMaxLength = sizeof(dataBuf); return NOERROR;}
			STDMETHODIMP GetBufferAndLength(BYTE **ppBuffer, DWORD *pLength){
				if (ppBuffer){		*ppBuffer = dataBuf;}
				if (pLength){		*pLength = dataBuffLen;}
				return NOERROR;
			}//GetBufferAndLength
			void Init(unsigned long ulData){		dataBuffLen = ulData;}//init

		protected:
			BYTE dataBuf[AudioSamplesPerSecond * AudioBlockAlign];					// buffer used to hold audio data returned by IMediaObject
			unsigned long dataBuffLen;														//size of current data buffer
	};//CStaticMediaBuffer

	// Asynchronous IStream implementation that captures audio data from Kinect audio sensor in a background thread
	// and lets clients read captured audio from any thread.
	class KinAudioStream : public IStream{
	public:

		KinAudioStream(IMediaObject *pKinectDmo,KinAudioHandler* _kinAdSHndlr);
		~KinAudioStream();

		HRESULT StartCapture();
		HRESULT StopCapture();

		//////////////////////////////
		// IUnknown methods
		STDMETHODIMP_(unsigned long) AddRef() { return InterlockedIncrement(&numRef); }
		STDMETHODIMP_(unsigned long) Release()	{
			unsigned int ref = InterlockedDecrement(&numRef);
			if (ref == 0){delete this;}
			return ref;
		}//Release
		STDMETHODIMP QueryInterface(REFIID riid, void **ppv){
			if (riid == IID_IUnknown){
				AddRef();
				*ppv = (IUnknown*)this;
				return S_OK;
			} else if (riid == IID_IStream)	{
				AddRef();
				*ppv = (IStream*)this;
				return S_OK;
			} else {	return E_NOINTERFACE;	}
		}//QueryInterface

		//////////////////////////////
		// IStream methods
		STDMETHODIMP Read(void *,unsigned long,unsigned long *);
		STDMETHODIMP Seek(LARGE_INTEGER,DWORD,ULARGE_INTEGER *);

			//interface methods not implemented for this stream - not needed for speech recogn
		STDMETHODIMP Stat(STATSTG *,DWORD){ return E_NOTIMPL;}
		STDMETHODIMP Clone(IStream **){ return E_NOTIMPL;}
		STDMETHODIMP Write(const void *,unsigned long,unsigned long *){ return E_NOTIMPL;}
		STDMETHODIMP SetSize(ULARGE_INTEGER){ return E_NOTIMPL;}
		STDMETHODIMP CopyTo(IStream *,ULARGE_INTEGER,ULARGE_INTEGER *,ULARGE_INTEGER *){ return E_NOTIMPL;}
		STDMETHODIMP Commit(DWORD){ return E_NOTIMPL;}
		STDMETHODIMP Revert(){ return E_NOTIMPL;}	
		STDMETHODIMP LockRegion(ULARGE_INTEGER,ULARGE_INTEGER,DWORD){ return E_NOTIMPL;}
		STDMETHODIMP UnlockRegion(ULARGE_INTEGER,ULARGE_INTEGER,DWORD){ return E_NOTIMPL;}

	private ://methods

		CStaticMediaBuffer* GetWriteBuffer();
		void ReleaseBuffer(CStaticMediaBuffer* pBuffer);										// Release an audio buffer back into buffer pool.
		void ReleaseAllBuffers();																// Release all audio buffers back into buffer pool.
		void QueueCapturedData(BYTE *pData, unsigned int cbData);								// Add captured audio data to the circular queue of buffers ready for client reading.
		void QueueCapturedBuffer(CStaticMediaBuffer *pBuffer);									// Add buffer full of captured audio data to the circular queue of buffers ready for client reading.
		void ReadOneBuffer(BYTE **ppbData, unsigned long* pcbData);								// Perform one read operation from current read buffer
		static DWORD WINAPI CaptureThread(LPVOID pParam);										// Starting address for audio capture thread.
		DWORD WINAPI CaptureThread();															// Audio capture thread. Captures audio data in a loop until it is signaled to stop - Non-zero if thread ended successfully, zero in case of failure.
		BOOL IsCapturing() {	return (NULL != eventStop) && (WAIT_OBJECT_0 != WaitForSingleObject(eventStop,0) );}		// Indicates whether this stream is currently capturing audio data.

	private : //variables
		unsigned int numRef;																	// Number of references to this object - this should be replaced with instantiating a shared pointer
		IMediaObject* kinCapDMO;																// Media object used to capture audio
		KinAudioHandler* kinAdSHndlr;														// owning audio stream handler
		HANDLE eventStop;																		// Event used to signal that capture thread should stop capturing audio
		HANDLE eventDataReady;																	// Event used to signal that there's captured audio data ready to be read
		HANDLE hndlAudCapThread;																// Audio capture thread
		stack<CStaticMediaBuffer*> bufPool;														// Pool of unused buffers ready to be used for writing captured audio data
		queue<CStaticMediaBuffer*> readBufQueue;												// Circular buffer queue that contains audio data ready for reading by stream clients
		CStaticMediaBuffer* currWriteBuf;														// Buffer where most recently captured audio data is being written
		CStaticMediaBuffer* currReadBuf;														// Buffer from which stream client is currently reading audio data
		unsigned long currReadBufIDX;															// Next index to be read within current read buffer
		unsigned long numBytesRead;																// Total number of bytes read so far by audio stream client
		CRITICAL_SECTION mtxAudData;															// mutex used to synchronize multithreaded access to captured audio data

		};//class KinAudioStream
	}//namespace kinect
}//namespace rtql8
#endif