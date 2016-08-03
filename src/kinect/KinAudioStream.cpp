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

#include "KinHndlrConsts.h"
#include "KinAudioStream.h"
#include "KinAudioHandler.h"

namespace rtql8{
	namespace kinect{
		KinAudioStream::KinAudioStream(IMediaObject *pKinectDmo, KinAudioHandler* _kinAdSHndlr) : numRef(1), kinAdSHndlr(_kinAdSHndlr), currWriteBuf(NULL), currReadBuf(NULL), currReadBufIDX(0), 
				numBytesRead(0), eventStop(NULL), eventDataReady(NULL),  hndlAudCapThread(NULL){
			pKinectDmo->AddRef();
			kinCapDMO = pKinectDmo;
			InitializeCriticalSection(&mtxAudData);								//setup mutex
		}//cstrctr

		KinAudioStream::~KinAudioStream(){
			if(NULL != kinCapDMO){		kinCapDMO->Release();		kinCapDMO = NULL;	}
			DeleteCriticalSection(&mtxAudData);									//dispose mutex
		}//dstrctr

		// Starts capturing audio data from Kinect sensor.
		HRESULT KinAudioStream::StartCapture(){

			eventStop = CreateEvent( NULL, TRUE, FALSE, NULL );
			eventDataReady = CreateEvent( NULL, FALSE, FALSE, NULL );
			numBytesRead = 0;

			//build buffer pool
			for (int i = 0; i < NumBuffers; ++i){
				CStaticMediaBuffer *pBuf = new CStaticMediaBuffer();
				bufPool.push(pBuf);
			}

			currWriteBuf = NULL;
			hndlAudCapThread = CreateThread(NULL, 0, CaptureThread, this, 0, NULL);
			return S_OK;
		}//StartCapture

		// stops capturing audio data from Kinect sensor.
		HRESULT KinAudioStream::StopCapture(){
			if ( NULL != eventStop ){
				// Signal the thread
				SetEvent(eventStop);

				// Wait for thread to stop
				if ( NULL != hndlAudCapThread ){
					WaitForSingleObject( hndlAudCapThread, INFINITE );
					CloseHandle( hndlAudCapThread );
					hndlAudCapThread = NULL;
				}
				CloseHandle( eventStop );
				eventStop = NULL;
			}

			if (NULL != eventDataReady){
				SetEvent(eventDataReady);
				CloseHandle(eventDataReady);
				eventDataReady = NULL;
			}

			// Release all buffers to buffer pool and then free all buffers in pool
			ReleaseAllBuffers();
			while (!bufPool.empty()){
				CStaticMediaBuffer* mediaBuffer = bufPool.top();
				delete mediaBuffer;
				bufPool.pop();
			}

			return S_OK;
		}//StopCapture

		//////////////////////////////
		// IStream methods
		STDMETHODIMP KinAudioStream::Read(void *pBuffer, unsigned long cbBuffer, unsigned long *pcbRead){
			HRESULT hr = S_OK;
		   if (pcbRead == NULL) {			 return E_INVALIDARG;  }
			unsigned long bytesPendingToRead = cbBuffer;
			while (bytesPendingToRead > 0 && IsCapturing()){
				ReadOneBuffer((BYTE**)&pBuffer, &bytesPendingToRead);
				if (NULL == currReadBuf) {//no data, wait ...
					WaitForSingleObject(eventDataReady, INFINITE);
				}
			}//while 
			unsigned long bytesRead = cbBuffer - bytesPendingToRead;
			numBytesRead += bytesRead;
			*pcbRead = bytesRead;
			return hr;
		}//Read

		STDMETHODIMP KinAudioStream::Seek(LARGE_INTEGER dlibMove,DWORD dwOrigin, ULARGE_INTEGER *plibNewPosition ){
			if (plibNewPosition != NULL){	plibNewPosition->QuadPart = numBytesRead + dlibMove.QuadPart;}
			return S_OK;
		}//Seek
	
		//////////////////////////////
		// Private KinAudioStream methods
		//////////////////////////////

		// get next avail buffer for writing audio data. first attempt to find a free buffer in buffer pool ; if none available, reuse the oldest buffer from circular queue of buffers ready for 
		// reading.
		// <returns>buffer or null</returns>
		CStaticMediaBuffer *KinAudioStream::GetWriteBuffer(){
			CStaticMediaBuffer *pBuf = NULL;
			EnterCriticalSection(&mtxAudData);
				//Get a free buffer if available. Otherwise, get the oldest buffer
				if (bufPool.size() > 0){   
					pBuf = bufPool.top();
					bufPool.pop();
					pBuf->SetLength(0);
				} else if (readBufQueue.size() > 0) {
					pBuf = readBufQueue.front();
					readBufQueue.pop();
					pBuf->SetLength(0);
				}
			LeaveCriticalSection(&mtxAudData);
			return pBuf;
		}//GetWriteBuffer

		// Release an audio buffer back into buffer pool.
		// <param name="pBuffer">Buffer to be released.</param>
		void KinAudioStream::ReleaseBuffer(CStaticMediaBuffer* pBuffer){
			if (NULL != pBuffer){																//yoda
				EnterCriticalSection(&mtxAudData);
					pBuffer->SetLength(0);
					bufPool.push(pBuffer);
				LeaveCriticalSection(&mtxAudData);
			}//if not null
		}//release

		// Release all audio buffers back into buffer pool.
		void KinAudioStream::ReleaseAllBuffers(){
			EnterCriticalSection(&mtxAudData);
				while (readBufQueue.size() > 0){
					CStaticMediaBuffer *pBuf = readBufQueue.front();
					readBufQueue.pop();
					ReleaseBuffer(pBuf);
				}
				if (NULL != currReadBuf){	ReleaseBuffer(currReadBuf);	}			//yoda
				currReadBufIDX = 0;
				currReadBuf = NULL;
			LeaveCriticalSection(&mtxAudData);
		}//ReleaseAllBuffers

		// Add captured audio data to the circular queue of buffers ready for client reading.
		// <param name="pData">Pointer to audio data to be added to queue.</param>
		// <param name="cbData">Number of bytes to be added to queue.</param>
		void KinAudioStream::QueueCapturedData(BYTE *pData, unsigned int cbData){
			BYTE *pWriteData = NULL;
			DWORD cbWriteData = 0;
			DWORD cbMaxLength = 0;

			//no bytes to add
			if (cbData <= 0){	return;	}
			if (NULL == currWriteBuf){			currWriteBuf = GetWriteBuffer();	}

			currWriteBuf->GetBufferAndLength(&pWriteData, &cbWriteData);
			currWriteBuf->GetMaxLength(&cbMaxLength);

			if (cbWriteData + cbData < cbMaxLength)	{
				memcpy(pWriteData + cbWriteData, pData, cbData);
				currWriteBuf->SetLength(cbWriteData + cbData);
			} else {
				QueueCapturedBuffer(currWriteBuf);

				currWriteBuf = GetWriteBuffer();
				currWriteBuf->GetBufferAndLength(&pWriteData, &cbWriteData);

				memcpy(pWriteData, pData, cbData);
				currWriteBuf->SetLength(cbData);
			}//if write + data less than max length, else
		}//QueueCapturedData

		// Add buffer full of captured audio data to the circular queue of buffers ready for client reading.
		// <param name="pBuffer">Buffer holding captured audio data.</param>
		void KinAudioStream::QueueCapturedBuffer(CStaticMediaBuffer *pBuffer){
			EnterCriticalSection(&mtxAudData);
			readBufQueue.push(pBuffer);
			SetEvent(eventDataReady);
			LeaveCriticalSection(&mtxAudData);
		}//QueueCapturedBuffer


		// single read op from current read buffer until client buffer is full or read buffer is empty
		// if read buf is empty, get next (oldest) available buffer
		// <param name="ppbData">
		// In/Out pointer to client's audio data buffer to be filled.
		// <param name="pcbData">
		// In/Out pointer to number of bytes remaining to be filled in client's audio data buffer.
		void KinAudioStream::ReadOneBuffer(BYTE **ppbData, unsigned long* pcbData){
			EnterCriticalSection(&mtxAudData);

			//Do we already have a buffer we are reading from? if not grab one from the queue
			if (NULL == currReadBuf) {
				if(readBufQueue.size() != 0){
					currReadBuf = readBufQueue.front();
					readBufQueue.pop();
				}
			}

			if (NULL != currReadBuf) {
				//Copy as much data as we can or need
				BYTE *pData = NULL;
				DWORD dwDataLength = 0;
				currReadBuf->GetBufferAndLength(&pData, &dwDataLength);

				unsigned long cbToCopy = min(dwDataLength - currReadBufIDX, *pcbData);
				memcpy(*ppbData, pData + currReadBufIDX, cbToCopy);
				*ppbData = (*ppbData)+cbToCopy;
				*pcbData = (*pcbData)-cbToCopy;
				currReadBufIDX += cbToCopy;

				//If we are done with this buffer put it back in the queue
				if (currReadBufIDX >= dwDataLength){
					ReleaseBuffer(currReadBuf);
					currReadBuf = NULL;
					currReadBufIDX = 0;

					if(readBufQueue.size() != 0){
						currReadBuf = readBufQueue.front();
						readBufQueue.pop();
					}
				}
			}//if read buffer not null

			LeaveCriticalSection(&mtxAudData);
		}//ReadOneBuffer

		// Starting address for audio capture thread.
		// <param name="pParam">
		// Thread data passed to the function using the lpParameter parameter of the CreateThread function.
		DWORD WINAPI KinAudioStream::CaptureThread(LPVOID pParam){
			KinAudioStream *pthis = (KinAudioStream *) pParam;
			return pthis->CaptureThread();
		}//CaptureThread

		// Audio capture thread. Captures audio data in a loop until it is signaled to stop.
		DWORD WINAPI KinAudioStream::CaptureThread(){
			HANDLE mmHandle = NULL;
			DWORD mmTaskIndex = 0;
			HRESULT hr = S_OK;
			bool bContinue = true;
			BYTE *pbOutputBuffer = NULL;
			CStaticMediaBuffer outputBuffer;
			DMO_OUTPUT_DATA_BUFFER OutputBufferStruct = {0};
			OutputBufferStruct.pBuffer = &outputBuffer;
			DWORD dwStatus = 0;
			unsigned long cbProduced = 0;
			double bangle, bpos, bconf;

			// Set high priority to avoid getting preempted while capturing sound
	//		mmHandle = AvSetMmThreadCharacteristicsW(L"Audio", &mmTaskIndex);					
			mmHandle = AvSetMmThreadCharacteristics("Audio", &mmTaskIndex);						//TODO verify this substitution is acceptable

			while (bContinue){
				if (WAIT_OBJECT_0 == WaitForSingleObject(eventStop, 0)){						//stop event triggered
					bContinue = false;	continue;
				}

				do{
					outputBuffer.Init(0);
					OutputBufferStruct.dwStatus = 0;
					hr = kinCapDMO->ProcessOutput(0, 1, &OutputBufferStruct, &dwStatus);
					if (FAILED(hr)){		bContinue = false;	break;}
					if (S_FALSE == hr){				cbProduced = 0;	} 
					else {				outputBuffer.GetBufferAndLength(&pbOutputBuffer, &cbProduced);		}
					if(cbProduced > 0){
						hr = kinAdSHndlr->kinNuiAudSrc->GetBeam(&bangle);																	//where is kinect listening
						if(FAILED(hr)){					clog<<"\tAudio Stream : Failed to retrieve Kinect Mic Array listening angle"<<std::endl;}
						hr = kinAdSHndlr->kinNuiAudSrc->GetPosition(&bpos, &bconf);														//where kinect things sound source is
						if(FAILED(hr)){					clog<<"\tAudio Stream : Failed to retrieve speech source location"<<std::endl;}
						if (0 != bpos){	
							kinAdSHndlr->setCurrBeamAngle(bangle);
							kinAdSHndlr->setSourceAngle(bpos);
							kinAdSHndlr->setSourceAngleConf(bconf);
							clog<<"\tAudio Stream : Angle detected : "<<bangle<<" Position and Conf detected : "<<bpos<<"|"<<bconf<<std::endl;
						}
						// Queue audio data to be read by IStream client
						QueueCapturedData(pbOutputBuffer, cbProduced);
					}
				} while (OutputBufferStruct.dwStatus & DMO_OUTPUT_DATA_BUFFERF_INCOMPLETE);

				Sleep(5);			//sleep 5ms
			}//while
			SetEvent(eventDataReady);
			AvRevertMmThreadCharacteristics(mmHandle);
			if (FAILED(hr))	{		return 0;}
			return 1;
		}//CaptureThread


	}//kinect namespace
}//namespace rtql8