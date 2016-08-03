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
#include "KinAudioHandler.h"

extern LPCWSTR GrammarFileName;

namespace rtql8{
	namespace kinect{

		KinAudioHandler::KinAudioHandler(INuiSensor* _oKin, KinectHandler* _owningKH) : KinStreamHandler(_oKin, _owningKH, _H_AUDIO),
						kinAudioStream(nullptr), kinNuiAudSrc(nullptr),  kinSpchStream(nullptr),   spchRecognizer(nullptr),   spchRecogCntxt(nullptr),   
						spchRecogGrammar(nullptr), currResult(""), currConf(0), currBeamAngle(0),	currSourceAngle(0), currSourceConf(0)

		{
				//how sample instantiated thread - investigate
			HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);			//initialize com apartment for multithreaded com object
			if (SUCCEEDED(hr)) {         HRESULT tmpRes = initKinStreamHandler();		}			//derived class version				
		}//cnstrctr

		KinAudioHandler::~KinAudioHandler(){
			CoUninitialize();										//release com apartment
			audSafeRelease(kinNuiAudSrc);
			audSafeRelease(kinAudioStream);
			audSafeRelease(kinSpchStream);
			audSafeRelease(spchRecognizer);
			audSafeRelease(spchRecogCntxt);
			audSafeRelease(spchRecogGrammar);
		}//dstrctr

		//initialize this handler's values, verify appropriate speech engines/sapi installed
		HRESULT KinAudioHandler::initKinStreamHandler(){
			HRESULT tmpRes = KinStreamHandler::initKinStreamHandler();		//base class
			if (CLSID_ExpectedRecognizer != CLSID_SpInprocRecognizer){
				clog<<"\tAudio Handler : !!!!   This program requires the correct version of sapi.h for Kinect."<<std::endl;
				clog<<"\tAudio Handler : !!!!   Please use the speech libraries included in the kinect sdk"<<std::endl;
				return EXIT_FAILURE;
			}
			initHndlrDataStructs();
			return tmpRes;
		}//initKinStreamHandler

		void KinAudioHandler::initHndlrDataStructs(){
			initVars();				//vars inited via initKinDataStream
		}

		void KinAudioHandler::initVars(){
			STRM_Flags = vector<bool>(AU_NumFlags, false);
			currResult = "";
			currConf = 0; 
			currBeamAngle = 0;	
			currSourceAngle = 0;
			currSourceConf = 0;
		}//initVars

		//initialize kinect audio stream object
		HRESULT KinAudioHandler::initKinDataStream(){
			clog<<"\tAudio Handler Init : init stream"<<std::endl;
			HRESULT hr = InitializeAudioStream();
			if (FAILED(hr))  {			clog<<"\tAudio Handler Init : Could not initialize audio stream : "<<hr<<std::endl;	return hr;	}
			clog<<"\tAudio Handler Init : create SR"<<std::endl;
			hr = CreateSpeechRecognizer();
			if (FAILED(hr))  {			clog<<"\tAudio Handler Init : Could not create speech recognizer. Please ensure that Microsoft Speech SDK and other sample requirements are installed : "<<hr<<std::endl;return hr;}
			clog<<"\tAudio Handler Init : load speech grammar"<<std::endl;
			hr = LoadSpeechGrammar();
			if (FAILED(hr))  {			clog<<"\tAudio Handler Init : Could not load speech grammar. Please ensure that grammar configuration file was properly deployed : "<<hr<<std::endl;	return hr;	}
			clog<<"\tAudio Handler Init : start SR"<<std::endl;
			hr = StartSpeechRecognition();
			if (FAILED(hr))  {			clog<<"\tAudio Handler Init : Could not start recognizing speech : "<<hr<<std::endl;	return hr;	}
			clog<<"\tAudio Handler Init : completed : "<<hr<<std::endl;
			return hr;
		}//initKinDataStream

		//initialize audio stream and assign audio beam ptr
		HRESULT KinAudioHandler::InitializeAudioStream(){
			IMediaObject*       pDMO = NULL;
			IPropertyStore*     pPropStore = NULL;
			IStream*            pStream = NULL;

			////// Get the audio source
			HRESULT hr = owningKinect->NuiGetAudioSource(&kinNuiAudSrc);
			if (SUCCEEDED(hr)){		
				hr = kinNuiAudSrc->QueryInterface(IID_IMediaObject, (void**)&pDMO);
				if (SUCCEEDED(hr)){
					hr = kinNuiAudSrc->QueryInterface(IID_IPropertyStore, (void**)&pPropStore);
					// Set AEC-MicArray DMO system mode. This must be set for the DMO to work properly.
					// Possible values are:
					//   SINGLE_CHANNEL_AEC = 0;  OPTIBEAM_ARRAY_ONLY = 2;  OPTIBEAM_ARRAY_AND_AEC = 4;   SINGLE_CHANNEL_NSAGC = 5
					PROPVARIANT pvAudSysMode;
					PropVariantInit(&pvAudSysMode);
					pvAudSysMode.vt = VT_I4;
					pvAudSysMode.lVal = (LONG)(2);							// Use OPTIBEAM_ARRAY_ONLY setting. Set OPTIBEAM_ARRAY_AND_AEC instead if you expect to have sound playing from speakers - need auto echo cancellation.
					hr = pPropStore->SetValue(MFPKEY_WMAAECMA_SYSTEM_MODE, pvAudSysMode);
					if(FAILED(hr)){		clog<<"\tAudio Handler Init : Failed setting DMO object System Mode code : "<<hr<<std::endl;}
					PropVariantClear(& pvAudSysMode);
						//enable modification of DMO features
					PROPVARIANT pvFeatMode;
					PropVariantInit(&pvFeatMode);
					pvFeatMode.vt = VT_BOOL;
					pvFeatMode.boolVal = VARIANT_TRUE;
					hr = pPropStore->SetValue(MFPKEY_WMAAECMA_FEATURE_MODE,pvFeatMode);								//needed to set features of dmo object
					if(FAILED(hr)){		clog<<"\tAudio Handler Init : Failed setting DMO object Feature Modification Enable Mode code : "<<hr<<std::endl;}
					PropVariantClear(& pvFeatMode);
						//enable manually setting the listening beam
					PROPVARIANT pvMicArrMode;
					PropVariantInit(&pvMicArrMode);
					pvMicArrMode.vt = VT_I4;
					pvMicArrMode.lVal = (LONG)(MICARRAY_EXTERN_BEAM);
					hr = pPropStore->SetValue(MFPKEY_WMAAECMA_FEATR_MICARR_MODE, pvMicArrMode);					//to be able to use setbeam
					if(FAILED(hr)){		clog<<"\tAudio Handler Init : Failed setting DMO object MICARR Mode code : "<<hr<<std::endl;}
					PropVariantClear(&pvMicArrMode);

					// Set DMO output format
					WAVEFORMATEX wfxOut = {AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0};
					DMO_MEDIA_TYPE mt = {0};
					MoInitMediaType(&mt, sizeof(WAVEFORMATEX));
					setMediaType(mt, wfxOut);

					hr = pDMO->SetOutputType(0, &mt, 0);

					if (SUCCEEDED(hr)){
						kinAudioStream = new KinAudioStream(pDMO,this);																				//build audio stream handler
						hr = kinAudioStream->QueryInterface(IID_IStream, (void**)&pStream);
						if (SUCCEEDED(hr)){
							hr = CoCreateInstance(CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpStream), (void**)&kinSpchStream);		//make thread for audio stream handler
							if (SUCCEEDED(hr)){			hr = kinSpchStream->SetBaseStream(pStream, SPDFID_WaveFormatEx, &wfxOut);		}		//if thread created
						}//if stream created
					}//if DMO output set
					MoFreeMediaType(&mt);					//frees media object
				}//if audio source interface found
			}//if audio source found

			audSafeRelease(pStream);
			audSafeRelease(pPropStore);
			audSafeRelease(pDMO);
			return hr;
		}//InitializeAudioStream

		//set media type based on wave format
		void KinAudioHandler::setMediaType(DMO_MEDIA_TYPE& mt, WAVEFORMATEX& wfxOut){
			mt.majortype = MEDIATYPE_Audio;
			mt.subtype = MEDIASUBTYPE_PCM;
			mt.lSampleSize = 0;
			mt.bFixedSizeSamples = TRUE;
			mt.bTemporalCompression = FALSE;
			mt.formattype = FORMAT_WaveFormatEx;	
			memcpy(mt.pbFormat, &wfxOut, sizeof(WAVEFORMATEX));
		}//setMediaType

		//create speech recognition engine
		HRESULT KinAudioHandler::CreateSpeechRecognizer(){
			ISpObjectToken *pEngineToken = NULL;
    
			HRESULT hr = CoCreateInstance(CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpRecognizer), (void**)&spchRecognizer);
			if (SUCCEEDED(hr)){
				spchRecognizer->SetInput(kinSpchStream, FALSE);
				hr = SpFindBestToken(SPCAT_RECOGNIZERS,L"Language=409;Kinect=True",NULL,&pEngineToken);
				if (SUCCEEDED(hr)){
					spchRecognizer->SetRecognizer(pEngineToken);
					hr = spchRecognizer->CreateRecoContext(&spchRecogCntxt);
					if (SUCCEEDED(hr)){    hr = spchRecognizer->SetPropertyNum(L"AdaptationOn", 0);   }	//turns off acoustic model adaptation
				}//if input for recognizer found
			}//if instance created
			audSafeRelease(pEngineToken);
			return hr;
		}//CreateSpeechRecognizer

		/// Load speech recognition grammar into recognizer.
		HRESULT KinAudioHandler::LoadSpeechGrammar(){
			HRESULT hr = spchRecogCntxt->CreateGrammar(1, &spchRecogGrammar);
			if (SUCCEEDED(hr)){	
				wstringstream fs;
				fs<<GrammarFileName;
				const std::wstring tmpFN = fs.str();												//file name, const defined to extend lifetime
				LPCWSTR grmFileName = tmpFN.c_str();
				try {
                    wclog<<"\tAudio Handler Init : Loading Grammar file name : "<<grmFileName<<endl;
					hr = spchRecogGrammar->LoadCmdFromFile(grmFileName, SPLO_STATIC);					// Populate recognition grammar from file
				} 
				catch (...) {
					wclog<<"\tAudio Handler : Grammar file named : `"<<grmFileName<<"` not found"<<std::endl;
					hr = -1;
				}
			}//if grammar created		
		   return hr;
		}//LoadSpeechGrammar

		//start audio capture and speech recognition
		HRESULT KinAudioHandler::StartSpeechRecognition(){
			HRESULT hr = kinAudioStream->StartCapture();
			if (SUCCEEDED(hr)) {																		//currently always succeeds
				spchRecogGrammar->SetRuleState(NULL, NULL, SPRS_ACTIVE);								// specify that all top level rules in grammar are now active
				spchRecognizer->SetRecoState(SPRST_ACTIVE_ALWAYS);										// specify that engine should always be reading audio
				spchRecogCntxt->SetInterest(SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION));			// specify that we're only interested in receiving recognition events
				hr = spchRecogCntxt->Resume(0);															// ensure that engine is not in paused state
				if (SUCCEEDED(hr))  {			      m_hNextEvent = spchRecogCntxt->GetNotifyEventHandle();   }
			}//if engine was started        
			return hr;
		}//StartSpeechRecognition

		// Process recently triggered speech recognition events.
		void KinAudioHandler::processSpeech(){
			if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextEvent, 0)){											//checking if event triggered	
				//clog<<"-----------KinAudioHandler : process speech event triggered "<<std::endl;
				SPEVENT curEvent;
				unsigned long fetched = 0;
				HRESULT hr = S_OK;

				spchRecogCntxt->GetEvents(1, &curEvent, &fetched);
				while (fetched > 0){																			//while there are speech events to process
					switch (curEvent.eEventId){																	//enum of eventid's in sapi.h
						case SPEI_RECOGNITION	:	{
							if (SPET_LPARAM_IS_OBJECT == curEvent.elParamType){									// check if this is recognizer-originated event							
								ISpRecoResult* result = reinterpret_cast<ISpRecoResult*>(curEvent.lParam);		
								SPPHRASE* pPhrase = NULL;                    
								hr = result->GetPhrase(&pPhrase);
								if (SUCCEEDED(hr)){																//phrase retreived from recognizer
									//hr = kinNuiAudSrc->GetBeam(& currBeamAngle);								//where is kinect listening
									//if(FAILED(hr)){clog<<"\tAudio Handler : Failed to retrieve Kinect Mic Array listening angle"<<std::endl;}
									//hr = kinNuiAudSrc->GetPosition(& currSourceAngle, & currSourceConf);		//where kinect things sound source is
									//if(FAILED(hr)){clog<<"\tAudio Handler : Failed to retrieve speech source location"<<std::endl;}
									if ((NULL != pPhrase) && (nullptr != pPhrase->pProperties) && (nullptr != pPhrase->pProperties->pFirstChild)){
										//clog<<"phrase length : "<<pPhrase->pProperties->ulCountOfElements<<std::endl;
										const SPPHRASEPROPERTY* pSemanticTag = pPhrase->pProperties->pFirstChild;
										if (pSemanticTag->SREngineConfidence > Aud_ConfThresh){
											clog<<"\tAudio Handler : Speech recognized with confidence : "<<pSemanticTag->SREngineConfidence<<" from angle : "<<currSourceAngle<<" conf : "<<currSourceConf<<std::endl;
											//MapSpeechTag(pSemanticTag->pszValue, pPhrase->pElements->pszDisplayText, pSemanticTag->SREngineConfidence);
											MapSpeechTag(pSemanticTag->pszValue, pSemanticTag->SREngineConfidence);
										}//if confidence greater than threshold
										else {//spoken phrase not found in grammar
											clog<<"\tAudio Handler : Speech not recognized "<<std::endl;
											currConf = 0;
											currResult = "";
										}
									}//if phrase not null
									else{																			//speech phrase null
										clog<<"\tAudio Handler : Speech null "<<std::endl;

									}
									::CoTaskMemFree(pPhrase);														//com free from getPhrase
								}//if phrase gotten - if failed then skips this area
							}//if from recognizer
							break;}//SPEI_RECOGNITION == event
					}//switch
					spchRecogCntxt->GetEvents(1, &curEvent, &fetched);
				}//while
			}//if event triggered for audio/speech handler
		}//processSpeech

		////process speech detected for result string
		//string KinAudioHandler::MapSpeechTag(LPCWSTR pszSpeechTag, LPCWSTR pszSpeechWord, float conf){
		//	clog<<"recognized string:  ";
		//	wclog<<pszSpeechTag;
		//	clog<<" word : ";
		//	wclog<<pszSpeechWord;
		//	clog<<" confidence : "<<conf;
		//	clog<<std::endl;
		//	std::wstringstream ws(wstringstream::in | wstringstream::out);
		//	ws<<pszSpeechTag<<L"|"<<pszSpeechWord;
		//	string tmp(ws.str().begin(), ws.str().end());
		//	currResult = tmp;
		//	return "";
		//}

		////process speech detected for result string
		string KinAudioHandler::MapSpeechTag(LPCWSTR pszSpeechTag, float _conf){
			clog<<"\tAudio Handler : recognized string:  ";
			wclog<<pszSpeechTag;
			clog<<std::endl;

			std::wstringstream ws;
			ws<<pszSpeechTag;

			std::wstring tmpWstr = ws.str();
			std::string tmp(tmpWstr.begin(), tmpWstr.end());
			currResult = tmp;										//current result of speech recognition
			currConf = _conf;
			return "";
		}

		//string KinAudioHandler::getSpchRecogResult(){	return currResult;	}
		void KinAudioHandler::setBeamAngle(double beamAngle){
			HRESULT hr = kinNuiAudSrc->SetBeam(beamAngle);
			if(FAILED(hr)){clog<<"\tAudio Handler : Failed to set Kinect Mic Array listening angle"<<std::endl;}		
			else {
				hr = kinNuiAudSrc->GetBeam(& currBeamAngle);								//reset where kinect is listening to match new setting - use this in case new value doesn't match passed value 
				if(FAILED(hr)){clog<<"\tAudio Handler : Failed to retrieve Kinect Mic Array listening angle"<<std::endl;}
			}
		}//setBeamAngle

		string KinAudioHandler::getSpchRecogResult(){	return currResult;	}
		double KinAudioHandler::getBeamAngle(){			return currBeamAngle;}
		double KinAudioHandler::getSourceAngle(){		return currSourceAngle;}
		double KinAudioHandler::getSourceAngleConf(){	return currSourceConf;}
		float  KinAudioHandler::getSpchRecogConf(){		return currConf;}
				//return appropriate kinect flag (or 0) for this stream corresponding to current setting of state machine flag 
		unsigned long KinAudioHandler::getKHSMFlagMode(int idx, bool state){ return 0l;}//getKHSMFlagMode

		inline bool KinAudioHandler::setFlag(int idx, bool val){
			this->STRM_Flags[idx] = val;
			switch (idx){
				case _AU_SEATED_IDX		:{break;}
				case _AU_NEAR_IDX		:{break;}
				default : {break;}
			}//switch	
			return true;
		}//setFlag
	
	}//namespace kinect
}//namespace rtql8
