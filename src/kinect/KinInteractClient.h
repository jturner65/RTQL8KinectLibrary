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

#ifndef __KININTERACTCLIENT_H__
#define __KININTERACTCLIENT_H__

/*
	describes kinect interaction for particular location in gui
*/
using namespace std;

namespace rtql8 {
	namespace kinect{
		class KinInteractClient : public INuiInteractionClient{
		public:
			KinInteractClient(void){}
			~KinInteractClient(void){}
				//from INuiInteractionClient interface
			STDMETHODIMP_(unsigned long) AddRef() { return 2;     }
			STDMETHODIMP_(unsigned long) Release() { return 1;     }
			STDMETHODIMP QueryInterface(REFIID riid, void **ppv) { return S_OK;  }
				//using stdcall to match interface header
			HRESULT __stdcall GetInteractionInfoAtLocation(DWORD skeletonTrackingID, NUI_HAND_TYPE handType, FLOAT x, FLOAT y, NUI_INTERACTION_INFO *pInteractionInfo);

		private :		//variables - need to describe UI here so GetInteractionInfoAtLocation can return appropriate values.



		};//kinInteractClient
	}//namespace KinectHandlerNS
}//namespace rtql8
#endif