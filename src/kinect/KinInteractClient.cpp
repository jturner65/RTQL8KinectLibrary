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
#include "KinInteractClient.h"

using namespace std;

namespace rtql8{

	namespace kinect{
		//implementation of interface method determining NUI_INTERACTION_INFO of calling skeleton, hand and location
		//so results go into pInteractInfo

		/// Gets interaction information available for a specified location in UI.
		/// <param name="skeletonTrackingId"> The skeleton tracking ID for which the interaction information is being retrieved.
		/// <param name="handType"> Hand type for which the interaction information is being retrieved.
		/// <param name="x"> X-coordinate of UI location for which interaction information is being retrieved.
		/// 0.0 corresponds to left edge of interaction region and 1.0 corresponds to right edge
		/// of interaction region.
		/// <param name="y"> Y-coordinate of UI location for which interaction information is being retrieved.
		/// 0.0 corresponds to top edge of interaction region and 1.0 corresponds to bottom edge
		/// of interaction region.
		/// <param name="pInteractionInfo">[Out] On successful return, will contain interaction information corresponding to
		/// specified UI location.
		/// <returns>
		/// S_OK if successful.
		/// E_POINTER if the value of <paramref name="pInteractionInfo"/> is NULL.

		 //   DWORD State; // A combination of NUI_HANDPOINTER_STATE flags
			//NUI_HAND_TYPE HandType; // Left hand vs right hand
			//FLOAT X; // Horizontal position, adjusted relative to UI (via INuiInteractionClient)
			//FLOAT Y; // Vertical position, adjusted relative to UI (via INuiInteractionClient)
			//FLOAT PressExtent;  // Progress towards a press action relative to UI (via INuiInteractionClient)
			//					// 0.0 represents press origin and 1.0 represents press trigger point
			//FLOAT RawX; // Unadjusted horizontal position
			//FLOAT RawY; // Unadjusted vertical position
			//FLOAT RawZ; // Unadjusted arm extension
			//			// 0.0 represents hand close to shoulder and 1.0 represents fully extended arm
			//NUI_HAND_EVENT_TYPE HandEventType; // Grip, grip release or no event

		/// </returns>
		HRESULT KinInteractClient::GetInteractionInfoAtLocation(DWORD skeletonTrackingID, NUI_HAND_TYPE handType, FLOAT x, FLOAT y, NUI_INTERACTION_INFO *pInteractInfo)  {
			if(pInteractInfo){				//if correct interaction info format then send back data corresponding to accepted interaction grip event, modify to handle locations in future
					//determine these values based upon UI/user input - need to define interaction zone for application
				pInteractInfo->IsPressTarget         = false;
				pInteractInfo->PressTargetControlId  = 0;
				pInteractInfo->PressAttractionPointX = 0;
				pInteractInfo->PressAttractionPointY = 0;
				pInteractInfo->IsGripTarget          = true;
				clog<<"KinInteractClient : GetInteractionAtInfo called :skel : "<< skeletonTrackingID<<" : hand type :"<<handType<<std::endl;
				return S_OK;
			}
			return E_POINTER;			//error - invalid pInteractInfo pointer
		}//GetInteractionInfoAtLocation

	}//namespace kinect
}//namespace rtql8