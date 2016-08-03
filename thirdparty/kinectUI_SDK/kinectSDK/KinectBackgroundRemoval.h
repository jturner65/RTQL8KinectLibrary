// -----------------------------------------------------------------------------------------
// <copyright file="KinectBackgroundRemoval.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// -----------------------------------------------------------------------------------------

#pragma once

/// <summary>
/// Struct that represents a frame in the INuiBackgroundRemovedColorStream
/// </summary>
typedef struct _NUI_BACKGROUND_REMOVED_COLOR_FRAME
{
	/// time stamp for the background removed color frame.
	LARGE_INTEGER  liTimeStamp;

	/// pointer to the color data whose background is removed.
	const BYTE*    pBackgroundRemovedColorData;

	/// length of the background removed color frame, in terms of number of bytes the color data occupies
	UINT           backgroundRemovedColorDataLength;

    /// the image resolution of the background removed color frame
    NUI_IMAGE_RESOLUTION backgroundRemovedColorFrameResolution;

	/// average depth in millimeters of the user in the foreground
	USHORT         averageDepth;

	/// the player that is currently in the foreground. If the output frame does not have a tracked skeleton,
    /// this will be NUI_SKELETON_INVALID_TRACKING_ID
	DWORD          trackedPlayerID;

} NUI_BACKGROUND_REMOVED_COLOR_FRAME;


#if defined(__cplusplus)
extern "C" {
#endif

#pragma pack(push, 16)

#undef  INTERFACE
#define INTERFACE   INuiBackgroundRemovedColorStream

/// <summary>
/// Interface for setting the parameters of the depth transformer filter.
/// </summary>
DECLARE_INTERFACE_IID_(INuiBackgroundRemovedColorStream, IUnknown, "2A44A803-B828-4F29-A050-48CB4B7D3192")
{
	/////////////////////////////////////////////
	// INuiBackgroundRemovedColorStream methods

	/// <summary>
	/// Enables generation of background removed color frames. 
	/// </summary>
	/// <param name="colorResolution">
	/// [in] The resolution of the Color Frames which shall be passed to the background remover
	/// </param>
	/// <param name="depthResolution">
	/// [in] The resolution of the Depth Frames which shall be passed to the background remover
	/// The depth frame data should be the extended depth format
	/// </param>
	/// <param name="hNextFrameEvent">
	/// [in] A handle to an application-allocated, manual reset event that will be set whenever a
	/// new frame of background removed color data is available, and will be reset whenever the latest frame data
	/// is returned. This is optional and can be NULL or INVALID_HANDLE_VALUE. 
	/// </param>
	/// <returns>
	/// S_OK if stream was successfully enabled.
    /// E_INVALIDARG if the color or depth resolution is not supported.
	/// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
	/// </returns>
	STDMETHOD(Enable)(THIS_ 
		_In_ NUI_IMAGE_RESOLUTION colorResolution,
		_In_ NUI_IMAGE_RESOLUTION depthResolution,
		_In_opt_ HANDLE hNextFrameEvent) PURE;

	/// <summary>
	/// Configures the Stream to perform background removal such that only those pixels corresponding
	/// to the player's skeleton tracking ID will remain. If there is no tracked skeleton with this tracking ID, 
	/// then no players will be tracked (the output frame will be completely transparent).
	/// </summary>
	/// <param name="trackingID">
	/// [in] The tracked player that is considered to be in the foreground.
	/// </param>
	/// </summary>
	STDMETHOD(SetTrackedPlayer)(THIS_
		_In_ DWORD trackingID) PURE;

	/// <summary>
	/// Processes specified depth data. 
	/// </summary>
	/// <param name="depthDataLength">
	/// [in] Number of NUI_DEPTH_IMAGE_PIXEL in specified depth data buffer. 
	/// </param>
	/// <param name="pDepthData">
	/// [in] Depth data buffer to process.
	/// </param>
	/// <param name="liTimeStamp">
	/// [in] Time when depth data buffer was generated.
	/// </param>
	/// <returns>
	/// S_OK if depth data was successfully processed.
	/// E_INVALIDARG if <paramref name="pDepthData"/> is NULL or if <paramref name="depthDataLength"/>
	/// is not the expected number of bytes for the registered depth image type.
	/// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
	/// </returns>
	STDMETHOD(ProcessDepth)(THIS_ 
		_In_ UINT depthDataLength,
		_In_count_(depthDataLength) const BYTE* pDepthData, 
		_In_ LARGE_INTEGER liTimeStamp) PURE;

	/// <summary>
	/// Processes specified color data. 
	/// </summary>
	/// <param name="colorDataLength">
	/// [in] Number of bytes in specified color data buffer. 
	/// </param>
	/// <param name="pColorData">
	/// [in] Color data buffer to process. (ARGB data format)
	/// </param>
	/// <param name="liTimeStamp">
	/// [in] Time when color data buffer was generated.
	/// </param>
	/// <returns>
	/// S_OK if color data was successfully processed.
	/// E_INVALIDARG if <paramref name="pColorData"/> is NULL or if <paramref name="colorDataLength"/>
	/// is not the expected number of bytes for the registered color image type.
	/// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
	/// </returns>
	STDMETHOD(ProcessColor)(THIS_ 
		_In_ UINT colorDataLength,
		_In_count_(colorDataLength) const BYTE* pColorData, 
		_In_ LARGE_INTEGER liTimeStamp) PURE;

	/// <summary>
    /// Processes specified skeleton data. 
    /// </summary>
    /// <param name="skeletonCount">
    /// [in] Number of elements in specified skeleton data array. 
    /// </param>
    /// <param name="pSkeletonData">
    /// [in] Skeleton data array to be processed.
    /// </param>
    /// <param name="liTimeStamp">
    /// [in] Time when skeleton data was generated.
    /// </param>
    /// <returns>
    /// S_OK if skeleton data was successfully processed.
    /// E_INVALIDARG if <paramref name="pSkeletonData"/> is NULL or if <paramref name="skeletonCount"/>
    /// is different from NUI_SKELETON_COUNT.
    /// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
    /// </returns>
    STDMETHOD(ProcessSkeleton)(THIS_ 
        _In_ UINT skeletonCount,
        _In_count_(skeletonCount) const NUI_SKELETON_DATA* pSkeletonData,
        _In_ LARGE_INTEGER liTimeStamp) PURE;

	/// <summary>
	/// Gets the next frame of data from the background removed color stream
	/// </summary>
	/// <param name="dwMillisecondsToWait">
	/// [in] The time in milliseconds that GetNextFrame must wait before returning without a frame. 
	/// </param>
	/// <param name="pFrame">
	/// [out] pointer to player background removed  color frame.
	/// </param>
	/// <returns>
	/// S_OK if the background removed color frame was successfully retrieved.
	/// E_NUI_FRAME_NO_DATA if interaction stream is disabled, or if the waiting timeout expired
	/// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
	/// </returns>
	/// <remarks>
	/// If this stream was opened in eventing mode, calling this function will result in resetting the waitable handle with which the frame was initialized
	/// </remarks>
	STDMETHOD(GetNextFrame)(THIS_
		_In_ DWORD dwMillisecondsToWait,
		_Out_ NUI_BACKGROUND_REMOVED_COLOR_FRAME* pFrame) PURE;

	/// <summary>
	/// Releases the specified frame of data.
	/// </summary>
	/// <param name="pFrame">
	/// [in] the frame whose data will be released
	/// </param>
	/// <returns>
	/// S_OK if the frame is successfully released
	/// E_INVALIDARG if pFrame is null
	/// </returns>
	STDMETHOD(ReleaseFrame)(THIS_
		_In_ const NUI_BACKGROUND_REMOVED_COLOR_FRAME* pFrame) PURE;

	/// <summary>
	/// Disables generation of background removed color frames. 
	/// </summary>
	/// <returns>
	/// S_OK if stream was successfully disabled.
	/// </returns>
	/// <remarks>
	/// When a background removed color stream is disabled, GetNextFrame will return E_NUI_FRAME_NO_DATA
	/// immediately, regardless of the wait timeout.
	/// </remarks>
	STDMETHOD(Disable)(THIS) PURE;

	END_INTERFACE

	};

#undef  INTERFACE
#pragma pack(pop)   // align

	HRESULT NUIAPI NuiCreateBackgroundRemovedColorStream(_In_  INuiSensor *pNuiSensor, _Out_ INuiBackgroundRemovedColorStream **ppBackgroundRemovedColorStream);

#if defined(__cplusplus)
}
#endif