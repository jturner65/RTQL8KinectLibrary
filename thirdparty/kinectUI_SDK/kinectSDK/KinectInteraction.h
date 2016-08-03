//------------------------------------------------------------------------------
// <copyright file="KinectInteraction.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#pragma pack(push, 16)

#define NUI_USER_HANDPOINTER_COUNT 2

/// <summary>
/// Enumeration used to denominate a specific hand of a user.
/// </summary>
enum NUI_HAND_TYPE
{
    NUI_HAND_TYPE_NONE = 0,
    NUI_HAND_TYPE_LEFT,
    NUI_HAND_TYPE_RIGHT,
};

/// <summary>
/// Enumeration used to denominate an event triggered by some hand of some user.
/// </summary>
enum NUI_HAND_EVENT_TYPE
{
    NUI_HAND_EVENT_TYPE_NONE = 0,   // No change from last event, or an undefined change.
    NUI_HAND_EVENT_TYPE_GRIP,       // Hand grip
    NUI_HAND_EVENT_TYPE_GRIPRELEASE // Hand grip release
};

/// <summary>
/// State flags that can be applied to a hand pointer.
/// Hand pointer could be tracked vs not tracked, pressed vs not pressed, etc.
/// </summary>
enum NUI_HANDPOINTER_STATE
{
    NUI_HANDPOINTER_STATE_NOT_TRACKED = 0x00,
    NUI_HANDPOINTER_STATE_TRACKED = 0x01,           // Hand is tracked
    NUI_HANDPOINTER_STATE_ACTIVE = 0x02,            // Hand is active within or near interaction region. In this state it is a candidate to become primary hand for user.
    NUI_HANDPOINTER_STATE_INTERACTIVE = 0x04,       // Hand is in interaction region
    NUI_HANDPOINTER_STATE_PRESSED = 0x08,           // Press happened in interaction region. Only possible in InteractionAdjuseHandPointers.
    NUI_HANDPOINTER_STATE_PRIMARY_FOR_USER = 0x10   // This hand is the primary hand for user.
};

/// <summary>
/// Handpointer information, including position (relative to UI) and state.
/// </summary>
struct NUI_HANDPOINTER_INFO
{
    DWORD State; // A combination of NUI_HANDPOINTER_STATE flags
    NUI_HAND_TYPE HandType; // Left hand vs right hand
    FLOAT X; // Horizontal position, adjusted relative to UI (via INuiInteractionClient)
    FLOAT Y; // Vertical position, adjusted relative to UI (via INuiInteractionClient)
    FLOAT PressExtent;  // Progress towards a press action relative to UI (via INuiInteractionClient)
                        // 0.0 represents press origin and 1.0 represents press trigger point
    FLOAT RawX; // Unadjusted horizontal position
    FLOAT RawY; // Unadjusted vertical position
    FLOAT RawZ; // Unadjusted arm extension
                // 0.0 represents hand close to shoulder and 1.0 represents fully extended arm
    NUI_HAND_EVENT_TYPE HandEventType; // Grip, grip release or no event
};

/// <summary>
///  Represents interaction information available about a specific user, including tracking ID
///  and associated hand pointers.
/// </summary>
struct NUI_USER_INFO
{
    DWORD SkeletonTrackingId;
    NUI_HANDPOINTER_INFO HandPointerInfos[NUI_USER_HANDPOINTER_COUNT];
};

/// <summary>
/// Frame of interaction data produced by <see cref="INuiInteractionStream"/>.
/// </summary>
struct NUI_INTERACTION_FRAME
{
    LARGE_INTEGER TimeStamp;
    NUI_USER_INFO UserInfos[NUI_SKELETON_COUNT];
};

/// <summary>
/// Represents interaction information available for a specified location in UI.
/// </summary>
struct NUI_INTERACTION_INFO
{
    BOOL IsPressTarget;
    DWORD PressTargetControlId;
    FLOAT PressAttractionPointX;
    FLOAT PressAttractionPointY;
    BOOL IsGripTarget;
};


#undef  INTERFACE
#define INTERFACE   INuiInteractionClient

/// <summary>
/// Interface that lets the Kinect Interaction framework discover information about
/// the UI elements of framework clients.
/// </summary>
DECLARE_INTERFACE_IID_(INuiInteractionClient, IUnknown, "634c4492-6561-4da4-b8d8-b1b3c6efbffc")
{
    BEGIN_INTERFACE

    /////////////////////////////////////////////
    // IUnknown methods
    STDMETHOD(QueryInterface)(THIS_ REFIID riid, void **ppv) PURE;
    STDMETHOD_(ULONG,AddRef)(THIS) PURE;
    STDMETHOD_(ULONG,Release)(THIS) PURE;

    /////////////////////////////////////////////
    // INuiInteractionClient methods

    /// <summary>
    /// Gets interaction information available for a specified location in UI.
    /// </summary>
    /// <param name="skeletonTrackingId">
    /// The skeleton tracking ID for which the interaction information is being retrieved.
    /// </param>
    /// <param name="handType">
    /// Hand type for which the interaction information is being retrieved.
    /// </param>
    /// <param name="x">
    /// X-coordinate of UI location for which interaction information is being retrieved.
    /// 0.0 corresponds to left edge of interaction region and 1.0 corresponds to right edge
    /// of interaction region.
    /// </param>
    /// <param name="y">
    /// Y-coordinate of UI location for which interaction information is being retrieved.
    /// 0.0 corresponds to top edge of interaction region and 1.0 corresponds to bottom edge
    /// of interaction region.
    /// </param>
    /// <param name="pInteractionInfo">
    /// [Out] On successful return, will contain interaction information corresponding to
    /// specified UI location.
    /// </param>
    /// <returns>
    /// S_OK if successful.
    /// E_POINTER if the value of <paramref name="pInteractionInfo"/> is NULL.
    /// </returns>
    STDMETHOD(GetInteractionInfoAtLocation)(THIS_ DWORD skeletonTrackingId, NUI_HAND_TYPE handType, FLOAT x, FLOAT y, _Out_ NUI_INTERACTION_INFO *pInteractionInfo) PURE;

    END_INTERFACE
};

#undef  INTERFACE
#define INTERFACE   INuiInteractionStream

/// <summary>
/// Represents a stream of interaction data frames.
/// Each frame contains information about users interacting with system, such as
/// position of user hands within a comfortable interaction region defined relative
/// to their body.
/// This is the main entry point for the Kinect Interaction framework, which also
/// exposes configuration properties of the framework.
/// </summary>
/// <remarks>
/// Interaction stream instances start out disabled, and clients must call
/// INuiInteractionStream::Enable before being able to retrieve interaction frames
/// from INuiInteractionStream::GetNextFrame.
/// </remarks>
DECLARE_INTERFACE_IID_(INuiInteractionStream, IUnknown, "d7b983d8-250d-4a71-ac18-e4514f676f2e")
{
    BEGIN_INTERFACE

    /////////////////////////////////////////////
    // IUnknown methods
    STDMETHOD(QueryInterface)(THIS_ REFIID riid, void **ppv) PURE;
    STDMETHOD_(ULONG,AddRef)(THIS) PURE;
    STDMETHOD_(ULONG,Release)(THIS) PURE;

    /////////////////////////////////////////////
    // INuiInteractionStream methods

    /// <summary>
    /// Enables generation of interaction frames. 
    /// </summary>
    /// <param name="hNextFrameEvent">
    /// [in] A handle to an application-allocated, manual reset event that will be set whenever a
    /// new frame of skeleton data is available, and will be reset whenever the latest frame data
    /// is returned. This is optional and can be NULL. 
    /// </param>
    /// <returns>
    /// S_OK if stream was successfully enabled.
    /// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
    /// </returns>
    STDMETHOD(Enable)(THIS_ 
        _In_opt_ HANDLE hNextFrameEvent) PURE;

    /// <summary>
    /// Processes specified depth data. 
    /// </summary>
    /// <param name="depthDataLength">
    /// [in] Number of bytes in specified depth data buffer. 
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
    /// is not the expected number of bytes for a 640x480 depth image.
    /// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
    /// </returns>
    STDMETHOD(ProcessDepth)(THIS_ 
        _In_ UINT depthDataLength,
        _In_count_(depthDataLength) const BYTE *pDepthData,
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
    /// <param name="pAccelerometerReading">
    /// [in] Current reading from Kinect Sensor's accelerometer.
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
        _In_count_(skeletonCount) const NUI_SKELETON_DATA *pSkeletonData, 
        _In_ Vector4 *pAccelerometerReading,
        _In_ LARGE_INTEGER liTimeStamp) PURE;
    
    /// <summary>
    /// Gets the next frame of data from the interaction stream
    /// </summary>
    /// <param name="dwMillisecondsToWait">
    /// [in] The time in milliseconds that GetNextFrame must wait before returning without a frame. 
    /// </param>
    /// <param name="pInteractionFrame">
    /// [out] A pointer to a NUI_INTERACTION_FRAME structure that contains the next frame in the
    /// interaction stream. This parameter cannot be NULL.
    /// </param>
    /// <returns>
    /// S_OK if interaction frame was successfully retrieved.
    /// E_POINTER if <paramref name="pInteractionFrame"/> is NULL.
    /// E_NUI_FRAME_NO_DATA if interaction stream is disabled, or if the waiting timeout expired
    /// before a frame was available.
    /// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
    /// </returns>
    STDMETHOD(GetNextFrame)(THIS_ 
        _In_ DWORD dwMillisecondsToWait,
        _Out_ NUI_INTERACTION_FRAME *pInteractionFrame) PURE;
        
    /// <summary>
    /// Disables generation of interaction frames. 
    /// </summary>
    /// <returns>
    /// S_OK if stream was successfully enabled.
    /// INuiSensor status failure code if NUI sensor associated with stream is no longer connected.
    /// </returns>
    /// <remarks>
    /// When an interaction stream is disabled, GetNextFrame will return E_NUI_FRAME_NO_DATA
    /// immediately, regardless of the wait timeout.
    /// </remarks>
    STDMETHOD(Disable)(THIS) PURE;

    END_INTERFACE
};

#pragma pack(pop)   // align

/// <summary>
/// Create the main entry point into Kinect Interaction framework.
/// </summary>
/// <param name="pNuiSensor">
/// [In] Kinect sensor reference.
/// </param>
/// <param name="pInteractionClient">
/// [In] Abstraction for client that lets the interaction stream discover information about
/// the UI elements owned by client.
/// </param>
/// <param name="ppInteractionStream">
/// [Out] On success, will receive newly created interaction stream instance.
/// </param>
/// <returns>
/// S_OK on success, failure code otherwise.
/// </returns>
HRESULT NUIAPI NuiCreateInteractionStream(_In_  INuiSensor *pNuiSensor, _In_ INuiInteractionClient *pInteractionClient, _Out_ INuiInteractionStream **ppInteractionStream);

#if defined(__cplusplus)
}
#endif