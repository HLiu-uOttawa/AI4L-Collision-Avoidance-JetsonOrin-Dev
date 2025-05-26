/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using System;
using System.Threading;
using Logging;
using VmbNET;

namespace CameraEvents
{
    /// <summary>
    /// Example program to receive camera events.
    /// </summary>
    class CameraEvents
    {
        static void Main(string[] args)
        {
            Console.WriteLine("////////////////////////////////////");
            Console.WriteLine("/// VmbNET Camera Events Example ///");
            Console.WriteLine("////////////////////////////////////\n");

            string cameraId = "";
            if (args.Length == 0)
            {
                // no arguments are OK.
            }
            else if (args.Length == 1 && args[0] != "-h")
            {
                cameraId = args[0];
            }
            else
            {
                Console.WriteLine("Usage:");
                Console.WriteLine("  CameraEvents.exe [cameraId]");
                Console.WriteLine("  CameraEvents.exe [ -h ]\n");
                Console.WriteLine("Parameters:");
                Console.WriteLine("  cameraId    ID or extended ID of the camera to use (using the first camera found if not specified)");
                Console.WriteLine("  [ -h ]      Display this usage information)\n");
                return;
            }

            IVmbSystem.Logger = LoggerCreator.CreateLogger();

            // startup Vimba X
            using var vmbSystem = IVmbSystem.Startup();

            ICamera camera;
            if (cameraId.Length > 0)
            {
                try
                {
                    // get camera with user-supplied ID
                    camera = vmbSystem.GetCameraByID(cameraId);
                }
                catch (VmbNETException e)
                {
                    Console.WriteLine($"Received VmbNET error: \"{e.Message}\" when trying to get camera.");
                    return;
                }
            }
            else
            {
                // get first detected camera
                var cameras = vmbSystem.GetCameras();
                if (cameras.Count > 0)
                {
                    camera = cameras[0];
                }
                else
                {
                    Console.WriteLine("No cameras were found.");
                    return;
                }
            }

            // open the camera
            using var openCamera = camera.Open();

            var features = openCamera.Features;

            if (!features.EventSelector.Exists)
            {
                Console.WriteLine($"No event selector is available for this camera.");
                return;
            }

            // switch all event notifications off
            foreach (var validName in features.EventSelector.EnumEntriesByName.Keys)
            {
                try
                {
                    features.EventSelector = validName;
                    features.EventNotification = "Off";
                }
                catch (VmbException<FeatureError> e) when (e.ErrorCode == VmbErrorCode.NotAvailable || e.ErrorCode == VmbErrorCode.NotFound || e.ErrorCode == VmbErrorCode.NotImplemented)
                {
                    Console.WriteLine($"Notification for event selector \"{validName}\" cannot be set to \"Off\" for this camera.");
                    return;
                }
            }

            // switch AcquisitionStart event notification on
            if (features.EventSelector.EnumEntriesByName.ContainsKey("AcquisitionStart"))
            {
                try
                {
                    features.EventSelector = "AcquisitionStart";
                    features.EventNotification = "On";
                }
                catch (VmbException<FeatureError> e) when (e.ErrorCode == VmbErrorCode.NotAvailable || e.ErrorCode == VmbErrorCode.NotFound || e.ErrorCode == VmbErrorCode.NotImplemented)
                {
                    Console.WriteLine($"Notification for event selector \"AcquisitionStart\" cannot be set to \"On\" for this camera.");
                    return;
                }
            }
            else
            {
                Console.WriteLine($"No acquisition start event is available for this camera.");
                return;
            }

            // used to make the main thread wait for the event handler to be called
            var signal = new ManualResetEvent(false);

            features.EventAcquisitionStart.Changed += (EventHandler<FeatureChangedEventArgs>)(
                (_, e) => {
                              Console.WriteLine($"Event received: {e.Feature.Name}");

                              // tell the main thread to continue
                              signal.Set();
                          });

            // start the acquisition in the camera, which will trigger the event
            features.AcquisitionStart();

            // wait for the event handler to signal that it has finished
            signal.WaitOne();

            // stop the acquisition
            features.AcquisitionStop();
        }
    }
}
