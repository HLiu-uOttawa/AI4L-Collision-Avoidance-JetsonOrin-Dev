/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using Logging;
using VmbNET;
using System;
using System.Linq;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace SynchronousGrab
{
    /// <summary>
    /// Example program to fetch images synchronously grabbed with VmbNET.
    /// </summary>
    class SynchronousGrab
    {
        static async Task Main(string[] args)
        {
            Console.WriteLine("///////////////////////////////////////");
            Console.WriteLine("/// VmbNET Synchronous Grab Example ///");
            Console.WriteLine("///////////////////////////////////////\n");

            bool frameCountFound = false;
            frameCountFound = int.TryParse(new List<string>(args).Find(arg => {
                                                                                  if (frameCountFound) return true;
                                                                                  else
                                                                                  {
                                                                                      frameCountFound = arg == "-fc";
                                                                                      return false;
                                                                                  }
                                                                              }),
                                           out var frameCount);

            string cameraId = "";
            var allocationMode = ICapturingModule.AllocationModeValue.AnnounceFrame;
            if (frameCountFound && args.Length == 2 && args[0] != "-h")
            {
                // Only frame count specification arguments are OK
            }
            else if (frameCountFound && (args.Length == 3 || args.Length == 4) && args[0] != "-h")
            {
                if (args[0] == "-x")
                {
                    allocationMode = ICapturingModule.AllocationModeValue.AllocAndAnnounceFrame;
                    if (args.Length == 4)
                    {
                        cameraId = args[1];
                    }
                }
                else
                {
                    cameraId = args[0];
                }
            }
            else
            {
                Console.WriteLine("Usage:");
                Console.WriteLine("  SynchronousGrab.exe [ -x ] [cameraId] -fc frameCount");
                Console.WriteLine("  SynchronousGrab.exe [ -h ]\n");
                Console.WriteLine("Parameters:");
                Console.WriteLine("  [ -x ]      If present, frame buffers are allocated by the transport layer)");
                Console.WriteLine("  cameraId    ID or extended ID of the camera to use (using the first camera found if not specified)");
                Console.WriteLine("  frameCount  Number of frames to capture");
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
                    Console.WriteLine("No cameras found.");
                    return;
                }
            }

            // open the camera
            using var openCamera = camera.Open();

            var features = openCamera.Features;

            // for GigE cameras only, adjust the packet size if possible
            if (openCamera.TransportLayer.Type == TransportLayerType.GEV && openCamera.Stream.Features.GVSPAdjustPacketSize.Exists)
            {
                Console.WriteLine($"trying to adjust the package size");
                await (Task)openCamera.Stream.Features.GVSPAdjustPacketSize(TimeSpan.FromSeconds(1));
            }

            // prepare stream 0 for capturing
            using var streamCapture = openCamera.PrepareCapture(allocationMode, 5);

            // start the acquisition in the camera
            features.AcquisitionStart();

            // loop over captured frames
            Enumerable.Range(0, frameCount).ToList().ForEach(_ =>
                                                             {
                                                                 using var frame = streamCapture.WaitForFrame(TimeSpan.FromMilliseconds(1000));
                                                                 Console.WriteLine($"Received frame with ID: {frame.Id}, status: {frame.FrameStatus}.");
                                                             });

            // stop the acquisition, terminate the capturing, close the camera and shutdown Vimba X
            features.AcquisitionStop();
        }
    }
}
