/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using Logging;
using VmbNET;
using System;
using System.Linq;
using System.Threading.Tasks;

namespace ChunkData
{
    /// <summary>
    /// Example program to get chunk data from captured frames.
    /// </summary>
    class ChunkData
    {
        static async Task Main(string[] args)
        {
            Console.WriteLine("/////////////////////////////////");
            Console.WriteLine("/// VmbNET Chunk Data Example ///");
            Console.WriteLine("/////////////////////////////////\n");

            bool frameCountFound = false;
            frameCountFound = int.TryParse(args.FirstOrDefault(arg => {
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
                Console.WriteLine("  ChunkData.exe [ -x ] [cameraId] -fc frameCount");
                Console.WriteLine("  ChunkData.exe [ -h ]\n");
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

            // configure chunk data
            try
            {
                features.ChunkModeActive = false;

                // disable all chunk data
                foreach (var entryName in features.ChunkSelector.EnumEntriesByName.Keys)
                {
                    features.ChunkSelector = entryName;
                    features.ChunkEnable = false;
                }

                // enable selected chunk data
                foreach (var entryName in new string[] { "Timestamp", "Width", "Height", "ExposureTime" })
                {
                    features.ChunkSelector = entryName;
                    features.ChunkEnable = true;
                }

                features.ChunkModeActive = true;
            }
            catch (VmbException<FeatureError> e) when (e.ErrorCode == VmbErrorCode.NotAvailable || e.ErrorCode == VmbErrorCode.NotImplemented || e.ErrorCode == VmbErrorCode.NotFound)
            {
                Console.WriteLine($"Chunk data is not available for this camera ({e.Message}).");
                return;
            }
            catch (VmbNETException e)
            {
                Console.WriteLine($"An unexpected error occured when enabling Chunk Data: {e.Message}");
                return;
            }

            // prepare stream 0 for capturing
            using var streamCapture = openCamera.PrepareCapture(allocationMode, 5);

            // start the acquisition in the camera
            features.AcquisitionStart();

            // loop over captured frames
            Enumerable.Range(0, frameCount).ToList().ForEach(_ =>
            {
                using var frame = streamCapture.WaitForFrame(TimeSpan.FromMilliseconds(1000));
                if(frame.FrameStatus == IFrame.FrameStatusValue.Completed && frame.ContainsChunkData)
                {
                    // add chunk data access callback to retrieve the chunk data
                    frame.AccessChunkData(featureContainer =>
                                          {
                                              var chunkFeatures = featureContainer.Features;
                                              Console.WriteLine($"Received completed frame with ChunkTimestamp: {chunkFeatures.ChunkTimestamp}, "
                                                                + $"ChunkWidth: {chunkFeatures.ChunkWidth}, "
                                                                + $"ChunkHeight: {chunkFeatures.ChunkHeight}, "
                                                                + "ChunkExposureTime: " + string.Format("{0:F2}", (double)chunkFeatures.ChunkExposureTime));
                                          }
                                         );
                }
                else if(frame.FrameStatus != IFrame.FrameStatusValue.Completed)
                {
                    Console.WriteLine("Received an incomplete frame");
                }
                else
                {
                    Console.WriteLine("Frame contains no chunk data");
                }
            });

            // stop the acquisition, terminate the capturing, close the camera and shutdown Vimba X
            features.AcquisitionStop();
        }
    }
}
