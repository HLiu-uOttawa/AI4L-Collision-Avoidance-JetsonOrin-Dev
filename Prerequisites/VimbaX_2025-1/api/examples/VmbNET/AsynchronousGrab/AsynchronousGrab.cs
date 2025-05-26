/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using System;
using System.Threading;
using Logging;
using VmbNET;

namespace AsynchronousGrab
{
    /// <summary>
    /// Example program to acquire images asynchronously.
    /// </summary>
    class AsynchronousGrab
    {
        static void Main()
        {
            Console.WriteLine("////////////////////////////////////////");
            Console.WriteLine("/// VmbNET Asynchronous Grab Example ///");
            Console.WriteLine("////////////////////////////////////////\n");

            IVmbSystem.Logger = LoggerCreator.CreateLogger();

            // startup Vimba X
            using var vmbSystem = IVmbSystem.Startup();

            // get all cameras found by Vimba X
            var cameras = vmbSystem.GetCameras();

            if (cameras.Count > 0)
            {
                // open the first camera found
                using (var openCamera = cameras[0].Open())
                {
                    Console.WriteLine($"Opened camera with ID {openCamera.Id}");

                    // register an event handler for the "FrameReceived" event
                    openCamera.FrameReceived += (s, e) =>
                    {
                        using var frame = e.Frame;
                        Console.WriteLine($"Frame received! ID={frame.Id}");
                    }; // IDisposable: leaving the scope will automatically requeue the frame

                    using var acquisition = openCamera.StartFrameAcquisition();
                    Thread.Sleep(5000);
                } // IDisposable: leaving the scope will automatically stop the acquisition and close the camera

                Console.WriteLine("Acquisition stopped and camera closed.");
            } 
            else
            {
                Console.WriteLine("No cameras found.");
            }
        } // IDisposable: leaving the scope will automatically shutdown VmbSystem
    }
}
