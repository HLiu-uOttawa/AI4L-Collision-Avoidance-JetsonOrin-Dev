/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using System;
using Logging;
using VmbNET;

namespace ListCameras
{
    /// <summary>
    /// Example program to list all cameras found by Vimba X.
    /// </summary>
    class ListCameras
    {
        static void Main()
        {
            Console.WriteLine("///////////////////////////////////");
            Console.WriteLine("/// VmbNET List Cameras Example ///");
            Console.WriteLine("///////////////////////////////////\n");

            IVmbSystem.Logger = LoggerCreator.CreateLogger();

            // startup Vimba X
            using var vmbSystem = IVmbSystem.Startup();

            // get all cameras found by Vimba X
            var cameras = vmbSystem.GetCameras();

            foreach (var camera in cameras)
            {
                Console.WriteLine($"Camera serial: {camera.Serial}");
                Console.WriteLine($"ID: {camera.Id}");
                Console.WriteLine($"Extended ID: {camera.IdExtended}");
                Console.WriteLine($"Name: {camera.Name}");
                Console.WriteLine($"Model name: {camera.ModelName}");
                Console.WriteLine($"Interface name: {camera.Interface.Name}");
                Console.WriteLine($"Interface ID: {camera.Interface.Id}");
                Console.WriteLine($"Transport layer name: {camera.TransportLayer.Name}");
                Console.WriteLine($"Transport layer ID: {camera.TransportLayer.Id}");
                Console.WriteLine($"Transport layer path: {camera.TransportLayer.Path}\n");
            }
        }
    }
}
