/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using System;
using System.Collections.Generic;
using Logging;
using VmbNET;

namespace ListFeatures
{
    /// <summary>
    /// Example program to list all features of a module.
    /// </summary>
    /// 
    /// <remarks>
    /// Since this example only prints the details of each feature,
    /// they are handled as dynamic types to minimize the lines of code.
    /// For more complicated applications involving a feature,
    /// conversion to the underlying type (e.g. <see cref="IIntFeature"/>) should be performed.
    /// </remarks>
    class ListFeatures
    {
        static void Main(string[] args)
        {
            Console.WriteLine("////////////////////////////////////");
            Console.WriteLine("/// VmbNET List Features Example ///");
            Console.WriteLine("////////////////////////////////////\n");

            IVmbSystem.Logger = LoggerCreator.CreateLogger();

            // startup Vimba X
            using var vmbSystem = IVmbSystem.Startup();

            // Get the feature container corresponding to the user input.
            // If the user input is invalid, print the usage information.
            // In some cases, a disposable open camera and possibly one of its disposable streams is instantiated,
            // which must then be disposed after use. Nullable types are used to detect such instantiations.

            IFeatureContainer? featureContainer = null;
            IOpenCamera? openCamera = null;
            IStream? stream = null;

            if ((args.Length == 1 || args.Length == 2) && args[0] == "-t")
            {
                // get the desired transport layer as a feature container
                var transportLayers = vmbSystem.GetTransportLayers();
                if (args.Length == 2)
                {
                    if (int.TryParse(args[1], out int transportLayerIndex))
                    {
                        if (transportLayerIndex < transportLayers.Count)
                        {
                            featureContainer = transportLayers[transportLayerIndex];
                        }
                        else
                        {
                            Console.WriteLine((transportLayers.Count > 0 ? $"Only {transportLayers.Count}" : "No") + " transport layers were found.");
                            return;
                        }
                    }
                    else
                    {
                        Console.WriteLine($"\"{args[1]}\" is not a valid index.");
                        return;
                    }
                }
                else
                {
                    if (transportLayers.Count > 0)
                    {
                        featureContainer = transportLayers[0];
                    }
                    else
                    {
                        Console.WriteLine("No transport layers were found.");
                        return;
                    }
                }
            }
            else if ((args.Length == 1 || args.Length == 2) && args[0] == "-i")
            {
                // get the desired interface as a feature container
                var interfaces = vmbSystem.GetInterfaces();
                if (args.Length == 2)
                {
                    if (int.TryParse(args[1], out int interfaceIndex))
                    {
                        if (interfaceIndex < interfaces.Count)
                        {
                            featureContainer = interfaces[interfaceIndex];
                        }
                        else
                        {
                            Console.WriteLine((interfaces.Count > 0 ? $"Only {interfaces.Count}" : "No") + " interfaces were found.");
                            return;
                        }
                    }
                    else
                    {
                        Console.WriteLine($"\"{args[1]}\" is not a valid index.");
                        return;
                    }
                }
                else
                {
                    if (interfaces.Count > 0)
                    {
                        featureContainer = interfaces[0];
                    }
                    else
                    {
                        Console.WriteLine("No interfaces were found.");
                        return;
                    }
                }
            }
            else if ((args.Length == 1 || args.Length == 2) && args[0] == "-c")
            {
                // Get the desired camera as a feature container.
                // The camera must be opened first.
                if(GetOpenCamera(args, out IOpenCamera? openCamera_))
                {
                    openCamera = openCamera_;
                }
               else
                {
                    return;
                }

                featureContainer = openCamera;
            }
            else if ((args.Length == 1 || args.Length == 2) && args[0] == "-l")
            {
                // Get the desired camera as a feature container.
                // The camera must be opened first.
                if (GetOpenCamera(args, out IOpenCamera? openCamera_))
                {
                    openCamera = openCamera_;
                }
                else
                {
                    return;
                }

                featureContainer = openCamera?.LocalDevice;
            }
            else if ((args.Length == 2 || args.Length == 3) && args[0] == "-s")
            {
                // Get the desired camera as a feature container.
                // The camera must be opened first.
                if (args.Length == 3)
                {
                    string[] argsSliced = { args[0], args[1] };
                    if (GetOpenCamera(argsSliced, out IOpenCamera? openCamera_))
                    {
                        openCamera = openCamera_;
                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    var cameras = vmbSystem.GetCameras();
                    if(cameras.Count >= 0)
                    {
                        var camera = cameras[0];
                        openCamera = camera.Open();
                    }
                }

                var streamIndexString = args.Length == 2 ? args[1] : args[2];

                if (uint.TryParse(streamIndexString, out uint streamIndex))
                {
                    try
                    {
                        stream = openCamera?.OpenStream(streamIndex);
                    }
                    catch (VmbNETException e)
                    {
                        Console.WriteLine($"Received VmbNET error: \"{e.Message}\" when trying to open stream.");
                        return;
                    }
                }
                else
                {
                    Console.WriteLine($"\"{streamIndexString}\" is not a valid index.");
                    return;
                }

                featureContainer = stream;
            }
            else
            {
                Console.WriteLine("Usage:");
                Console.WriteLine("  ListFeatures.exe -t [ transportLayerIndex ]");
                Console.WriteLine("  ListFeatures.exe -i [ interfaceIndex ]");
                Console.WriteLine("  ListFeatures.exe -c [ cameraId ]");
                Console.WriteLine("  ListFeatures.exe -l [ cameraId ]");
                Console.WriteLine("  ListFeatures.exe -s [ [ cameraId ] streamIndex ]");
                Console.WriteLine("  ListFeatures.exe -h");
                Console.WriteLine("Parameters:");
                Console.WriteLine("  -t                     List features of a transport layer (the first one if no index is specified)");
                Console.WriteLine("  transportLayerIndex    The index of the transport layer as it appears in the transport layer list");
                Console.WriteLine("  -i                     List features of an interface (the first one if no index is specified)");
                Console.WriteLine("  interfaceIndex         The index of the interface as it appears in the interface list");
                Console.WriteLine("  -c                     List features of a camera (the first one if no ID is specified)");
                Console.WriteLine("  cameraId               The camera ID");
                Console.WriteLine("  -l                     List features of a camera's local device (the first camera if no camera ID is specified)");
                Console.WriteLine("  -s                     List features of a camera's stream (the first camera / stream if no camera ID / stream index is specified)");
                Console.WriteLine("  streamIndex            The stream's index\n");

                return;
            }

            if (featureContainer == null)
            {
                Console.WriteLine("Features could not be obtained.");
                return;
            }

            // print the properties of each feature

            foreach (var feature in featureContainer.Features)
            {
                Console.WriteLine($"Feature name: {feature.Name}");
                Console.WriteLine($"Display name: {feature.DisplayName}");
                Console.WriteLine($"Tooltip: {feature.Tooltip}");
                Console.WriteLine($"Description: {feature.Description}");
                Console.WriteLine($"SFNC namespace: {feature.SfncNamespace}");

                try
                {
                    // print the allowed values of value features, if available
                    IFeature.TypeValue type = feature.Type;
                    switch (type)
                    {
                        case IFeature.TypeValue.Int:
                        case IFeature.TypeValue.Float:
                            Console.WriteLine($"Range: [{feature.Minimum}..{feature.Maximum}]");
                            break;
                        case IFeature.TypeValue.Enum:
                            IEnumerable<string> validNames = feature.EnumEntriesByName.Keys;
                            Console.WriteLine($"Valid entries: [{string.Join(",", validNames)}]");
                            break;
                        case IFeature.TypeValue.Raw:
                            Console.WriteLine($"Maximum size (bytes): {feature.MaximumSize}");
                            break;
                        default:
                            break;
                    }
                }
                catch (VmbException<FeatureError> e) when (e.ErrorCode == VmbErrorCode.NotAvailable)
                {
                    Console.WriteLine("Valid values: <not available>");
                }
                catch (VmbNETException e)
                {
                    Console.WriteLine($"While getting the valid values, the following VmbNET exception was thrown: {e.Message}");
                    return;
                }
                catch (Exception e)
                {
                    Console.WriteLine($"While getting the valid values, the following exception was thrown: {e.Message}");
                    return;
                }

                try
                {
                    // print the value of value features, if available
                    IFeature.TypeValue type = feature.Type;
                    switch (type)
                    {
                        case IFeature.TypeValue.Int:
                        case IFeature.TypeValue.Float:
                        case IFeature.TypeValue.Bool:
                        case IFeature.TypeValue.String:
                        case IFeature.TypeValue.Enum:
                        case IFeature.TypeValue.Raw:
                            Console.WriteLine($"Value: {feature.Value}");
                            break;
                        default:
                            break;
                    }
                }
                catch (VmbException<FeatureError> e) when (e.ErrorCode == VmbErrorCode.NotAvailable || e.ErrorCode == VmbErrorCode.InvalidAccess)
                {
                    Console.WriteLine($"Value: " + ((e.ErrorCode == VmbErrorCode.NotAvailable) ? "<not available>" : "<not accessible>"));
                }
                catch (VmbNETException e)
                {
                    Console.WriteLine($"While getting the value, the following VmbNET exception was thrown: {e.Message}");
                    return;
                }
                catch (Exception e)
                {
                    Console.WriteLine($"While getting the value, the following exception was thrown: {e.Message}");
                    return;
                }

                Console.WriteLine();
            }

            stream?.Dispose();
            openCamera?.Dispose();
        }

        private static bool GetOpenCamera(string[] args, out IOpenCamera? openCamera)
        {
            using var vmbSystem = IVmbSystem.Startup();

            ICamera? camera = null;
            openCamera = null;
            if (args.Length == 2)
            {
                try
                {
                    // get camera with user-supplied ID
                    camera = vmbSystem.GetCameraByID(args[1]);
                }
                catch (VmbNETException e)
                {
                    Console.WriteLine($"Received VmbNET error: \"{e.Message}\" when trying to get camera.");
                    return false;
                }
            }
            else
            {
                var cameras = vmbSystem.GetCameras();
                if (cameras.Count > 0)
                {
                    camera = cameras[0];
                }
                else
                {
                    Console.WriteLine("No cameras were found.");
                    return false;
                }
            }

            try
            {
                openCamera = camera?.Open();
            }
            catch (VmbNETException e)
            {
                Console.WriteLine($"Received VmbNET error: \"{e.Message}\" when trying to open camera.");
                return false;
            }

            return true;
        }
    }
}
