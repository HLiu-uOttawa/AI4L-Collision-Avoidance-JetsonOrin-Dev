
# VmbNET

The .NET API of the [Vimba X SDK](https://www.alliedvision.com).

Vimba X is a fully GenICam compliant SDK and the successor of Vimba. VmbNET is the .NET API that is
provided by this SDK. It provides access to the full functionality of Vimba X from .NET applications.

VmbNET can be used on Windows64, Linux64 and Linux ARM.

# Installation

To use VmbNET, a .NET runtime providing .NET Standard 2.0 is required.
These are .NET Framework 4.6.2 or above and .NET Core 2.0 or above.
A ready-to-install nuget package
for VmbNET (whose file name ends with `.nupkg`) that conforms to .NET Standard 2.0
can be found as part of the Vimba X installation.
This nuget package can be installed in an application in various ways.

## 1. Installation with Visual Studio 2019 (Windows64 only)

After opening your application's project file with Visual Studio 2019,
1. Open the NuGet Package Manager for the project by right-clicking it in Solution Explorer
and selecting `Manage NuGet Packages...`.
2. Click the cog wheel after `Package source:` to open the 
package sources, and add the full path to the directory containing the `.nupkg` file
by clicking the `+` icon and entering this path into the `Source:` field.
The `Name:` field can be whatever you like, but you will need it just now.
Close this window to return to the NuGet Package Manager.
3. Open the drop-down menu after `Package source:` and either select `All` or the `Name:` field value you used
when adding the package source for VmbNET.
4. Select the "Browse" tab and search for `VmbNET`.
Clik the VmbNET NuGet package that is displayed.
5. In the pane that appears, click `Install` to install the version shown.

## 2. Installation from the command line

* Add the full path to the directory containing the `.nupkg` file by executing

    ```C#
    dotnet nuget add source <path-to-directory-containing-VmbNET-NuGet-package> -n <user-chosen-name-for-this-package-source>
    ```

* Install VmbNET to your package by changing directory to the location of your project file and executing

    ```C#
    dotnet add package VmbNET
    ```

* Then build your application by changing directory to the location of your project file and executing

    ```C#
    dotnet build -c Release .
    ```

# Usage

Below is a minimal example demonstrating how to acquire frames for 2 seconds from the stream of the first camera found by VmbNET. It
highlights the general usage of VmbNET. More complete code examples can be found in the `Examples`
directory.

```C#
   using VmbNET;

   class Program
   {
       static void Main()
	   {
           using var vmb = IVmbSystem.Startup(); // API startup (loads transport layers)

           var cam = vmb.GetCameras()[0]; // Get the first available camera

           using var openCam = cam.Open(); // Open the camera 

           openCam.Features.ExposureTime = 5000.0; // Set the exposure time value 

           // Register an event handler for incoming frames
           openCam.FrameReceived += (s,e) => 
           {
               using var frame = e.Frame;
               Console.WriteLine($"Frame Received! ID={frame.Id}");
           }; // IDisposable: Frame is automatically requeued

           // Convenience function to start acquisition
           using var acquisition = openCam.StartFrameAcquisition(); 

           Thread.Sleep(2000);	  
		   
       } // IDisposable: Stops acquisition, closes camera, shuts down Vimba X
   }
```
