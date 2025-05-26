
VmbNET Examples
===============

Programming examples using the Vimba X .NET API "VmbNET".

Beta Disclaimer
---------------
THE SOFTWARE IS PRELIMINARY AND STILL IN TESTING AND VERIFICATION PHASE AND IS
PROVIDED ON AN “AS IS” AND “AS AVAILABLE” BASIS AND IS BELIEVED TO CONTAIN DEFECTS.
A PRIMARY PURPOSE OF THIS EARLY ACCESS IS TO OBTAIN FEEDBACK ON PERFORMANCE AND
THE IDENTIFICATION OF DEFECT SOFTWARE, HARDWARE AND DOCUMENTATION.

Compile instructions & tool chain requirements
----------------------------------------------

### Requirements

* .NET runtime version with which .NET Standard 2.0 is compatible (>= .NET Framework 4.6.1 or >= .NET Core 2)
* To use the Vimba X transport layers: Vimba X
* Editors:
  * Windows: Visual Studio >= 2019
  * Linux: Visual Studio Code
* Platforms: Windows 64-bit, Linux 64-bit or Linux ARM

### Compilation

Options such as the .NET runtime version or C# language version can be set in the *.csproj of the example that you want to use.

The examples can be built using Visual Studio (Windows only) or Visual Studio Code in the usual way.
Alternatively, use the `dotnet` command:
Open a command prompt in the directory of the example that you want to use and run

```console
dotnet build -c <configuration> .
```

where `<configuration>` is one of `Release` or `Debug`.
For example, to build a release version, run

```console
dotnet build -c Release
```
