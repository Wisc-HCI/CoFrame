# cobots_augmented_reality_ui

Developer notes for the collaborative manufacturing education system. This document captures the tool setup, design patterns, and deployment process for the application.

## Setup
For this project, you will need to install the Hololens tool-chain for Unity which can be done by following this [tutorial](https://docs.microsoft.com/en-us/windows/mixed-reality/install-the-tools). Note, to use the Hololens emulator you will need to be running at least Windows 10 pro as the emulator uses Hyper-V. Version of Unity used for this project is 2018.4.10f1 which will install Visual Studios 2017 but for deployment to Hololens we need to use Visual Studios 2019 (install seperatly or also install Unity 2019 within the Unity Hub).

Next our project makes use of the following external Unity Assets. These are already configured within the UI project but if a new project is started then these must be installed.

- Microsoft's [Mixed Reality Toolkit (MRTK)](https://microsoft.github.io/MixedRealityToolkit-Unity/README.html)
  - [Github Repository](https://github.com/microsoft/MixedRealityToolkit-Unity)
- [Siemens/ros-sharp](https://github.com/siemens/ros-sharp)
- [qian256/HoloLensARToolKit](https://github.com/qian256/HoloLensARToolKit)

## Design Pattern and Methodology
Our AR application uses an MVC pattern.

## Deployment

TODO
