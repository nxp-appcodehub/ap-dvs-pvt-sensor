<p align="center">
	<img width="200" height="150" src="dvs_pvt_sensor/images/icon.jpg">
</p>

<h1 align="center">Dynamic Voltage Scaling Using PVT Sensor</h1>

This repository holds the [Dynamic Voltage Scaling using PVT Sensor App SW Pack](https://www.nxp.com/dvs-pvt-sensor) and depends on the MCUXpresso SDK overall delivery.

## Resources
* Purchase supported board.
    * [MIMXRT595-EVK](https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt595-evaluation-kit:MIMXRT595-EVK)
* Install [MCUXpresso IDE v11.6.0+](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).
* [Application Note AN13695](https://www.nxp.com/doc/AN13695) - Covers technical details of the software pack.
* [Lab Guide](https://github.com/NXPmicro/appswpacks-dvs-pvt-sensor/blob/mcux_release_github/dvs_pvt_sensor/app/evkmimxrt595/doc/evkmimxrt595_dvs_pvt_sensor_lab_guide.pdf) and [Video Walkthrough](https://www.nxp.com/pages/:TIP-APP-SW-PACK-DYNAMIC-VOLTAGE) - Walks you through downloading, importing, and running the software pack.

## Assemble the Application
You need to have both Git and [West](https://docs.zephyrproject.org/latest/develop/west/index.html) installed, then execute below commands to gather the whole APP-SW-PACKS/DVS-PVT-SENSOR delivery at revision ```${revision}``` and place it in a folder named ```appswpacks_dvs_pvt_sensor```. 
```
west init -m https://github.com/nxp-mcuxpresso/appswpacks-dvs-pvt-sensor --mr ${revision} appswpacks_dvs_pvt_sensor
cd appswpacks_dvs_pvt_sensor
west update
```
Replace ```${revision}``` with any SDK revision you wish to achieve. This can be ```mcux_release_github``` if you want the latest state, or any commit SHA.

## Build and Run the Application
To build and run the application please refer to the lab guide or check the steps in [Run a project using MCUXpresso IDE](https://github.com/NXPmicro/mcux-sdk/blob/main/docs/run_a_project_using_mcux.md).

## Application Overview
This software application pack demonstrates the use of the PVT sensor on i.MX RT500 to implement dynamic voltage scaling (DVS).

After initializing the necessary hardware, it launches the following tasks:
1. **Workload Task:** Runs Coremark for ~10 seconds, prints the results, and then delays for 5 seconds to allow the FreeRTOS idle task to enable deep sleep mode.
2. **PVT Task:** The PVT task is in charge of handling DVS in the application. The general flow of the task is shown below. \
![flowchart](dvs_pvt_sensor/images/flowchart.png)
## Other Reference Applications
For other rapid-development software bundles please visit the [Application Software Packs](https://www.nxp.com/appswpack) page.

For SDK examples please go to the [MCUXpresso SDK](https://github.com/NXPmicro/mcux-sdk/) and get the full delivery to be able to build and run examples that are based on other SDK components.