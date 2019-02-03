# Firmware Update Process for TNC3 on Microsoft Windows 10

This document outlines the firmware update process for the Mobilinkd TNC3
for Microsoft Window, focusing on Windows 10.  The update process should
be similar enough for all Windows versions back to Windows 7.

Please note that the firmware update software is cross-platform, so the
actual firmware update process is the same for Windows, Linux and MacOS.

The first part of this document outlines the download and installation
process of the firmware programming software and the required device
drivers.

The second part of this document outlines the firmware upgraded process
using the STM32CubeProgrammer software.

The third part of this document outlines troubleshooting steps.

# Installing the Firmware Programmer and Device Drivers

The Mobilinkd TNC3 uses the STM32CubeProgrammer software from
STMicroelectronics to perform firmware updates.  This is a comprehensive
and cross-platform firmware update tool.

You will be performing the following steps:

 - Downloading the software
 - Unzipping the software
 - Running the installer
 - Installing Java (if not already installed)
 - Installing the firmware update software
 - Installing the device drivers
 
## Download the Software

The firmware update tool is available from here:
https://s3.amazonaws.com/mobilinkd/en.stm32cubeprog-1.4.0.zip

### Download the File

[!Download file](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/download.png])

### Open the File

[!ZIP file contents](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/zip_contents.png])

### Extract the Files

Double-click the executable file.  Windows will ask you to extract all
files.

[!Extract all files](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/extract_all.png])

Select "Extract all".

[!Extract to](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/extract_to.png])

Make sure that "Show extracted files when complete" is selected.  Once
complete, you will be presented with a new file browser showing the
extracted files.

[!Extracted files](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/extractes.png])

### Install Java

Double-click the SetupSTM32CubeProgrammer-1.4.0.exe to run the installer.

You may be presented with a warning screen.

[!Install warning](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_warning.png])

Select "Yes" to continue.

At this point you may be presented with a message saying that Jave is
required.  This section will guide you through the installation
process.  If this does not appear for you, please skip to the next
section.

[!Requires Java](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/requires_java.png])

Select "OK" to continue.  You will be presented with a browser window
to download the Java Runtime Environment.

[!Download Java](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/java_download.png])

Windows may ask you to approve this software installation.

[!Download Java](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/java_download.png])

Select "Yes" to continue.  You will see the Java installation window appear.

[!Installing Java](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/installing_java.png])

Once complete, you will need to go back to the File Browser showing the 
STM32CubeProgrammer extracted files.

[!Extracted files](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/extractes.png])

Double-click the SetupSTM32CubeProgrammer-1.4.0.exe to run the installer.

You may be presented with a warning screen.

[!Install warning](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_warning.png])

Select "Yes" to continue.

### Install STM32CubeProgrammer

Follow the installation screens, choosing the defaults.

[!Install programmer step 1](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_1.png])

Select "Next".

[!Install programmer step 2](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_2.png])

Skip the information... select "Next".

[!Install programmer step 3](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_3.png])

Accept the terms of the software license agreement and press "Next".

[!Install programmer step 4](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_4.png])

Unless you have a very good reason to change the location of the installed
software, use the default path and select "Next.

You may be presented with a dialog informing you the location will be created.
Select "OK".

[!Install programmer step 4.1](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_4_1.png])

The next screen asks which components should be installed.  You should just
select the default components as these are all that is required for the
firmware update process on the Mobilinkd TNC3.

[!Install programmer step 5](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_5.png])

The installation process will begin...

[!Install programmer step 6](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_6.png])

After the software is installed, the installer will install the required
device drivers.

[!Install programmer step 6.1](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_6_install_driver.png])

Select "Next" to proceed with the installation.  You may be asked to confirm
the installation of the drivers.

[!Install programmer step 6.2](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_driver_confirm.png])

Select "Install" to proceed.

[!Install programmer step 6.3](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_6_drivers_now_installing.png])

[!Install programmer step 6.4](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_6_installed_drivers.png])

Select "Finish" to proceed.

[!Install programmer step 6.5](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_6_finished.png])

When complete, select "Finish" to proceed. The installer will finalize the
installation.  Accept the defaults for the next two screens.

[!Install programmer step 7](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_7.png])

[!Install programmer step 8](https://s3.amazonaws.com/mobilinkd/TNC3/FirmwareUpdateGuide/install_programmer_8.png])

The firmware update software 

























