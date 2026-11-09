#Main Readme

## Driver Station Setup Guide

This guide covers locating the physical hardware, connecting to the robot, and deploying code for the 2026 season.

1. Locating the Hardware

To set up the Driver Station, locate the following items from the Programming Cart:
• Top Drawer: Find the pink computer case (labeled "Narnia"). This contains the Driver Station laptop and its power charger.
• Side Sleeves / Second Drawer: Locate the orange and blue connection cables.

2. Cable Configuration


Orange USB Cable (Joystick / Controller)

• Purpose: Connects your controller to the laptop.
• Setup: Plug the cable into the left USB-A port of the laptop.
• Note: For the current 2026 codebase, an Xbox Controller is strictly required.

Blue USB-B Cable (Direct Tether)

• Purpose: Bypasses the wireless radio to establish direct communication with the robot if the radio is malfunctioning.
• Setup: Plug the USB-A end into the right side of the laptop, and the USB-B end directly into the roboRIO.

3. Driver Station & Software Configuration


Signing In

1. Open the Driver Station laptop.
2. Sign in using the team credentials provided by your Team Captain.

Programming & Deploying Code

If you are working on the programming team, follow these steps to deploy changes:
1. Open VS Code: Launch the FRC WPILib VS Code editor to access your local environment.
2. Deploy Code: After making changes, press Ctrl + Shift + P and select WPILib: Deploy Robot Code.
	• Note: You do not need to build the code separately. The deploy command automatically builds the project first and will block the deployment if compilation fails.
3. Verify Connection: Once deployment succeeds, check the Driver Station interface to ensure Communications, Robot Code, and Joysticks all show green status indicators.
