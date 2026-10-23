# Main ReadMe

## Setting upDriverstation
In order to properly setup your Driverstation, you must actually find the driverstation. (Challenge #1)
Please find the pink computer case within Narnia within the programming cart in the top drawer. 
This pink case will have the Driverstation laptop and charger found in the main and side sleeves.
Occasionally you may find an orange and/or blue usb-b cable which may also prove useful for your setup. 

### The Orange Cable
Allows for joystick communication to the robot when plugged in (by default) the left side of the driverstation laptop into the USB-A port.
We do advise for the use of an Xbox controller in this current codebase therefore please use an Xbox controller for this year 2026.

### The Blue USB-B Cable
The Blue cable you may find within the Driverstation pink case in the side sleeve or in the second drawer of the programming cart allows for direct
connection to your robot thus bypassing the need for a radio in the event that said radio isn't working. To use this cable properly, please plug in
the USB-A end of the cable into the right side of the driverstation laptop (default) then the other usb-b end into the robo Rio to establish direct comms.

### Configuring Driverstation Itself
Upon opening the Driverstation laptop, please use the credential password given to you by your captain to sign in. 
In the case that you are in programming please follow the instructions below : {

Open FRC Wpilib VsCode editor which will allow you to make changes to your local env and actively change/deploy new code to the robot
Everytime you make changes in that code editor, be sure to Ctrl+Shift+P then hit deploy to push your code onto the robo Rio. Don't worry
it will stop you from pushing code that can't compile because the deploy function builds before then so don't waste your time trying to run two
separate commands. After deploying, you will now have registered your changes onto the robot. Now ensure all comms, robot code, and joysticks work.


In the case that you aren't in programming or just want the robot to function please follow these instructions:
upon signing in, please open the "Driverstation" application

In this application, ensure that all 3 red bars have turned green, if not keep reading.
If you don't have comms, configure the blue cable for a wired connection to the robot; for wireless comms please turn on the robot normally and 
connect to its network. Don't wait on the wifi UI of windows to tell you that you are connected, wait on the driverstation application's
red bar to turn green.

If the joysticks bar is not green, please ensure the configuration in driverstation supports the port that you are using for joysticks, by default it 
uses the left side of the driverstations USB-A port.

Now hit enable in driverstation and play with the joystick. Disable whenever someones hands are close to the robot using the enter key.


## Code maintainers....






