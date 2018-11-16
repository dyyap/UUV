# Sub Start Up Quick Guide


**Items needed for Quick Motor StartUp** 
  1. Blue Robotics Motor
  2. A bucket of water 
  3. Motor connector with to PWM and Power 
  4. Intel NUC
  5. PixHawk
  6. PowerSource
  7. UUV cameras
  8. Monitor, mouse, and keyboard,HDMI cable
  
**Software Required for Startup**

  QGroundControl
  
  c_uart_interface_exampleOPENCV folder
  
**Pixhawk Update**
  1. Connect the Pixhawk to another computer with QGroundContol
  2. Find the gears in the top right corner and go to firmware and follow instructions
  
## Start up

  Put the test motor in the bucket of water (it is lubricated with water and will fry if not submerged in water)
  
  Attach the HDMI cable from the monitor to the NUC as well as the mouse and keyboard.
  
  Attach Pixhawk to NUC.
  
  Open QGroundControl on the NUC and check to see if Pixhawk and the NUC are connected.
  
  Put in the PWM cable on the far right of the Pixhawk.
  
  Plug in the cameras on the two steel disks of the sub into the nuc (one should be a usb and the other should be a usb-c).
  
  Connect the black and red cables of the power source to the respective cables of the motor cables.
  
  Turn on the power source and make sure it is reading between 12-16V and 1A and then **PRESS THE ON/OFF BUTTON**.
  
  Open a terminal on the NUC and navigate to the c_uart_interface_exampleOPENCV folder
  
      cd C:Path\to>c_uart_interface_exampleOPENCV
  
  Type *make* to compile mavlink_control.cpp
  
  Run the executable
  
      sudo ./mavlinkcontrol
  
  
  
  
