***How to use this software***

Download & install Arduino IDE v1.0 from http://arduino.cc/en/main/software
Copy the CONTENTS of the PID Libraries folder to your Arduino libraries folder.

***To use the Arduino Autotune backend and Processing backend
Download and install the latest version of Processing (I used V1.5.1)
Download and install the ControlP5 library https://code.google.com/p/controlp5/downloads/list
Make sure you get the version of ControlP5 that is compatible with your version of Processing!

Now open Arduino IDE and open the AutotuneMAX31855.ino file
Click on the Verify button (looks like a "tick")
After it compiles, click on the Upload button (looks like a "->"), in the lower left corner when "Compiling" changes to "Uploading", tap the Reset buttin on the controller.
If you get an "avrdude" error, it indicates the Reset button press timing was incorrect. The Reset button press must be done as soon as soon as "Compiling" changes to "Uploading".

Open Processing and open the PID_FrontEnd_v03.pde file
Search (CTRL F) the file for "//EDIT" (without the quotations), these are the parts that the end user can/should change.
Click the Run button (looks like a "Play" button)
This will open a new window displaying the temperature, output and toggle buttons. If the temperature and setpoint traces fail to appear, close the window and click the Runn button again.
The value below or inside the toggle buttons is what the button has been set to, the value to the RHS of the button is what the Arduino backend is set to. 
To change a toggle setting, click on it, enter a value (for P,I,D, etc), click the SEND_TO_ARDUINO button.

To Autotune the controller, put the controller into Manual Mode, then starting from a very low Output value (~50), slowly increase until the system has reached equilibrium with
the Setpoint. Toggle the TOGGLE_TUNING button to ON, check that TUNING_AM is set to MANUAL, then click SEND_TO_ARDUINO.
The Autotune uses the Set Response method to obtain values.
When the TOGGLE_TUNING state to the RHS of the TOGGLE_TUNING button changes from ON to OFF, tuning is complete. The values to the RHS of each P, I and D are the Autotuned
values.

***To use the PID Menu based Controller:
Open the Arduino IDE
Open the Arduino_PID_for_Espresso_V2_0.ino file
Enter the values obtained from the Autotune process into lines 120-122.
Click on the Verify button (looks like a "tick")
After it compiles, click on the Upload button (looks like a "->"), in the lower left corner when "Compiling" changes to "Uploading", tap the Reset buttin on the controller.
If you get an "avrdude" error, it indicates the Reset button press timing was incorrect. The Reset button press must be done as soon as soon as "Compiling" changes to "Uploading".

Once the code has uploaded, the display will read Arduino PID for Espresso, then change to the measured temp and "Menu".

To activate changes, button presses

Press ENTER to enter the Menu. The first item is Brew Espresso, press ENTER to set the Setpoint to the preselected Brew Temperature. The menu will return to root and the controller will move to the Brew Temperature Setpoint.

Press ENTER to enter the Menu. Move right to Steam Milk, press ENTER to set the Setpoint to the preselected Steaming Temperature. The menu will return to root.

Press ENTER to enter the Menu, move right to Brew Temp. the currect setting of Brew Temp will be displayed on the LCD. Press ENTER to enable changing of the Setpoint. The display will read Set Brew Point.
Use the L and R buttons to decrease and increase the Brewing Setpoint. When you have the Brew Setpoint you want, press ENTER to set it. The menu will return to root and the controller will move to the Setpoint. 
Now when you select Brew Espresso, the setpoint will move to the setpoint you have selected.

Press ENTER to enter the Menu, move right to Steam Temp, the currect setting of Steam Temp will be displayed on the LCD. Press ENTER to enable changing of the Steam Temp setting. The display will read Set Steam Point.
Use the L and R buttons to decrease and increase the Steaming Setpoint. When you have the Steaming Setpoint you want, press ENTER to set it. The menu will return to root and the controller will move to the Setpoint.
Now when you select Steam Milk, the setpoint will move to the setpoint you have selected.

Press ENTER to enter the Menu, move right to Offset. The current temp offset between the boiler and the grouphead will be displayed on the LCD. Press ENTER to enable changing of the Offset setting. the display will read Set Offset.
Us the L and R button to decrease and increase the Offset between the boiler and grouphead. When you have the offset you want, press ENTER. The menu will return to root and the controller will move to the Setpoint + Offset.

Currently in the Menu, there is a menu item named Scale, this is for a future update, where the user will be able to select whether to diplay temps in Celcius or Fahrenheit. 

If you are in the Menu and wish to exit without changing any settings, press the ESC button.