## TeamCode Module

//link to other readmes
//talk abt basic robot code

## Creating your own OpModes



* IMU:      Code used to get the angles from the internal gyroscope
* OpenCV:   Various pipelines and robot code to use vision to locate blocks
* Roadrunner: Odometry code that can run autonomous code very precisely
* Tensorflow:  Vision code using tensorflow, does not currently work
* Vuforia:	Vision code for location using the pictures on the walls



```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

