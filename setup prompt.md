# Adding a new arm

Here's the steps for connecting a new arm. First we want to go through the steps where Users are required for input, and then we want to go and connect everything that we can do manually. 

Highlevel, please look at the repo to udnerstand how it's setup followign the YAM and SO101 exampels for knowing what needs to be implemented. For arms with Canables, we will need to add an additional C++ connector to let the arm run even if python stops.  We should follow 3 steps.

First. Get user necessary information. This should happen through discussion with the user, and by lookign at user provided info and repos. Second. We want to connect everything, first connecting the arm to the controller, and then wiring the local runner that integrates with models. Lastly, we want to first let the user verify that the controller works, and then to verify that they are able to send requests throguh the API via the local runner. 

Before we laucnh the user needs to provide (or guide us in finding):
1. **Calibration:** Provide the calibration file for each arm. - Specifying which arm is which, and whcih cameras are which
4. **Zero and home:** Agree on the zero and home poses to use.
5. **Operator account:** Specify which account(s) should have access — here, `andrew2` and `isaac`.
4. **Arm assignment (if bimanual):** Arm A is left; Arm B is right.
5. **Camera assignment (if multiple):** Swap the two wrist feeds.
6. **Mounting layout (if bimanual):** User confirmed the standard XLeRobot layout: arm bases 26.6 cm apart, at equal height and with the same orientation. Use the mounting transforms from [XLeRobot's combined URDF](https://github.com/Vector-Wangel/XLeRobot/blob/main/simulation/Maniskill/assets/xlerobot/xlerobot.urdf) for arm-to-arm collision checking.
6. Safety: Safety needs to be added. This should happen through discussion with the user or by looking at their harness to see if they already have such guiderails

The two repos that need to be customized for the user are  
1) https://github.com/andlyu/blupe-playground-robot-controller
   This repo connects with the arm, and adds safety guardrails to controll the phsycial hardware. Here we need to make sure taht all the sensors are connected, and passed thorugh to the API, and all the ports are connected.
3) https://github.com/andlyu/blupe-remote-yam\
       This repo connects with the model and allows us to run policies. Currently we need the arm configs for IK, Safety, and we need to update the prompt. 

Here's the other possible issues that can happen
1. Issues 1) The tunnel should eb directly from teh device connected to the arm
. 




