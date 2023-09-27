## Instruction for using choreography function of Choreonoid

The following instruction is based on a video lecture by Shin-ichiro Nakaoka.

### Preparation

Build Chorenoid from source.
Version 2.0.0 or later is required.
In CMake, turn on the following options:
- BUILD_POSE_SEQ_PLUGIN=ON
- BUILD_BALANCER_PLUGIN=ON
- BUILD_MOCAP_PLUGIN=ON
- BUILD_HRP4C_HANDLER=ON
- BUILD_MEDIA_PLUGIN=ON

The command-line should look like:
```
cmake . -DBUILD_POSE_SEQ_PLUGIN=ON -DBUILD_BALANCER_PLUGIN=ON -DBUILD_MOCAP_PLUGIN=ON -DBUILD_HRP4C_HANDLER=ON -DBUILD_MEDIA_PLUGIN=ON 
```

The BUILD_HRP4C_HANDLER option is needed if you want to use custom IK for the HRP-4C model.

### Step by step instruction of choreography

- Start up Choreonoid.
<img src="fig/dance/screen1.png" width="500"/>

Check the message view to see if required plug-ins are properly loaded.

- Create a world

Add a world item (File -> New -> World).

<img src="fig/dance/screen2.png" width="500"/>

- Load robot model

Select World in the item view, and select File -> Load -> Body.
In the load dialog, navigate to the source directory of choreonoid and select choreonoid/share/model/HRP4C/HRP4C.body.

<img src="fig/dance/screen3.png" width="500"/>

In the item view, make sure that HRP-4C is listed below World as its child item.

You can switch between the "edit mode" and the "view mode" by right-clicking on the scene view.
In the "edit mode", you can select a link of the robot, and change its pose by dragging the 3D cursor.

<img src="fig/dance/screen4.png" width="500"/>

- Load floor

Select World in the item view, and select File -> Load -> Body.
In the load dialog, navigate to the source directory of choreonoid and select choreonoid/share/model/misc/floor.body.

<img src="fig/dance/screen5.png" width="500"/>

To avoid selecting and moving the floor accidentally, it is recommended to fix the floor.
To do so, switch to the edit mode, right-click the floor and select "Lock location".

Floor is necessary for computing physical interaction between the robot and the floor.
However, while editing the motion of the robot, it is often convenient if the floor is not shown.
To hide the floor, uncheck the checkbox of Floor in the item view.

- Add PoseSeq item

Select HRP-4C in the item view, and select File->New->PoseSeq.
Make sure that the created PoseSeq item is shown below HRP-4C in the item view.

Select View->Show View->Pose Roll and show the Pose Roll view.
This view is used for choreography.
Select PoseSeq in the item view.
Now, the name of the PoseSeq item ("PoseSeq" in this case) will be shown in the bottom left of the pose roll view.

Up to this point is basic preparation for pose editing.
Save the project by selecting File->Save Project As.
The project and pose sequence will be saved to separate files (.cnoid file and .pseq file respectively).

<img src="fig/dance/screen6.png" width="500"/>

- Adjust window layout for pose editing

You can grab the tab of each view and drag it to change the layout of views according to your needs.
In pose editing, Pose Roll view and Joint Displacement view are often used, so these views
 would better be shown bigger.
Moreover in the Joint Displacement view, you can show joint names in addition to joint IDs by
 right-clicking its tab and checking "Joint name".
Shown below is an example layout.

<img src="fig/dance/screen8.png" width="500"/>


- Edit initial pose

To start simulation in a stable manner, the feet of the robot should be just touching the floor.

First, enable collision cheking.
To do so, select World item, and in the property view just below the item view,
 switch "Collision detection" property to True.

Next, enable visualization of collision.
To do so, check the check box of World item.
Moreover, push the button shown below in the tool bar.

<img src="fig/dance/screen7.png" width="200"/>

Now, reset the pose of the robot.
There are some buttons for reseting the pose of the robot.

<img src="fig/dance/screen9.png" width="200"/>

<img src="fig/dance/screen10.png" width="200"/>

The initial pose is the pose where all joint angles are zero,
 while the standard pose is a more useful starting configuration with knees bent slightly.
Here, reset to the standard pose.

Now, we would like to adjust the height of the robot so that its feet are just touching the floor.
For this purpose, we need use FK instead of IK.
To do this, turn on the "Enable forward kinematics" button and 
 turn off "Use preset kinematics" and "Enable inverse kinematics" buttons, as shown below.

<img src="fig/dance/screen11.png" width="200"/>

Then you should be able to drag the WAIST link (highlighted yellow in the figure below)
 to move the robot without chaninging its posture.

*If this doesn't work, try switching to view mode and back to edit mode.

<img src="fig/dance/screen12.png" width="500"/>
 
As you move the robot gradually down, eventually the feet will start to penetrate into the floor,
 and green lines indicating collision will appear, and grow longer as the penetration gets deeper.
Stop lowering the robot when these green lines have just started to show up.

Alternatively, you can change the position of the selected in in the Link Position view.
This way is more useful for precise positioning.

In the case of HRP-4C, the ideal z coordinate of the base link in the standard posture is 0.719m.

- Insert initial pose to pose sequence

Make sure that the robot is in a proper initial pose as described above.
Now, left-drag in the Pose Roll view and bring the seek bar (black vertical line) to the left most (time 0.0) and click "Insert".
Then the currently edited pose will be inserted as the first key pose of the pose sequence.

<img src="fig/dance/screen13.png" width="300"/>

- Insert other key poses

You can create new key poses in the same manner;
 edit a new pose in the Scene view, and then in the Pose Roll view, position the seek bar, and push Insert.
After inserting some key poses, it will look like this:

<img src="fig/dance/screen14.png" width="500"/>


