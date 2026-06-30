# VnoidJudge Plugin

The VnoidJudge plugin is a Choreonoid plugin that performs the automatic judgement of the Humanoid Virtual Athletics Challenge (HVAC). It monitors the state of the robot during simulation and performs the following judgement and display functions:

- Checking the total mass (it must be above the lower limit)
- Monitoring the joint power (POWER) and detecting the limit violation
- Monitoring the bounding box (BB) size of the robot and detecting the limit violation
- Judging the clearance of each section of the field and recording the clear times
- Displaying the judgement status on the scene view as a HUD and visualizing the section regions
- Saving the judgement result to a text file

## How to build

This plugin is built as a part of the vnoid repository, integrated into the build of Choreonoid.

1. Clone the vnoid repository into the `ext` directory of the Choreonoid source directory
2. Turn on `VNOID_BUILD_CNOID` and `BUILD_VNOID_JUDGE_PLUGIN` in the CMake build settings of Choreonoid
3. Build Choreonoid

Note that this plugin must be combined with a version of Choreonoid at commit `cb45151ae` (2026-07-05) or later.

## Components

| Component | Role |
|---|---|
| VnoidJudgeTargetItem | An item that specifies the robot to be judged. When it is placed as a child of the robot's BodyItem, it attaches the judgement device (VnoidJudgeTargetDevice) to the robot. This item also holds the limit settings of the judgement (the parameters of the competition rules). |
| VnoidJudgeItem | The "judge" item that performs the judgement. It is placed as a child of a simulator item. It is responsible for the judgement processing during simulation and the display of the HUD and the section BBs. |
| VnoidJudgeTargetDevice | An internal device that holds the judgement state (power, BB, elapsed time, and section states). It is automatically attached by VnoidJudgeTargetItem, so the user does not need to handle it directly. The state is recorded frame by frame through the device state recording mechanism, and the judgement display is also reproduced in playback. |

## Setup

1. Prepare a usual simulation project in which the robot's BodyItem and the field's BodyItem are placed under a world item
2. Place a VnoidJudgeTargetItem as a child of the robot's BodyItem (with the BodyItem selected, it can be created from the main menu "File" - "New" - "VnoidJudgeTarget")
3. Set the limit values with the properties of the VnoidJudgeTargetItem if necessary
4. Place a VnoidJudgeItem as a child of the simulator item (with the simulator item selected, "File" - "New" - "VnoidJudge")
5. **Turn on the check** of the VnoidJudgeItem in the item tree

**Note: The judgement displays on the scene view such as the HUD and the section bounding boxes are not displayed at all unless the check of the VnoidJudgeItem is turned on**. If the judgement result does not appear on the screen after running the simulation, first make sure that the VnoidJudgeItem is checked.

Conversely, when the HUD display is in the way, you can intentionally hide it by turning off the check. The check only switches the display; even if the check is off, the judgement processing itself is performed properly, and the result recording, the message output, and the saving of the judgement result file all work as usual.

An example of the item tree structure:

```
World
 ├─ SR1 (BodyItem)
 │   ├─ SimpleController (any controller item)
 │   └─ JudgeTarget (VnoidJudgeTargetItem)
 ├─ HVAC2023_athletics_field (BodyItem)
 └─ AISTSimulator (any simulator item)
     └─ VnoidJudge (VnoidJudgeItem)
```

The judgement starts when the simulation starts, and the result of the total mass check is shown in the message view at the beginning. When a section is cleared or a limit is exceeded, a notification is shown in the message view each time.

## Section definitions of the field

The sections to be judged for clearance are described as `hvac_field_sections` at the **top level** of the field's body file (the same level as `links:` etc.). The plugin does not assume any particular number or structure of the sections; it only refers to this definition.

```yaml
hvac_field_sections:
  - name: "Section1-Easy"
    pre_stage_bb:
      - [ -0.75, -2.0, -0.2 ]
      - [ 0.75, 2.2, 4.0 ]
    section_bb:
      - [ 0.75, 0.0, -0.2 ]
      - [ 5.25, 2.2, 4.0 ]
    post_stage_bb:
      - [ 5.25, -2.0, -0.2 ]
      - [ 6.75, 2.2, 4.0 ]
  - name: "Section1-Hard"
    ...
  - new_row
  - name: "Section2-Easy"
    ...
```

- `name`: The section name. It is displayed as is on the HUD labels and in the result file
- `pre_stage_bb` / `section_bb` / `post_stage_bb`: The axis-aligned bounding boxes of the pre stage, the section, and the post stage in the world coordinates. Each box is specified by the two points of the minimum and maximum corners
- `new_row`: When this is inserted, the following sections are arranged in the next row of the HUD display (it does not affect the judgement). If `new_row` is not used, the sections are automatically wrapped into rows of three columns
- The maximum number of sections is eight. The order of the sections in the list is used as the internal management order

When creating and editing the section bounding boxes, it is convenient to build the project structure in Choreonoid in advance and enable the bounding box display of the VnoidJudgeItem (the "Show section bounding boxes" property). Every time you edit the body file with a text editor, reload the field's body item ("Reload" in the context menu or Ctrl + R), and the edited bounding boxes are immediately visualized on the scene view, so you can edit them efficiently while checking the coordinate values. In addition, since the names of the sections are listed in the section display of the HUD, you can also check whether the section definitions themselves are written and loaded correctly.

## Judgement details

### Mass check

At the beginning of the simulation, the plugin checks whether the total mass of all the links of the robot is above the lower limit (the "Mass lower limit" property), and shows the result in the message view.

### POWER (joint power)

The plugin monitors |joint velocity x joint torque| of each joint. To avoid spikes of the instantaneous values, the moving average over a time window (the "Power average window [s]" property, 0.01 seconds by default) is taken, and the maximum value over all the joints is used as the POWER. When the maximum POWER exceeds the limit (the "Power limit [W]" property), a notification with the name of the exceeding joint is shown in the message view, and the POWER display of the HUD turns red.

**The joint torques must be output from the simulation for the POWER to be calculated correctly.** With AISTSimulatorItem, PhysXSimulatorItem, and MuJoCoSimulatorItem, set the "Drive effort output" property of the simulator item to true. Note that the plugin has not been tested with other simulator items.

### Bounding box (BB)

The BB of the robot is an axis-aligned bounding box in the root link's coordinate frame, computed from the point sets obtained by reducing the mesh vertices of each link with the convex hull. The box does not inflate when the whole robot rotates, and fits the robot shape tightly. When the maximum size of each axis exceeds the limit (the "BBox upper limit X/Y/Z" properties), a notification is shown in the message view and the HUD display turns red.

**If you want to display the BB of the robot on the scene view, set the "Show bounding box" property of the robot's BodyItem to true** (this is a function of the Choreonoid framework). The BB used for this display has the same definition as the one used for the judgement.

### Section clear judgement

Each section is cleared by the following steps:

1. The BB of the robot (in the world coordinates) is fully contained in the BB of the pre stage
2. The robot's BB is fully contained in the BB of the section (the section attempt starts)
3. The robot's BB is fully contained in the BB of the post stage and stays there for one second -> cleared. The clear time is recorded

The judgement is maintained while the robot is crossing the boundary of the adjacent regions (while it intersects two or more regions). When the robot intersects only one region, it is required to be fully contained in that region, and going out of the course resets the attempt. When the robot fully returns from the section to the pre stage, the state goes back to the pre stage standby (the cleared sections are never reset).

## HUD display

When the check of the VnoidJudgeItem is turned on, the following are displayed on the scene view:

- Top left: the current and maximum values of the POWER with the exceeding joint, and the size of each axis of the robot's BB (the current and maximum values)
- Top right: the elapsed time (TIME)
- Bottom center: the status of each section

The section status display is as follows (the colors are synchronized with the section BB visualization):

| Display | Meaning | Color |
|---|---|---|
| `Section1-Easy:` | Standby | Normal color |
| `Section1-Easy: PRE STAGE` | Staying in the pre stage | Light blue |
| `Section1-Easy: IN SECTION` | Attempting the section | Yellow |
| `Section1-Easy: POST STAGE` | Staying in the post stage (before one second passes) | Orange |
| `Section1-Easy: CLEAR 12.34s` | Cleared (with the time) | Green |

The font size, the text color, and the panel transparency can be changed with the properties of the VnoidJudgeItem.

Since the judgement states are recorded frame by frame as the state of the judgement device, the HUD and the section BB displays reflect **the judgement states at the time** in conjunction with the playback and the time bar operations after the simulation.

## Section BB visualization

When the "Show section bounding boxes" property of the VnoidJudgeItem is true (default), the BBs of the pre stage, the section, and the post stage of each section are displayed on the scene view as wireframes. The colors are the same as the status colors described above, and gray in standby. Since they are displayed even when the simulation is not running, they can also be used to check the section definitions of the field.

## Saving the judgement result

By selecting "Save the judgement result to a file" from the context menu (right click) of the VnoidJudgeItem, the judgement result of **the current playback state** can be saved to a text file. If you want to save the final result of the simulation, advance the playback to the final time in advance (right after the simulation execution, the final state is reproduced as is).

The default file name is "project name-result.txt". An output example:

```
Project: vnoid-judge-test
Target: SR1
Elapsed time: 30.000 s
Total mass: 45.000 kg (OK)
Power average window: 0.01 s
Max power: 350.2 W (joint RLEG_KNEE, ID 4) (OK)
Max bounding box: 0.512 0.605 1.630
BBox: OK
Section Section1-Easy: CLEAR at 12.340 s
Section Section2-Easy: CLEAR at 25.100 s
```

The cleared sections are listed in the order of the clear times. If no section has been cleared, `No section has been cleared.` is output.

For the automation with scripts, the VnoidJudgeItem provides the `saveJudgementResult(filename)` function, which saves the result of the current playback state to the specified file without a dialog.

## Properties

### VnoidJudgeTargetItem

| Property | Meaning | Default |
|---|---|---|
| Power limit [W] | The upper limit of the POWER | 1000.0 |
| Mass lower limit | The lower limit of the total mass [kg] | 30.0 |
| BBox upper limit X/Y/Z | The upper limits of the BB size of each axis [m] | 2.0 |

### VnoidJudgeItem

| Property | Meaning | Default |
|---|---|---|
| Power average window [s] | The time window of the moving average of the POWER | 0.01 |
| Show section bounding boxes | Enables the section BB visualization | true |
| Font size | The font size of the HUD | 18 |
| Text color | The text color of the HUD | White |
| Panel transparency | The transparency of the HUD panels | 0.4 |

## Log saving with WorldLogFileItem

Since the judgement states are recorded as the state of the judgement device, the log of the judgement result is also saved together when the simulation log is saved to a file using WorldLogFileItem. Even after reloading the project with the saved log, the judgement states (the HUD and the section BB displays) are reproduced by the playback of the WorldLogFileItem, and the judgement result file can be saved at any playback time.
