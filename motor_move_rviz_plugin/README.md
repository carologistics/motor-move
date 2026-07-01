# motor_move_rviz_plugin

RViz panel for sending `motor_move_msgs/action/MotorMove` goals and comparing
ideal motion against live action feedback.

Load it in RViz with:

```text
Panels -> Add New Panel -> motor_move_rviz_plugin/MotorMovePanel
```

The panel is independent from the `motor_move` package. It only depends on
`motor_move_msgs` for the action type.

Fields:

| Field | Description |
|---|---|
| `Action` | Action server name, for example `/robotinobase1/motor_move_action` |
| `Param node` | Node whose motion parameters are set |
| `Frame` | Frame for the outgoing goal pose |
| `X`, `Y`, `Rot` | Goal pose in that frame |
| `Max linear`, `Linear accel` | Linear dynamic parameters sent with `ros2 param` semantics |
| `Max angular`, `Angular accel` | Angular dynamic parameters sent with `ros2 param` semantics |
| `Linear kp`, `Angular kp` | Proportional damping caps near the target |

Changing any motion parameter sends a parameter update immediately after a
short debounce. A checkmark is shown when the update succeeds.

Press `Send` to send the action goal. The speed and position-error plots keep
the ideal curve and add the real feedback curve while the action runs. The
ideal preview assumes the current motor_move behavior: rotate first, then drive
the linear part.
