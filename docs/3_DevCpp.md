# C++ Library Development

## C++ Notes

### Overview
- C++ is used on this project over other languages for it's performance and embedded programming capabilities
- C++ is an object-oriented programming language, meaning that the code is organized into 'objects', or classes, with specific purposes and functions

### Pointers
- The objects in the code are usually single instance, meaning they only exist once in memory, and are shared to other objects using its pointer, or memory address
- Typically, using pointers requires allocating and deallocating space in memory (using `free` and `delete`), but it makes the code prone to memory-leaks (when memory is repeatedly allocated, but never de-allocated), which fills up the memory with useless objects
- To combat this, we use the STL's (C++ Standard Library) implementation of 'special pointers', namely `shared_ptr`, which automatically de-allocates the memory when there are no instances left
- We use the method `std::make_shared<ObjectType>(param1, param2, ...)` to construct a new `shared_ptr` of type `ObjectType` with a constructor of parameters `param1`, `param2`, `...`
- In the code, the shortcut `ObjectType::Ptr` is defined to be used instead of `shared_ptr<ObjectType>`

### Abstraction
- Object-oriented code also allows for the use of abstracted classes, or outlines, that other classes can derive from
- This is useful if there is some common structure for a set of objects used by another object, but each with their own implementation
- For example, we have a `MotorController` abstract class that defines the set of functions for all motor controller types and is used by the `LegController` class to control the motors, but the actual method of control is determined by how the `MotorController` class functions are overwritten by say the `ODriveController` or `MuJoCoController` classes, both of which are motor controller instances
- This enables us to have custom definitions/implementations for certain parts of the code, without having to edit the code at a higher level

## Library Architecture

### Robot Class

- The highest level of the library is the `Robot` class
- The Robot class contains the functions to every subcomponent that runs the physical robot
- Every robot needs to define `6` subclasses to have full functionality
  1. `MotorController`: Interface between the library and the hardware/simulation 
     - ODriveController
     - MuJoCoController
  2. `LegDynamics`: Equations for forward and inverse kinematics, and Jacobian of a leg 
     - FiveBar2DLegDynamics
     - UnitreeA1LegDynamics
  3. `Localization`: Current time, and the position and orientation of the robot
     - SystemLocalization *(System time only)*
     - ROS2Localization
     - MuJoCoLocalization
  4. `RobotParameters`: Parameters required to run model predictive control 
     - STARQRobotParameters
     - UnitreeA1RobotParameters
  5. `MPCSolver`: Interface with external MPC/QP solvers
     - OSQP
  6. `PlanningModel`: SBMPO planning model
     - STARQPlanningModel

### Current Robot Definitions

| Name | MotorControllers | LegDynamics | Localization | RobotParameters | PlanningModel | MPCSolver |
| ---- | ---------------- | ----------- | ------------ | --------------- | ------------- | --------- |
| **STARQ** | ODrive | FiveBar2D | ROS2 | STARQ | STARQ | OSQP |
| **UnitreeA1** | MuJoCo | UnitreeA1 | MuJoCo | UnitreeA1 | STARQ | OSQP |

### Using a Robot

- Include a robot into your program using:
  - `#include "starq/starq/starq_robot.hpp"`
  - `#include "starq/unitree/unitree_a1_mujoco_robot.hpp"`
- See the available functions [here](../starq/include/starq/robot.hpp)
- See robot specific functions for:
  - [STARQ Robot](../starq/include/starq/starq/starq_robot.hpp)
  - [Unitree A1 Robot](../starq/include/starq/unitree/unitree_a1_mujoco_robot.hpp)
- See examples in the `tests` and `utils` directories

### Creating a Custom Robot

