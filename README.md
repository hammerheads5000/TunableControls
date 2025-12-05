# TunableControls
Runtime-tunable versions of WPILib's PIDController and ProfiledPIDController using AdvantageKit's LoggedTunableNumber. It exposes any constant that you might want to tune. This allows super quick tuning without redeploying.

This also has an advantage over just putting a PIDController to the dashboard using the sendable interface because it avoids possible issues that can come up when you have new controllers being created with the same name, such as when a command has its own PIDController and is instantiated multiple times.

<details>
  <summary><i>Pedantic note</i></summary>

  _Yes, calling these constants is technically incorrect because they can be changed, and so are not constant. However, I do not care, and I just use constants and parameters interchangably._
</details>

## Installation
It's super simple to install! Just copy TunableControls.java into your project (and LoggedTunableNumber.java if it's not already there). If you put it somewhere other than frc/robot/utils, then change the first line of both files to match the new file path.

> [!IMPORTANT]
> [AdvantageKit](https://docs.advantagekit.org/) is required to use any of this!

## Usage
Define a set of parameters with `ControlConstants` (All constants have defaults, so none are strictly necessary to set. However, I strongly recommend at least setting PID and tolerance for an unprofiled controller, and all of these for a profiled one):
```java
ControlConstants CONTROL_CONSTANTS = new ControlConstants()
        .withPID(10, 0, 0)         // kP, kI, kD
        .withTolerance(0.04, 0.1)  // tolerance, velTolernace
        .withFeedforward(2.9, 0.0) // kV, kA (only used for profiled controllers)
        .withPhysical(0.0, 0.2)    // kS, kG (only used for profiled controllers)
        .withProfile(2.0, 3.0);    // maxVel, maxAcc (only used for profiled controllers)
```
Then create a `TunableConstants` object from these constants:
```java
TunableControlConstants TUNABLE_CONSTANTS = new TunableControlConstants("Elevator", CONTROL_CONSTANTS);
```
This will automatically publish all these values to the "Tuning" NetworkTable, which can then be tuned in AdvantageScope or from another dashboard. AdvantageScope will save tunable values between deploys as long as it stays open, but __make sure to update the `ControlConstants` object when done tuning!__

Finally, create a `TunablePIDController` or `TunableProfiledController` object:
```java
TunableProfiledController controller = new TunableProfiledController(ElevatorConstants.TUNABLE_CONSTANTS);
```
Which can fully replace a `PIDController` or `ProfiledPIDController` in code with no other changes (for the most part).

> [!NOTE]
> `TunableProfiledController` will include feedforward control in its output from the `calculate` method, eliminating the need for a separate `SimpleMotorFeedforward` or `ElevatorFeedforward` object (`ArmFeedforward` is yet to be implemented).

Feedforward constants (`kS`, `kG`, `kV`, `kA`) are continuously updated. The rest of the constants are updated any time the `setGoal` method is called, or they can also be updated manually using `updateParams`.

## Tuning Demonstration
<a href="https://www.youtube.com/watch?v=gsnrGc5PhoA"><img width="720" height="480" alt="Demo video" src="https://github.com/user-attachments/assets/b11c7be8-68c8-42b1-b0e7-7895cf1e5cda" /></a>


## Planned Features
- [ ] Make into installable vendordep
- [ ] Add option for ArmFeedforward
- [ ] Add ability to enable/disable
- [ ] Add "slots" feature (different sets of parameters that can be switched between)
- [ ] Add version that integrates with [AtlantisKit](https://github.com/Atlantis2679/AtlantisKit)
