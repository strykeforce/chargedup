```mermaid
stateDiagram-v2
    [*] --> Stow
    Stow --> stowToHigh : toHigh()
    stowToHigh --> high : armInPos
    high --> scoreToStow : toStow()
    scoreToStow --> Stow : armInPos

    state stowToHigh {
        [*] --> elbowToHigh
        elbowToHigh --> elevatorToHigh: elbowFinished
        elevatorToHigh --> shoulderToHigh: elevatorFinished
        shoulderToHigh --> [*]: shoulderFinished
    }

    state scoreToStow {
        [*] --> shoulderToStow
        shoulderToStow --> elevatorToStow: shoulderFinished
        elevatorToStow --> elbowToStow: elevatorFinished
        elbowToStow --> [*]: elbowFinished
    }

    Stow --> stowToMid : toMid()
    stowToMid --> mid : armInPos
    mid --> scoreToStow : toStow()

    state stowToMid {
        [*] --> elbowToMid
        elbowToMid --> elevatorToMid: elbowFinished
        elevatorToMid --> shoulderToMid: elevatorFinished
        shoulderToMid --> [*]: shoulderFinished
    }

    Stow --> stowToLow : toLow()
    stowToLow --> low : armInPos
    low --> scoreToStow : toStow()

    state stowToLow {
        [*] --> elbowToLow
        elbowToLow --> shoulderToLow: elbowFinished
        shoulderToLow --> elevatorToLow: shoulderFinished
        elevatorToLow --> [*]: elevatorFinished
    }

    Stow --> stowToIntakeStage : toIntakeStage()
    stowToIntakeStage --> intakeStage : armInPos
    intakeStage --> intakeStageToIntake : toIntake()
    intakeStageToIntake --> intake
    intake --> intakeToStow : toStow()
    intakeToStow --> Stow : armInPos
    intakeStage --> intakeToStow : toStow()

    state stowToIntakeStage {
        state elbowAndShoulder <<fork>>
        state shoulderAndElbow <<join>>
        [*] --> elbowAndShoulder
        elbowAndShoulder --> shoulderToIntake
        elbowAndShoulder --> elbowToIntake
        shoulderToIntake --> shoulderAndElbow
        elbowToIntake --> shoulderAndElbow
        shoulderAndElbow --> [*] : elbowFinished
    }

    state intakeStageToIntake {
        [*] --> elevatorToIntake
        elevatorToIntake --> [*]: elevatorFinished
    }

    state intakeToStow {
        state elevatorAndShoulder <<fork>>
        state shoulderAndElevator <<join>>
        [*] --> elevatorAndShoulder
        elevatorAndShoulder --> elevatorToStow2
        elevatorAndShoulder --> shoulderToStow2
        elevatorToStow2 --> shoulderAndElevator
        shoulderToStow2 --> shoulderAndElevator
        shoulderAndElevator --> elbowToStow2
        elbowToStow2 --> [*]
    }

   Stow --> stowToFloor : toFloor()
   stowToFloor --> floor : armInPos
   floor --> floorToStow : toStow()
   floorToStow --> Stow : armInPos

   state stowToFloor {
    [*] --> elbowToFloor
    elbowToFloor --> shoulderToFloor: elbowFinished
    shoulderToFloor --> elevatorToFloor: shoulderFinished
    elevatorToFloor --> [*]: elevatorFinished
   }

   state floorToStow {
    [*] --> shoulderToStow3
    shoulderToStow3 --> elevatorToStow3: shoulderFinished
    elevatorToStow3 --> elbowToStow3: elevatorFinished
    elbowToStow3 --> [*]: elbowFinished
   }
   ```