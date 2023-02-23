``` mermaid
stateDiagram-v2
    [*] --> stow
    stow --> intakeStage
    intakeStage --> pickupFromIntake: beamBreak
    pickupFromIntake --> stow: haveCube

    state intakeStage {
        [*] --> armAndIntakeStage
        armAndIntakeStage --> openHand: armInPos
        openHand --> [*]: handOpen
    }

    state pickupFromIntake{
        [*] --> retractIntake
        retractIntake --> armIntake : intakeFinished
        armIntake --> closeHand: armInPos
        closeHand --> waitIntake: handClosed
        waitIntake --> [*]: delayElapsed
    }


    stow --> autoScore
    autoScore --> releaseGamepiece: driver button AND autoStage done
    releaseGamepiece --> stow: noGamepiece

    state autoScore {
        state driveAndArm <<fork>>
        state armAndDrive <<join>>
        [*] --> driveAndArm
        driveAndArm --> driveToPos: Cone/Cube \n L/R \n 1/2/3
        driveAndArm --> armToScore: 1/2/3
        driveToPos --> armAndDrive
        armToScore --> armAndDrive
        armAndDrive --> [*]: driveDone
    }

    state releaseGamepiece {
        [*] --> waitScore
        waitScore --> [*]: delayElapsed
    }


    stow --> manualScore
    manualScore --> releaseGamepiece : driver button AND armInPos

    state manualScore {
        [*] --> armToScore2: 1/2/3
        armToScore2 --> [*]: armInPos
    }

    stow --> autoShelf
    autoShelf --> waitShelf
    waitShelf --> stow: odomDeltaDone

    state autoShelf {
        state driveAndArm2 <<fork>>
        state armAndDrive2 <<join>>
        [*] --> driveAndArm2
        driveAndArm2 --> driveToPos2
        driveAndArm2 --> armToShelf
        driveToPos2 --> armAndDrive2 
        armToShelf --> armAndDrive2
        armAndDrive2 --> [*]: driveDone
    }

    state grabGamepiece {
        [*] --> handGrab
        handGrab --> [*]: handFinished
    }

    stow --> toManualShelf
    toManualShelf --> manualShelf : robotInPos
    manualShelf --> waitShelf : odomDeltaDone
    
    state toManualShelf {
    [*] --> armToShelf2
    armToShelf2 --> handOpen : armInPos
    handOpen --> [*] : handInPos
    }

    state manualShelf {
        [*] --> closeHand2 : proxSensorThres
        closeHand2 --> [*] : handInPos
    }
    
    state waitShelf {
    [*] --> waitOdomDelta
    waitOdomDelta --> [*] : drive >1m
    }

    stow --> floorPickup
    floorPickup --> grabGamepiece
    grabGamepiece --> stow

    state floorPickup {
        [*] --> armToFloor
        armToFloor --> [*]
    }

```
