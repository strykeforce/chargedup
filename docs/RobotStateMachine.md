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

    stow --> toAutoShelf
    toAutoShelf --> autoDrive : handFinished
    autoDrive --> autoShelf : autoDriveFinished
    autoShelf --> waitShelf : haveGamepiece
    waitShelf --> stow: odomDeltaDone

    state toAutoShelf {
        [*] --> armToShelf3
        armToShelf3 --> openHand2 : armInPos
        openHand2 --> [*] : handInPos
    }

    state autoDrive {
        state checkFail <<choice>>
        [*] --> checkFail
        checkFail --> autoFailed : noOdomReset
        checkFail --> driveAutoPath : OdomReset
        driveAutoPath --> [*] : autoDriveFinished

    }

    state autoShelf {
        [*] --> waitGamepiece
        waitGamepiece --> closeHand3 : sensorSeePiece
        closeHand3 --> [*] : handInPos
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
