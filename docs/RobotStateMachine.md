``` mermaid
stateDiagram-v2
    [*] --> stow
    stow --> intakeStage
    intakeStage --> intake: beamBreak
    intake --> stow: haveCube

    state intakeStage {
        [*] --> armIntakeStage
        armIntakeStage --> openHand: armInPos
        openHand --> [*]: handOpen
    }

    state intake{
        [*] --> armIntake
        armIntake --> closeHand: armInPos
        closeHand --> waitIntake: handClosed
        waitIntake --> [*]: delayElapsed
    }


    stow --> autoScore
    autoScore --> releaseGamepiece: 
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
        startDelay --> [*]: delayElapsed
    }


    stow --> manualScore
    manualScore --> releaseGamepiece

    state manualScore {
        [*] --> armToScore2: 1/2/3
        armToScore2 --> [*]: armInPos
    }

    stow --> autoShelf
    autoShelf --> grabGamepiece
    grabGamepiece --> stow: haveCone

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
        [*] --> waitShelf
        startDelay2 --> [*]: delayElapsed
    }

    stow --> manualShelf
    manualShelf --> grabGamepiece

    state manualShelf {
        [*] --> armToShelf2
        armToShelf2 --> waitProx
        waitProx --> [*]: inRange
    }

    stow --> floorPickup
    floorPickup --> grabGamepiece

    state floorPickup {
        [*] --> armToFloor
        armToFloor --> [*]
    }

```
