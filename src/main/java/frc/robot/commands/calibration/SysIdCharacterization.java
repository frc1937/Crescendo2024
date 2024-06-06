package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface SysIdCharacterization {
    SysIdRoutine getSysIdRoutine();

    default Command sysIdQuastaticTest(SysIdRoutine.Direction direction) {
        return getSysIdRoutine().quasistatic(direction);
    }

    default Command sysIdDynamicTest(SysIdRoutine.Direction direction) {
        return getSysIdRoutine().dynamic(direction);
    }
}

