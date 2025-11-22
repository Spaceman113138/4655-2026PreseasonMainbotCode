package frc.robot.LoggingFiles.logging.revlib;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.LoggingFiles.logging.revlib.structs.SparkBaseStruct;

@CustomLoggerFor(SparkFlex.class)
public class SparkFlexLogger extends ClassSpecificLogger<SparkFlex> {

  private static final SparkFlexStruct struct = new SparkFlexStruct();

  public SparkFlexLogger() {
    super(SparkFlex.class);
  }

  @Override
  public void update(EpilogueBackend backend, SparkFlex sparkFlex) {
    backend.log("", sparkFlex, struct);
  }

  private static class SparkFlexStruct extends SparkBaseStruct<SparkFlex> {

    @Override
    public Class<SparkFlex> getTypeClass() {
      return SparkFlex.class;
    }

    @Override
    public String getTypeName() {
      return "SparkFlex";
    }
  }
}
