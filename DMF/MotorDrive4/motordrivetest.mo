within ;
model MotorDriveTest
  extends DriveLib.MotorDrive;
  Modelica.Blocks.Sources.Step Step1 annotation (Placement(transformation(
          extent={{-100,-10},{-80,10}}, rotation=0)));
equation
  connect(Step1.y,positionerror.u1)             annotation (Line(points={{-79,0},
          {-68,0}}));
  annotation (__Dymola_Commands(file="MotorDriveTest.mos" "Simulate motor and plot result"),
    Dymola(checkSum="3294755121:4183752949"),
    version="4",
 versionBuild=1,
    versionDate="2019-02-27",
 dateModified="2019-02-27 15:32:00Z",
    uses(DriveLib(version="4"), Modelica(version="4.0.0")),
    conversion(from(version="2", script="ConvertFromMotorDriveTest_2.mos")));
end MotorDriveTest;
