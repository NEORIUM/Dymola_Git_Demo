within ;
package Demo_DriveLib
  extends Modelica.Icons.Package;
  model Motor "A basic model of an electrical dc motor."

    Modelica.Electrical.Analog.Sources.SignalVoltage Vs annotation (Placement(
          transformation(
          origin={-70,0},
          extent={{10,-10},{-10,10}},
          rotation=90)));
    Modelica.Electrical.Analog.Basic.Ground G annotation (Placement(
          transformation(extent={{-80,-60},{-60,-40}}, rotation=0)));
    Modelica.Electrical.Analog.Basic.Resistor Ra(R=0.5) annotation (Placement(
          transformation(extent={{-60,30},{-40,50}}, rotation=0)));
    Modelica.Electrical.Analog.Basic.Inductor La(L=0.05, i(fixed=true))
                                                         annotation (Placement(
          transformation(extent={{-20,30},{0,50}}, rotation=0)));
    Modelica.Electrical.Analog.Basic.RotationalEMF emf(k=1) annotation (
        Placement(transformation(extent={{0,-10},{20,10}}, rotation=0)));
    Modelica.Blocks.Interfaces.RealInput inPort         annotation (Placement(
          transformation(extent={{-108,-10},{-90,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia Jm(J=0.005)
                                                      annotation (Placement(
          transformation(extent={{38,-10},{58,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation (Placement(
          transformation(extent={{90,-10},{110,10}}, rotation=0)));
  equation
    connect(Ra.n, La.p) annotation (Line(points={{-40,40},{-20,40}}));
    connect(La.n, emf.p) annotation (Line(points={{0,40},{10,40},{10,10}}));
    connect(emf.flange,   Jm.flange_a) annotation (Line(points={{20,0},{38,0}}));
    connect(Ra.p, Vs.p) annotation (Line(points={{-60,40},{-70,40},{-70,10}}));
    connect(Vs.n, emf.n) annotation (Line(points={{-70,-10},{-70,-20},{10,-20},
            {10,-10}}));
    connect(G.p, Vs.n) annotation (Line(points={{-70,-40},{-70,-10}}));
    connect(Jm.flange_b, flange_b) annotation (Line(points={{58,0},{100,0}}));
    connect(inPort,Vs.v)       annotation (Line(points={{-99,0},{-82,4.28626e-16}}));
    annotation (
      Icon(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2}), graphics={
          Rectangle(
            extent={{-60,40},{60,-40}},
            lineColor={0,0,0},
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,0,0}),
          Rectangle(
            extent={{-80,-80},{80,-100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-38,-20},{-60,-80},{60,-80},{40,-20},{-40,-20},{-38,-20}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-90,0},{-60,0}}),
          Rectangle(
            extent={{60,8},{90,-8}},
            lineColor={160,160,164},
            fillColor={160,160,164},
            fillPattern=FillPattern.Solid),
          Text(extent={{-80,100},{80,60}}, textString=
                                               "%name")}),
      Documentation(info="A basic model of an electrical dc motor.
"));
  end Motor;

  partial model MotorDrive
    parameter Modelica.Units.SI.Radius r=0.5 "Radius of load";
    parameter Modelica.Units.SI.Mass m=80 "Mass of load";
    Demo_DriveLib.Motor motor annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.IdealGear gearbox(
      ratio=100,
      useSupport=false) annotation (Placement(
          transformation(extent={{30,-10},{50,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia load(
      J=0.5*m*r*r,
      phi(fixed=true, start=0),
      w(fixed=true, start=0)) annotation (Placement(
          transformation(extent={{60,-10},{80,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor phiload annotation (Placement(
          transformation(
          origin={80,-30},
          extent={{-10,-10},{10,10}},
          rotation=270)));
    Modelica.Blocks.Math.Feedback positionerror annotation (Placement(
          transformation(extent={{-70,-10},{-50,10}}, rotation=0)));
    Modelica.Blocks.Continuous.PID controller(
      Ti=0.5,
      Td=0.1,
      xd_start=0) annotation (Placement(
          transformation(extent={{-40,-10},{-20,10}}, rotation=0)));
  equation
    connect(gearbox.flange_b, load.flange_a) annotation (Line(points={{50,0},{
            60,0}}));
    connect(load.flange_b,phiload.flange)    annotation (Line(points={{80,0},{
            80,-20}}));
    connect(positionerror.u2,phiload.phi)           annotation (Line(points={{
            -60,-8},{-60,-50},{80,-50},{80,-41}}));
    connect(motor.flange_b, gearbox.flange_a) annotation (Line(points={{10,0},{
            30,0}}));
    connect(positionerror.y,controller.u)             annotation (Line(points={
            {-51,0},{-42,0}}));
    connect(controller.y,       motor.inPort) annotation (Line(points={{-19,0},
            {-9.9,0}}));
  end MotorDrive;

  model TestMotor

    Demo_DriveLib.Motor Motor1 annotation (Placement(transformation(extent={{20,0},{40,20}}, rotation=0)));
    Modelica.Blocks.Sources.Step Step1 annotation (Placement(transformation(
            extent={{-20,0},{0,20}}, rotation=0)));
  equation
    connect(Step1.y,       Motor1.inPort) annotation (Line(points={{1,10},{20.1,
            10}}));
  end TestMotor;
  annotation (
    version="4",
 versionBuild=1,
    versionDate="2019-02-27",
 dateModified="2019-02-27 15:32:00Z",
    uses(Modelica(version="4.0.0")),
    conversion(from(version="2", script="ConvertFromDriveLib_2.mos")));
end Demo_DriveLib;
