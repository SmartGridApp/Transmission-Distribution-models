package TransmissionDistributionModels
  extends Modelica.Icons.Package;
  import Modelica.Constants.pi;
  import PowerSystems.Types.SI;
  import PowerSystems.Types.SIpu;

  model subModelGeneric "Two generator generation and power transmission to fixed load"
    extends Modelica.Icons.Example;
    PowerSystems.Generic.Generator generator annotation(
      Placement(transformation(extent = {{-20, 0}, {0, 20}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 1e3, phi(fixed = true, start = 0), w(fixed = true, start = 50 * 2 * pi)) annotation(
      Placement(transformation(extent = {{-50, 0}, {-30, 20}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(useSupport = false, tau_constant = 3e6 / 50 / 2 / 2 / pi) annotation(
      Placement(transformation(extent = {{-90, 0}, {-70, 20}})));
    PowerSystems.Generic.FixedLoad fixedLoad(P(displayUnit = "MW") = 200000, phi = 0.643328) "0.2MW, 0.8pf lag" annotation(
      Placement(visible = true, transformation(extent = {{120, -22}, {140, -2}}, rotation = 0)));
    PowerSystems.Generic.Generator generator1 annotation(
      Placement(transformation(extent = {{-20, -40}, {0, -20}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1e3, phi(fixed = true, start = 0), w(fixed = true, start = 50 * 2 * pi)) annotation(
      Placement(transformation(extent = {{-50, -40}, {-30, -20}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(useSupport = false, tau_constant = 3e6 / 50 / 2 / pi) annotation(
      Placement(transformation(extent = {{-90, -40}, {-70, -20}})));
    PowerSystems.Generic.Impedance STLine30km(L = 0.021, R = 0.9) annotation(
      Placement(visible = true, transformation(extent = {{18, 70}, {38, 90}}, rotation = 0)));
    PowerSystems.Generic.Impedance STLine30kmCopy(L = 0.021, R = 0.9) annotation(
      Placement(visible = true, transformation(origin = {48, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    inner PowerSystems.System system(dynType = PowerSystems.Types.Dynamics.SteadyState) annotation(
      Placement(transformation(extent = {{-100, 80}, {-80, 100}})));
    PowerSystems.Generic.Transformer transformer1(ratio = 0.6) annotation(
      Placement(visible = true, transformation(origin = {8, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    PowerSystems.Generic.Transformer transformer2(ratio = 0.6) annotation(
      Placement(visible = true, transformation(origin = {30, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
    PowerSystems.Generic.Transformer transformer3(ratio = 3.3) annotation(
      Placement(visible = true, transformation(origin = {58, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerSystems.Generic.Sensors.PMeter pMeterLoad annotation(
      Placement(visible = true, transformation(origin = {94, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerSystems.Generic.Sensors.PMeter pMeter1 annotation(
      Placement(visible = true, transformation(origin = {48, 36}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    PowerSystems.Generic.Sensors.PMeter pMeter2 annotation(
      Placement(visible = true, transformation(origin = {4, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    connect(transformer2.terminal_n, STLine30kmCopy.terminal_p) annotation(
      Line(points = {{40, -80}, {48, -80}, {48, -52}, {48, -52}}, color = {0, 120, 120}));
    connect(pMeter2.terminal_n, transformer2.terminal_p) annotation(
      Line(points = {{4, -70}, {4, -70}, {4, -80}, {20, -80}, {20, -80}}, color = {0, 120, 120}));
    connect(generator1.terminal, pMeter2.terminal_p) annotation(
      Line(points = {{0, -30}, {4, -30}, {4, -50}, {4, -50}, {4, -50}}, color = {0, 120, 120}));
    connect(STLine30kmCopy.terminal_n, transformer3.terminal_p) annotation(
      Line(points = {{48, -32}, {48, -12}}, color = {0, 120, 120}));
    connect(pMeter1.terminal_n, transformer3.terminal_p) annotation(
      Line(points = {{48, 26}, {48, -12}}, color = {0, 120, 120}));
    connect(STLine30km.terminal_n, pMeter1.terminal_p) annotation(
      Line(points = {{38, 80}, {48, 80}, {48, 46}}, color = {0, 120, 120}));
    connect(pMeterLoad.terminal_n, fixedLoad.terminal) annotation(
      Line(points = {{104, -12}, {120, -12}, {120, -12}, {120, -12}}, color = {0, 120, 120}));
    connect(transformer3.terminal_n, pMeterLoad.terminal_p) annotation(
      Line(points = {{68, -12}, {84, -12}, {84, -12}, {84, -12}}, color = {0, 120, 120}));
    connect(transformer1.terminal_n, STLine30km.terminal_p) annotation(
      Line(points = {{8, 58}, {8, 58}, {8, 80}, {18, 80}, {18, 80}}, color = {0, 120, 120}));
    connect(generator.terminal, transformer1.terminal_p) annotation(
      Line(points = {{0, 10}, {8, 10}, {8, 38}, {8, 38}, {8, 38}}, color = {0, 120, 120}));
    connect(inertia.flange_b, generator.flange) annotation(
      Line(points = {{-30, 10}, {-20, 10}}, color = {0, 0, 0}));
    connect(constantTorque.flange, inertia.flange_a) annotation(
      Line(points = {{-70, 10}, {-50, 10}}, color = {0, 0, 0}));
    connect(inertia1.flange_b, generator1.flange) annotation(
      Line(points = {{-30, -30}, {-20, -30}}, color = {0, 0, 0}));
    connect(constantTorque1.flange, inertia1.flange_a) annotation(
      Line(points = {{-70, -30}, {-50, -30}}, color = {0, 0, 0}));
    annotation(
      experiment(StopTime = 100),
      Documentation(info = "<html>
<p>
This sub-model is a very basic part of power distribution in island mode, i.e. without connection to a larger net.
It contains the following components:
<ul>
<li>Two generators as source,</li>
<li>Two transformers for stepping up generation voltage,</li>
<li>One Transformer for stepping down voltage at the customer end,</li>
<li>Short Transmission lines with parameters R=0.03 ohm/km and L=0.7 mH/km,</li>
<li>A fixed load of 0.2 MW at 0.8 p.f. lag.</li>
</ul>
</p>
  </html>"),
      Diagram(coordinateSystem(extent = {{-100, -100}, {150, 100}})),
      Icon(coordinateSystem(extent = {{-100, -100}, {150, 100}})),
      __OpenModelica_commandLineOptions = "");
  end subModelGeneric;

  model SubModel1ph "1 phase model having AC source feeding to a 1 phase dynamic load having one faulty line"
    extends Modelica.Icons.Example;
    inner PowerSystems.System system annotation(
      Placement(transformation(extent = {{-100, 80}, {-80, 100}})));
    PowerSystems.AC1ph_DC.Sources.ACvoltage voltage1(V_nom = 6600, alpha0 = 0.5235987755983) annotation(
      Placement(transformation(extent = {{-90, -20}, {-70, 0}})));
    PowerSystems.AC1ph_DC.Transformers.TrafoStray trafo(redeclare record Data = PowerSystems.Examples.Data.Transformers.TrafoStray1ph(V_nom = {20e3, 132e3}, S_nom = 100e6, f_nom = 50)) annotation(
      Placement(visible = true, transformation(extent = {{-66, -20}, {-46, 0}}, rotation = 0)));
    PowerSystems.AC1ph_DC.Lines.Tline line(redeclare record Data = PowerSystems.AC1ph_DC.Lines.Parameters.Line(V_nom = 132e3), len = 30000) annotation(
      Placement(transformation(extent = {{20, -40}, {40, -20}})));
    PowerSystems.AC1ph_DC.Breakers.Switch switch1(V_nom = 132e3, I_nom = 2500) annotation(
      Placement(transformation(extent = {{-40, 0}, {-20, 20}})));
    PowerSystems.AC1ph_DC.Lines.FaultTline lineF(redeclare record Data = PowerSystems.AC1ph_DC.Lines.Parameters.Line(V_nom = 132e3), len = 30000) annotation(
      Placement(transformation(extent = {{20, 0}, {40, 20}})));
    PowerSystems.AC1ph_DC.Breakers.Switch switch2(V_nom = 132e3, I_nom = 2500) annotation(
      Placement(transformation(extent = {{50, 0}, {70, 20}})));
    PowerSystems.AC1ph_DC.Faults.Fault_ab ab annotation(
      Placement(transformation(extent = {{20, 40}, {40, 60}})));
    PowerSystems.AC1ph_DC.Sources.ACvoltage voltage2(V_nom = 3300, alpha0 = 0.5235987755983) annotation(
      Placement(visible = true, transformation(extent = {{130, -20}, {110, 0}}, rotation = 0)));
    PowerSystems.Control.Relays.SwitchRelay relay1(t_switch = {0.15, 0.2}, n = 1) annotation(
      Placement(transformation(extent = {{-60, 40}, {-40, 60}})));
    PowerSystems.Control.Relays.SwitchRelay relay2(t_switch = {0.153, 0.21}, n = 1) annotation(
      Placement(transformation(extent = {{90, 40}, {70, 60}})));
    PowerSystems.AC1ph_DC.Sensors.PVImeter meterL(S_nom = 1000e6, V_nom = 132e3) annotation(
      Placement(transformation(extent = {{-10, -40}, {10, -20}})));
    PowerSystems.AC1ph_DC.Sensors.PVImeter meterF(S_nom = 1000e6, V_nom = 132e3) annotation(
      Placement(transformation(extent = {{-10, 0}, {10, 20}})));
    PowerSystems.AC1ph_DC.Nodes.GroundOne grd1 annotation(
      Placement(transformation(extent = {{-90, -20}, {-110, 0}})));
    PowerSystems.AC1ph_DC.Nodes.GroundOne grd2 annotation(
      Placement(visible = true, transformation(extent = {{130, -20}, {150, 0}}, rotation = 0)));
    PowerSystems.Blocks.Signals.Transient[2] transSig(s_end = {2, 3}, s_start = {1, 2}) annotation(
      Placement(visible = true, transformation(origin = {16, -126}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerSystems.AC1ph_DC.Loads.ZloadAC zLoadAC(use_pq_in = true) annotation(
      Placement(visible = true, transformation(origin = {50, -126}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
    PowerSystems.AC1ph_DC.Nodes.BusBar bus1 annotation(
      Placement(visible = true, transformation(origin = {-40, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerSystems.AC1ph_DC.Nodes.BusBar bus2 annotation(
      Placement(visible = true, transformation(origin = {70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerSystems.AC1ph_DC.Sensors.PVImeter meterLoad(S_nom = 1000e6, V_nom = 132e3) annotation(
      Placement(visible = true, transformation(origin = {50, -88}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    connect(line.term_n, meterLoad.term_p) annotation(
      Line(points = {{40, -30}, {50, -30}, {50, -78}, {50, -78}}, color = {0, 0, 255}));
    connect(meterLoad.term_n, zLoadAC.term) annotation(
      Line(points = {{50, -98}, {50, -98}, {50, -116}, {50, -116}}, color = {0, 0, 255}));
    connect(line.term_n, bus2.term) annotation(
      Line(points = {{40, -30}, {70, -30}, {70, -10}, {70, -10}}, color = {0, 0, 255}));
    connect(transSig.y, zLoadAC.pq_in) annotation(
      Line(points = {{26, -126}, {40, -126}}, color = {0, 0, 127}));
    connect(grd2.term, voltage2.neutral) annotation(
      Line(points = {{130, -10}, {130, -10}}, color = {0, 0, 255}));
    connect(voltage2.term, bus2.term) annotation(
      Line(points = {{110, -10}, {70, -10}}, color = {0, 0, 255}));
    connect(bus1.term, meterL.term_p) annotation(
      Line(points = {{-40, -12}, {-40, -12}, {-40, -30}, {-10, -30}, {-10, -30}}, color = {0, 0, 255}));
    connect(trafo.term_n, bus1.term) annotation(
      Line(points = {{-46, -10}, {-40, -10}, {-40, -12}, {-40, -12}}, color = {0, 0, 255}));
    connect(bus1.term, switch1.term_p) annotation(
      Line(points = {{-40, -12}, {-40, -12}, {-40, 10}, {-40, 10}}, color = {0, 0, 255}));
    connect(switch2.term_n, bus2.term) annotation(
      Line(points = {{70, 10}, {70, -10}}, color = {0, 0, 255}));
    connect(voltage1.term, trafo.term_p) annotation(
      Line(points = {{-70, -10}, {-66, -10}}, color = {0, 120, 120}));
    connect(meterL.term_n, line.term_p) annotation(
      Line(points = {{10, -30}, {20, -30}}, color = {0, 110, 110}));
    connect(switch1.term_n, meterF.term_p) annotation(
      Line(points = {{-20, 10}, {-10, 10}}, color = {0, 110, 110}));
    connect(meterF.term_n, lineF.term_p) annotation(
      Line(points = {{10, 10}, {20, 10}}, color = {0, 110, 110}));
    connect(lineF.term_n, switch2.term_p) annotation(
      Line(points = {{40, 10}, {50, 10}}, color = {0, 110, 110}));
    connect(lineF.term_f, ab.term) annotation(
      Line(points = {{30, 20}, {30, 40}}, color = {0, 110, 110}));
    connect(grd1.term, voltage1.neutral) annotation(
      Line(points = {{-90, -10}, {-90, -10}}, color = {0, 0, 255}));
    connect(relay1.y[1], switch1.control) annotation(
      Line(points = {{-40, 50}, {-30, 50}, {-30, 20}}, color = {255, 0, 255}));
    connect(relay2.y[1], switch2.control) annotation(
      Line(points = {{70, 50}, {66, 50}, {60, 50}, {60, 20}}, color = {255, 0, 255}));
    annotation(
      Documentation(info = "<html>
  <p>
  This 1 phase sub-model is a basic part of power distribution with a dynamic 1 ph load where sudden fault is cleared by short-time line switched off.
  It contains the following components:
  <ul>
  <li>Transformer and AC voltage sources (6.6 kV and 3.3 kV),</li>
  <li>One healthy and one faulty transmission line each of 30km,</li>
  <li>One dynamic 1 phase load(drawing active and reactive power changing with time),</li>
  <li>Breaker switches and relays to clear the line fault for a certain time,</li>
  <li>Meters to check certain voltage/current/power.</li>
  </ul>
  </p>
  </html>"),
      experiment(StopTime = 0.5, Interval = 2.5e-5),
      Diagram(coordinateSystem(extent = {{-100, -150}, {150, 100}})),
      Icon(coordinateSystem(extent = {{-100, -150}, {150, 100}})),
      __OpenModelica_commandLineOptions = "");
  end SubModel1ph;













  model Submodel3ph "3 phase model having three PV sources feeding to a 3 phase dynamic load having one faulty line"
    extends Modelica.Icons.Example;
    inner PowerSystems.System system(dynType = PowerSystems.Types.Dynamics.FreeInitial)  annotation(
      Placement(visible = true, transformation(extent = {{-160, 100}, {-140, 120}}, rotation = 0)));
    PowerSystems.AC3ph.Nodes.BusBar bus1 annotation(
      Placement(visible = true, transformation(extent = {{-172, -20}, {-152, 0}}, rotation = 0)));
    PowerSystems.AC3ph.Sensors.Psensor sensor1(term_p(v(start = {20e3, 0, 0}))) annotation(
      Placement(visible = true, transformation(extent = {{-152, -20}, {-132, 0}}, rotation = 0)));
    PowerSystems.AC3ph.Nodes.BusBar bus2 annotation(
      Placement(visible = true, transformation(extent = {{154, -20}, {174, 0}}, rotation = 0)));
    PowerSystems.AC3ph.Sensors.Psensor sensor2(term_p(v(start = {20e3, 0, 0}))) annotation(
      Placement(visible = true, transformation(extent = {{154, -20}, {134, 0}}, rotation = 0)));
    PowerSystems.AC3ph.Sensors.Psensor sensor3(term_p(v(start = {20e3, 0, 0}))) annotation(
      Placement(transformation(origin = {0, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    PowerSystems.AC3ph.Sensors.Psensor sensorLoad annotation(
      Placement(visible = true, transformation(origin = {0, -146}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    PowerSystems.AC3ph.Loads.Zload load( S_nom = 1500e6, V_nom = 400000,pq0 = {0.95, 0.2}, use_pq_in = true) annotation(
      Placement(visible = true, transformation(origin = {0, -182}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
    PowerSystems.Blocks.Signals.Transient[2] pq_change(each t_duration = 0.1, s_start = {0.95, 0.2}, s_end = {0.2, 0.1}, each t_change = 2) annotation(
      Placement(visible = true, transformation(extent = {{-40, -192}, {-20, -172}}, rotation = 0)));
    PowerSystems.AC3ph.Transformers.TrafoStray trafo1 annotation(
      Placement(visible = true, transformation(origin = {-184, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerSystems.AC3ph.Transformers.TrafoStray trafo2 annotation(
      Placement(visible = true, transformation(origin = {186, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    PowerSystems.AC3ph.Transformers.TrafoStray trafo3 annotation(
      Placement(visible = true, transformation(origin = {0, 78}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    PowerSystems.AC3ph.Nodes.BusBar bus3 annotation(
      Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  PowerSystems.AC3ph.Nodes.GroundOne grd1 annotation(
      Placement(visible = true, transformation(origin = {-240, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Nodes.GroundOne grd3 annotation(
      Placement(visible = true, transformation(origin = {0, 154}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  PowerSystems.AC3ph.Nodes.GroundOne grd2 annotation(
      Placement(visible = true, transformation(origin = {248, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Lines.PIline line1(len = 10000)  annotation(
      Placement(visible = true, transformation(origin = {-86, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Lines.PIline line3(len = 10000)  annotation(
      Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PowerSystems.AC3ph.Nodes.BusBar bus4 annotation(
      Placement(visible = true, transformation(origin = {2, -66}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PowerSystems.AC3ph.Transformers.TrafoStray trafo4 annotation(
      Placement(visible = true, transformation(origin = {0, -100}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  PowerSystems.AC3ph.Sensors.PVImeter PVImeter2 annotation(
      Placement(visible = true, transformation(origin = {86, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Lines.FaultTline LineF(len = 10000)  annotation(
      Placement(visible = true, transformation(origin = {54, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Breakers.Switch BkrSwitch1(I_nom = 2500, V_nom = 400000)  annotation(
      Placement(visible = true, transformation(origin = {116, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Breakers.Switch BkrSwitch2(I_nom = 2500, V_nom = 400000)  annotation(
      Placement(visible = true, transformation(origin = {26, -40}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
  PowerSystems.AC3ph.Faults.Fault_abc fault_abc(epsG = 1e-5)  annotation(
      Placement(visible = true, transformation(origin = {54, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PowerSystems.Control.Relays.SwitchRelay relay1(t_switch = {0.15, 0.2})  annotation(
      Placement(visible = true, transformation(origin = {116, 32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PowerSystems.Control.Relays.SwitchRelay relay2(t_switch = {0.153, 0.21})  annotation(
      Placement(visible = true, transformation(origin = {60, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  PowerSystems.AC3ph.Sensors.PVImeter PVImeter1 annotation(
      Placement(visible = true, transformation(origin = {-54, -36}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PowerSystems.AC3ph.Sensors.PVImeter PVImeter3 annotation(
      Placement(visible = true, transformation(origin = {0, -24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PowerSystems.AC3ph.Sources.InfBus infBus annotation(
      Placement(visible = true, transformation(origin = {-212, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Sources.InfBus infBus1 annotation(
      Placement(visible = true, transformation(origin = {218, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PowerSystems.AC3ph.Sources.InfBus infBus2 annotation(
      Placement(visible = true, transformation(origin = {2, 122}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PowerSystems.Blocks.Signals.TransientPhasor transPh1 annotation(
      Placement(visible = true, transformation(origin = {-234, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PowerSystems.Blocks.Signals.TransientPhasor transPh11 annotation(
      Placement(visible = true, transformation(origin = {222, 34}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PowerSystems.Blocks.Signals.TransientPhasor transPh12 annotation(
      Placement(visible = true, transformation(origin = {44, 128}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(transPh11.y, infBus1.vPhasor_in) annotation(
      Line(points = {{212, 34}, {212, 34}, {212, 0}, {212, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(transPh1.y, infBus.vPhasor_in) annotation(
      Line(points = {{-224, 24}, {-206, 24}, {-206, 0}, {-206, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(transPh12.y, infBus2.vPhasor_in) annotation(
      Line(points = {{34, 128}, {12, 128}, {12, 116}, {12, 116}}, color = {0, 0, 127}, thickness = 0.5));
    connect(infBus2.term, trafo3.term_p) annotation(
      Line(points = {{2, 112}, {0, 112}, {0, 88}, {0, 88}}, color = {0, 120, 120}));
    connect(grd3.term, infBus2.neutral) annotation(
      Line(points = {{0, 144}, {2, 144}, {2, 132}, {2, 132}}, color = {0, 0, 255}));
    connect(infBus1.term, trafo2.term_p) annotation(
      Line(points = {{208, -10}, {196, -10}, {196, -10}, {196, -10}}, color = {0, 120, 120}));
    connect(grd2.term, infBus1.neutral) annotation(
      Line(points = {{238, -10}, {228, -10}, {228, -10}, {228, -10}}, color = {0, 0, 255}));
    connect(grd1.term, infBus.neutral) annotation(
      Line(points = {{-230, -10}, {-222, -10}, {-222, -10}, {-222, -10}}, color = {0, 0, 255}));
    connect(infBus.term, trafo1.term_p) annotation(
      Line(points = {{-202, -10}, {-194, -10}, {-194, -10}, {-194, -10}}, color = {0, 120, 120}));
    connect(PVImeter3.term_n, bus4.term) annotation(
      Line(points = {{0, -34}, {0, -34}, {0, -66}, {2, -66}}, color = {0, 120, 120}));
    connect(line3.term_n, PVImeter3.term_p) annotation(
      Line(points = {{0, 0}, {0, 0}, {0, -14}, {0, -14}}, color = {0, 120, 120}));
    connect(PVImeter1.term_n, bus4.term) annotation(
      Line(points = {{-54, -46}, {-54, -46}, {-54, -66}, {2, -66}, {2, -66}}, color = {0, 120, 120}));
    connect(line1.term_n, PVImeter1.term_p) annotation(
      Line(points = {{-76, -10}, {-56, -10}, {-56, -10}, {-54, -10}, {-54, -26}, {-54, -26}}, color = {0, 120, 120}));
    connect(relay2.y, BkrSwitch2.control) annotation(
      Line(points = {{50, -40}, {36, -40}, {36, -40}, {36, -40}}, color = {255, 0, 255}, thickness = 0.5));
    connect(relay1.y, BkrSwitch1.control) annotation(
      Line(points = {{116, 22}, {116, 22}, {116, 0}, {116, 0}}, color = {255, 0, 255}, thickness = 0.5));
    connect(BkrSwitch2.term_n, bus4.term) annotation(
      Line(points = {{26, -50}, {26, -50}, {26, -66}, {2, -66}, {2, -66}}, color = {0, 120, 120}));
    connect(LineF.term_p, BkrSwitch2.term_p) annotation(
      Line(points = {{44, -10}, {26, -10}, {26, -30}, {26, -30}, {26, -30}}, color = {0, 120, 120}));
    connect(PVImeter2.term_n, LineF.term_n) annotation(
      Line(points = {{76, -10}, {64, -10}, {64, -10}, {64, -10}}, color = {0, 120, 120}));
    connect(BkrSwitch1.term_n, PVImeter2.term_p) annotation(
      Line(points = {{106, -10}, {96, -10}, {96, -10}, {96, -10}}, color = {0, 120, 120}));
    connect(sensor2.term_n, BkrSwitch1.term_p) annotation(
      Line(points = {{134, -10}, {126, -10}, {126, -10}, {126, -10}}, color = {0, 120, 120}));
    connect(fault_abc.term, LineF.term_f) annotation(
      Line(points = {{54, 16}, {54, 16}, {54, 0}, {54, 0}}, color = {0, 120, 120}));
    connect(trafo4.term_n, sensorLoad.term_p) annotation(
      Line(points = {{0, -110}, {0, -110}, {0, -136}, {0, -136}}, color = {0, 120, 120}));
    connect(bus4.term, trafo4.term_p) annotation(
      Line(points = {{2, -66}, {0, -66}, {0, -90}, {0, -90}}, color = {0, 120, 120}));
    connect(sensorLoad.term_n, load.term) annotation(
      Line(points = {{0, -156}, {0, -172}}, color = {0, 110, 110}));
    connect(pq_change.y, load.pq_in) annotation(
      Line(points = {{-20, -182}, {-10, -182}}, color = {0, 0, 127}));
    connect(sensor1.term_n, line1.term_p) annotation(
      Line(points = {{-132, -10}, {-96, -10}, {-96, -10}, {-96, -10}}, color = {0, 120, 120}));
    connect(sensor3.term_n, line3.term_p) annotation(
      Line(points = {{0, 30}, {0, 30}, {0, 20}, {0, 20}}, color = {0, 120, 120}));
    connect(trafo2.term_n, bus2.term) annotation(
      Line(points = {{176, -10}, {164, -10}, {164, -10}, {164, -10}}, color = {0, 120, 120}));
    connect(bus2.term, sensor2.term_p) annotation(
      Line(points = {{164, -10}, {154, -10}}, color = {0, 110, 110}));
    connect(trafo1.term_n, bus1.term) annotation(
      Line(points = {{-174, -10}, {-162, -10}}, color = {0, 120, 120}));
    connect(bus1.term, sensor1.term_p) annotation(
      Line(points = {{-162, -10}, {-152, -10}}, color = {0, 110, 110}));
    connect(sensor3.term_p, bus3.term) annotation(
      Line(points = {{0, 50}, {0, 50}, {0, 60}, {0, 60}}, color = {0, 120, 120}));
    connect(trafo3.term_n, bus3.term) annotation(
      Line(points = {{0, 68}, {0, 68}, {0, 60}, {0, 60}}, color = {0, 120, 120}));
    annotation(
      experiment(StopTime = 30),
      Diagram(coordinateSystem(extent = {{-250, -200}, {250, 200}}, initialScale = 0.1)),
      Icon(coordinateSystem(extent = {{-250, -200}, {250, 200}})),
      __OpenModelica_commandLineOptions = "");
  end Submodel3ph;





























  annotation(
    uses(PowerFlow(version = "0.3"), PowerSystems(version = "0.6.0"), Buildings(version = "3.0.0"), Modelica(version = "3.2.2")));
end TransmissionDistributionModels;