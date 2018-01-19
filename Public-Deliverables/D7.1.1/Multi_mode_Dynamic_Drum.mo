within ;
package Multi_mode_Dynamic_Drum
  model Multi_mode_Drum

    inner parameter Modelica.SIunits.AbsolutePressure P0=50e5
      "Fluid initial pressure (active if steady_state=false)";
    inner parameter Modelica.SIunits.Volume Veps1=0.01
      "Lower limit for transition to one-phase state";
    inner parameter Modelica.SIunits.Volume Veps2=0.1
      "Upper limit for transition to two-phases state";

    inner parameter Real Vf0=0.5
      "Fraction of initial water volume in the drum (active if steady_state=false)";

    inner parameter Modelica.SIunits.Density p_rhol=0
      "If > 0, fixed fluid density";
    inner parameter Modelica.SIunits.Density p_rhov=0
      "If > 0, fixed fluid density";

    inner Modelica.SIunits.SpecificEnthalpy hlstart
      "Initialization liquid phase specific enthalpy";
    inner Modelica.SIunits.SpecificEnthalpy hvstart
      "Initialization gas phase specific enthalpy";

    inner Modelica.SIunits.AbsolutePressure Plstart
      "Initialization liquid phase pressure";

    inner Modelica.SIunits.Volume Vlstart "Initial liquid phase volume";

    inner Modelica.SIunits.Volume Vvstart "Initial gas phase volume";
    inner Modelica.SIunits.SpecificEnthalpy hl(start=hlstart, fixed=true)
      "Liquid phase specific enthalpy";

    inner Modelica.SIunits.SpecificEnthalpy hv(start=hvstart, fixed=true)
      "Gas phase specific enthalpy";

    inner Real xl "Mass vapor fraction in the liquid phase";
    inner Real xv "Mass vapor fraction in the gas phase";
    inner Real x "Made-up outlet vapor fraction"; /*to solve oscillations issue in only liquid phase*/
    inner Real y "Made-up outlet liquid fraction"; /*to solve oscillations issue in only vapor phase*/

    inner Real x_e "Inlet vapor fraction";
    inner Real x_e2 "Inlet 2 vapor fraction";
    inner Real x_e3 "Inlet 3 vapor fraction";

    inner Modelica.SIunits.MassFlowRate Qevap
      "Evaporation mass flow rate from the liquid phase";
    inner Modelica.SIunits.MassFlowRate Qcond
      "Condensation mass flow rate from the vapor phase";
    inner Modelica.SIunits.Volume Vl(start=Vlstart, fixed=true)
      "Liquid phase volume";

    inner Modelica.SIunits.Volume Vv( start = Vvstart, fixed = true)
      "Vapor phase volume";

    inner Modelica.SIunits.Density rhol "Liquid phase density";

    inner Modelica.SIunits.Density rhov "Gas phase density";

    inner Modelica.SIunits.Density rholsat "Saturated liquid phase density";
    inner Modelica.SIunits.Density rhovsat "Saturated gas phase density";

    inner Modelica.SIunits.SpecificEnthalpy hlsat
      "Saturated liquid phase specific enthalpy";
    inner Modelica.SIunits.SpecificEnthalpy hvsat
      "Saturated gas phase specific enthalpy";

    inner Modelica.SIunits.Temperature Tsat "Saturation temperature";
    inner Modelica.SIunits.AbsolutePressure Pl(start=Plstart, fixed=true)
      "Fluid average pressure";

    inner Modelica.SIunits.Temperature Tl "Liquid phase temperature";
    inner Modelica.SIunits.Temperature Tv "Gas phase temperature";

    inner Integer mode "Current state of the state machine";

    inner Modelica.SIunits.AbsolutePressure Pbottom
      "Pressure at the bottom of the liquid phase";

    inner parameter Real Ccond=0.01 "Condensation coefficient";
    inner parameter Real Cevap=0.09 "Evaporation coefficient";
    inner parameter Real Xlo=0.0025
      "Vapor mass fraction in the liquid phase from which the liquid starts to evaporate";
    inner parameter Real Xvo=0.9975
      "Vapor mass fraction in the gas phase from which the liquid starts to condensate";
    inner parameter Real Kvl=1000
      "Heat exchange coefficient between the liquid and gas phases";
    inner Modelica.SIunits.Power Wvl
      "Thermal power exchanged from the gas phase to the liquid phase";

    inner parameter Boolean steady_state = true
      "true: start from steady state - false: start from (P0, Vl0)";

    inner Modelica.SIunits.SpecificEnthalpy hs "Outlet specific enthalpy";

    inner Modelica.SIunits.SpecificEnthalpy he "Inlet specific enthalpy";

    inner Real xmv "Mass vapor fraction in the ascending tube";

    inner Modelica.SIunits.SpecificEnthalpy hs2 "Outlet specific enthalpy";

    inner Modelica.SIunits.SpecificEnthalpy hd "Downcomer specific enthalpy";

    inner Modelica.SIunits.Position zl(start=1.05) "Liquid level in drum";
    inner Modelica.SIunits.Area Al "Cross sectional area of the liquid phase";
    inner Modelica.SIunits.Angle theta "Angle";
    inner Modelica.SIunits.Area Avl(start=1.0)
      "Heat exchange surface between the liquid and gas phases";

    inner Modelica.SIunits.Area Alp
      "Liquid phase surface on contact with the wall";
    inner Modelica.SIunits.Area Avp
      "Gas phase surface on contact with the wall";
    inner Modelica.SIunits.Area Ape "Wall surface on contact with the outside";

    inner Modelica.SIunits.Power Wpl
      "Thermal power exchanged from the liquid phase to the wall";
    inner Modelica.SIunits.Power Wpv
      "Thermal power exchanged from the gas phase to the wall";
    inner Modelica.SIunits.Power Wpa
      "Thermal power exchanged from the outside to the wall";

    inner Modelica.SIunits.Temperature Tp(start=550) "Wall temperature";
    inner Modelica.SIunits.Temperature Ta "External temperature";

    inner parameter Modelica.SIunits.CoefficientOfHeatTransfer Klp=400
      "Heat exchange coefficient between the liquid phase and the wall";
    inner parameter Modelica.SIunits.CoefficientOfHeatTransfer Kvp=100
      "Heat exchange coefficient between the gas phase and the wall";
    inner parameter Modelica.SIunits.CoefficientOfHeatTransfer Kpa=25
      "Heat exchange coefficient between the wall and the outside";
    inner parameter Modelica.SIunits.Mass Mp=117e3 "Wall mass";
    inner parameter Modelica.SIunits.SpecificHeatCapacity cpp=600
      "Wall specific heat";

    inner parameter Boolean Vertical=true
      "true: vertical cylinder - false: horizontal cylinder";
    inner parameter Modelica.SIunits.Radius R=1.05
      "Radius of the drum cross-sectional area";
    inner parameter Modelica.SIunits.Length L=16.27 "Drum length";

  protected
    inner constant Real pi=Modelica.Constants.pi "pi";
    inner constant Modelica.SIunits.Acceleration g=Modelica.Constants.g_n
      "Gravity constant";
    inner parameter Modelica.SIunits.Volume V=pi*R^2*L "Drum volume";

  public
    Liquid_1 state1
      annotation (Placement(transformation(extent={{-8,46},{12,66}})));

    block Liquid_1 "Single phase liquid (no vapor)"
      outer output Modelica.SIunits.SpecificEnthalpy hl
        "Liquid phase specific enthalpy";
      outer output Modelica.SIunits.MassFlowRate Qevap
        "Evaporation mass flow rate from the liquid phase";
      outer output Real xl "Mass vapor fraction in the liquid phase";
      outer output Real x "Made-up outlet vapor fraction";

      outer output Modelica.SIunits.AbsolutePressure Plstart
        "Initialization liquid phase pressure";

      outer output Modelica.SIunits.SpecificEnthalpy hlstart
        "Initialization liquid phase specific enthalpy";
      outer output Modelica.SIunits.Volume Vlstart
        "Initial liquid phase volume";

      outer output Modelica.SIunits.Volume Vvstart "Initial gas phase volume";

      outer output Modelica.SIunits.Density rhol "Liquid phase density";
      outer output Modelica.SIunits.Density rhov "Gas phase density";

      outer output Modelica.SIunits.Volume Vl "Liquid phase volume";
      outer output Modelica.SIunits.Volume Vv "Vapor phase volume";

      outer output Modelica.SIunits.AbsolutePressure Pl "Liquid phase pressure";

      outer output Modelica.SIunits.Temperature Tl "Liquid phase temperature";
      outer output Modelica.SIunits.Temperature Tv "Gas phase temperature";

      outer Modelica.SIunits.Area Al "Cavity cross-sectional area";

      outer Modelica.SIunits.Volume V "Cavity volume";

      outer Modelica.SIunits.Acceleration g=Modelica.Constants.g_n
        "Gravity constant";

      outer Modelica.SIunits.SpecificEnthalpy hvsat
        "Saturated gas phase specific enthalpy";

      Modelica.SIunits.MassFlowRate BVl
        "Right hand side of the mass balance equation of the liquid phase";
      Modelica.SIunits.MassFlowRate BVv
        "Right hand side of the mass balance equation of the gas phase";
      Modelica.SIunits.MassFlowRate BVl1
        "Right hand side of the mass balance equation of the liquid phase with constraints";
      Modelica.SIunits.MassFlowRate BVv1
        "Right hand side of the mass balance equation of the gas phase with constraints";

      outer Modelica.SIunits.Density p_rhol "If > 0, fixed fluid density";
      outer Modelica.SIunits.Density p_rhov "If > 0, fixed fluid density";

      outer Real Cevap "Evaporation coefficient";
      outer Real Xlo
        "Vapor mass fraction in the liquid phase from which the liquid starts to evaporate";

       Modelica.SIunits.Power BHl
        "Right hand side of the energy balance equation of the liquid phase";

      outer output Integer mode;

      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce1;

      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce2;
      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce3;

      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cs1;
      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cs2;

      outer ThermoSysPro.Thermal.Connectors.ThermalPort Cth;

      outer ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
        prol;

      outer ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
        prod;

      outer output Modelica.SIunits.SpecificEnthalpy hs
        "Outlet specific enthalpy";
      outer output Modelica.SIunits.SpecificEnthalpy he
        "Inlet specific enthalpy";

      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cd
        "Evaporation loop outlet";
      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Cm
        "Evaporation loop inlet";

      outer Real xmv;
      outer Modelica.SIunits.SpecificEnthalpy hlsat
        "Saturated liquid phase specific enthalpy";

      outer output Modelica.SIunits.SpecificEnthalpy hs2
        "Outlet specific enthalpy";

      outer Real Wpl;

      outer Real x_e "Inlet 1 vapor fraction";
      outer Real x_e2 "Inlet 2 vapor fraction";
      outer Real x_e3 "Inlet 3 vapor fraction";

      outer Real rhovsat;
      outer output Real hv;
      outer output Modelica.SIunits.SpecificEnthalpy hd
        "Downcomer specific enthalpy";

    equation
      /* For restart */

      Plstart = Pl;
      hlstart = hl;
      Vlstart = V;
      Vvstart = 0;

      Vl + Vv = V;

      /*Made-up outlet vapor fraction*/

    if noEvent(Cs1.Q>0) then
      x = max(min(1,Qevap/Cs1.Q),0);
    else
      x = 0;
    end if;

      /* Liquid phase mass balance equation */

      BVl = Ce1.Q + Ce2.Q + Ce3.Q - Cd.Q - (1-x)*Cs1.Q - Cs2.Q + Cm.Q - Qevap;

      BVl1 = BVl;

      (prol.ddph*der(Pl) + prol.ddhp*der(hl))*Vl = BVl1;

      /*Onset of bubble*/

      BVv = -x*Cs1.Q + Qevap;
      BVv1 = (if (((Vv <= 0) and (BVv <= 0)) or
                          ((Vv >= V) and (BVv >= 0)))
                          then 0 else BVv);

      rhov*der(Vv) = BVv1;

      /* Liquid phase energy balance equation */

       BHl = - Cs2.Q*(Cs2.h - (hl - Pl/rhol)) - Qevap*(hvsat - (hl - Pl/rhol)) + Ce1.Q*(Ce1.h - (hl - Pl/rhol))
       + Ce2.Q*(Ce2.h - (hl - Pl/rhol)) + Ce3.Q*(Ce3.h - (hl - Pl/rhol)) - Cd.Q*(Cd.h - (hl - Pl/rhol)) - (1-x)*Cs1.Q*(Cs1.h - (hl - Pl/rhol))
       + Cm.Q*(Cm.h - (hl - Pl/rhol)) - Wpl + Cth.W;

       Vl*((Pl/rhol*prol.ddph - 1)*der(Pl) + (Pl/rhol*prol.ddhp + rhol)*der(hl)) = BHl;

      /* Evaporation mass flow rates */

      Qevap = if (xl > Xlo) then Cevap*rhol*Vl*(xl - Xlo) else 0;

      /* Fluid thermodynamic properties*/

      Tl = prol.T;

      if (p_rhol > 0) then
        rhol = p_rhol;
      else
        rhol = prol.d;
      end if;

      xl = prol.x;

      Tv = Tl;

      rhov = rhovsat;

      hv = hvsat;

     /* Connectors*/

      hs = hvsat*x +hl*(1-x);
      he = hl;
      hs2 = hl;
      hd = hl;

     /* Mode*/

      mode = 1;

       annotation (
        Icon(graphics={Text(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              textString="%name")}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}),
                graphics={Text(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              textString="%stateName",
              fontSize=10)}),
        __Dymola_state=true,
        showDiagram=true,
        singleInstance=true);
    end Liquid_1;

    block Vapor_2 "Single phase vapor (no liquid)"
      outer output Modelica.SIunits.SpecificEnthalpy hv
        "Gas phase specific enthalpy";
      outer output Modelica.SIunits.MassFlowRate Qcond
        "Condensation mass flow rate from the vapor phase";
      outer output Real xv "Mass vapor fraction in the gas phase";
      outer output Real y "Made-up outlet liquid fraction";

      outer output Modelica.SIunits.AbsolutePressure Plstart
        "Initialization liquid phase pressure";

      outer output Modelica.SIunits.SpecificEnthalpy hvstart
        "Initialization gas phase specific enthalpy";
      outer output Modelica.SIunits.Volume Vlstart
        "Initial liquid phase volume";

      outer output Modelica.SIunits.Volume Vvstart "Initial gas phase volume";

      outer output Modelica.SIunits.Density rhol "Liquid phase density";
      outer output Modelica.SIunits.Density rhov "Gas phase density";
      outer output Modelica.SIunits.Volume Vl "Liquid phase volume";
      outer output Modelica.SIunits.Volume Vv "Vapor phase volume";

      outer output Modelica.SIunits.AbsolutePressure Pl "Liquid phase pressure";

      outer output Modelica.SIunits.Temperature Tv "Gas phase temperature";
      outer output Modelica.SIunits.Temperature Tl "Liquid phase temperature";

      outer Modelica.SIunits.Area Al "Cavity cross-sectional area";
      outer Modelica.SIunits.Acceleration g=Modelica.Constants.g_n
        "Gravity constant";

      outer Modelica.SIunits.Volume V "Cavity volume";

      outer Modelica.SIunits.SpecificEnthalpy hlsat
        "Saturated liquid phase specific enthalpy";

      Modelica.SIunits.MassFlowRate BVl
        "Right hand side of the mass balance equation of the liquid phase";
      Modelica.SIunits.MassFlowRate BVv
        "Right hand side of the mass balance equation of the gas phase";
      Modelica.SIunits.MassFlowRate BVl1
        "Right hand side of the mass balance equation of the liquid phase with constraints";
      Modelica.SIunits.MassFlowRate BVv1
        "Right hand side of the mass balance equation of the gas phase with constraints";

      outer Modelica.SIunits.Density p_rhol "If > 0, fixed fluid density";
      outer Modelica.SIunits.Density p_rhov "If > 0, fixed fluid density";

      outer Real Ccond "Condensation coefficient";
      outer Real Xvo
        "Vapor mass fraction in the gas phase from which the liquid starts to condensate";

      Modelica.SIunits.Power BHv
        "Right hand side of the energy balance equation of the gas phase";

      outer output Integer mode;

      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce1;

      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce2;
      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce3;

      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cs1;
      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cs2;

      outer ThermoSysPro.Thermal.Connectors.ThermalPort Cth;

      outer ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
        prov;

      outer output Modelica.SIunits.SpecificEnthalpy hs
        "Outlet specific enthalpy";
      outer output Modelica.SIunits.SpecificEnthalpy he
        "Inlet specific enthalpy";

      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cd
        "Evaporation loop outlet";
      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Cm
        "Evaporation loop inlet";

      outer Real xmv;

      outer Modelica.SIunits.SpecificEnthalpy hvsat
        "Saturated gas phase specific enthalpy";

      outer output Modelica.SIunits.SpecificEnthalpy hs2
        "Outlet specific enthalpy";

      outer Real Wpv;

      outer Real x_e "Inlet 1 vapor fraction";
      outer Real x_e2 "Inlet 2 vapor fraction";
      outer Real x_e3 "Inlet 3 vapor fraction";

      outer Real rholsat;
      outer output Real hl;
      outer output Modelica.SIunits.SpecificEnthalpy hd
        "Downcomer specific enthalpy";

    equation
      /* For restart*/

      Plstart = Pl;
      hvstart = hv;
      Vlstart = 0;
      Vvstart = V;

      Vl + Vv = V;

      /*Made-up outlet liquid fraction*/

    if noEvent(Cs2.Q + Cd.Q >0) then
      y = max(min(1,Qcond/(Cs2.Q + Cd.Q)),0);
    else
      y = 0;
    end if;

      /* Onset of droplet */

      BVl = Qcond - y*(Cs2.Q + Cd.Q);
      BVl1 = (if (((Vl <= 0) and (BVl <= 0)) or
                          ((Vl >= V) and (BVl >= 0)))
                          then 0 else BVl);
      rhol*der(Vl) = BVl1;

      /* Vapor phase mass balance equation */

      BVv = Ce1.Q + Ce2.Q + Ce3.Q - Cs1.Q - (1-y)*(Cs2.Q + Cd.Q) - Qcond + Cm.Q;

      BVv1 = BVv;

      (prov.ddph*der(Pl) + prov.ddhp*der(hv))*Vv = BVv1;

      /* Gas phase energy balance equation */

      BHv = Cm.Q*( Cm.h - (hv - Pl/rhov)) - (1-y)*Cd.Q*(Cd.h - (hv -Pl/rhov))
      - Cs1.Q*(Cs1.h - (hv - Pl/rhov)) - (1-y)*Cs2.Q*(Cs2.h - (hv - Pl/rhov)) - Qcond*(hlsat - (hv - Pl/rhov)) + Ce1.Q*(Ce1.h - (hv-Pl/rhov))
      + Ce2.Q*(Ce2.h - (hv-Pl/rhov)) + Ce3.Q*(Ce3.h - (hv - Pl/rhov)) - Wpv + Cth.W;

      Vv*((Pl/rhov*prov.ddph - 1)*der(Pl) + (Pl/rhov*prov.ddhp + rhov)*der(hv)) = BHv;

      /* Condensation mass flow rates */

     Qcond = if (xv < Xvo) then Ccond*rhov*Vv*(Xvo - xv) else 0;

     /*Fluid thermodynamic properties*/

      xv = prov.x;

      Tl = Tv;

       rhol = rholsat;
       Tv = prov.T;

       if (p_rhov > 0) then
        rhov = p_rhov;
      else
        rhov = prov.d;
      end if;

      hl = hlsat;

      /*Connectors*/

      he = hv;
      hs = hv;
      hs2 = y*hlsat + (1-y)*hv;
      hd = hs2;

      /*Mode*/

      mode = 2;
      annotation (
        Icon(graphics={Text(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              textString="%name")}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}),
                graphics={Text(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              textString="%stateName",
              fontSize=10)}),
        __Dymola_state=true,
        showDiagram=true,
        singleInstance=true);
    end Vapor_2;
    Vapor_2 state2
      annotation (Placement(transformation(extent={{-10,-68},{10,-48}})));
    Two_phases_3 state3
      annotation (Placement(transformation(extent={{46,-10},{66,10}})));
    ThermoSysPro.Properties.WaterSteam.Common.PropThermoSat
      lsat annotation (Placement(transformation(extent={{-92,-68},{-72,-48}})));
    ThermoSysPro.Properties.WaterSteam.Common.PropThermoSat
      vsat annotation (Placement(transformation(extent={{-60,-68},{-40,-48}})));
    inner ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce1
      annotation (Placement(transformation(extent={{-120,66},{-100,86}}),
          iconTransformation(extent={{-120,66},{-100,86}})));
    inner ThermoSysPro.WaterSteam.Connectors.FluidOutlet                        Cs1
      annotation (Placement(transformation(extent={{90,70},{110,90}}),
          iconTransformation(extent={{90,70},{110,90}})));
    ThermoSysPro.InstrumentationAndControl.Connectors.OutputReal
      yLevel "Water level" annotation (Placement(transformation(extent={{100,-10},
              {120,10}}), iconTransformation(extent={{100,-10},{120,10}})));

    inner ThermoSysPro.Thermal.Connectors.ThermalPort Cth
      annotation (Placement(transformation(extent={{-10,16},{10,36}})));
    inner ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      prol annotation (Placement(transformation(extent={{-92,12},{-72,32}})));
    inner ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      prov annotation (Placement(transformation(extent={{-62,12},{-40,32}})));
    inner ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      prod annotation (Placement(transformation(extent={{-94,-40},{-72,-20}})));
    inner ThermoSysPro.WaterSteam.Connectors.FluidInlet Cm
      "Evaporation loop inlet"
      annotation (Placement(transformation(extent={{86,-96},{106,-76}})));
    inner ThermoSysPro.WaterSteam.Connectors.FluidOutlet                        Cd
      "Evaporation loop outlet"
      annotation (Placement(transformation(extent={{-116,-98},{-96,-78}})));
    inner ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      prom annotation (Placement(transformation(extent={{-62,-42},{-40,-22}})));
    inner ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce2
      annotation (Placement(transformation(extent={{-120,12},{-100,32}})));
    inner ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce3
      annotation (Placement(transformation(extent={{-120,-46},{-100,-26}})));

    inner ThermoSysPro.WaterSteam.Connectors.FluidOutlet                        Cs2
      annotation (Placement(transformation(extent={{88,-58},{108,-38}})));

    ThermoSysPro.Thermal.Connectors.ThermalPort Cex
      annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      proe1 annotation (Placement(transformation(extent={{-88,68},{-68,88}})));
    ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      proe2 annotation (Placement(transformation(extent={{-56,68},{-36,88}})));
    ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
      proe3 annotation (Placement(transformation(extent={{-88,42},{-68,62}})));
  initial equation

    der(Tp) = 0;

  equation
    /* Liquid volume */

    if Vertical then
       theta = 1;
       Al = pi*R^2;
       Vl = Al*zl;
       Avl = Al;
    else
       theta = Modelica.Math.asin(max(-0.9999,min(0.9999,(R - zl)/R)));
       Al = (pi/2 - theta)*R^2 - R*(R - zl)*Modelica.Math.cos(theta);
       Vl = Al*L;
       Avl = 2*R*Modelica.Math.cos(theta)*L;
    end if;

    /* Liquid level */
    yLevel.signal = zl;

    /* Liquid surface and vapor surface on contact with wall */

    Alp = if Vertical then 2*pi*R*zl + Al else (pi - 2*theta)*R*L + 2*Al;
    Avp = if Vertical then 2*pi*R*(L - zl) + Al else (pi + 2*theta)*R*L + 2*Al;

    /* Wall surface on contact with the outside */
    Ape = Alp + Avp;

    /* Energy balance equation at the wall */
    Mp*cpp*der(Tp) = Wpl + Wpv + Wpa;

    /* Heat exchange between the liquid phase and the wall */
    Wpl = Klp*Alp*(Tl - Tp);

    /* Heat exchange between the gas phase and the wall */
    Wpv = Kvp*Avp*(Tv - Tp);

    /* Heat exchange between the wall and the outside */
    Wpa = Kpa*Ape*(Ta - Tp);

    /* Fluid thermodynamic properties*/
    (lsat,vsat) =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_sat_P(
       Pl);

    Tsat = lsat.T;

    hlsat = lsat.h;
    hvsat = vsat.h;

    rholsat = lsat.rho;
    rhovsat = vsat.rho;

    prol =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_Ph(
       Pl, hl);
  //   prolt = ThermoSysPro.Properties.WaterSteam.IF97.Water_PT(Pl, Tl);

    prov =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_Ph(
       Pl, hv);
  //   provt = ThermoSysPro.Properties.WaterSteam.IF97.Water_PT(Pl, Tv, 2);

    prod =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_Ph(
       Pbottom, Cd.h);

    proe1 =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_Ph(
       Ce1.P, Ce1.h);
    proe2 =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_Ph(
       Ce2.P, Ce2.h);
    proe3 =
      ThermoSysPro.Properties.WaterSteam.IF97_packages.IF97_wAJ.Water_Ph(
       Ce3.P, Ce3.h);

     x_e = proe1.x;
     x_e2 = proe2.x;
     x_e3 = proe3.x;

    prom =
      ThermoSysPro.Properties.WaterSteam.IF97.Water_Ph(Pl,
      Cm.h);

    xmv = if noEvent(Cm.Q > 0) then prom.x else 0;

    /* Pressure at the bottom of the cavity */

    Pbottom = Pl + prod.d*g*zl;

    /* Unconnected connectors */

    if (cardinality(Cd) == 0) then
      Cd.Q = 0;
      Cd.h = 1.e5;
      Cd.a = true;
    end if;

    if (cardinality(Cm) == 0) then
      Cm.Q = 0;
      Cm.h = 1.e5;
      Cm.b = true;
    end if;

    if (cardinality(Ce1) == 0) then
      Ce1.Q = 0;
      Ce1.h = 1.e5;
      Ce1.b = true;
    end if;

    if (cardinality(Ce2) == 0) then
      Ce2.Q = 0;
      Ce2.h = 1.e5;
      Ce2.b = true;
    end if;

    if (cardinality(Ce3) == 0) then
      Ce3.Q = 0;
      Ce3.h = 1.e5;
      Ce3.b = true;
    end if;

    if (cardinality(Cs1) == 0) then
      Cs1.Q = 0;
      Cs1.h = 1.e5;
      Cs1.a = true;
    end if;

    if (cardinality(Cs2) == 0) then
      Cs2.Q = 0;
      Cs2.h = 1.e5;
      Cs2.a = true;
    end if;

  /*Connectors*/

    Ce1.P = Pl;
    Ce2.P = Pl;
    Ce3.P = Pl;

    Ce1.h_vol = he;
    Ce2.h_vol = he;
    Ce3.h_vol = he;

    Cs1.P = Pl;
    Cs2.P = Pl;

    Cs1.h_vol = hs;
    Cs2.h_vol = hs2;

    Cd.P = Pbottom;
    Cm.P = Pl;

    Cm.h_vol = he;
    Cd.h_vol = hd;

    Cth.T = Tl;

    Cex.T = Ta;
    Cex.W = Wpa;

  public
    block Two_phases_3 "Two-phase (liquid and vapor)"

      outer output Modelica.SIunits.SpecificEnthalpy hl
        "Liquid phase specific enthalpy";
      outer output Modelica.SIunits.SpecificEnthalpy hv
        "Gas phase specific enthalpy";
      outer output Modelica.SIunits.MassFlowRate Qevap
        "Evaporation mass flow rate from the liquid phase";
      outer output Modelica.SIunits.MassFlowRate Qcond
        "Condensation mass flow rate from the vapor phase";
      outer output Real xl "Mass vapor fraction in the liquid phase";
      outer output Real xv "Mass vapor fraction in the gas phase";

      outer output Modelica.SIunits.AbsolutePressure Plstart
        "Initialization liquid phase pressure";

      outer output Modelica.SIunits.SpecificEnthalpy hlstart
        "Initialization liquid phase specific enthalpy";
      outer output Modelica.SIunits.SpecificEnthalpy hvstart
        "Initialization gas phase specific enthalpy";
      outer output Modelica.SIunits.Volume Vlstart
        "Initial liquid phase volume";

      outer output Modelica.SIunits.Volume Vvstart "Initial gas phase volume";

      outer output Modelica.SIunits.Density rhol "Liquid phase density";
      outer output Modelica.SIunits.Density rhov "Gas phase density";

      outer output Modelica.SIunits.Volume Vl "Liquid phase volume";

      outer output Modelica.SIunits.Volume Vv "Vapor phase volume";

      outer output Modelica.SIunits.AbsolutePressure Pl "Liquid phase pressure";

      outer output Modelica.SIunits.Temperature Tv "Gas phase temperature";
      outer output Modelica.SIunits.Temperature Tl "Liquid phase temperature";

      outer Modelica.SIunits.Area Avl
        "Heat exchange surface between the liquid and gas phases";

      outer Modelica.SIunits.Volume V "Cavity volume";

      outer Modelica.SIunits.Acceleration g=Modelica.Constants.g_n
        "Gravity constant";

      outer Modelica.SIunits.SpecificEnthalpy hlsat
        "Saturated liquid phase specific enthalpy";
      outer Modelica.SIunits.SpecificEnthalpy hvsat
        "Saturated gas phase specific enthalpy";

      Modelica.SIunits.MassFlowRate BVl
        "Right hand side of the mass balance equation of the liquid phase";
      Modelica.SIunits.MassFlowRate BVv
        "Right hand side of the mass balance equation of the gas phase";
      Modelica.SIunits.MassFlowRate BVl1
        "Right hand side of the mass balance equation of the liquid phase with constraints";
      Modelica.SIunits.MassFlowRate BVv1
        "Right hand side of the mass balance equation of the gas phase with constraints";

      outer Modelica.SIunits.Density p_rhol "If > 0, fixed fluid density";
      outer Modelica.SIunits.Density p_rhov "If > 0, fixed fluid density";

      outer Real Ccond "Condensation coefficient";
      outer Real Cevap "Evaporation coefficient";
      outer Real Xlo
        "Vapor mass fraction in the liquid phase from which the liquid starts to evaporate";
      outer Real Xvo
        "Vapor mass fraction in the gas phase from which the liquid starts to condensate";

      Modelica.SIunits.Power BHl
        "Right hand side of the energy balance equation of the liquid phase";
      Modelica.SIunits.Power BHv
        "Right hand side of the energy balance equation of the gas phase";
      outer Real Kvl=1000
        "Heat exchange coefficient between the liquid and gas phases";
      outer output Modelica.SIunits.Power Wvl
        "Thermal power exchanged from the gas phase to the liquid phase";

      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce1;

      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cs1;
      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cs2;

      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce2;
      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Ce3;

      outer ThermoSysPro.Thermal.Connectors.ThermalPort Cth;

      outer ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
        prol;

      outer ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
        prov;

      outer ThermoSysPro.Properties.WaterSteam.Common.ThermoProperties_ph
        prod;

      outer output Integer mode;

      outer output Modelica.SIunits.SpecificEnthalpy hs
        "Outlet specific enthalpy";
      outer output Modelica.SIunits.SpecificEnthalpy he
        "Inlet specific enthalpy";

      outer ThermoSysPro.WaterSteam.Connectors.FluidOutlet Cd
        "Evaporation loop outlet";
      outer ThermoSysPro.WaterSteam.Connectors.FluidInlet Cm
        "Evaporation loop inlet";

      outer Real xmv;

      outer output Modelica.SIunits.SpecificEnthalpy hs2
        "Outlet specific enthalpy";

      outer Real Wpv;
      outer Real Wpl;

      outer Real x_e "Inlet 1 vapor fraction";
      outer Real x_e2 "Inlet 2 vapor fraction";
      outer Real x_e3 "Inlet 3 vapor fraction";
      outer Boolean steady_state;
      outer Real Vf0;
      outer Real P0;
      outer output Modelica.SIunits.SpecificEnthalpy hd
        "Downcomer specific enthalpy";

    initial equation

          if steady_state then
          der(hl) = 0;
          der(hv) = 0;
          der(Vl) = 0;

          der(Pl) = 0;

         else
           hl = hlsat;
           hv = hvsat;
           Vl = Vf0*V;
           Pl = P0;

         end if;

    equation
      /*For restart*/

      Plstart = Pl;
      hlstart = hl;
      hvstart = hv;
      Vlstart = Vl;
      Vvstart = Vv;

      Vl + Vv = V;

      /* Liquid phase mass balance equation */

      BVl = (1 - x_e)*Ce1.Q + (1 - x_e2)*Ce2.Q + (1 - x_e3)*Ce3.Q - Qevap + Qcond + (1-xmv)*Cm.Q - Cd.Q - Cs2.Q;

      BVl1 = BVl;

      rhol*der(Vl) + (prol.ddph*der(Pl) + prol.ddhp*der(hl))*Vl = BVl1;

      /* Vapor phase mass balance equation */

      BVv = x_e*Ce1.Q + x_e2*Ce2.Q + x_e3*Ce3.Q - Cs1.Q + Qevap - Qcond + xmv*Cm.Q;

      BVv1 = BVv;

      rhov*der(Vv) + (prov.ddph*der(Pl) + prov.ddhp*der(hv))*Vv = BVv1;

     /* Liquid phase energy balance equation */

      BHl = Qcond*(hlsat - (hl - Pl/rhol)) - Qevap*(hvsat - (hl - Pl/rhol)) + (1 - x_e)*Ce1.Q*((if (x_e > 0) then hlsat else Ce1.h) - (hl - Pl/rhol))
        + (1 - x_e2)*Ce2.Q*((if (x_e > 0) then hlsat else Ce2.h) - (hl - Pl/rhol)) + (1 - x_e3)*Ce3.Q*((if (x_e > 0) then hlsat else Ce3.h) - (hl - Pl/rhol)) - Cd.Q*(Cd.h - (hl - Pl/rhol))
      + (1 - xmv)*Cm.Q*((if (xmv > 0) then hlsat else Cm.h) - (hl - Pl/rhol)) + Wvl + Cth.W - Cs2.Q*(Cs2.h - (hl - Pl/rhol)) - Wpl;

      Vl*((Pl/rhol*prol.ddph - 1)*der(Pl) + (Pl/rhol*prol.ddhp + rhol)*der(hl)) = BHl;

      /* Gas phase energy balance equation */

      BHv = xmv*Cm.Q*((if (xmv < 1) then hvsat else Cm.h) - (hv - Pl/rhov)) - Cs1.Q*(Cs1.h - (hv - Pl/rhov))
      + Qevap*(hvsat - (hv - Pl/rhov)) - Qcond*(hlsat - (hv - Pl/rhov)) + x_e*Ce1.Q*((if (x_e < 1) then hvsat else Ce1.h) - (hv-Pl/rhov))
      + x_e2*Ce2.Q*((if (x_e2 < 1) then hvsat else Ce2.h) - (hv-Pl/rhov)) + x_e3*Ce3.Q*((if (x_e3 < 1) then hvsat else Ce3.h) - (hv-Pl/rhov)) - Wvl - Wpv;

      Vv*((Pl/rhov*prov.ddph - 1)*der(Pl) + (Pl/rhov*prov.ddhp + rhov)*der(hv)) = BHv;

      /* Heat exchange between liquid and gas phases */

      Wvl = Kvl*Avl*(Tv - Tl);

      /* Condensation and evaporation mass flow rates */

      Qcond = if (xv < Xvo) then Ccond*rhov*Vv*(Xvo - xv) else 0;
      Qevap = if (xl > Xlo) then Cevap*rhol*Vl*(xl - Xlo) else 0;

      /*Fluid thermodynamic properties*/

      xl = prol.x;
      xv = prov.x;

      Tl = prol.T;

      if (p_rhol > 0) then
        rhol = p_rhol;
      else
        rhol = prol.d;
      end if;

      Tv = prov.T;

      if (p_rhov > 0) then
        rhov = p_rhov;
      else
        rhov = prov.d;
      end if;

      /*Connectors*/

      he = hl;
      hs = hv;
      hs2 = hl;
    //   hd = noEvent(min(hlsat, hl));
    hd = hl;
      /*Mode*/

      mode = 3;
       annotation (
        Icon(graphics={Text(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              textString="%name")}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}),
                graphics={Text(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              textString="%stateName",
              fontSize=10)}),
        __Dymola_state=true,
        showDiagram=true,
        singleInstance=true);
    end Two_phases_3;
  equation
    transition(
        state1,
        state3,Vv > Veps2,
        immediate=true,
        reset=true,
        synchronize=false,
        priority=1) annotation (Line(
        points={{14,56},{45,11}},
        color={175,175,175},
        thickness=0.25,
        smooth=Smooth.Bezier), Text(
        string="%condition",
        extent={{-16,16},{-16,22}},
        lineColor={95,95,95},
        fontSize=10,
        textStyle={TextStyle.Bold},
        horizontalAlignment=TextAlignment.Right));
    transition(
        state2,
        state3,Vl > Veps2,
        immediate=true,
        reset=true,
        synchronize=false,
        priority=1) annotation (Line(
        points={{12,-60},{45,-11}},
        color={175,175,175},
        thickness=0.25,
        smooth=Smooth.Bezier), Text(
        string="%condition",
        extent={{-34,-16},{-34,-22}},
        lineColor={95,95,95},
        fontSize=10,
        textStyle={TextStyle.Bold},
        horizontalAlignment=TextAlignment.Left));
    transition(
        state3,
        state1,Vv <= Veps1,
        immediate=true,
        reset=true,
        synchronize=false,
        priority=1) annotation (Line(
        points={{56,12},{56,80},{0,80},{0,68}},
        color={175,175,175},
        thickness=0.25,
        smooth=Smooth.Bezier), Text(
        string="%condition",
        extent={{-4,4},{-4,10}},
        lineColor={95,95,95},
        fontSize=10,
        textStyle={TextStyle.Bold},
        horizontalAlignment=TextAlignment.Right));
    transition(
        state3,
        state2,Vl <= Veps1,
        priority=2,
        immediate=true,
        reset=true,
        synchronize=false) annotation (Line(
        points={{56,-12},{56,-82},{0,-82},{0,-70}},
        color={175,175,175},
        thickness=0.25,
        smooth=Smooth.Bezier), Text(
        string="%condition",
        extent={{4,-4},{4,-10}},
        lineColor={95,95,95},
        fontSize=10,
        textStyle={TextStyle.Bold},
        horizontalAlignment=TextAlignment.Left));
    initialState(state3) annotation (Line(
        points={{68,0},{84,0},{88,0}},
        color={175,175,175},
        thickness=0.25,
        smooth=Smooth.Bezier,
        arrow={Arrow.Filled,Arrow.None}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}})),           Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Ellipse(
            extent={{48,0},{-48,96}},
            lineColor={0,0,255},
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{0,48},{48,48},{48,56},{46,66},{40,76},{34,84},{24,92},{14,96},
                {4,98},{0,98},{-6,98},{-14,96},{-24,92},{-34,84},{-42,74},{-46,64},
                {-48,56},{-48,48},{0,48}},
            lineColor={0,0,255},
            smooth=Smooth.None,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{94,-98},{4,-8}},
            lineColor={0,0,255},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-4,-98},{-94,-8}},
            lineColor={0,0,255},
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid)}));
  end Multi_mode_Drum;

  model Test_steady

    Multi_mode_Drum Drum(
      zl(fixed=true, start=1.05),
      xl(start=0),
      xv(start=1),
      Avl(start=3.46),
      Tl(start=592, displayUnit="K"),
      Tv(start=604, displayUnit="K"),
      steady_state=true,
      Vertical=false,
      hl(start=1.45445e6, fixed=false),
      hv(start=2.65793e+06, fixed=false),
      hd(start=1.45445e6),
      hs(start=2.65793e+06),
      he(start=1.45445e6),
      Vv(start=25, fixed=false),
      P0=13000000,
      rhol(start=675),
      rhov(start=78),
      Pl(start=13000000, fixed=true),
      Tp(start=600.15, fixed=false)) annotation (extent=[-61,16; 1,78],
        Placement(transformation(extent={{-55,16},{7,78}})));
    ThermoSysPro.WaterSteam.PressureLosses.ControlValve FeedwaterValve(
      Cv(start=100),
      C1(
        P(start=130e5),
        h_vol(start=1400e3),
        Q(start=75),
        h(start=1400e3)),
      Cvmax(fixed=false) = 200) annotation (extent=[-120,74; -100,94]);
    ThermoSysPro.WaterSteam.PressureLosses.ControlValve SteamValve(
      Pm(start=132e5),
      Cvmax(fixed=true) = 50000,
      Cv(start=25000)) annotation (extent=[40,74; 60,94]);
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Constante
      ConsigneNiveauBallon(k(fixed=true) = 0.5)
      annotation (extent=[-160,120; -140,140]);
    ThermoSysPro.WaterSteam.HeatExchangers.DynamicTwoPhaseFlowPipe
      TubeEcranBoucleEvaporatoire(
      T0=fill(400, 10),
      heb(start={10409,10268,10127,9985,9842,9698,9552,9406,9258,9111}),
      advection=false,
      z2=10,
      simplified_dynamic_energy_balance=false,
      P(start={13000000,13000000,13000000,13000000,13000000,13000000,13000000,
            13000000,13000000,13000000,13000000,13000000}),
      D=0.03,
      ntubes=1400,
      h(start={1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,
            1400e3,1400e3,1400e3}),
      L=20,
      Q(start=fill(150, 11))) annotation (extent=[16,-38; -4,-18], rotation=90);
    ThermoSysPro.Thermal.BoundaryConditions.HeatSource SourceC3(
      option_temperature=2,
      T0={290,290,290,290,290,290,290,290,290,290},
      W0={1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7})
      annotation (extent=[26,-38; 46,-18], rotation=-90);
    ThermoSysPro.Thermal.HeatTransfer.HeatExchangerWall heatExchangerWall(Ns=10, L=
          20) annotation (extent=[8,-38; 28,-18], rotation=90);
    ThermoSysPro.WaterSteam.PressureLosses.LumpedStraightPipe
      lumpedStraightPipe(
      L=20,
      z1=20,
      lambda(fixed=false) = 0.03,
      C1(P(start=130e5)),
      mode=1,
      Q(fixed=true, start=130))
      annotation (extent=[-76,-18; -56,-38], rotation=90);
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Constante
      ConsigneNiveauBallon1(k(fixed=true) = 0.5)
      annotation (extent=[0,120; 20,140]);
    ThermoSysPro.WaterSteam.BoundaryConditions.SourceP sourceP(
      h0=1400000,
      P0(fixed=true) = 13200000,
      option_temperature=2) annotation (extent=[-196,68; -176,88]);
    ThermoSysPro.WaterSteam.BoundaryConditions.SinkQ sinkQ(Q0(fixed=
            false) = 75) annotation (extent=[126,68; 146,88]);
  equation
    connect(heatExchangerWall.WT1, SourceC3.C) annotation (points=[20,-28; 26.2,
          -28], style(color=46, rgbcolor={191,95,0}));
    connect(TubeEcranBoucleEvaporatoire.CTh, heatExchangerWall.WT2) annotation (
        points=[9,-28; 16,-28],    style(color=46, rgbcolor={191,95,0}));
    connect(ConsigneNiveauBallon.y, FeedwaterValve.Ouv)
      annotation (points=[-139,130; -110,130; -110,95], style(smooth=0));
    connect(sourceP.C, FeedwaterValve.C1)  annotation (points=[-176,78; -120,78],
        style(
        color=3,
        rgbcolor={0,0,255},
        smooth=0));
    connect(SteamValve.C2, sinkQ.C) annotation (points=[60,78; 126,78],  style(
        color=3,
        rgbcolor={0,0,255},
        smooth=0));
    connect(TubeEcranBoucleEvaporatoire.C1, lumpedStraightPipe.C2)
      annotation (points=[6,-38; 6,-70; -66,-70; -66,-38]);
    connect(ConsigneNiveauBallon1.y, SteamValve.Ouv)
      annotation (points=[21,130; 50,130; 50,95], style(smooth=0));
    connect(FeedwaterValve.C2, Drum.Ce1) annotation (Line(points={{-100,78},{
            -74,78},{-74,70.56},{-58.1,70.56}}, color={0,0,255}));
    connect(Drum.Cs1, SteamValve.C1) annotation (Line(points={{7,71.8},{22,71.8},
            {28,71.8},{28,78},{40,78}}, color={0,0,255}));
    connect(Drum.Cm, TubeEcranBoucleEvaporatoire.C2) annotation (Line(points={{
            5.76,20.34},{5.76,0.17},{6,0.17},{6,-18}}, color={0,0,0}));
    connect(Drum.Cd, lumpedStraightPipe.C1) annotation (Line(points={{-56.86,
            19.72},{-66,19.72},{-66,-18}}, color={0,0,255}));
    annotation (
      Coordsys(
        extent=[-200, -100; 140, 200],
        grid=[2, 2],
        component=[20, 20]),
      Window(
        x=0.43,
        y=0,
        width=0.57,
        height=0.63),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
              140,200}}), graphics={Text(
            extent={{-192,190},{-48,162}},
            lineColor={28,108,200},
            textString="No script, just steady state")}));
  end Test_steady;

  model Cas_test

    ThermoSysPro.WaterSteam.BoundaryConditions.SourceQ sourceQ(Q0=0, h0=
          8e5)
      annotation (Placement(transformation(extent={{-126,-8},{-106,12}})));
    ThermoSysPro.WaterSteam.BoundaryConditions.SinkQ sinkQ(Q0=0, h0=
          4e6) annotation (Placement(transformation(extent={{96,2},{116,22}})));
    ThermoSysPro.WaterSteam.PressureLosses.SingularPressureLoss
      controlValve
      annotation (Placement(transformation(extent={{-74,-8},{-54,12}})));
    ThermoSysPro.WaterSteam.PressureLosses.SingularPressureLoss
      steamvalve annotation (Placement(transformation(extent={{42,8},{62,28}})));
    ThermoSysPro.WaterSteam.BoundaryConditions.SinkQ sinkQ1(Q0=0, h0=
          0.8e6)
      annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
    ThermoSysPro.WaterSteam.PressureLosses.SingularPressureLoss
      controlValve2
      annotation (Placement(transformation(extent={{34,-16},{54,4}})));
    Multi_mode_Drum drum__v1(
      R=1,
      L=0.5,
      p_rhov=0,
      steady_state=false,
      Veps2=0.1,
      P0=100000,
      Vf0=0.5) annotation (Placement(transformation(extent={{-26,-8},{-6,12}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Trapezoide
      Vapor_empty(
      periode=500,
      n=1,
      largeur=5,
      offset=0,
      rising=10,
      falling=10,
      amplitude=1,
      startTime=50)
      annotation (Placement(transformation(extent={{-160,84},{-140,104}})));
    ThermoSysPro.Thermal.BoundaryConditions.HeatSource heatSource(
        option_temperature=2)
      annotation (Placement(transformation(extent={{-36,56},{-16,76}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Trapezoide
      Vaporization_not_all_liquid_water(
      periode=500,
      n=1,
      startTime=200,
      rising=50,
      falling=50,
      largeur=150,
      offset=0,
      amplitude=5e6)
      annotation (Placement(transformation(extent={{-156,54},{-136,74}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Trapezoide
      Liquid_empty(
      periode=500,
      n=1,
      offset=0,
      rising=20,
      falling=20,
      largeur=220,
      amplitude=3.5,
      startTime=350)
      annotation (Placement(transformation(extent={{-154,-96},{-134,-76}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Trapezoide
      Liquid_filling(
      periode=500,
      n=1,
      offset=0,
      amplitude=4,
      startTime=3000,
      rising=50,
      falling=50,
      largeur=200)
      annotation (Placement(transformation(extent={{-156,26},{-136,46}})));
  equation
    connect(sourceQ.C, controlValve.C1) annotation (Line(points={{-106,2},{-78,
            2},{-74,2}},         color={0,0,255}));
    connect(sinkQ.C, steamvalve.C2)
      annotation (Line(points={{96,12},{82,12},{82,18},{62,18}}));
    connect(sinkQ1.C, controlValve2.C2)
      annotation (Line(points={{100,-80},{82,-80},{82,-6},{54,-6}}));
    connect(controlValve.C2, drum__v1.Ce1) annotation (Line(points={{-54,2},{
            -40,2},{-40,9.6},{-27,9.6}},
                                       color={0,0,255}));
    connect(steamvalve.C1, drum__v1.Cs1)
      annotation (Line(points={{42,18},{-6,18},{-6,10}}));
    connect(controlValve2.C1, drum__v1.Cs2)
      annotation (Line(points={{34,-6},{18,-6},{18,-2.8},{-6.2,-2.8}}));
    connect(sinkQ.IMassFlow,Vapor_empty. y)
      annotation (Line(points={{106,17},{94,17},{94,94},{-139,94}}));
    connect(drum__v1.Cth, heatSource.C[1]) annotation (Line(points={{-16,4.6},{
            -16,4.6},{-16,56.2},{-26,56.2}},   color={191,95,0}));
    connect(heatSource.ISignal, Vaporization_not_all_liquid_water.y)
      annotation (Line(points={{-26,71},{-34,71},{-34,64},{-135,64}}));
    connect(Liquid_empty.y, sinkQ1.IMassFlow) annotation (Line(points={{-133,
            -86},{126,-86},{126,-76},{126,-75},{110,-75}}, color={0,0,0}));
    connect(Liquid_filling.y, sourceQ.IMassFlow) annotation (Line(points={{-135,
            36},{-132,36},{-132,34},{-116,34},{-116,7}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-160,
              -100},{140,120}}), graphics={Text(
            extent={{-138,-28},{-6,-52}},
            lineColor={28,108,200},
            textString="8000secs, 500int, Dassl, no script")}),
                                  Icon(coordinateSystem(extent={{-160,-100},{
              140,120}})));
  end Cas_test;

  model Test_recirculation

    Multi_mode_Drum Drum(
      Avl(start=3.46),
      Tv(start=604, displayUnit="K"),
      steady_state=true,
      hlstart(start=1454446.125, fixed=false),
      hvstart(start=2657931.5, fixed=false),
      Vlstart(start=28.1764, fixed=true),
      Vertical=true,
      P0=13000000,
      Veps1=1e-6,
      Veps2=1e-4,
      Plstart(start=13000000, fixed=true),
      Tl(start=592),
      Tp(start=600.15, fixed=false)) annotation (extent=[-61,16; 1,78],
        Placement(transformation(extent={{-55,18},{7,80}})));
    ThermoSysPro.WaterSteam.PressureLosses.ControlValve FeedwaterValve(
      Cv(start=100),
      Cvmax(fixed=false) = 200,
      C1(
        h_vol(start=1400e3),
        Q(start=75),
        h(start=1400e3),
        P(start=13000000, fixed=false))) annotation (extent=[-120,74; -100,94]);
    ThermoSysPro.WaterSteam.PressureLosses.ControlValve SteamValve(
      Pm(start=132e5),
      Cvmax(fixed=true) = 50000,
      Cv(start=25000)) annotation (extent=[40,74; 60,94]);
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Constante
      ConsigneNiveauBallon(k(fixed=true) = 0.5) annotation (extent=[-160,120; -140,
          140], Placement(transformation(extent={{4,120},{24,140}})));
    ThermoSysPro.WaterSteam.HeatExchangers.DynamicTwoPhaseFlowPipe
      TubeEcranBoucleEvaporatoire(
      T0=fill(400, 10),
      heb(start={10409,10268,10127,9985,9842,9698,9552,9406,9258,9111}),
      advection=false,
      z2=10,
      simplified_dynamic_energy_balance=false,
      P(start={13000000,13000000,13000000,13000000,13000000,13000000,13000000,
            13000000,13000000,13000000,13000000,13000000}),
      D=0.03,
      ntubes=1400,
      h(start={1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,
            1400e3,1400e3,1400e3}),
      L=20,
      Q(start=fill(150, 11))) annotation (extent=[16,-38; -4,-18], rotation=90);
    ThermoSysPro.Thermal.BoundaryConditions.HeatSource SourceC3(
      option_temperature=2,
      T0={290,290,290,290,290,290,290,290,290,290},
      W0={1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7})
      annotation (extent=[26,-38; 46,-18], rotation=-90);
    ThermoSysPro.Thermal.HeatTransfer.HeatExchangerWall heatExchangerWall(Ns=10, L=
          20) annotation (extent=[8,-38; 28,-18], rotation=90);
    ThermoSysPro.WaterSteam.PressureLosses.LumpedStraightPipe
      lumpedStraightPipe(
      L=20,
      z1=20,
      lambda(fixed=false) = 0.03,
      C1(P(start=130e5)),
      mode=1,
      Q(fixed=true, start=130))
      annotation (extent=[-76,-18; -56,-38], rotation=90);
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Sinusoide
      ConsigneNiveauBallon1(
      startTime=50,
      period=1500,
      offset=132e5,
      amplitude=30e5)    annotation (extent=[0,120; 20,140], Placement(
          transformation(extent={{-228,68},{-208,88}})));
    ThermoSysPro.WaterSteam.BoundaryConditions.SinkQ sinkQ(Q0(fixed=true) = 75)
                         annotation (extent=[126,68; 146,88]);
    ThermoSysPro.WaterSteam.BoundaryConditions.SourceP sourceP(
      h0=1400000,
      P0(fixed=true) = 13200000,
      option_temperature=2) annotation (extent=[-196,68; -176,88], Placement(
          transformation(extent={{-196,68},{-176,88}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Constante
      ConsigneNiveauBallon2(k(fixed=true) = 0.5) annotation (extent=[-160,120;
          -140,140], Placement(transformation(extent={{-144,114},{-124,134}})));
    Trapezoide_modif
      ConsigneNiveauBallon3(
      offset=1e7,
      rising=5,
      periode=1500,
      amplitude=-1e7,
      falling=45,
      largeur=500,
      startTime=1000)    annotation (extent=[0,120; 20,140], Placement(
          transformation(extent={{-224,-92},{-204,-72}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Sinusoide
      ConsigneNiveauBallon4(
      startTime=50,
      period=1500,
      offset(fixed=false) = 75,
      amplitude=40)      annotation (extent=[0,120; 20,140], Placement(
          transformation(extent={{-226,142},{-206,162}})));
  equation
    connect(heatExchangerWall.WT1, SourceC3.C) annotation (points=[20,-28; 26.2,
          -28], style(color=46, rgbcolor={191,95,0}));
    connect(TubeEcranBoucleEvaporatoire.CTh, heatExchangerWall.WT2) annotation (
        points=[9,-28; 16,-28],    style(color=46, rgbcolor={191,95,0}));
    connect(SteamValve.C2, sinkQ.C) annotation (points=[60,78; 126,78],  style(
        color=3,
        rgbcolor={0,0,255},
        smooth=0));
    connect(TubeEcranBoucleEvaporatoire.C1, lumpedStraightPipe.C2)
      annotation (points=[6,-38; 6,-70; -66,-70; -66,-38]);
    connect(FeedwaterValve.C2, Drum.Ce1) annotation (Line(points={{-100,78},{
            -74,78},{-74,72.56},{-58.1,72.56}}, color={0,0,255}));
    connect(Drum.Cs1, SteamValve.C1) annotation (Line(points={{7,73.8},{7,71.8},
            {28,71.8},{28,78},{40,78}}, color={0,0,255}));
    connect(Drum.Cm, TubeEcranBoucleEvaporatoire.C2) annotation (Line(points={{5.76,
            22.34},{5.76,0.17},{6,0.17},{6,-18}},      color={0,0,0}));
    connect(Drum.Cd, lumpedStraightPipe.C1) annotation (Line(points={{-56.86,
            21.72},{-66,21.72},{-66,-18}}, color={0,0,255}));
    connect(SteamValve.Ouv, ConsigneNiveauBallon.y)
      annotation (Line(points={{50,95},{38,95},{38,130},{25,130}}));
    connect(FeedwaterValve.C1, sourceP.C)
      annotation (Line(points={{-120,78},{-176,78}}));
    connect(FeedwaterValve.Ouv, ConsigneNiveauBallon2.y)
      annotation (Line(points={{-110,95},{-116,95},{-116,124},{-123,124}}));
    connect(sinkQ.IMassFlow, ConsigneNiveauBallon4.y)
      annotation (Line(points={{136,83},{120,83},{120,152},{-205,152}}));
    connect(ConsigneNiveauBallon3.y, SourceC3.ISignal) annotation (Line(points={{-203,
            -82},{86,-82},{86,-28},{41,-28}},      color={0,0,0}));
    annotation (
      Coordsys(
        extent=[-200, -100; 140, 200],
        grid=[2, 2],
        component=[20, 20]),
      Window(
        x=0.43,
        y=0,
        width=0.57,
        height=0.63),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{
              140,200}}), graphics={
          Text(
            extent={{-236,34},{-110,-70}},
            lineColor={28,108,200},
            textString="4000s,10000intervals, Dassl with script
(takes a long time)

")}));
  end Test_recirculation;

  model Test_heating

    Multi_mode_Drum Drum(
      zl(fixed=true, start=1.05),
      xl(start=0),
      xv(start=1),
      Avl(start=3.46),
      Tl(start=592, displayUnit="K"),
      Tv(start=604, displayUnit="K"),
      Vv(start=39, fixed=false),
      hs(start=2689980),
      rhol(start=996),
      rhov(start=60),
      steady_state=true,
      Vertical=false,
      he(start=1400000),
      hlstart(start=1.45445e6),
      hvstart(start=2.65793e+06),
      P0=13000000,
      Plstart(start=13000000, fixed=true),
      Tp(start=600.15, fixed=false),
      hd(start=1.45445e6)) annotation (extent=[-61,16; 1,78], Placement(
          transformation(extent={{-55,14},{7,76}})));
    ThermoSysPro.WaterSteam.PressureLosses.ControlValve FeedwaterValve(
      Cv(start=100),
      Cvmax(fixed=false) = 200,
      C1(
        h_vol(start=1400e3),
        Q(start=75),
        h(start=1400e3),
        P(start=13000000, fixed=false))) annotation (extent=[-120,74; -100,94]);
    ThermoSysPro.WaterSteam.PressureLosses.ControlValve SteamValve(
      Pm(start=132e5),
      Cvmax(fixed=true) = 50000,
      Cv(start=25000)) annotation (extent=[40,74; 60,94]);
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Constante
      ConsigneNiveauBallon(k(fixed=true) = 0.5) annotation (extent=[-160,120; -140,
          140], Placement(transformation(extent={{4,120},{24,140}})));
    ThermoSysPro.WaterSteam.HeatExchangers.DynamicTwoPhaseFlowPipe
      TubeEcranBoucleEvaporatoire(
      T0=fill(400, 10),
      heb(start={10409,10268,10127,9985,9842,9698,9552,9406,9258,9111}),
      advection=false,
      z2=10,
      simplified_dynamic_energy_balance=false,
      P(start={13000000,13000000,13000000,13000000,13000000,13000000,13000000,
            13000000,13000000,13000000,13000000,13000000}),
      D=0.03,
      ntubes=1400,
      h(start={1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,1400e3,
            1400e3,1400e3,1400e3}),
      L=20,
      Q(start=fill(150, 11))) annotation (extent=[16,-38; -4,-18], rotation=90);
    ThermoSysPro.Thermal.BoundaryConditions.HeatSource SourceC3(
      option_temperature=2,
      T0={290,290,290,290,290,290,290,290,290,290},
      W0={1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7,1e7})
      annotation (extent=[26,-38; 46,-18], rotation=-90);
    ThermoSysPro.Thermal.HeatTransfer.HeatExchangerWall heatExchangerWall(Ns=10, L=
          20) annotation (extent=[8,-38; 28,-18], rotation=90);
    ThermoSysPro.WaterSteam.PressureLosses.LumpedStraightPipe
      lumpedStraightPipe(
      L=20,
      z1=20,
      lambda(fixed=false) = 0.03,
      C1(P(start=130e5)),
      mode=1,
      Q(fixed=true, start=130))
      annotation (extent=[-76,-18; -56,-38], rotation=90);
    Trapezoide_modif
      ConsigneNiveauBallon1(
      startTime=50,
      offset=0,
      amplitude=50e5,
      periode=10000,
      n=1,
      largeur=425) annotation (extent=[0,120; 20,140], Placement(transformation(
            extent={{-106,160},{-86,180}})));
    ThermoSysPro.WaterSteam.BoundaryConditions.SinkQ sinkQ(Q0(fixed=
            false) = 75) annotation (extent=[126,68; 146,88]);
    ThermoSysPro.WaterSteam.BoundaryConditions.SourceP sourceP(
      h0=1400000,
      P0(fixed=true) = 13200000,
      option_temperature=2) annotation (extent=[-196,68; -176,88], Placement(
          transformation(extent={{-196,68},{-176,88}})));
    ThermoSysPro.InstrumentationAndControl.Blocks.Sources.Constante
      ConsigneNiveauBallon2(k(fixed=true) = 0.5) annotation (extent=[-160,120;
          -140,140], Placement(transformation(extent={{-144,114},{-124,134}})));
    ThermoSysPro.Thermal.BoundaryConditions.HeatSource heatSource(
        option_temperature=2, W0={0})
      annotation (Placement(transformation(extent={{-42,132},{-22,152}})));
  equation
    connect(heatExchangerWall.WT1, SourceC3.C) annotation (points=[20,-28; 26.2,
          -28], style(color=46, rgbcolor={191,95,0}));
    connect(TubeEcranBoucleEvaporatoire.CTh, heatExchangerWall.WT2) annotation (
        points=[9,-28; 16,-28],    style(color=46, rgbcolor={191,95,0}));
    connect(SteamValve.C2, sinkQ.C) annotation (points=[60,78; 126,78],  style(
        color=3,
        rgbcolor={0,0,255},
        smooth=0));
    connect(TubeEcranBoucleEvaporatoire.C1, lumpedStraightPipe.C2)
      annotation (points=[6,-38; 6,-70; -66,-70; -66,-38]);
    connect(FeedwaterValve.C2, Drum.Ce1) annotation (Line(points={{-100,78},{
            -74,78},{-74,68.56},{-58.1,68.56}}, color={0,0,255}));
    connect(Drum.Cs1, SteamValve.C1) annotation (Line(points={{7,69.8},{22,69.8},
            {28,69.8},{28,78},{40,78}}, color={0,0,255}));
    connect(Drum.Cm, TubeEcranBoucleEvaporatoire.C2) annotation (Line(points={{5.76,
            18.34},{5.76,0.17},{6,0.17},{6,-18}},      color={0,0,0}));
    connect(Drum.Cd, lumpedStraightPipe.C1) annotation (Line(points={{-56.86,
            17.72},{-66,17.72},{-66,-18}}, color={0,0,255}));
    connect(SteamValve.Ouv, ConsigneNiveauBallon.y)
      annotation (Line(points={{50,95},{38,95},{38,130},{25,130}}));
    connect(FeedwaterValve.C1, sourceP.C)
      annotation (Line(points={{-120,78},{-176,78}}));
    connect(FeedwaterValve.Ouv, ConsigneNiveauBallon2.y)
      annotation (Line(points={{-110,95},{-116,95},{-116,124},{-123,124}}));
    connect(Drum.Cth, heatSource.C[1]) annotation (Line(points={{-24,53.06},{
            -24,53.06},{-24,132.2},{-32,132.2}}, color={191,95,0}));
    connect(heatSource.ISignal, ConsigneNiveauBallon1.y)
      annotation (Line(points={{-32,147},{-58,147},{-58,170},{-85,170}}));
    annotation (
      Coordsys(
        extent=[-200, -100; 140, 200],
        grid=[2, 2],
        component=[20, 20]),
      Window(
        x=0.43,
        y=0,
        width=0.57,
        height=0.63),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
              140,200}}), graphics={
    Text(   extent={{-278,0},{-76,-78}},
            lineColor={28,108,200},
            textString="1000secs 500 int Dassl, with script

")}));
  end Test_heating;

  block Trapezoide_modif
    parameter Real amplitude=1 "Amplitude du trapèze";
    parameter Real rising(final min=0) = 0.5 "Durée du front montant (s)";
    parameter Real largeur(final min=0) = 2 "Largeur du trapèze (s)";
    parameter Real falling(final min=0) = 0.5 "Durée du front descendant (s)";
    parameter Real periode(final min=Modelica.Constants.small) = 4
      "Période (s)";
    parameter Integer n=-1
      "Nombre de périodes (< 0 nombre de périodes infini)";
    parameter Real offset=0 "Décalage de la sortie";
    parameter Real startTime=0 "Instant de départ de l'échelon";

  protected
    Real T0(final start=startTime, fixed = true)
      "Instant de départ de la période courante";
    Integer counter(start=n);
    Integer counter2(start=n);
  public
    ThermoSysPro.InstrumentationAndControl.Connectors.OutputReal y
                                         annotation (extent=[100, -10; 120, 10]);
  equation

    when ((pre(counter2) <> 0) and sample(startTime, periode)) then
      T0 = time;
      counter2 = pre(counter);
      counter = pre(counter) - (if pre(counter) > 0 then 1 else 0);
    end when;

    y.signal = offset + (if (time < startTime or (counter2 == 0) or (time >= T0
       + rising + largeur + falling)) then 0 else if (time < T0 + rising) then (
      time - T0)*amplitude/rising else if (time < T0 + rising + largeur) then
      amplitude else (T0 + rising + largeur - time)*amplitude/falling + amplitude);
    annotation (
      Coordsys(
        extent=[-100, -100; 100, 100],
        grid=[2, 2],
        component=[20, 20]),
      Icon(
        Text(extent=[-150, 150; 150, 110], string="%name"),
        Rectangle(extent=[-100, -100; 100, 100], style(color=3, fillColor=7)),
        Line(points=[-80, 68; -80, -80], style(color=8)),
        Polygon(points=[-80, 90; -88, 68; -72, 68; -80, 90], style(color=8,
              fillColor=8)),
        Line(points=[-90, -70; 82, -70], style(color=8)),
        Polygon(points=[90, -70; 68, -62; 68, -78; 90, -70], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Line(points=[-81, -70; -60, -70; -30, 40; 9, 40; 39, -70; 61, -70; 90, 40],
             style(color=0))),
      Window(
        x=0.16,
        y=0.11,
        width=0.6,
        height=0.6),
      Diagram(
        Polygon(points=[-80, 90; -88, 68; -72, 68; -80, 90], style(color=8,
              fillColor=8)),
        Line(points=[-80, 68; -80, -80], style(color=8)),
        Line(points=[-90, -70; 82, -70], style(color=8)),
        Polygon(points=[90, -70; 68, -62; 68, -78; 90, -70], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[-45, -30; -47, -41; -43, -41; -45, -30], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Line(points=[-45, -31; -45, -70], style(
            color=8,
            pattern=1,
            thickness=1,
            arrow=0)),
        Polygon(points=[-45, -70; -47, -60; -43, -60; -45, -70; -45, -70], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Text(
          extent=[-86, -43; -43, -55],
          string="offset",
          style(color=9)),
        Text(
          extent=[-47, -69; -1, -87],
          string="startTime",
          style(color=9)),
        Text(
          extent=[-84, 91; -43, 71],
          string="y",
          style(color=9)),
        Text(
          extent=[70, -80; 94, -100],
          string="temps",
          style(color=9)),
        Line(points=[-29, 82; -30, -70], style(color=8, pattern=2)),
        Line(points=[-10, 59; -10, 40], style(color=8, pattern=2)),
        Line(points=[20, 59; 20, 39], style(color=9, pattern=2)),
        Line(points=[40, 59; 40, -30], style(color=8, pattern=2)),
        Line(points=[-20, 76; 61, 76], style(color=8)),
        Line(points=[-29, 56; 40, 56], style(color=8)),
        Text(
          extent=[-2, 86; 25, 77],
          string="periode",
          style(color=9)),
        Text(
          extent=[-8, 70; 21, 60],
          string="largeur",
          style(color=9)),
        Line(points=[-42, 40; -10, 40], style(color=8, pattern=2)),
        Line(points=[-39, 40; -39, -19], style(
            color=8,
            pattern=1,
            thickness=1,
            arrow=0)),
        Text(
          extent=[-77, 14; -40, 0],
          string="amplitude",
          style(color=9)),
        Polygon(points=[-29, 56; -22, 58; -22, 54; -29, 56], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[-10, 56; -17, 58; -17, 54; -10, 56], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[-29, 76; -20, 78; -20, 74; -29, 76], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[61, 76; 53, 78; 53, 74; 61, 76], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Line(points=[-80, -30; -30, -30; -10, 40; 20, 40; 40, -30; 60, -30; 80,
              40; 100, 40], style(color=0, thickness=2)),
        Polygon(points=[-39, 40; -41, 29; -37, 29; -39, 40], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[-39, -29; -41, -19; -37, -19; -39, -29; -39, -29], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Line(points=[61, 84; 60, -30], style(color=8, pattern=2)),
        Polygon(points=[39, 56; 32, 58; 32, 54; 39, 56], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[20, 56; 27, 58; 27, 54; 20, 56], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[20, 56; 13, 58; 13, 54; 20, 56], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Polygon(points=[-12, 56; -5, 58; -5, 54; -12, 56], style(
            color=8,
            fillColor=8,
            fillPattern=1)),
        Text(
          extent=[-34, 70; -5, 60],
          string="rising",
          style(color=9)),
        Text(
          extent=[16, 70; 45, 60],
          string="falling",
          style(color=9))),
      Documentation(info="<html>
<p><b>Adapted from the Modelica.Blocks.Sources library</b></p>
</HTML>
<html>
<p><b>Version 1.0</b></p>
</HTML>
"));
  end Trapezoide_modif;
  annotation (uses(Modelica(version="3.2.1")));
end Multi_mode_Dynamic_Drum;
