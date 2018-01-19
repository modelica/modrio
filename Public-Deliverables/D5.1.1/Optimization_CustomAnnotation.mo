within ;
package Optimization_CustomAnnotation
  "Custom annotation based optimization setups for optimization problems"
    extends Modelica.Icons.Package;

  package Examples "Examples demonstrating the use of the optimization setup"
    extends Modelica.Icons.ExamplesPackage;

    package Integrator
      "Tiny example to demonstrate setup of optimization by optimizing an integrator"
      extends Modelica.Icons.Package;
      model Model "Model to be used for the optimization"
        extends Modelica.Icons.Example;
        input Real u;
        Real x(start=1, fixed=true);
      equation
        der(x) = u;
      end Model;

      model TrajectoryOptimizationSetupGUI
        "Optimization setup for model Integrator with graphical definition"
        extends Model(u annotation (Optimization_CustomAnnotation(Tuners(Input(
                    min=-1, max=100)))));

        Modelica.Blocks.Sources.RealExpression u_signal(y=u)
          annotation (Placement(transformation(extent={{-60,58},{-40,78}})));
        Modelica.Blocks.Sources.RealExpression x_signal(y=x)
          annotation (Placement(transformation(extent={{-60,32},{-40,52}})));
        GUIBlocks.Criteria.IntegralOfLeastSquares u_leastSquares
          annotation (Placement(transformation(extent={{-28,62},{-16,74}})));
        GUIBlocks.Criteria.IntegralOfLeastSquares x_leastSquares
          annotation (Placement(transformation(extent={{-28,12},{-16,24}})));
        GUIBlocks.Tasks.TrajectoryOptimization trajectoryOptimization(stopTime=
              1.0)
          annotation (Placement(transformation(extent={{-60,80},{-40,100}})));
        GUIBlocks.Constraints.LowerBound x_min(min=0.8)
          annotation (Placement(transformation(extent={{-28,36},{-16,48}})));
      equation

        connect(u_signal.y, u_leastSquares.u) annotation (Line(
            points={{-39,68},{-29.2,68}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(x_signal.y, x_leastSquares.u) annotation (Line(
            points={{-39,42},{-34,42},{-34,18},{-29.2,18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(x_signal.y, x_min.u) annotation (Line(
            points={{-39,42},{-29.2,42}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),        graphics));
      end TrajectoryOptimizationSetupGUI;

      model TrajectoryOptimizationSetupTextual
        "Optimization setup for model Integrator with textual definition"
        import Optimization_CustomAnnotation.TextualBlocks.*;

        extends Model(u annotation (Optimization_CustomAnnotation(Tuners(Input(
                    min=-1, max=100)))));

        Tasks.TrajectoryOptimization trajectoryOptimization(stopTime=1);
        Criteria.Integral criterion(signal=x^2 + u^2);
        Constraints.LowerBound lowerBound(signal=x, min=0.8);
      end TrajectoryOptimizationSetupTextual;

      model TrajectoryOptimizationSetupTextual2
        "Optimization setup for model Integrator with textual definition 2"
        import Optimization_CustomAnnotation.TextualBlocks.*;
        import Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.*;

        extends Model(u annotation (Optimization_CustomAnnotation(Tuners(Input(
                    min=-1, max=100)))));

        Tasks.TrajectoryOptimization trajectoryOptimization(stopTime=1);
        Criteria.Integral criterion(signal=x^2 + u^2);
        OptimizationVariable x_o=OptimizationVariable(x);
        Constraint x_Constraint(condition=x_o >= 0.8);
      end TrajectoryOptimizationSetupTextual2;

      model TrajectoryOptimizationSetupGUI2
        "Optimization setup for model Integrator with graphical definition"
        extends Model;

        GUIBlocks.Criteria.IntegralOfLeastSquares u_leastSquares
          annotation (Placement(transformation(extent={{-28,62},{-16,74}})));
        GUIBlocks.Criteria.IntegralOfLeastSquares x_leastSquares
          annotation (Placement(transformation(extent={{56,32},{68,44}})));
        GUIBlocks.Tasks.TrajectoryOptimization trajectoryOptimization(stopTime=
              1.0)
          annotation (Placement(transformation(extent={{-60,80},{-40,100}})));
        GUIBlocks.Constraints.LowerBound x_min(min=0.8)
          annotation (Placement(transformation(extent={{56,54},{68,66}})));
        Model2 model2_1
          annotation (Placement(transformation(extent={{-20,28},{0,48}})));
        GUIBlocks.Tuners.Input Input(min=-1, max=100)
          annotation (Placement(transformation(extent={{-78,28},{-58,48}})));
      equation

        connect(Input.u, model2_1.u) annotation (Line(
            points={{-61.6,38},{-22,38}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(model2_1.x, x_leastSquares.u) annotation (Line(
            points={{1,38},{54.8,38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(x_min.u, model2_1.x) annotation (Line(
            points={{54.8,60},{28,60},{28,38},{1,38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(Input.u, u_leastSquares.u) annotation (Line(
            points={{-61.6,38},{-50,38},{-50,68},{-29.2,68}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}})));
      end TrajectoryOptimizationSetupGUI2;


      model Model2 "Model to be used for the optimization"
        extends Modelica.Icons.Example;

        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealOutput x(start=1,fixed=true)
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        der(x) = u;
      end Model2;
    end Integrator;

    package DrumBoiler
      "Example demonstrating the online-optimization of a simple drum boiler model"
      extends Modelica.Icons.Package;

      model Model
        "Complete drum boiler model, including evaporator and supplementary components"
        extends Modelica.Icons.Example;

        parameter Boolean use_inputs=false
          "use external inputs instead of test data contained internally";

        Modelica.Fluid.Examples.DrumBoiler.BaseClasses.EquilibriumDrumBoiler
          evaporator(
          m_D=300e3,
          cp_D=500,
          V_t=100,
          V_l_start=67,
          redeclare package Medium = Modelica.Media.Water.StandardWater,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          p_start=100000) annotation (Placement(transformation(extent={{-46,-30},
                  {-26,-10}}, rotation=0)));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow furnace
          annotation (Placement(transformation(
              origin={-36,-53},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Fluid.Sources.FixedBoundary sink(
          nPorts=1,
          p=Modelica.SIunits.Conversions.from_bar(0.5),
          redeclare package Medium = Modelica.Media.Water.StandardWaterOnePhase,
          T=500) annotation (Placement(transformation(
              origin={90,-20},
              extent={{-10,-10},{10,10}},
              rotation=180)));

        Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package
            Medium = Modelica.Media.Water.StandardWater) annotation (Placement(
              transformation(
              origin={30,-20},
              extent={{10,10},{-10,-10}},
              rotation=180)));
        Modelica.Fluid.Sensors.Temperature temperature(redeclare package Medium
            = Modelica.Media.Water.StandardWater) annotation (Placement(
              transformation(
              origin={-3,-1},
              extent={{10,10},{-10,-10}},
              rotation=180)));
        Modelica.Fluid.Sensors.Pressure pressure(redeclare package Medium =
              Modelica.Media.Water.StandardWater) annotation (Placement(
              transformation(extent={{10,18},{30,38}}, rotation=0)));
        Modelica.Blocks.Continuous.PI controller(
          T=120,
          k=10,
          initType=Modelica.Blocks.Types.Init.InitialState) annotation (
            Placement(transformation(extent={{-49,23},{-63,37}}, rotation=0)));
        Modelica.Fluid.Sources.MassFlowSource_h pump(
          nPorts=1,
          h=5e5,
          redeclare package Medium = Modelica.Media.Water.StandardWater,
          use_m_flow_in=true) annotation (Placement(transformation(extent={{-80,
                  -30},{-60,-10}}, rotation=0)));
        Modelica.Blocks.Math.Feedback feedback annotation (Placement(
              transformation(extent={{-22,20},{-42,40}}, rotation=0)));
        Modelica.Blocks.Sources.Constant levelSetPoint(k=67) annotation (
            Placement(transformation(extent={{-38,48},{-24,62}}, rotation=0)));
        Modelica.Blocks.Interfaces.RealOutput T_S "steam temperature"
          annotation (Placement(transformation(extent={{100,48},{112,60}},
                rotation=0)));
        Modelica.Blocks.Interfaces.RealOutput p_S "steam pressure" annotation (
            Placement(transformation(extent={{100,22},{112,34}}, rotation=0)));
        Modelica.Blocks.Interfaces.RealOutput qm_S "steam flow rate"
          annotation (Placement(transformation(extent={{100,-2},{112,10}},
                rotation=0)));
        Modelica.Blocks.Interfaces.RealOutput V_l "liquid volume inside drum"
          annotation (Placement(transformation(extent={{100,74},{112,86}},
                rotation=0)));
      public
        Modelica.Blocks.Math.Gain MW2W(k=1e6) annotation (Placement(
              transformation(extent={{-54,-75.5},{-44,-64.5}}, rotation=0)));
        Modelica.Blocks.Math.Gain Pa2bar(k=1e-5) annotation (Placement(
              transformation(extent={{37,23},{47,33}}, rotation=0)));
        Modelica.Thermal.HeatTransfer.Celsius.FromKelvin K2degC annotation (
            Placement(transformation(extent={{38,49},{48,59}}, rotation=0)));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMin=0, uMax=500) annotation (
           Placement(transformation(
              origin={-78,30},
              extent={{-7,7},{7,-7}},
              rotation=180)));
        Modelica.Fluid.Valves.ValveLinear SteamValve(
          redeclare package Medium = Modelica.Media.Water.StandardWater,
          dp_nominal=9000000,
          m_flow_nominal=180) annotation (Placement(transformation(extent={{50,
                  -10},{70,-30}}, rotation=0)));

        inner Modelica.Fluid.System system
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
        Modelica.Blocks.Sources.TimeTable q_F_Tab(table=[0, 0; 3600, 400; 7210,
              400]) if not use_inputs annotation (Placement(transformation(
                extent={{-90,-80},{-70,-60}}, rotation=0)));
        Modelica.Blocks.Sources.TimeTable Y_Valve_Tab(table=[0, 0; 900, 1; 7210,
              1]) if not use_inputs annotation (Placement(transformation(extent=
                 {{30,-80},{50,-60}}, rotation=0)));
        Modelica.Blocks.Interfaces.RealInput q_F(unit="MW") if use_inputs
          "fuel flow rate" annotation (Placement(transformation(extent={{-112,-56},
                  {-100,-44}})));
        Modelica.Blocks.Interfaces.RealInput Y_Valve if use_inputs
          "valve opening" annotation (Placement(transformation(extent={{-112,-96},
                  {-100,-84}})));
        Modelica.Blocks.Sources.RealExpression sigma_D_expr(y=-1e3*der(
              evaporator.T_D) + 1e-5*evaporator.p)
          annotation (Placement(transformation(extent={{24,-108},{82,-92}})));
        Modelica.Blocks.Interfaces.RealOutput sigma_D(unit="N/mm2")
          "thermal stress of drum" annotation (Placement(transformation(extent=
                  {{100,-68},{112,-56}}, rotation=0)));
      equation
        connect(furnace.port, evaporator.heatPort)
          annotation (Line(points={{-36,-43},{-36,-30}}, color={191,0,0}));
        connect(controller.u, feedback.y)
          annotation (Line(points={{-47.6,30},{-41,30}}, color={0,0,127}));
        connect(massFlowRate.m_flow, qm_S)
          annotation (Line(points={{30,-9},{30,4},{106,4}}, color={0,0,127}));
        connect(evaporator.V, V_l) annotation (Line(points={{-32,-9},{-32,16},{
                -4,16},{-4,80},{106,80}}, color={0,0,127}));
        connect(MW2W.y, furnace.Q_flow) annotation (Line(points={{-43.5,-70},{-36,
                -70},{-36,-63}}, color={0,0,127}));
        connect(pressure.p, Pa2bar.u)
          annotation (Line(points={{31,28},{36,28}}, color={0,0,127}));
        connect(Pa2bar.y, p_S)
          annotation (Line(points={{47.5,28},{106,28}}, color={0,0,127}));
        connect(K2degC.Celsius, T_S)
          annotation (Line(points={{48.5,54},{106,54}}, color={0,0,127}));
        connect(controller.y, limiter.u)
          annotation (Line(points={{-63.7,30},{-69.6,30}}, color={0,0,127}));
        connect(limiter.y, pump.m_flow_in) annotation (Line(points={{-85.7,30},
                {-90,30},{-90,-12},{-80,-12}}, color={0,0,127}));
        connect(temperature.T, K2degC.Kelvin) annotation (Line(points={{4,-1},{
                4,-1},{8,-1},{8,54},{37,54}}, color={0,0,127}));
        connect(pressure.port, massFlowRate.port_a)
          annotation (Line(points={{20,18},{20,-20}}, color={0,127,255}));
        connect(pump.ports[1], evaporator.port_a)
          annotation (Line(points={{-60,-20},{-46,-20}}, color={0,127,255}));
        connect(massFlowRate.port_b, SteamValve.port_a)
          annotation (Line(points={{40,-20},{50,-20}}, color={0,127,255}));
        connect(SteamValve.port_b, sink.ports[1]) annotation (Line(points={{70,
                -20},{75,-20},{80,-20}}, color={0,127,255}));
        connect(evaporator.port_b, massFlowRate.port_a)
          annotation (Line(points={{-26,-20},{20,-20}}, color={0,127,255}));
        connect(temperature.port, massFlowRate.port_a) annotation (Line(
            points={{-3,-11},{-3,-20},{20,-20}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(q_F_Tab.y, MW2W.u) annotation (Line(
            points={{-69,-70},{-55,-70}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(Y_Valve_Tab.y, SteamValve.opening) annotation (Line(
            points={{51,-70},{60,-70},{60,-28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(q_F, MW2W.u) annotation (Line(
            points={{-106,-50},{-62,-50},{-62,-70},{-55,-70}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(Y_Valve, SteamValve.opening) annotation (Line(
            points={{-106,-90},{60,-90},{60,-28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(evaporator.V, feedback.u2) annotation (Line(
            points={{-32,-9},{-32,6},{-32,6},{-32,22}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(levelSetPoint.y, feedback.u1) annotation (Line(
            points={{-23.3,55},{-16,55},{-16,30},{-24,30}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sigma_D_expr.y, sigma_D) annotation (Line(
            points={{84.9,-100},{94,-100},{94,-62},{106,-62}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics={
              Text(
                extent={{-151,165},{138,102}},
                lineColor={0,0,255},
                textString="%name"),
              Text(
                extent={{-79,67},{67,21}},
                lineColor={0,0,0},
                textString="drum"),
              Text(
                extent={{-90,-14},{88,-64}},
                lineColor={0,0,0},
                textString="boiler")}),
          experiment(StopTime=5400),
          Documentation(info="<html>
<p>The DrumBoiler example provides a simple test case for optimization and estimation. It exhibits the following features:</p>
<p><ul>
<li>simple model based on MSL, including Media and Fluid</li>
<li>simple embedded control using Blocks, including PI controller and discontinuous limiter</li>
<li>three states: drum boiler pressure, drum boiler water level, integral part of controller</li>
</ul></p>
<p>The optimization should care about</p>
<p><ul>
<li>usage of regular Modelica simulation model as optimization contraints</li>
<li>initialization of states from parameters</li>
<li>treatment of nominal values (e.g. 100 bar is represented as 1e7 Pa in the pressure state, while the valve is fully open at 1)</li>
<li>physical state/output constraint for thermal stress</li>
<li>controller state/output constraint for limiter (i.e. avoid negative feedwater flow)</li>
<li>appropriate discretization of path contraints, avoiding oscillation around sample points</li>
</ul></p>
<p>The test case is taken from: </p>
<p><i>R. Franke, M. Rode, K. Kr&uuml;ger: On-line Optimization of Drum Boiler Startup, Modelica 2003. </i></p>
<p>See: <a href=\"https://www.modelica.org/events/Conference2003/papers/h29_Franke.pdf\">https://www.modelica.org/events/Conference2003/papers/h29_Franke.pdf</a></p>
<p><h4><font color=\"#008000\">Simulation model for optimization and estimation</font></h4></p>
<p>The simulation model can be translated to an ODE of the form:</p>
<pre>  der(x) = f(x,u)
  y = h(x,u)</pre>
<p>with:</p>
<pre>  x = {evaporator.p, evaporator.V_l, controller.x}
  u = {q_F, Y_Valve}
  y = {T_S, p_S, qm_S, V_l, sigma_D}</pre>
<p>Compared to the base model Modelica.Fluid.Examples.DrumBoiler.DrumBoiler, the fifth output sigma_D adds a simplified model for thermal stress and membrane stress</p>
<pre>  sigma_D = -1e3*der(evaporator.T_D) + 1e-5*evaporator.p</pre>
<p>The initial states are defined by model parameters to:</p>
<pre>  x(0) = f0(p)</pre>
<p>with the parameter values:</p>
<pre>  evaporator.p_start = 1 bar
  evaporator.V_start = 67 m3
  controller.x_start = 0</pre>
<p><h4><font color=\"#008000\">Trajectory optimization</font></h4></p>
<p>The aim is to obtain an optimal startup control for fuel flow rate and steam flow considering a constraint on termal stress. </p>
<p>This is:</p>
<pre> 3600s
   &int;  1e-3*(p_S - 110)^2 + 1e-4*(qm_S - 180)^2 dt  --&GT; min
   0                                                   u(t) </pre>
<p>subject to the model:</p>
<pre>  der(x) = f(x,u)
  x(0) = f0(p)</pre>
<p>the control bounds:</p>
<pre>  0          &LT;= Y_Valve  &LT;= 1
  0          &LT;=   q_F    &LT;= 500 MW
  -25 MW/min &LT;= der(q_F) &LT;= 25 MW/min
  q_F(0) = 0</pre>
<p>and the state/output constraint:</p>
<pre>  -150 N/mm2 &LT;= sigma_D</pre>
<p>An appropriate discretization of the control inputs is piecewise linear with a discretization step size of 60s. The model variables can be initialized with the results of an initial-value simulation keeping the valve fully open and constantly ramping up the fuel flow rate, e.g. by 400MW/1h (see also table data provided with the simulation model in q_F_tab and Y_Valve_tab).</p>
<p><h4><font color=\"#008000\">Initial state and parameter estimation</font></h4></p>
<p>The aim is to estimate initial states and the heating value of the fuel for given measurement data. The conversion parameter MW2W.k is used instead of an explicitly modeled heating value. </p>
<p>The estimation problem is:</p>
<pre>  10
   &Sigma;  [(y(60*kk) - y_measured(60*kk))./{100,100,100,10,100}].^2  --&GT; min
  kk=0                                                             x(0), MW2W.k </pre>
<p>subject to the model:</p>
<pre>  der(x) = f(p,x,u)
  x(0) = f0(p)</pre>
<p>The measurement data can be generated by performing an initial value simulation, e.g. using the provided table data as inputs. The estimation should start from a point away from the solution, e.g. MW2W.k=5e5. Moreover, noise may be added to the simulated measurement data.</p>
<p><h4><font color=\"#008000\">Steady-state optimization</font></h4></p>
<p>The goal of the steady-state optimization is to find values for the fuel flow rate q_F and the opening of the steam valve Y_Valve for which the heat input is minimized, subject to required steam pressure and mass flow rate.</p>
<p>This is:</p>
<pre>  q_F  --&GT;  min
            x,u </pre>
<p>subject to the steady-state model:</p>
<pre>  0 = f(x,u)</pre>
<p>and the constraints:</p>
<pre>  0        &LT;= q_F     &LT;= 500 MW
  0        &LT;= Y_Valve &LT;= 1
  150 kg/s &LT;= qm_S    &LT;= 200 kg/s
  100 bar  &LT;= p_S     &LT;= 120 bar</pre>
<p>The solution can be found at q_F = 328 MW, Y_Valve = 0.63 with the states evaporator.p = 120 bar, evaporator.V_liquid = 67 m3, and controller.x = 15.</p>
</html>"));
      end Model;

      model TrajectoryOptimizationSetupGUI
        extends Model(
          use_inputs=true,
          q_F annotation (Optimization_CustomAnnotation(Tuners(Input(
                  min=0,
                  max=500,
                  derMin=-25/60,
                  derMax=25/60,
                  u_tStart=0,
                  u_init=[0.0, 0.0; 60*60, 400])))),
          Y_Valve annotation (Optimization_CustomAnnotation(Tuners(Input(
                  min=0,
                  max=1,
                  u_init=[0.0, 1.0])))));

        GUIBlocks.Constraints.LowerBound sigma_D_bound(min=-150)
          annotation (Placement(transformation(extent={{128,-68},{140,-56}})));
        GUIBlocks.Criteria.IntegralOfLeastSquares p_S_criterion(reference=110,
            nominal=1000)
          annotation (Placement(transformation(extent={{130,26},{142,38}})));
        GUIBlocks.Criteria.IntegralOfLeastSquares qm_S_criterion(reference=180,
            nominal=1e4)
          annotation (Placement(transformation(extent={{130,-10},{142,2}})));
        GUIBlocks.Tasks.TrajectoryOptimization trajectoryOptimization(tolerance=1e-6,
            stopTime=3600)
          annotation (Placement(transformation(extent={{-100,74},{-80,94}})));
      equation
        connect(sigma_D, sigma_D_bound.u) annotation (Line(
            points={{106,-62},{126.8,-62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(p_S, p_S_criterion.u) annotation (Line(
            points={{106,28},{118,28},{118,32},{128.8,32}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(qm_S_criterion.u, qm_S) annotation (Line(
            points={{128.8,-4},{118,-4},{118,4},{106,4}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}})), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-160,-120},{160,100}}),
              graphics));
      end TrajectoryOptimizationSetupGUI;

      model TrajectoryOptimizationSetupTextual
        import Optimization_CustomAnnotation.TextualBlocks.*;

        extends Model(
          use_inputs=true,
          q_F annotation (Optimization_CustomAnnotation(Tuners(Input(
                  min=0,
                  max=500,
                  derMin=-25/60,
                  derMax=25/60,
                  u_tStart=0,
                  u_init=[0.0, 0.0; 60*60, 400])))),
          Y_Valve annotation (Optimization_CustomAnnotation(Tuners(Input(
                  min=0,
                  max=1,
                  u_init=[0.0, 1.0])))));

        Tasks.TrajectoryOptimization trajectoryOptimization(stopTime=3600, tolerance=1e-6);
        Constraints.LowerBound sigma_D_bound(signal=sigma_D, min=-150);
        Criteria.IntegralOfLeastSquares p_S_criterion(signal=p_S, reference=110,  nominal=1000);
        Criteria.IntegralOfLeastSquares qm_S_criterion(signal=qm_S, reference=180,  nominal=1e4);
      end TrajectoryOptimizationSetupTextual;

      model TrajectoryOptimizationSetupTextual2
        import Optimization_CustomAnnotation.TextualBlocks.*;
        import Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.*;

        extends Model(
          use_inputs=true,
          q_F annotation (Optimization_CustomAnnotation(Tuners(Input(
                  min=0,
                  max=500,
                  derMin=-25/60,
                  derMax=25/60,
                  u_tStart=0,
                  u_init=[0.0, 0.0; 60*60, 400])))),
          Y_Valve annotation (Optimization_CustomAnnotation(Tuners(Input(
                  min=0,
                  max=1,
                  u_init=[0.0, 1.0])))));

        Tasks.TrajectoryOptimization trajectoryOptimization(stopTime=3600, tolerance=1e-6);
        OptimizationVariable sigma_D_o=OptimizationVariable(sigma_D);
        Constraint sigma_D_Constraint(condition=sigma_D_o >= -150);
        Criteria.IntegralOfLeastSquares p_S_criterion(signal=p_S, reference=110, nominal=1000);
        Criteria.IntegralOfLeastSquares qm_S_criterion(signal=qm_S, reference=180, nominal=1e4);
      end TrajectoryOptimizationSetupTextual2;

    end DrumBoiler;

  end Examples;

  package Tuners
    extends Modelica.Icons.Package;

    record Parameter "Parameter to be optimized"
      extends Optimization_CustomAnnotation.Internal.Icons.Record;
      parameter Boolean active=true
        "= true, if parameter is optimized. Otherwise, it is left at its binding expression.";
      parameter Real initialGuess=Internal.Constants.UndefinedReal
        "Value to start the optimization. If unspecified, the parameter's default value should be used.";
      parameter Real min=-Modelica.Constants.inf
        "Minimum value for optimization";
      parameter Real max=Modelica.Constants.inf
        "Maximum value for optimization";
    end Parameter;

    record Input "Time-dependent input function to be optimized"
      extends Optimization_CustomAnnotation.Internal.Icons.Record;
      parameter Boolean active=true
        "= true, if the input is optimized. Otherwise, it is left at the initial guess u_init.";

      parameter Real min=-Modelica.Constants.inf "Minimum value of input u";
      parameter Real max=Modelica.Constants.inf "Maximum value of input u";

      parameter Real derMin=-Modelica.Constants.inf "Minimum value of der(u)"
        annotation (Dialog(group="Band constraint on derivative"));

      parameter Real derMax=Modelica.Constants.inf "Maximum value of der(u)"
        annotation (Dialog(group="Band constraint on derivative"));

      parameter Real u_tStart=Internal.Constants.UndefinedReal
        annotation (Dialog(group="Required value of input at startTime"));
      parameter Real der_u_tStart=Internal.Constants.UndefinedReal
        annotation (Dialog(group="Required value of input at startTime"));
      parameter Real der2_u_tStart=Internal.Constants.UndefinedReal
        annotation (Dialog(group="Required value of input at startTime"));

      parameter Real u_tStop=Internal.Constants.UndefinedReal
        annotation (Dialog(group="Required value of input at stopTime"));
      parameter Real der_u_tStop=Internal.Constants.UndefinedReal
        annotation (Dialog(group="Required value of input at stopTime"));
      parameter Real der2_u_tStop=Internal.Constants.UndefinedReal
        annotation (Dialog(group="Required value of input at stopTime"));

      parameter Real u_init[:, 2]=[Internal.Constants.UndefinedReal, Internal.Constants.UndefinedReal]
        "Initial definition of input; first column time points, second column u-values (by default: linear interpolation; extrapolation through first/last two points). Solvers will typically need values to be specified."
        annotation (Dialog(group="Initial input trajectory"));
    end Input;

  end Tuners;

  package GUIBlocks
    "Configure the optimization setup graphically with special blocks"
    package Tasks "Global options of optimization setup tasks"
      extends Modelica.Icons.Package;
      model ModelPredictiveControl
        "Global options for model predictive control (online-optimization to optimize control inputs)"
        extends
          Optimization_CustomAnnotation.TextualBlocks.Tasks.ModelPredictiveControl;
      end ModelPredictiveControl;

      model TrajectoryOptimization
        "Global options for trajectory optimization (offline-optimization to optimize input trajectories)"
        extends
          Optimization_CustomAnnotation.TextualBlocks.Tasks.TrajectoryOptimization;
      end TrajectoryOptimization;

      model StaticOptimization
        "Global options for static optimization (single instant-optimization to optimize parameters)"
        extends
          Optimization_CustomAnnotation.TextualBlocks.Tasks.StaticOptimization;
      end StaticOptimization;
    end Tasks;

    extends Modelica.Icons.Package;

    package Criteria
      extends Modelica.Icons.Package;
      block Integral
        "The integral of the variable is minimized (Lagrange term)"

        extends Optimization_CustomAnnotation.Internal.Criteria.Integral;
        extends Optimization_CustomAnnotation.Internal.Icons.CriterionGUI;

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Criteria(Integral(active=active,
                  nominal=nominal)))), Placement(transformation(extent={{-140,-20},
                  {-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}),
                         graphics={
              Line(
                points={{-81,-80},{-73,-68.53},{-65,-39.5},{-57,-2.522},{-49,
                    32.75},{-41,58.8},{-33,71.51},{-29,73},{-24,70},{-9,40.06},
                    {-1,20.55},{7,4.459},{15,-5.271},{23,-7.629},{31,-3.428},{
                    39,5.21},{47,15.56},{55,25.03},{63,31.66},{71,34.5},{79,
                    33.61}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{-78,84},{-86,62},{-70,62},{-78,84}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-78,62},{-78,-86}}, color={192,192,192}),
              Line(points={{-88,-76},{84,-76}}, color={192,192,192}),
              Polygon(
                points={{92,-76},{70,-68},{70,-84},{92,-76}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-68,-48},{-54,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-62,-26},{-32,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-56,0},{-10,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-50,26},{14,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-44,48},{36,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{14,-4},{58,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{34,0},{76,-72}},
                color={255,0,0},
                smooth=Smooth.None),
              Text(
                extent={{-240,-120},{260,-140}},
                lineColor={0,0,0},
                textString="nominal = %nominal"),
              Line(
                points={{48,16},{94,-74}},
                color={255,0,0},
                smooth=Smooth.None)}));
      end Integral;

      block IntegralOfLeastSquares
        "The integral of the least squares error with respect to a reference value is minimized (special Lagrange term)"
        extends Optimization_CustomAnnotation.Internal.Criteria.Integral;
        extends Optimization_CustomAnnotation.Internal.Icons.CriterionGUI;
        parameter Real reference=0.0
          "Reference value. Minimize Integral( (input-reference)^2/nominal * dt)";
        Modelica.Blocks.Interfaces.RealInput u annotation (Placement(
              transformation(extent={{-140,-20},{-100,20}}), iconTransformation(
                extent={{-140,-20},{-100,20}})));
        final output Real leastSquares=(u - reference)^2 annotation (
            Optimization_CustomAnnotation(Internal(Criteria(Integral(active=active,
                  nominal=nominal)))));
        annotation (Icon(graphics={
              Text(
                extent={{-250,-130},{250,-150}},
                lineColor={0,0,0},
                textString="reference = %reference"),
              Text(
                extent={{-250,-180},{250,-200}},
                lineColor={0,0,0},
                textString="nominal = %nominal"),
              Line(
                points={{-81,-80},{-73,-68.53},{-65,-39.5},{-57,-2.522},{-49,
                    32.75},{-41,58.8},{-33,71.51},{-29,73},{-24,70},{-9,40.06},
                    {-1,20.55},{7,4.459},{15,-5.271},{23,-7.629},{31,-3.428},{
                    39,5.21},{47,15.56},{55,25.03},{63,31.66},{71,34.5},{79,
                    33.61}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{-78,84},{-86,62},{-70,62},{-78,84}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-78,62},{-78,-86}}, color={192,192,192}),
              Line(points={{-88,-76},{84,-76}}, color={192,192,192}),
              Polygon(
                points={{92,-76},{70,-68},{70,-84},{92,-76}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-78,20},{82,20}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-78,-30},{-68,-50}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-78,0},{-62,-26}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-70,20},{-56,0}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-46,44},{-30,20}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-38,62},{-10,20}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{10,20},{26,-6}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{24,20},{36,2}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{64,32},{72,20}},
                color={255,0,0},
                smooth=Smooth.None)}));
      end IntegralOfLeastSquares;

      block Point
        "The variable value at one or several fixed time instants is minimized (Mayer term)"
        extends Optimization_CustomAnnotation.Internal.Criteria.Point;
        extends Optimization_CustomAnnotation.Internal.Icons.CriterionGUI;

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Criteria(Point(
                active=active,
                nominal=nominal,
                T=T)))), Placement(transformation(extent={{-140,-20},{-100,20}}),
              iconTransformation(extent={{-140,-20},{-100,20}})));

        annotation (Icon(graphics={
              Line(
                points={{-81,-80},{-73,-68.53},{-65,-39.5},{-57,-2.522},{-49,
                    32.75},{-41,58.8},{-33,71.51},{-29,73},{-24,70},{-9,40.06},
                    {-1,20.55},{7,4.459},{15,-5.271},{23,-7.629},{31,-3.428},{
                    39,5.21},{47,15.56},{55,25.03},{63,31.66},{71,34.5},{79,
                    33.61}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{-78,84},{-86,62},{-70,62},{-78,84}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-78,62},{-78,-86}}, color={192,192,192}),
              Line(points={{-88,-76},{84,-76}}, color={192,192,192}),
              Polygon(
                points={{92,-76},{70,-68},{70,-84},{92,-76}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-1,22},{-1,-76}},
                color={255,0,0},
                smooth=Smooth.None),
              Polygon(
                points={{-1,20},{-9,43},{8,43},{-1,20}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-1,100},{-1,21}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-240,-120},{260,-140}},
                lineColor={0,0,0},
                textString="T = %T"),
              Text(
                extent={{-240,-170},{260,-190}},
                lineColor={0,0,0},
                textString="nominal = %nominal")}), Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={1,1})));
      end Point;

      block VariableAtStopTime
        "The variable value at the final time instant is minimized (Mayer term). The final time instant might be determined by optimization. This is described in the Task definition."
        extends
          Optimization_CustomAnnotation.Internal.Criteria.VariableAtStopTime;
        extends Optimization_CustomAnnotation.Internal.Icons.CriterionGUI;

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Criteria(VariableAtStopTime(active=
                    active, nominal=nominal)))), Placement(transformation(extent=
                 {{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},
                  {-100,20}})));

        annotation (Icon(graphics={
              Line(
                points={{-81,-80},{-73,-68.53},{-65,-39.5},{-57,-2.522},{-49,
                    32.75},{-41,58.8},{-33,71.51},{-29,73},{-24,70},{-9,40.06},
                    {-1,20.55},{7,4.459},{15,-5.271},{23,-7.629},{31,-3.428},{
                    39,5.21},{47,15.56},{55,25.03},{63,31.66},{71,34.5},{78,34}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{-78,84},{-86,62},{-70,62},{-78,84}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-78,62},{-78,-86}}, color={192,192,192}),
              Line(points={{-88,-76},{84,-76}}, color={192,192,192}),
              Polygon(
                points={{92,-76},{70,-68},{70,-84},{92,-76}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{77,34},{69,57},{86,57},{77,34}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(
                points={{77,100},{77,21}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-240,-120},{260,-140}},
                lineColor={0,0,0},
                textString="nominal = %nominal"),
              Line(
                points={{77,22},{77,-76}},
                color={255,0,0},
                smooth=Smooth.None)}));

      end VariableAtStopTime;

      block PointLeastSquares
        "The variable value at a grid of fixed time instant is minimized in a least squares sense with respect to reference values at these time instants (Mayer term)"
        extends
          Optimization_CustomAnnotation.Internal.Criteria.PointLeastSquares;
        extends Optimization_CustomAnnotation.Internal.Icons.CriterionGUI;

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Criteria(PointLeastSquares(
                active=active,
                nominal=nominal,
                T=T,
                reference=reference)))), Placement(transformation(extent={{-140,
                  -20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,
                  20}})));

        annotation (Icon(graphics={
              Line(
                points={{-81,-80},{-73,-68.53},{-65,-39.5},{-57,-2.522},{-49,
                    32.75},{-41,58.8},{-33,71.51},{-29,73},{-24,70},{-9,40.06},
                    {-1,20.55},{7,4.459},{15,-5.271},{23,-7.629},{31,-3.428},{
                    39,5.21},{47,15.56},{55,25.03},{63,31.66},{71,34.5},{79,
                    33.61}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{-78,84},{-86,62},{-70,62},{-78,84}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-78,62},{-78,-86}}, color={192,192,192}),
              Line(points={{-88,-76},{84,-76}}, color={192,192,192}),
              Polygon(
                points={{92,-76},{70,-68},{70,-84},{92,-76}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{0,18},{-8,44},{8,44},{0,18}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-240,-120},{260,-140}},
                lineColor={0,0,0},
                textString="nominal = %nominal"),
              Line(
                points={{56,100},{56,-30}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-78,-30},{82,-30}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{0,100},{0,-30}},
                color={255,0,0},
                smooth=Smooth.None),
              Line(
                points={{-52,100},{-52,-30}},
                color={255,0,0},
                smooth=Smooth.None),
              Polygon(
                points={{56,26},{48,52},{64,52},{56,26}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-52,20},{-60,46},{-44,46},{-52,20}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={1,1})));
      end PointLeastSquares;
    end Criteria;

    package Constraints
      extends Modelica.Icons.Package;
      block Band "The variable is constrained to stay in a defined band"
        import Optimization_CustomAnnotation.Internal.Constants.*;
        extends Optimization_CustomAnnotation.Internal.Constraints.Band;
        extends Optimization_CustomAnnotation.Internal.Icons.ConstraintGUI;

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Band(
                active=active,
                min=min,
                max=max,
                nominal=nominal,
                T=T)))), Placement(transformation(extent={{-140,-20},{-100,20}}),
              iconTransformation(extent={{-140,-20},{-100,20}})));

        annotation (Icon(graphics={
              Line(points={{-84,66},{-84,-90}}, color={192,192,192}),
              Polygon(
                points={{-84,88},{-92,66},{-76,66},{-84,88}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-84,32},{87,32}}, color={255,0,0}),
              Line(
                points={{-84,-8},{-67,0},{-55,9},{-46,20},{-39,29},{-32,33},{-27,
                    30},{-12,0.06},{-4,-19.45},{4,-35.541},{12,-45.271},{20,-47.629},
                    {28,-43.428},{36,-34.79},{44,-24.44},{52,-14.97},{60,-8.34},
                    {68,-5.5},{76,-6.39}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{3,32},{-5,10},{11,10},{3,32}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(
                points={{3,32},{3,-47}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{3,-47},{-5,-25},{11,-25},{3,-47}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(points={{-84,-47},{87,-47}}, color={255,0,0}),
              Text(
                extent={{-250,80},{250,60}},
                lineColor={0,0,0},
                textString="%max"),
              Text(
                extent={{-250,-65},{250,-85}},
                lineColor={0,0,0},
                textString="%min")}));
      end Band;

      block LowerBound "The variable is constrained by a lower bound"
        extends Optimization_CustomAnnotation.Internal.Icons.ConstraintGUI;
        parameter Boolean active=true
          "= true, if the constraint is taken into account";
        parameter Real min=0 "Minimum value of variable";
        parameter Real nominal=1.0
          "Scale constraint: min/nominal <= variable/nominal";
        parameter Real T[:]=fill(0.0, 0)
          "Time instants at which the constraint shall be fulfilled. If zero-dimension, constraint holds for all time instants";

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Band(
                active=active,
                min=min,
                nominal=nominal,
                T=T)))), Placement(transformation(extent={{-140,-20},{-100,20}}),
              iconTransformation(extent={{-140,-20},{-100,20}})));

        annotation (Icon(graphics={
              Line(points={{-84,66},{-84,-82}}, color={192,192,192}),
              Polygon(
                points={{-84,88},{-92,66},{-76,66},{-84,88}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-84,22},{-67,30},{-55,39},{-46,50},{-39,59},{-32,63},{
                    -27,60},{-12,30.06},{-4,10.55},{4,-5.541},{12,-15.271},{20,
                    -17.629},{28,-13.428},{36,-4.79},{44,5.56},{52,15.03},{60,
                    21.66},{68,24.5},{76,23.61}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Line(
                points={{3,62},{3,-17}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{3,-17},{-5,5},{11,5},{3,-17}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(points={{-84,-17},{87,-17}}, color={255,0,0}),
              Text(
                extent={{-250,-50},{250,-70}},
                lineColor={0,0,0},
                textString="%min")}));
      end LowerBound;

      block UpperBound "The variable is constrained by an upper bound"
        extends Optimization_CustomAnnotation.Internal.Icons.ConstraintGUI;
        parameter Boolean active=true
          "= true, if the constraint is taken into account";
        parameter Real max "Maximum value of variable";
        parameter Real nominal=1.0
          "Scale constraint: max/nominal >= variable/nominal";
        parameter Real T[:]=fill(0.0, 0)
          "Time instants at which the constraint shall be fulfilled. If zero-dimension, constraint holds for all time instants";

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Band(
                active=active,
                max=max,
                nominal=nominal,
                T=T)))), Placement(transformation(extent={{-140,-20},{-100,20}}),
              iconTransformation(extent={{-140,-20},{-100,20}})));

        annotation (Icon(graphics={
              Line(points={{-84,66},{-84,-82}}, color={192,192,192}),
              Polygon(
                points={{-84,88},{-92,66},{-76,66},{-84,88}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-84,-44},{-67,-36},{-55,-27},{-46,-16},{-39,-7},{-32,-3},
                    {-27,-6},{-12,-35.94},{-4,-55.45},{4,-71.541},{12,-81.271},
                    {20,-83.629},{28,-79.428},{36,-70.79},{44,-60.44},{52,-50.97},
                    {60,-44.34},{68,-41.5},{76,-42.39}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Line(
                points={{3,-4},{3,-83}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{3,-4},{-5,-28},{11,-28},{3,-4}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(points={{-84,-3.5},{87,-3.5}}, color={255,0,0}),
              Text(
                extent={{-250,60},{250,40}},
                lineColor={0,0,0},
                textString="%max")}), Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={1,1})));
      end UpperBound;

      block Equality "The variable is constrained to a required value"
        extends Optimization_CustomAnnotation.Internal.Constraints.Equality;
        extends Optimization_CustomAnnotation.Internal.Icons.ConstraintGUI;
        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Equality(
                active=active,
                required=required,
                nominal=nominal,
                T=T)))), Placement(transformation(extent={{-140,-20},{-100,20}}),
              iconTransformation(extent={{-140,-20},{-100,20}})));

        annotation (Icon(graphics={
              Line(points={{-84,66},{-84,-82}}, color={192,192,192}),
              Polygon(
                points={{-84,88},{-92,66},{-76,66},{-84,88}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(
                points={{3,-4},{3,-83}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{3,-4},{-5,-28},{11,-28},{3,-4}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Line(points={{-84,-3.5},{87,-3.5}}, color={255,0,0}),
              Text(
                extent={{-250,-130},{242,-150}},
                lineColor={0,0,0},
                textString="%required"),
              Line(
                points={{3,76},{3,-3}},
                color={255,0,0},
                smooth=Smooth.Bezier),
              Polygon(
                points={{3,-4},{-5,19},{12,19},{3,-4}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={1,1})));
      end Equality;
    end Constraints;

    package Tuners
      extends Modelica.Icons.Package;
      connector Input "The variable is constrained to stay in a defined band"
        import Optimization_CustomAnnotation.Internal.Constants.*;
        extends Optimization_CustomAnnotation.Tuners.Input;
        extends Optimization_CustomAnnotation.Internal.Icons.TunerGUI;

        Modelica.Blocks.Interfaces.RealInput u annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Tuner(
                active=active,
                min=min,
                max=max,
                nominal=nominal,
                derMin=derMin,
                derMax=derMax,
                u_tStart=u_tStart,
                der_u_tStart=der_u_tStart,
                der_u_tStop = der_u_tStop,
                der2_u_tStop = der2_u_tStop,
                u_init=u_init)))), Placement(transformation(extent={{44,-20},{
                  84,20}}),
              iconTransformation(extent={{44,-20},{84,20}})));

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}),
                         graphics={Polygon(
                points={{-60,90},{-24,2},{-64,-90},{92,0},{-60,90}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,170},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
                                   Polygon(
                points={{-68,90},{-32,2},{-72,-90},{84,0},{-68,90}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,170},
                fillPattern=FillPattern.Solid)}));
      end Input;

    end Tuners;
  end GUIBlocks;

  package TextualBlocks
    package Tasks "Global options of optimization setup tasks"
      extends Modelica.Icons.Package;
      model ModelPredictiveControl
        "Global options for model predictive control (online-optimization to optimize control inputs)"

        extends
          Optimization_CustomAnnotation.Internal.Tasks.ModelPredictiveControl;

        annotation (Optimization_CustomAnnotation(Internal(Tasks(
                   ModelPredictiveControl(method=method, tolerance=tolerance,
                   samplePeriod=samplePeriod, modelHorizon=modelHorizon,
                   controlHorizon=controlHorizon)))));

      end ModelPredictiveControl;

      model TrajectoryOptimization
        "Global options for trajectory optimization (offline-optimization to optimize input trajectories)"

        extends
          Optimization_CustomAnnotation.Internal.Tasks.TrajectoryOptimization;

        annotation (Optimization_CustomAnnotation(Internal(Tasks(
                    TrajectoryOptimization(method=method, tolerance=tolerance, startTime=startTime, freeStopTime=freeStopTime,
                     stopTime=stopTime, minStopTime=minStopTime,maxStopTime=maxStopTime)))));
      end TrajectoryOptimization;

      model StaticOptimization
        "Global options for static optimization (single instant-optimization to optimize parameters)"

        extends Optimization_CustomAnnotation.Internal.Tasks.StaticOptimization;

        annotation (Optimization_CustomAnnotation(Internal(Tasks(
                   StaticOptimization(method=method, tolerance=tolerance)))));

      end StaticOptimization;
    end Tasks;

    package Criteria
      extends Modelica.Icons.Package;
      block Integral
        "The integral of the variable is minimized (Lagrange term)"
        input Real signal annotation (Dialog,
            Optimization_CustomAnnotation(Internal(Criteria(Integral(active=active,
                  nominal=nominal)))));
        extends Optimization_CustomAnnotation.Internal.Criteria.Integral;
      end Integral;

      block IntegralOfLeastSquares
        "The integral of the least squares error with respect to a reference value is minimized (special Lagrange term)"
        input Real signal annotation (Dialog);
        extends Optimization_CustomAnnotation.Internal.Criteria.Integral;
        parameter Real reference=0.0
          "Reference value. Minimize Integral( (input-reference)^2/nominal * dt)";
        final output Real leastSquares=(signal - reference)^2 annotation (
            Optimization_CustomAnnotation(Internal(Criteria(Integral(active=active,
                  nominal=nominal)))));

      end IntegralOfLeastSquares;

      block Point
        "The variable value at one or several fixed time instants is minimized (Mayer term)"
        input Real signal annotation (
            Optimization_CustomAnnotation(Internal(Criteria(Point(
                active=active,
                nominal=nominal,
                T=T)))), Dialog);
        extends Optimization_CustomAnnotation.Internal.Criteria.Point;
      end Point;

      block VariableAtStopTime
        "The variable value at the final time instant is minimized (Mayer term). The final time instant might be determined by optimization. This is described in the Task definition."
         input Real signal annotation (
            Optimization_CustomAnnotation(Internal(Criteria(VariableAtStopTime(active=
                    active, nominal=nominal)))), Dialog);
         extends
          Optimization_CustomAnnotation.Internal.Criteria.VariableAtStopTime;
      end VariableAtStopTime;

      block PointLeastSquares
        "The variable value at a grid of fixed time instant is minimized in a least squares sense with respect to reference values at these time instants (Mayer term)"

        Modelica.Blocks.Interfaces.RealInput signal annotation (
            Optimization_CustomAnnotation(Internal(Criteria(PointLeastSquares(
                active=active,
                nominal=nominal,
                T=T,
                reference=reference)))), Dialog);

        extends
          Optimization_CustomAnnotation.Internal.Criteria.PointLeastSquares;
      end PointLeastSquares;
    end Criteria;

    package Constraints
      extends Modelica.Icons.Package;
      block Band "The variable is constrained to stay in a defined band"
        input Real signal annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Band(
                active=active,
                min=min,
                max=max,
                nominal=nominal,
                T=T)))), Dialog);
        extends Optimization_CustomAnnotation.Internal.Constraints.Band;
      end Band;

      block LowerBound "The variable is constrained by a lower bound"
        extends Optimization_CustomAnnotation.Internal.Icons.Record;
        input Real signal annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Band(
                active=active,
                min=min,
                nominal=nominal,
                T=T)))),Dialog);

        parameter Boolean active=true
          "= true, if the constraint is taken into account";
        parameter Real min=0 "Minimum value of variable";
        parameter Real nominal=1.0
          "Scale constraint: min/nominal <= variable/nominal";
        parameter Real T[:]=fill(0.0, 0)
          "Time instants at which the constraint shall be fulfilled. If zero-dimension, constraint holds for all time instants";
      end LowerBound;

      block UpperBound "The variable is constrained by an upper bound"
        extends Optimization_CustomAnnotation.Internal.Icons.Record;
        input Real signal annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Band(
                active=active,
                max=max,
                nominal=nominal,
                T=T)))), Dialog);

        parameter Boolean active=true
          "= true, if the constraint is taken into account";
        parameter Real max "Maximum value of variable";
         parameter Real nominal=1.0
          "Scale constraint: max/nominal >= variable/nominal";
        parameter Real T[:]=fill(0.0, 0)
          "Time instants at which the constraint shall be fulfilled. If zero-dimension, constraint holds for all time instants";
      end UpperBound;

      block Equality "The variable is constrained to a required value"
        input Real signal annotation (
            Optimization_CustomAnnotation(Internal(Constraints(Equality(
                active=active,
                required=required,
                nominal=nominal,
                T=T)))),Dialog);
        extends Optimization_CustomAnnotation.Internal.Constraints.Equality;
      end Equality;

       package Variant2
        block Constraint
            input
            Optimization_CustomAnnotation.Internal.Interfaces.BasicConstraint       condition;
            input Real[:] T=fill(0, 0);
            input Real nominal=1.0;
            input Boolean active=true;
            Real res=condition.res annotation (Optimization_CustomAnnotation(Internal(Constraints(
              Band(
              active=active,
              min=minValue,
              max=0.0,
              nominal=nominal,
              T=T)))));

        protected
            Real minValue;
        algorithm
            minValue := if condition.op <> "==" then 0.0 else -Modelica.Constants.inf;
        end Constraint;

        operator record OptimizationVariable
            Real exp;

            encapsulated operator 'constructor'
              import
              Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.OptimizationVariable;
              function fromReal
                input Real exp;
                output OptimizationVariable res(exp=exp);

              algorithm
                annotation (Inline=true);
              end fromReal;

            end 'constructor';

            encapsulated operator function '<'
              import
              Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.OptimizationVariable;
              import
              Optimization_CustomAnnotation.Internal.Interfaces.BasicConstraint;
              input OptimizationVariable lhs;
              input OptimizationVariable rhs;
              output BasicConstraint res(res=lhs.exp - rhs.exp, op="<");
              // lhs < rhs
            algorithm

              annotation (Inline=true);
            end '<';

            encapsulated operator function '<='
              import
              Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.OptimizationVariable;
              import
              Optimization_CustomAnnotation.Internal.Interfaces.BasicConstraint;
              input OptimizationVariable lhs;
              input OptimizationVariable rhs;
              output BasicConstraint res(res=lhs.exp - rhs.exp, op="<=");
              // lhs < rhs
            algorithm

              annotation (Inline=true);
            end '<=';

            encapsulated operator function '>'
              import
              Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.OptimizationVariable;
              import
              Optimization_CustomAnnotation.Internal.Interfaces.BasicConstraint;
              input OptimizationVariable lhs;
              input OptimizationVariable rhs;
              output BasicConstraint res(res=rhs.exp - lhs.exp, op="<");
              // rhs < lhs
            algorithm

              annotation (Inline=true);
            end '>';

            encapsulated operator function '>='
              import
              Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.OptimizationVariable;
              import
              Optimization_CustomAnnotation.Internal.Interfaces.BasicConstraint;
              input OptimizationVariable lhs;
              input OptimizationVariable rhs;
              output BasicConstraint res(res=rhs.exp - lhs.exp, op="<=");
              // rhs <= lhs
            algorithm

              annotation (Inline=true);
            end '>=';

            encapsulated operator function '=='
              import
              Optimization_CustomAnnotation.TextualBlocks.Constraints.Variant2.OptimizationVariable;
              import
              Optimization_CustomAnnotation.Internal.Interfaces.BasicConstraint;
              input OptimizationVariable lhs;
              input OptimizationVariable rhs;
              output BasicConstraint res(res=rhs.exp - lhs.exp, op="==");
              // rhs == lhs
            algorithm

              annotation (Inline=true);
            end '==';

        end OptimizationVariable;
       end Variant2;
    end Constraints;
  end TextualBlocks;

  package Internal
    extends Modelica.Icons.InternalPackage;
    package Tasks "Global options of optimization setup tasks"
      extends Modelica.Icons.Package;
      record ModelPredictiveControl
        "Global options for model predictive control (online-optimization to optimize control inputs)"
        extends Optimization_CustomAnnotation.Internal.Icons.Record;
        constant String version="1.0"
          "Version of the optimization specification used";
        parameter String method=""
          "Optimization method (empty string means: default method)";
        parameter Real tolerance=1e-3
          "Relative error tolerance for the optimization solution";
        parameter Modelica.SIunits.Time samplePeriod=0.1
          "Sample period for the online optimization";
        parameter Integer modelHorizon(min=1)=1
          "Number of samples for which the model behavior is predicted";
        parameter Integer controlHorizon(min=1)=1
          "Number of samples provided for the predicted control inputs";
      end ModelPredictiveControl;

      record TrajectoryOptimization
        "Global options for trajectory optimization (offline-optimization to optimize input trajectories)"
        import Optimization_CustomAnnotation.Internal.Constants.*;
        extends Optimization_CustomAnnotation.Internal.Icons.Record;
        constant String version="1.0"
          "Version of the optimization specification used";
        parameter String method=""
          "Optimization method (empty string means: default method)";
        parameter Real tolerance=1e-3
          "Relative error tolerance for the optimization solution";
        parameter Modelica.SIunits.Time startTime=0.0
          "Start time of simulation";
        parameter Boolean freeStopTime=false
          "= true: stop time shall be optimized"
          annotation (choices(checkBox=true));
        parameter Modelica.SIunits.Time stopTime=1.0
          "Stop time of simulation (freeStopTime=false) or initial guess of stop time (freeStopTime=true)";
        parameter Modelica.SIunits.Time minStopTime=UndefinedReal
          "Minimum value of stopTime (if freeStopTime=true)"
          annotation (Dialog(enable=freeStopTime));
        parameter Modelica.SIunits.Time maxStopTime=UndefinedReal
          "Maximum value of stopTime (if freeStopTime=true)"
          annotation (Dialog(enable=freeStopTime));
      end TrajectoryOptimization;

      record StaticOptimization
        "Global options for static optimization (single instant-optimization to optimize parameters)"
        extends Optimization_CustomAnnotation.Internal.Icons.Record;
        constant String version="1.0"
          "Version of the optimization specification used";
        parameter String method=""
          "Optimization method (empty string means: default method)";
        parameter Real tolerance=1e-3
          "Relative error tolerance for the optimization solution";
      end StaticOptimization;
    end Tasks;

    package Criteria
      extends Modelica.Icons.Package;
      record Integral
        "The integral of the variable is minimized (Lagrange term)"
        parameter Boolean active=true
          "= true, if criterion is optimized. Otherwise it is not summed into the cost function.";
        parameter Real nominal=1.0
          "Nominal value of criterion (variable/nominal is minimized)";
      end Integral;

      record Point
        "The variable value at one or several fixed time instants is minimized (Mayer term)"

        parameter Boolean active=true
          "= true, if criterion is optimized. Otherwise it is not summed into the cost function.";
        parameter Real nominal=1.0
          "Nominal value of criterion (variable/nominal is minimized)";
        parameter Modelica.SIunits.Time T[:]=fill(0.0, 0)
          "Time instants when variable/nominal should be summed into the cost function.";
      end Point;

      record VariableAtStopTime
        "The variable value at the final time instant is minimized (Mayer term). The final time instant might be determined by optimization. This is described in the Task definition."
        parameter Boolean active=true
          "= true, if criterion is optimized. Otherwise it is not summed into the cost function.";
        parameter Real nominal=1.0
          "Nominal value of criterion (variable/nominal is minimized)";
      end VariableAtStopTime;

      record PointLeastSquares
        "The variable value at a grid of fixed time instant is minimized in a least squares sense with respect to reference values at these time instants (Mayer term)"
        parameter Boolean active=true
          "= true, if criterion is optimized. Otherwise it is not summed into the cost function.";
        parameter Real nominal=1.0
          "Nominal value of criterion ((variable(T[i])-reference(T[i]))^2)/nominal is added to the cost function)";
        parameter Modelica.SIunits.Time T[:]=fill(0.0,0)
          "The values of the variable at the fixed time instants T are minimized.";
        parameter Real reference[size(T, 1)]=fill(0.0,size(T,1))
          "Reference values at T ((variable(T[i])-reference(T[i]))^2)/nominal is added to the cost function).";

      end PointLeastSquares;
    end Criteria;

    package Constraints "Package containing constraint definitions"
      record Band
        "The variable is constrained by an upper and/or lower bound. If they coincide, this is an equality constraint. Bound constraints must be specified using this annotation; the min and max attributes of variables should not be used as optimization bounds."
        parameter Boolean active=true
          "= true, if constraint is taken into account";
        parameter Real min=-Modelica.Constants.inf
          "Optional minimum value of variable";
        parameter Real max=Modelica.Constants.inf
          "Optional maximum value of variable";
        parameter Real nominal=1.0
          "Scale constraint: min/nominal <= variable/nominal <= max/nominal";
        parameter Real T[:]=fill(0.0, 0)
          "Time instants at which the band constraint shall be fulfilled. If zero-dimension, band constraint holds for all time instants";
      end Band;
      extends Modelica.Icons.Package;

      record Equality "The variable is constrained to a required value"
        parameter Boolean active=true
          "= true, if constraint is taken into account";
        parameter Real required=0.0 "Required value of variable";
        parameter Real nominal=1.0
          "Scale constraint: variable/nominal = required/nominal";
        parameter Real T[:]=fill(0.0, 0)
          "Time instants at which the equality constraint shall be fulfilled. If zero-dimension, equality constraint holds for all time instants";
      end Equality;
    end Constraints;

    package Constants
      extends Modelica.Icons.Package;

      constant Real UndefinedReal=Modelica.Constants.inf;
      constant Integer UndefinedInteger=Modelica.Constants.Integer_inf;

    end Constants;

    package Interfaces
      extends Modelica.Icons.InterfacesPackage;

        record BasicConstraint
          Real res;
          constant String op;
        end BasicConstraint;

    end Interfaces;

    package Icons
      partial block CriterionGUI "Partial Icon for a criterion"

        annotation (Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              initialScale=0.06), graphics={
              Text(
                extent={{-250,170},{250,110}},
                textString="%name",
                lineColor={0,0,255}),
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={255,137,0},
                lineThickness=5,
                fillColor={235,255,230},
                borderPattern=BorderPattern.Raised,
                fillPattern=FillPattern.Solid)}));

      end CriterionGUI;

      partial block ConstraintGUI "Partial icon for a constraint"
        annotation (Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              initialScale=0.06), graphics={
              Text(
                extent={{-250,170},{250,110}},
                textString="%name",
                lineColor={0,0,255}),
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={255,137,0},
                lineThickness=5,
                fillColor={255,255,231},
                borderPattern=BorderPattern.Raised,
                fillPattern=FillPattern.Solid)}));
      end ConstraintGUI;

      partial record Record "Icon for optimization annotation record"

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={
              Text(
                lineColor={0,0,255},
                extent={{-250,60},{250,100}},
                textString="%name"),
              Rectangle(
                origin={0,-23},
                lineColor={64,64,64},
                fillColor={255,252,156},
                fillPattern=FillPattern.Solid,
                extent={{-98,-75},{98,75}},
                radius=25),
              Line(points={{-100.0,0.0},{100.0,0.0}}, color={64,64,64}),
              Line(
                origin={0.0,-50.0},
                points={{-100.0,0.0},{100.0,0.0}},
                color={64,64,64}),
              Line(
                origin={0.0,-25.0},
                points={{0.0,75.0},{0.0,-75.0}},
                color={64,64,64})}));
      end Record;

      partial block TunerGUI "Partial icon for a constraint"
        annotation (Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              initialScale=0.06), graphics={
              Text(
                extent={{-250,170},{250,110}},
                textString="%name",
                lineColor={0,0,255}),
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={255,137,0},
                lineThickness=5,
                fillColor={255,170,170},
                borderPattern=BorderPattern.Raised,
                fillPattern=FillPattern.Solid)}));
      end TunerGUI;
    end Icons;

  end Internal;

  annotation (uses(Modelica(version="3.2.1")),
  Documentation(info="<html>  
  <p>
<i>This Modelica package is <u>free</u> software and
the use is completely at <u>your own risk</u>;
it can be redistributed and/or modified under the terms of the
Modelica license 2, see the license conditions (including the
disclaimer of warranty) at
<a href=\"https://www.Modelica.org/licenses/ModelicaLicense2\">
https://www.Modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>


<p>
<b>Copyright &copy; 2013-2015, DLR Institute of System Dynamics and Control, FH Bielefeld, Modelon AB</b>
</p> 
<p>
</p>
  </html>"));
end Optimization_CustomAnnotation;
