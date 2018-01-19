within ;
package MultiModeExamples
  "Multi-mode examples to evaluate multi-mode handling in tool prototypes - Version 0.2"
package UsersGuide "User's Guide"
  extends Modelica.Icons.Information;

package ReleaseNotes "Release notes"
  extends Modelica.Icons.ReleaseNotes;

  class Version_0_2 "Version 0.2 (Sept. 2, 2014)"
  extends Modelica.Icons.ReleaseNotes;

    annotation (Documentation(info="<html>

<p>
Clean-up of the library, improved documentation, adding further
test models from previous tests of Hilding Elmqvist and Martin Otter.
</p>

</html>"));
  end Version_0_2;

class Version_0_1 "Version 0.1 (Feb. 2, 2014)"
  extends Modelica.Icons.ReleaseNotes;

annotation (Documentation(info="<html>

<p>
First version of library combining test models from 
Hilding Elmqvist, Martin Otter and performing a clean-up of the
models and re-structuring of the library.
<p>

</html>"));
end Version_0_1;
 annotation (Documentation(info="<html>

<p>
This section summarizes the changes that have been performed
on package Properties2.
</p>
</html>"));
end ReleaseNotes;

class Contact "Contact"
  extends Modelica.Icons.Contact;

 annotation (Documentation(info="<html>
<dl>
<dt><b>Main Authors</b></dt>
<dd>Martin Otter<br>
    German Aerospace Center (DLR)<br>
    Robotics and Mechatronics Center<br>
    Institute of System Dynamics and Control<br>
    Postfach 1116<br>
    D-82230 Wessling<br>
    Germany<br>
    email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a><br>&nbsp;<br>

    Hilding Elmqvist<br>
    Dassault Systèmes AB<br>
    Research Park Ideon<br>
    Ole Römersväg 16<br>
    S-22370 Lund<br>
    Sweden<br>
    email: <a href=\"mailto:Hilding.Elmqvist@3ds.com\">Hilding.Elmqvist@3ds.com</a><br>
</dd>
</dl>


<p><b>Acknowledgements:</b></p>
<ul>
<li> This library is based on research performed within the ITEA2 project MODRIO.
     Partial financial support of the Swedish VINNOVA and the German BMBF for this
     development are highly appreciated.</li>
<li> This library was used to evaluated the Dymola prototype
     of Sven Erik Mattsson supporting
     multi-mode modeling.</li>
</ul>
</html>"));

end Contact;

annotation (DocumentationClass=true, Documentation(info="<html>
<p>
Library <b>MultiModeExamples</b> is Modelica package with a collection
of models to test multi-mode modeling.
</p>
</html>"));
end UsersGuide;
   extends Modelica.Icons.Package;
   package WithSpecialSwitch
    "Multi-mode examples with special switch components that are Modelica 3.2 compliant"
     model TwoBlocks "Switching between two input/output blocks"
       Integer state = if time < 0.5 then 1 else 2;
       extends Modelica.Icons.Example;
       Interfaces.ConnectInputsOf2States switchIn(state=state)
         annotation (Placement(transformation(extent={{-46,-6},{-34,6}})));
       Interfaces.ConnectOutputsOf2States switchOut(state=state)
         annotation (Placement(transformation(extent={{30,-6},{42,6}})));
       Modelica.Blocks.Continuous.FirstOrder firstOrder(       initType=Modelica.Blocks.Types.Init.SteadyState, T=0.2)
         annotation (Placement(transformation(extent={{-14,10},{6,30}})));
       Modelica.Blocks.Continuous.SecondOrder secondOrder(
         initType=Modelica.Blocks.Types.Init.SteadyState,
         w=25,
         D=0.2)
         annotation (Placement(transformation(extent={{-14,-36},{6,-16}})));
       Modelica.Blocks.Sources.Step step(startTime=0.1)
         annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
     equation
       when state==2 then
          reinit(secondOrder.y, firstOrder.y);
          reinit(secondOrder.yd, der(firstOrder.y));
       end when;

       connect(switchIn.y1, firstOrder.u) annotation (Line(
           points={{-33,4},{-26,4},{-26,20},{-16,20}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(firstOrder.y, switchOut.u1) annotation (Line(
           points={{7,20},{14,20},{14,4},{28,4}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(switchIn.y2, secondOrder.u) annotation (Line(
           points={{-33,-4},{-26,-4},{-26,-26},{-16,-26}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(secondOrder.y, switchOut.u2) annotation (Line(
           points={{7,-26},{16,-26},{16,-4},{28,-4}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(step.y, switchIn.u) annotation (Line(
           points={{-65,0},{-48,0}},
           color={0,0,127},
           smooth=Smooth.None));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), experiment(StopTime=1.1));
     end TwoBlocks;
      extends Modelica.Icons.Package;
     model CircuitCapacitorResistor "Switching from a capacitor to a resistor"
       extends Modelica.Icons.Example;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-32,-34},{-12,-14}})));
       Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
         annotation (Placement(transformation(extent={{-8,50},{12,70}})));
       Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-22,20})));
       Modelica.Electrical.Analog.Basic.Capacitor C(C=0.001, v(fixed=true))
                                                             annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={10,20})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=100) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={48,20})));
       Interfaces.ConnectPinsOf2States switch1(state=state) annotation (Placement(
             transformation(
             extent={{-6,6},{6,-6}},
             rotation=-90,
             origin={30,42})));
       Interfaces.ConnectPinsOf2States switch2(state=state) annotation (Placement(
             transformation(
             extent={{6,6},{-6,-6}},
             rotation=-90,
             origin={30,-2})));
     equation
      connect(voltage.p, R1.p) annotation (Line(
          points={{-22,30},{-22,60},{-8,60}},
          color={0,0,255},
          smooth=Smooth.None));

      connect(voltage.n, ground.p) annotation (Line(
          points={{-22,10},{-22,-14}},
          color={0,0,255},
          smooth=Smooth.None));
       connect(C.p, switch1.n1) annotation (Line(
           points={{10,30},{26,30},{26,36}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.p, switch1.n2) annotation (Line(
           points={{48,30},{34,30},{34,36}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(C.n, switch2.n1) annotation (Line(
           points={{10,10},{26,10},{26,4}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.n, switch2.n2) annotation (Line(
           points={{48,10},{34,10},{34,4}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R1.n, switch1.p) annotation (Line(
           points={{12,60},{30,60},{30,48}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(ground.p, switch2.p) annotation (Line(
           points={{-22,-14},{30,-14},{30,-8}},
           color={0,0,255},
           smooth=Smooth.None));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), experiment(StopTime=1.0));
     end CircuitCapacitorResistor;

     model CircuitCapacitorResistor_state1
      "Demonstrating state=1 of model CircuitCapacitorResistor"
       extends Modelica.Icons.Example;
       parameter Real Gsmall=1e-10;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-32,-34},{-12,-14}})));
       Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
         annotation (Placement(transformation(extent={{-8,50},{12,70}})));
       Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-22,20})));
       Modelica.Electrical.Analog.Basic.Capacitor C(C=0.001, v(fixed=true))
                                                             annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={10,20})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=100) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={48,20})));
       Utilities.Conductor conductor1(G=Gsmall)
         annotation (Placement(transformation(extent={{56,30},{76,50}})));
       Utilities.Conductor conductor2(G=Gsmall)
         annotation (Placement(transformation(extent={{56,-10},{76,10}})));
       Utilities.Ground g1
         annotation (Placement(transformation(extent={{76,28},{88,40}})));
       Utilities.Ground g2
         annotation (Placement(transformation(extent={{76,-12},{88,0}})));
     equation
      connect(voltage.p, R1.p) annotation (Line(
          points={{-22,30},{-22,60},{-8,60}},
          color={0,0,255},
          smooth=Smooth.None));

      connect(voltage.n, ground.p) annotation (Line(
          points={{-22,10},{-22,-14}},
          color={0,0,255},
          smooth=Smooth.None));
       connect(ground.p, C.n) annotation (Line(
           points={{-22,-14},{28,-14},{28,10},{10,10}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(C.p, R1.n) annotation (Line(
           points={{10,30},{28,30},{28,60},{12,60}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.p, conductor1.p) annotation (Line(
           points={{48,30},{48,40},{56,40}},
           color={95,95,95},
           smooth=Smooth.None));
       connect(R2.n, conductor2.p) annotation (Line(
           points={{48,10},{48,0},{56,0}},
           color={95,95,95},
           smooth=Smooth.None));
       connect(conductor1.n, g1.p) annotation (Line(
           points={{76,40},{82,40}},
           color={95,95,95},
           smooth=Smooth.None));
       connect(conductor2.n, g2.p) annotation (Line(
           points={{76,0},{82,0}},
           color={95,95,95},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})));
     end CircuitCapacitorResistor_state1;

     model CircuitCapacitorResistor_state2
      "Demonstrating state=2 of model CircuitCapacitorResistor"
       extends Modelica.Icons.Example;
       parameter Real Gsmall=1e-10;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-32,-34},{-12,-14}})));
       Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
         annotation (Placement(transformation(extent={{-8,50},{12,70}})));
       Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-22,20})));
       Modelica.Electrical.Analog.Basic.Capacitor C(C=0.001, v(fixed=true))
                                                             annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={28,20})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=100) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={60,20})));
       Utilities.Conductor conductor1(G=Gsmall)
         annotation (Placement(transformation(extent={{2,28},{22,48}})));
       Utilities.Conductor conductor2(G=Gsmall)
         annotation (Placement(transformation(extent={{2,-10},{22,10}})));
       Utilities.Ground g1
         annotation (Placement(transformation(extent={{-10,26},{2,38}})));
       Utilities.Ground g2
         annotation (Placement(transformation(extent={{-10,-12},{2,0}})));
     equation
      connect(voltage.p, R1.p) annotation (Line(
          points={{-22,30},{-22,60},{-8,60}},
          color={0,0,255},
          smooth=Smooth.None));

      connect(voltage.n, ground.p) annotation (Line(
          points={{-22,10},{-22,-14}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(conductor1.n, C.p) annotation (Line(
          points={{22,38},{28,38},{28,30}},
          color={95,95,95},
          smooth=Smooth.None));
      connect(conductor2.n, C.n) annotation (Line(
          points={{22,0},{28,0},{28,10}},
          color={95,95,95},
          smooth=Smooth.None));
      connect(conductor2.p, g2.p) annotation (Line(
          points={{2,0},{-4,0}},
          color={95,95,95},
          smooth=Smooth.None));
      connect(R1.n, R2.p) annotation (Line(
          points={{12,60},{40,60},{40,40},{40,40},{40,30},{60,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ground.p, R2.n) annotation (Line(
          points={{-22,-14},{48,-14},{48,10},{60,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(g1.p, conductor1.p) annotation (Line(
          points={{-4,38},{2,38}},
          color={95,95,95},
          smooth=Smooth.None));
       annotation (experiment(StopTime=1.0),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})));
     end CircuitCapacitorResistor_state2;

     model CircuitParallelCapacitorResistor
      "Switching between two resistors, that are in parallel to a capacitor"
       extends Modelica.Icons.Example;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-32,-34},{-12,-14}})));
       Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
         annotation (Placement(transformation(extent={{-8,50},{12,70}})));
       Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-22,20})));
       Modelica.Electrical.Analog.Basic.Resistor R3(R=200) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={10,20})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=100) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={48,20})));
       Interfaces.ConnectPinsOf2States switch1(state=state) annotation (Placement(
             transformation(
             extent={{-6,6},{6,-6}},
             rotation=-90,
             origin={30,42})));
       Interfaces.ConnectPinsOf2States switch2(state=state) annotation (Placement(
             transformation(
             extent={{6,6},{-6,-6}},
             rotation=-90,
             origin={30,-2})));
       Modelica.Electrical.Analog.Basic.Capacitor C2(
                                                    C=0.001, v(fixed=true))
                                                             annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={80,20})));
     equation
      connect(voltage.p, R1.p) annotation (Line(
          points={{-22,30},{-22,60},{-8,60}},
          color={0,0,255},
          smooth=Smooth.None));

      connect(voltage.n, ground.p) annotation (Line(
          points={{-22,10},{-22,-14}},
          color={0,0,255},
          smooth=Smooth.None));
       connect(R3.p, switch1.n1) annotation (Line(
           points={{10,30},{26,30},{26,36}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.p, switch1.n2) annotation (Line(
           points={{48,30},{34,30},{34,36}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R3.n, switch2.n1) annotation (Line(
           points={{10,10},{26,10},{26,4}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.n, switch2.n2) annotation (Line(
           points={{48,10},{34,10},{34,4}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R1.n, switch1.p) annotation (Line(
           points={{12,60},{30,60},{30,48}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(ground.p, switch2.p) annotation (Line(
           points={{-22,-14},{30,-14},{30,-8}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(switch1.p, C2.p) annotation (Line(
           points={{30,48},{30,60},{80,60},{80,30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(switch2.p, C2.n) annotation (Line(
           points={{30,-8},{30,-14},{80,-14},{80,10}},
           color={0,0,255},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})));
     end CircuitParallelCapacitorResistor;

     model CircuitParallelCapacitors
      "Varying index system (switching from a capacitor to a resistor and these components are in parallel to a capacitor)"
       extends Modelica.Icons.Example;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-32,-34},{-12,-14}})));
       Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
         annotation (Placement(transformation(extent={{-8,50},{12,70}})));
       Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-22,20})));
       Modelica.Electrical.Analog.Basic.Capacitor C1(C=0.001, v(fixed=true))
                                                              annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={10,20})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=100) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={48,20})));
       Interfaces.ConnectPinsOf2States_specialCase switch1(state=state) annotation (
           Placement(transformation(
             extent={{-6,6},{6,-6}},
             rotation=-90,
             origin={30,42})));
       Interfaces.ConnectPinsOf2States_specialCase switch2(state=state) annotation (
           Placement(transformation(
             extent={{6,6},{-6,-6}},
             rotation=-90,
             origin={30,-2})));
       Modelica.Electrical.Analog.Basic.Capacitor C2(
                                                    C=0.001, v(fixed=true))
                                                             annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={80,20})));
     equation
      connect(voltage.p, R1.p) annotation (Line(
          points={{-22,30},{-22,60},{-8,60}},
          color={0,0,255},
          smooth=Smooth.None));

      connect(voltage.n, ground.p) annotation (Line(
          points={{-22,10},{-22,-14}},
          color={0,0,255},
          smooth=Smooth.None));
       connect(C1.p, switch1.n1) annotation (Line(
           points={{10,30},{26,30},{26,36}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.p, switch1.n2) annotation (Line(
           points={{48,30},{34,30},{34,36}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(C1.n, switch2.n1) annotation (Line(
           points={{10,10},{26,10},{26,4}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.n, switch2.n2) annotation (Line(
           points={{48,10},{34,10},{34,4}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R1.n, switch1.p) annotation (Line(
           points={{12,60},{30,60},{30,48}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(ground.p, switch2.p) annotation (Line(
           points={{-22,-14},{30,-14},{30,-8}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(switch1.p, C2.p) annotation (Line(
           points={{30,48},{30,60},{80,60},{80,30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(switch2.p, C2.n) annotation (Line(
           points={{30,-8},{30,-14},{80,-14},{80,10}},
           color={0,0,255},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})),
         Documentation(info="<html>
<p>
This system has a state machine where a capacitor switches to a resistor.
Furthermore, a capacitor is connected in parallel to this state machine.
This system gives rise to the following issue:
</p>

<ul>
<li> If state=1, two capacitors are in parallel, and index reduction takes place.</li>
<li> If state=2, a capacitor is in parallel to a resistor and no index reduction
     takes place.</li>
</ul>

<p>
As a result, in the different states of the state machine, different index
reduction has to be performed. If the generic switch
<a href=\"modelica://MultiModeExamples.Interfaces.ConnectPinsOf2States\">ConnectPinsOf2States</a>
would be used, this system would fail during simulation, because the Pantelides
algorithm cannot differentiate only part of an equation.
</p>

<p>
In order to be able to process this system correctly, another type of
switch is used:
<a href=\"modelica://MultiModeExamples.Interfaces.ConnectPinsOf2States_specialCase\">ConnectPinsOf2States_specialCase</a>.
This switch makes the assumption that the potential outside of the state machine
is always known and then allows to rewrite the equations so that the two states are
decoupled (are no longer coupled algebraically). As a result, the Pantelides algorithm
can detect and can differentiate the potential equation of state1, and not
differentiating the separate potential equation of state2.
This approach is similar to the rewriting rule given in section 3.2.4 of
(Elmqvist, Mattsson, Otter 2014):
</p>

<ul>
<li> Perform Pantelides and BLT on the differentiated equations 
     with the generic switch equations
     <a href=\"modelica://MultiModeExamples.Interfaces.ConnectPinsOf2States\">ConnectPinsOf2States</a>.
     </li>
<li> If there is an algebraic loop where switch.n1.v and switch.n2.v are unknowns,
     and switch.p.v is a known (computed before the algebraic loop is entered),
     then rewrite the equations of the switch from
<pre>
  p.v = if state==1 then n1.v else n2.v;
  0 = p.i + (if state==1 then n1.i else n2.i);
  0 = if state==1 then Gsmall*n2.v - n2.i else Gsmall*n1.v - n1.i;
</pre>
to
<pre>
  n1.v = if state==1 then p.v else last(p.v)
  n2.v = if state==2 then p.v else last(p.v)
  0 = p.i + (if state==1 then n1.i else n2.i);
</pre>
These are the equations of the switch
<a href=\"modelica://MultiModeExamples.Interfaces.ConnectPinsOf2States_specialCase\">ConnectPinsOf2States_specialCase</a>
used in this example.
</li>
</ul>
     
<p>
Operator last(p.v) means the value of p.v at the time when the corresponding state was 
active the last time. In standard Modelica such an operator is not present and
therefore in the switch
<a href=\"modelica://MultiModeExamples.Interfaces.ConnectPinsOf2States_specialCase\">ConnectPinsOf2States_specialCase</a>
the dummy value last(p.v)=0 is used instead.
</p>

<p>
The same example is also present with the generic switch as
<a href=\"modelica://MultiModeExamples.WithSpecialSwitch_Fails.CircuitParallelCapacitors\">WithSpecialSwitch_Fails.CircuitParallelCapacitors</a>.  Translating this model results in an algebraic system of equations with
the unknowns C1.n.v, C1.i, R2.i, R2.n.v and switch1.p.v and switch2.p.v are computed before
the algebraic loop is entered. Therefore, the above rewriting rule can be applied.
</p>

<p>
Note, everything is fine when switching from state==1 to state==2. 
However, when switching back from state==2 to state==1 then C2.v and C1.v
have usually different values before the switching and the same value
after switching. As a result a Dirac impulse must occur which means that the values
of C2.v and C1.v are re-initialized at the transition from state==2 to state==1.
</p>
</html>"));
     end CircuitParallelCapacitors;

     model BreakingInertia1
      "Drive train with breaking inertia, variant 1 (switching from one to two unconnected inertias)"
       extends Modelica.Icons.Example;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Mechanics.Rotational.Sources.Torque torque
         annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
       Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=5000,
           d=20,
         stateSelect=StateSelect.never)
         annotation (Placement(transformation(extent={{74,-10},{94,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{104,-10},{124,10}})));
       Modelica.Blocks.Sources.Step step
         annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia(J=10,
         phi(fixed=true, start=0),
         w(fixed=true, start=0))
         annotation (Placement(transformation(extent={{0,10},{20,30}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=9,
         phi(fixed=true, start=0),
         w(fixed=true, start=0))
         annotation (Placement(transformation(extent={{-12,-30},{8,-10}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1,
         phi(fixed=true, start=0),
         w(fixed=true, start=0))
         annotation (Placement(transformation(extent={{16,-30},{36,-10}})));
       Interfaces.ConnectFlangesOf2States switch1(state=state)
         annotation (Placement(transformation(extent={{-50,-6},{-38,6}})));
       Interfaces.ConnectFlangesOf2States switch2(state=state)
         annotation (Placement(transformation(extent={{62,-6},{50,6}})));
     equation
       when state==2 then
         reinit(inertia1.phi, inertia.phi);
         reinit(inertia1.w, inertia.w);
         reinit(inertia2.phi, inertia.phi);
         reinit(inertia2.w, inertia.w);
       end when;

       connect(springDamper.flange_b,fixed. flange) annotation (Line(
           points={{94,0},{114,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(step.y,torque. tau) annotation (Line(
           points={{-93,0},{-82,0}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(torque.flange, switch1.flange_a) annotation (Line(
           points={{-60,0},{-50,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch1.flange_b1, inertia.flange_a) annotation (Line(
           points={{-38,4},{-26,4},{-26,20},{0,20}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch1.flange_b2, inertia1.flange_a) annotation (Line(
           points={{-38,-4},{-26,-4},{-26,-20},{-12,-20}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch2.flange_b1, inertia.flange_b) annotation (Line(
           points={{50,4},{42,4},{42,20},{20,20}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia2.flange_b, switch2.flange_b2) annotation (Line(
           points={{36,-20},{42,-20},{42,-4},{50,-4}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch2.flange_a, springDamper.flange_a) annotation (Line(
           points={{62,0},{74,0}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0, Tolerance=1e-006), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                 -100},{140,100}})),
         Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
     end BreakingInertia1;

     model BreakingInertia2
      "Drive train with breaking inertia, variant 2 (switching from one to two unconnected inertias)"
       extends Modelica.Icons.Example;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Mechanics.Rotational.Sources.Torque torque
         annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
       Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=5000,
           d=20,
         stateSelect=StateSelect.never)
         annotation (Placement(transformation(extent={{74,-10},{94,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{104,-10},{124,10}})));
       Modelica.Blocks.Sources.Step step
         annotation (Placement(transformation(extent={{-136,-10},{-116,10}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia(J=10, w(fixed=true,
             start=1),
         phi(fixed=true, start=0))
         annotation (Placement(transformation(extent={{0,10},{20,30}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=9, phi(fixed=true,
             start=0))
         annotation (Placement(transformation(extent={{-12,-30},{8,-10}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1,
         phi(fixed=true, start=0),
         w(fixed=true, start=0))
         annotation (Placement(transformation(extent={{16,-30},{36,-10}})));
       Interfaces.ConnectFlangesOf2States switch1(state=state)
         annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
       Interfaces.ConnectFlangesOf2States switch2(state=state)
         annotation (Placement(transformation(extent={{62,-6},{50,6}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia3(
                                                                J=10, w(fixed=true,
             start=1))
         annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
     equation
       when state==2 then
         reinit(inertia1.phi, inertia.phi);
         reinit(inertia1.w, inertia.w);
         reinit(inertia2.phi, inertia.phi);
         reinit(inertia2.w, inertia.w);
       end when;

       connect(springDamper.flange_b,fixed. flange) annotation (Line(
           points={{94,0},{114,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(step.y,torque. tau) annotation (Line(
           points={{-115,0},{-102,0}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(switch1.flange_b1, inertia.flange_a) annotation (Line(
           points={{-30,4},{-26,4},{-26,20},{0,20}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch1.flange_b2, inertia1.flange_a) annotation (Line(
           points={{-30,-4},{-26,-4},{-26,-20},{-12,-20}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch2.flange_b1, inertia.flange_b) annotation (Line(
           points={{50,4},{42,4},{42,20},{20,20}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia2.flange_b, switch2.flange_b2) annotation (Line(
           points={{36,-20},{42,-20},{42,-4},{50,-4}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch2.flange_a, springDamper.flange_a) annotation (Line(
           points={{62,0},{74,0}},
           color={0,0,0},
           smooth=Smooth.None));
      connect(torque.flange, inertia3.flange_a) annotation (Line(
          points={{-80,0},{-72,0}},
          color={0,0,0},
          smooth=Smooth.None));
       connect(inertia3.flange_b, switch1.flange_a) annotation (Line(
           points={{-52,0},{-42,0}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0, Tolerance=1e-006), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                 -100},{140,100}})),
         Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
     end BreakingInertia2;

     model BreakingShaft "Varying index system (breaking shaft)"
       extends Modelica.Icons.Example;
       Integer state = if time <= 0.5 then 1 else 2;
       Modelica.Mechanics.Rotational.Sources.Torque torque
         annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
       Modelica.Mechanics.Rotational.Components.Spring spring(c=5000)
         annotation (Placement(transformation(extent={{74,-10},{94,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{104,-10},{124,10}})));
       Modelica.Blocks.Sources.Step step
         annotation (Placement(transformation(extent={{-136,-10},{-116,10}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia2(
         J=1)
         annotation (Placement(transformation(extent={{40,-10},{60,10}})));
       Interfaces.ConnectFlangesOf2States_specialCase switch1(state=state)
         annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
       Interfaces.ConnectFlangesOf2States_specialCase switch2(state=state)
         annotation (Placement(transformation(extent={{32,-6},{20,6}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia1(
                       J=9,
         phi(fixed=true, start=0),
         w(fixed=true, start=0))
         annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
       Modelica.Mechanics.Rotational.Sources.ConstantTorque t1(final tau_constant=0)
         annotation (Placement(transformation(extent={{-10,-18},{-22,-6}})));
       Modelica.Mechanics.Rotational.Sources.ConstantTorque t2(final tau_constant=0)
         annotation (Placement(transformation(extent={{-2,-18},{10,-6}})));
     equation
       connect(spring.flange_b, fixed.flange) annotation (Line(
           points={{94,0},{114,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(step.y,torque. tau) annotation (Line(
           points={{-115,0},{-102,0}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(torque.flange, inertia1.flange_a) annotation (Line(
           points={{-80,0},{-72,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia1.flange_b, switch1.flange_a) annotation (Line(
           points={{-52,0},{-42,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia2.flange_b, spring.flange_a) annotation (Line(
           points={{60,0},{74,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch2.flange_a, inertia2.flange_a) annotation (Line(
           points={{32,0},{40,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch1.flange_b1, switch2.flange_b1) annotation (Line(
           points={{-30,4},{20,4}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(switch1.flange_b2, t1.flange) annotation (Line(
           points={{-30,-4},{-26,-4},{-26,-12},{-22,-12}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(t2.flange, switch2.flange_b2) annotation (Line(
           points={{10,-12},{14,-12},{14,-4},{20,-4}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.1,Tolerance=1e-006), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                 -100},{140,100}}), graphics),
         Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
         Documentation(info="<html>
<p>
This system has a state machine where two ridigly connected flanges switch
to not connected flanges (= breaking shaft).
This system gives rise to the following issue:
</p>

<ul>
<li> If state=1, two inertias are rigidly connected and index reduction takes place.</li>
<li> If state=2, two inertias are not connected and no index reduction
     takes place.</li>
</ul>

<p>
As a result, in the different states of the state machine, different index
reduction has to be performed. If the generic switch
<a href=\"modelica://MultiModeExamples.Interfaces.ConnectFlangesOf2States\">ConnectFlangesOf2States</a>
would be used, this system would fail during simulation, because the Pantelides
algorithm cannot differentiate only part of an equation.
</p>

<p>
In order to be able to process this system correctly, another type of
switch is used:
<a href=\"modelica://MultiModeExamples.Interfaces.ConnectFlangesOf2States_specialCase\">ConnectFlangesOf2States_specialCase</a>.
This switch works nicely, but it is not yet fully analzyed why this works
so well in this case.
</p>
</html>"));
     end BreakingShaft;

     package Utilities "Library of utility components"
       extends Modelica.Icons.UtilitiesPackage;
     model Conductor "Ideal linear electrical conductor"
       parameter Modelica.SIunits.Conductance G(start=1)
          "Conductance at temperature T_ref";
       parameter Modelica.SIunits.Temperature T_ref=300.15
          "Reference temperature";
       parameter Modelica.SIunits.LinearTemperatureCoefficient alpha=0
          "Temperature coefficient of conductance (G_actual = G_ref/(1 + alpha*(T_heatPort - T_ref))";
       extends Modelica.Electrical.Analog.Interfaces.OnePort;
       extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
                                                                    T = T_ref);
       Modelica.SIunits.Conductance G_actual
          "Actual conductance = G_ref/(1 + alpha*(T_heatPort - T_ref))";

     equation
       assert((1 + alpha*(T_heatPort - T_ref)) >= Modelica.Constants.eps, "Temperature outside scope of model!");
       G_actual = G/(1 + alpha*(T_heatPort - T_ref));
       i = G_actual*v;
       LossPower = v*i;
       annotation (
         Documentation(info="<html>
<p>The linear conductor connects the branch voltage <i>v</i> with the branch current <i>i</i> by <i>i = v*G</i>. The Conductance <i>G</i> is allowed to be positive, zero, or negative.</p>
</html>",
      revisions="<html>
<ul>
<li><i> August 07, 2009   </i>
       by Anton Haumer<br> temperature dependency of conductance added<br>
       </li>
<li><i> March 11, 2009   </i>
       by Christoph Clauss<br> conditional heat port added<br>
       </li>
<li><i> 1998   </i>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),
         Icon(coordinateSystem(
             preserveAspectRatio=true,
             extent={{-100,-100},{100,100}}), graphics={
               Rectangle(
                 extent={{-70,30},{70,-30}},
                 fillColor={255,255,255},
                 fillPattern=FillPattern.Solid,
                 lineColor={0,0,255}),
               Rectangle(extent={{-70,30},{70,-30}}, lineColor={135,135,135}),
               Line(points={{-90,0},{-70,0}}, color={0,0,255}),
               Line(points={{70,0},{90,0}}, color={0,0,255}),
               Line(
                 visible=useHeatPort,
                 points={{0,-100},{0,-30}},
                 color={127,0,0},
                 smooth=Smooth.None,
                 pattern=LinePattern.Dot),
               Text(
                 extent={{-152,87},{148,47}},
                 textString="%name",
                 lineColor={0,0,255}),
               Text(
                 extent={{-144,-38},{142,-70}},
                 lineColor={0,0,0},
                 textString="G=%G")}),
         Diagram(coordinateSystem(
             preserveAspectRatio=true,
             extent={{-100,-100},{100,100}}), graphics={
               Line(points={{-96,0},{-70,0}}, color={0,0,255}),
               Line(points={{70,0},{96,0}}, color={0,0,255}),
               Rectangle(extent={{-70,30},{70,-30}}, lineColor={0,0,255})}));
     end Conductor;

       model Ground "Ground node"

         Modelica.Electrical.Analog.Interfaces.Pin p annotation (Placement(
              transformation(
              origin={0,100},
              extent={{10,-10},{-10,10}},
              rotation=270)));
       equation
         p.v = 0;
         annotation (
           Documentation(info="<html>
<p>Ground of an electrical circuit. The potential at the ground node is zero. Every electrical circuit has to contain at least one ground object.</p>
</html>",
        revisions="<html>
<ul>
<li><i> 1998   </i>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"), Icon(coordinateSystem(
               preserveAspectRatio=false,
               extent={{-100,-100},{100,100}}), graphics={
               Line(points={{-60,50},{60,50}}, color={95,95,95}),
               Line(points={{-40,30},{40,30}}, color={95,95,95}),
               Line(points={{-20,10},{20,10}}, color={95,95,95}),
               Line(points={{0,90},{0,50}}, color={95,95,95}),
               Text(
                 extent={{-144,-19},{156,-59}},
                 textString="%name",
                 lineColor={0,0,255})}),
           Diagram(coordinateSystem(
               preserveAspectRatio=true,
               extent={{-100,-100},{100,100}}), graphics={
               Line(
                 points={{-60,50},{60,50}},
                 thickness=0.5,
                 color={0,0,255}),
               Line(
                 points={{-40,30},{40,30}},
                 thickness=0.5,
                 color={0,0,255}),
               Line(
                 points={{-20,10},{20,10}},
                 thickness=0.5,
                 color={0,0,255}),
               Line(
                 points={{0,96},{0,50}},
                 thickness=0.5,
                 color={0,0,255}),
               Text(
                 extent={{-24,-38},{22,-6}},
                 textString="p.v=0",
                 lineColor={0,0,255})}));
       end Ground;
       annotation (Documentation(info="<html>
<p>
This package contains copies of components of the Modelica Standard Library
that have slightly different icons and are used in some examples.
</p>
</html>"));
     end Utilities;
     annotation (Documentation(info="<html>
<p>
This package contains multi-mode examples that should work with any Modelica 3.2
tool, by using special switch components. These examples allow to analyze the
occuring equation structures and the needed symbolic transformation algorithms
for multi-mode systems, without actually supporting continuous-time state machines.
Note, in order to generate efficient code, the non-active equations should
not be evaluated. This is not possible when using Modelica 3.2 standard conformant
elements and is only possible when using continuous-time state machines.
</p>
</html>"));
   end WithSpecialSwitch;

  package WithSpecialSwitch_Fails
    "Multi-mode examples with special switch components that are Modelica 3.2 compliant, but simulation fails due to varying DAE index"
     extends Modelica.Icons.Package;
    model CircuitParallelCapacitors
      " Varying index system (switching from a capacitor to a resistor and these components are in parallel to a capacitor)"
      extends Modelica.Icons.Example;
      Integer state = if time <= 0.5 then 1 else 2;
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-32,-34},{-12,-14}})));
      Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
        annotation (Placement(transformation(extent={{-8,50},{12,70}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
       annotation (Placement(transformation(
           extent={{-10,-10},{10,10}},
           rotation=-90,
           origin={-22,20})));
      Modelica.Electrical.Analog.Basic.Capacitor C1(C=0.001) annotation (
         Placement(transformation(
           extent={{-10,-10},{10,10}},
           rotation=-90,
           origin={10,20})));
      Modelica.Electrical.Analog.Basic.Resistor R2(R=100) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={48,20})));
      Interfaces.ConnectPinsOf2States switch1(state=state) annotation (Placement(
            transformation(
            extent={{-6,6},{6,-6}},
            rotation=-90,
            origin={30,42})));
      Interfaces.ConnectPinsOf2States switch2(state=state) annotation (Placement(
            transformation(
            extent={{6,6},{-6,-6}},
            rotation=-90,
            origin={30,-2})));
      Modelica.Electrical.Analog.Basic.Capacitor C2(
                                                   C=0.001) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={80,20})));
    equation
     connect(voltage.p, R1.p) annotation (Line(
         points={{-22,30},{-22,60},{-8,60}},
         color={0,0,255},
         smooth=Smooth.None));

     connect(voltage.n, ground.p) annotation (Line(
         points={{-22,10},{-22,-14}},
         color={0,0,255},
         smooth=Smooth.None));
      connect(C1.p, switch1.n1) annotation (Line(
          points={{10,30},{26,30},{26,36}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(R2.p, switch1.n2) annotation (Line(
          points={{48,30},{34,30},{34,36}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(C1.n, switch2.n1) annotation (Line(
          points={{10,10},{26,10},{26,4}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(R2.n, switch2.n2) annotation (Line(
          points={{48,10},{34,10},{34,4}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(R1.n, switch1.p) annotation (Line(
          points={{12,60},{30,60},{30,48}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ground.p, switch2.p) annotation (Line(
          points={{-22,-14},{30,-14},{30,-8}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(switch1.p, C2.p) annotation (Line(
          points={{30,48},{30,60},{80,60},{80,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(switch2.p, C2.n) annotation (Line(
          points={{30,-8},{30,-14},{80,-14},{80,10}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
               -100},{100,100}})),
        Documentation(info="<html>
<p>
This system does not simulate, because it has a varying DAE index.
It is possible to reformulate this system with another type of switch,
in order that the system can be simulated, see example:
<a href=\"modelica://MultiModeExamples.WithSpecialSwitch.CircuitParallelCapacitors\">CircuitParallelCapacitors</a>.
</p>
</html>"));
    end CircuitParallelCapacitors;

    model BreakingShaft "Varying index system (breaking shaft)"
      extends Modelica.Icons.Example;
      Integer state = if time <= 0.5 then 1 else 2;
      Modelica.Mechanics.Rotational.Sources.Torque torque
        annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      Modelica.Mechanics.Rotational.Components.Spring spring(c=5000)
        annotation (Placement(transformation(extent={{74,-10},{94,10}})));
      Modelica.Mechanics.Rotational.Components.Fixed fixed
        annotation (Placement(transformation(extent={{104,-10},{124,10}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-136,-10},{-116,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Interfaces.ConnectFlangesOf2States switch1(state=state)
        annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
      Interfaces.ConnectFlangesOf2States switch2(state=state)
        annotation (Placement(transformation(extent={{32,-6},{20,6}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia1(     w(fixed=true,
            start=1), J=9)
        annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
      Modelica.Mechanics.Rotational.Sources.ConstantTorque t1(tau_constant=0)
        annotation (Placement(transformation(extent={{-10,-18},{-22,-6}})));
      Modelica.Mechanics.Rotational.Sources.ConstantTorque t2(tau_constant=0)
        annotation (Placement(transformation(extent={{-2,-18},{10,-6}})));
    equation

      connect(spring.flange_b, fixed.flange) annotation (Line(
          points={{94,0},{114,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(step.y,torque. tau) annotation (Line(
          points={{-115,0},{-102,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(torque.flange, inertia1.flange_a) annotation (Line(
          points={{-80,0},{-72,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(inertia1.flange_b, switch1.flange_a) annotation (Line(
          points={{-52,0},{-42,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(inertia2.flange_b, spring.flange_a) annotation (Line(
          points={{60,0},{74,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(switch2.flange_a, inertia2.flange_a) annotation (Line(
          points={{32,0},{40,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(switch1.flange_b1, switch2.flange_b1) annotation (Line(
          points={{-30,4},{20,4}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(switch1.flange_b2, t1.flange) annotation (Line(
          points={{-30,-4},{-26,-4},{-26,-12},{-22,-12}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(t2.flange, switch2.flange_b2) annotation (Line(
          points={{10,-12},{14,-12},{14,-4},{20,-4}},
          color={0,0,0},
          smooth=Smooth.None));
      annotation ( experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                -100},{140,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
        Documentation(info="<html>
<p>
This system does not simulate, because it has a varying DAE index.
It is possible to reformulate this system with another type of switch,
in order that the system can be simulated, see example:
<a href=\"modelica://MultiModeExamples.WithSpecialSwitch.BreakingShaft\">WithSpecialSwitch.BreakingShaft</a>.
</p>
</html>"));
    end BreakingShaft;

    model BreakingPendulum1 "Not yet completely implemented"
      extends Modelica.Icons.Example;
      Integer state = if time <= 0.5 then 1 else 2;
      inner Modelica.Mechanics.MultiBody.World world annotation (Placement(
            transformation(extent={{-120,0},{-100,20}},
                                                      rotation=0)));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(useAxisFlange=true,phi(fixed=true),
          w(fixed=true))                                             annotation (Placement(transformation(extent={{-94,0},
                {-74,20}}, rotation=0)));
      Modelica.Mechanics.Rotational.Components.Damper damper(
                                                  d=0.1)
        annotation (Placement(transformation(extent={{-94,32},{-74,52}}, rotation=0)));
      Modelica.Mechanics.MultiBody.Parts.BodyBox boxBody1(             width=0.06, r={0.25,
            0,0})
        annotation (Placement(transformation(extent={{-34,10},{-14,30}},
                                                                       rotation=0)));
      Interfaces.ConnectFramesOf2States connectFramesOf2States
        annotation (Placement(transformation(extent={{-58,4},{-46,16}})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox boxBody2(             width=0.06, r={0.25,
            0,0})
        annotation (Placement(transformation(extent={{-34,-10},{-14,10}},
                                                                       rotation=0)));
    equation

      connect(damper.flange_b,revolute1. axis) annotation (Line(points={{-74,42},{-70,
              42},{-70,28},{-84,28},{-84,20}},     color={0,0,0}));
      connect(revolute1.support,damper. flange_a) annotation (Line(points={{-90,20},
              {-90,28},{-104,28},{-104,42},{-94,42}},
                                                    color={0,0,0}));
      connect(world.frame_b,revolute1. frame_a)
        annotation (Line(
          points={{-100,10},{-94,10}},
          color={95,95,95},
          thickness=0.5));
      connect(connectFramesOf2States.frame_b1, boxBody1.frame_a) annotation (Line(
          points={{-46,14},{-40,14},{-40,20},{-34,20}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.None));
      connect(revolute1.frame_b, connectFramesOf2States.frame_a) annotation (Line(
          points={{-74,10},{-58,10}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.None));
      connect(connectFramesOf2States.frame_b2, boxBody2.frame_a) annotation (Line(
          points={{-46,6},{-40,6},{-40,0},{-34,0}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.None));

      annotation (experiment(StopTime=1.0),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                -100},{140,100}})),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
        Documentation(info="<html>
<p>
Goal: Breaking pendulum.
</p>
</html>"));
    end BreakingPendulum1;
    annotation (Documentation(info="<html>
<p>
This package contains multi-mode examples that are using special switch components.
These systems do not simulate in Dymola 2015 (and most likely in no other
Modelica tool), because the DAE index is changing when switching.
</p>
</html>"));
  end WithSpecialSwitch_Fails;

   package WithStateMachine
    "Multi-mode examples with continous-time state machines (requiring an extension to Modelica 3.3)"
     model TwoBlocks1
      "Bug in translator (switching between two input/output blocks, variant 1)"
       extends Modelica.Icons.Example;
       Modelica.Blocks.Sources.Step step(startTime=0.1)
         annotation (Placement(transformation(extent={{-54,-9},{-38,7}})));

       Modelica.Blocks.Interfaces.RealOutput y
        "Connector of Real output signal"
         annotation (Placement(transformation(extent={{16,-8},{28,4}})));
       Modelica.Blocks.Continuous.FirstOrder firstOrder(       initType=Modelica.Blocks.Types.Init.SteadyState, T=0.2)
         annotation (Placement(transformation(extent={{-16,8},{4,28}})));
       Modelica.Blocks.Continuous.SecondOrder secondOrder(
         initType=Modelica.Blocks.Types.Init.SteadyState,
         w=25,
         D=0.2)
         annotation (Placement(transformation(extent={{-16,-30},{4,-10}})));
     equation

       initialState(firstOrder) annotation (Line(
           points={{-6,30},{-6,30},{-6,40}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       transition(
             firstOrder,
             secondOrder,
             time > 0.5) annotation (Line(
           points={{-6,6},{-6,-8}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,4},{-4,10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       connect(step.y, firstOrder.u) annotation (Line(
           points={{-37.2,-1},{-30,-1},{-30,18},{-18,18}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(step.y, secondOrder.u) annotation (Line(
           points={{-37.2,-1},{-30,-1},{-30,-20},{-18,-20}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(firstOrder.y, y) annotation (Line(
           points={{5,18},{12,18},{12,-2},{22,-2}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(secondOrder.y, y) annotation (Line(
           points={{5,-20},{12,-20},{12,-2},{22,-2}},
           color={0,0,127},
           smooth=Smooth.None));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), experiment(StopTime=1.1));
     end TwoBlocks1;

     model TwoBlocks2
      "Bug in translator (switching between two input/output blocks, variant 2)"
       extends Modelica.Icons.Example;
       Modelica.Blocks.Sources.Step step(startTime=0.1)
         annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
       State1 state1 annotation (Placement(transformation(extent={{-20,10},{0,30}})));
       State2 state2
         annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
    public
       block State1

         Modelica.Blocks.Continuous.FirstOrder firstOrder(       initType=Modelica.Blocks.Types.Init.SteadyState, T=0.2)
           annotation (Placement(transformation(extent={{-50,-50},{50,50}})));
         Modelica.Blocks.Interfaces.RealInput u
          "Connector of Real input signal"
           annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
         Modelica.Blocks.Interfaces.RealOutput y
          "Connector of Real output signal"
           annotation (Placement(transformation(extent={{100,-10},{120,10}})));
       equation
         connect(firstOrder.u, u) annotation (Line(
             points={{-60,0},{-120,0}},
             color={0,0,127},
             smooth=Smooth.None));
         connect(firstOrder.y, y) annotation (Line(
             points={{55,0},{110,0}},
             color={0,0,127},
             smooth=Smooth.None));
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                   100}}), graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateName",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end State1;

       block State2

         Modelica.Blocks.Continuous.SecondOrder secondOrder(
           initType=Modelica.Blocks.Types.Init.SteadyState,
           w=25,
           D=0.2)
           annotation (Placement(transformation(extent={{-50,-50},{50,50}})));
         Modelica.Blocks.Interfaces.RealInput u
          "Connector of Real input signal"
           annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
         Modelica.Blocks.Interfaces.RealOutput y
          "Connector of Real output signal"
           annotation (Placement(transformation(extent={{100,-10},{120,10}})));
       equation
         connect(secondOrder.u, u) annotation (Line(
             points={{-60,0},{-120,0}},
             color={0,0,127},
             smooth=Smooth.None));
         connect(secondOrder.y, y) annotation (Line(
             points={{55,0},{110,0}},
             color={0,0,127},
             smooth=Smooth.None));
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                   100}}), graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateName",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end State2;
       Modelica.Blocks.Interfaces.RealOutput y
        "Connector of Real output signal"
         annotation (Placement(transformation(extent={{24,-10},{44,10}})));
     equation
       initialState(state1) annotation (Line(
           points={{-10,32},{-10,46}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(step.y, state1.u) annotation (Line(
           points={{-39,0},{-32,0},{-32,20},{-22,20}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(step.y, state2.u) annotation (Line(
           points={{-39,0},{-32,0},{-32,-20},{-22,-20}},
           color={0,0,127},
           smooth=Smooth.None));

       transition(
         state1,
         state2,
         time > 0.5,
         immediate=false) annotation (Line(
           points={{-10,8},{-10,-8}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,-4},{-4,-10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       connect(state1.y, y) annotation (Line(
           points={{1,20},{14,20},{14,0},{34,0}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(state2.y, y) annotation (Line(
           points={{1,-20},{14,-20},{14,0},{34,0}},
           color={0,0,127},
           smooth=Smooth.None));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), experiment(StopTime=1.1));
     end TwoBlocks2;
      extends Modelica.Icons.Package;
     model HybridModelWithReset "Simple hybrid automata"
         extends Modelica.Icons.Example;
       inner Real xstart(start=1, fixed=true);
       inner Real x(start=xstart, fixed=true);
       inner Integer mode(start=1);
       Boolean e = time>2.5;
       Boolean a = edge(e);
       model Mode1
         outer output Real x;
         outer output Real xstart;
         outer output Integer mode;
       equation
         mode = 1;
         der(x) = 1;
         xstart=1;

         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateText",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end Mode1;
       Mode1 mode1 annotation (Placement(transformation(extent={{-74,28},{-30,
                60}})));
       model Mode2
         outer output Real x;
         outer output Real xstart;
         outer output Integer mode;
       equation
         mode = 2;
         der(x) = -x;
         xstart = 2*x;
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateText",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end Mode2;
       Mode2 mode2
         annotation (Placement(transformation(extent={{-32,-30},{14,2}})));
       model Mode3
         outer output Real x;
         outer output Real xstart;
         outer output Integer mode;
       equation
         der(x) = 1 + sin(time+0.5);
         xstart = 1.5*x;
         mode = 3;
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateText",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end Mode3;
       Mode3 mode3 annotation (Placement(transformation(extent={{2,30},{52,62}})));
     initial equation
        pre(e) = false;

     equation
       transition(
         mode1,
         mode2,x > 2,
         priority=2,immediate=true,reset=true,synchronize=false)
                     annotation (Line(
           points={{-52,26},{-27.4,4}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-12,4},{-12,10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       transition(
         mode2,
         mode3,
         x <= 2) annotation (Line(
           points={{7.1,4},{24.5,28}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,-12},{-4,-6}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       initialState(mode1) annotation (Line(
           points={{-54.2,62},{-63,74}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       transition(
         mode1,
         mode3,a,
         priority=1,
         immediate=true,
         reset=true,
         synchronize=false) annotation (Line(
           points={{-28,36},{0,36}},
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
         mode3,
         mode1,
         x >= 3) annotation (Line(
           points={{0,52},{-28,52}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{4,4},{4,10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics={Text(
               extent={{-98,98},{-38,80}},
               lineColor={0,0,0},
               horizontalAlignment=TextAlignment.Left,
               textString="%declarations",
               fontSize=10)}));
     end HybridModelWithReset;

     model CircuitCapacitorResistor "Switching from a capacitor to a resistor"
       extends Modelica.Icons.Example;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-46,-74},{-26,-54}})));
       Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
         annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
       Modelica.Electrical.Analog.Basic.Capacitor C(C=0.001, v(fixed=true))
                                                         annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={0,-30})));
       Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-36,-28})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=100)
                                                         annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=-90,
             origin={56,-30})));
     equation
      connect(voltage.p, R1.p) annotation (Line(
          points={{-36,-18},{-36,0},{-12,0}},
          color={0,0,255},
          smooth=Smooth.None));

       initialState(C) annotation (Line(
           points={{-11,-30},{-20,-30}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       transition(
         C,
         R2,time > 0.5,
         priority=1,immediate=true,reset=true,synchronize=false)
                     annotation (Line(
           points={{11,-30},{45,-30}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,4},{-4,10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
      connect(voltage.n, ground.p) annotation (Line(
          points={{-36,-38},{-36,-54}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(C.p, R1.n) annotation (Line(
          points={{1.88738e-015,-20},{0,-20},{0,-14},{26,-14},{26,0},{8,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(R1.n, R2.p) annotation (Line(
          points={{8,0},{26,0},{26,-14},{56,-14},{56,-20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(C.n, R2.n) annotation (Line(
          points={{-1.83187e-015,-40},{-1.83187e-015,-48},{56,-48},{56,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ground.p, R2.n) annotation (Line(
          points={{-36,-54},{27,-54},{27,-48},{56,-48},{56,-40}},
          color={0,0,255},
          smooth=Smooth.None));
       annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}},
            grid={1,1})), Icon(
            coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={1,1})));
     end CircuitCapacitorResistor;

     model IdealRectifier "Ideal rectifier modelled with state machine"
      extends Modelica.Icons.Example;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-80,-78},{-60,-58}})));
       MultiModeModels.Closed closed
         annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
       MultiModeModels.Open open
         annotation (Placement(transformation(extent={{-10,50},{10,70}})));
       Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(     freqHz=10, V=1)
                                                                       annotation (
           Placement(transformation(
             extent={{-10,-10},{10,10}},
             rotation=270,
             origin={-70,-10})));
       Modelica.Electrical.Analog.Basic.Resistor load(R=1)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=270,
             origin={60,-30})));
       Modelica.Electrical.Analog.Basic.Capacitor C(C=1)         annotation (
           Placement(transformation(
             extent={{-10,-10},{10,10}},
             rotation=270,
             origin={40,-30})));
       Modelica.Electrical.Analog.Basic.Resistor Ri(R=1)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=180,
             origin={-50,30})));
     equation
       connect(sineVoltage.n, ground.p) annotation (Line(
           points={{-70,-20},{-70,-58}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(sineVoltage.n, C.n) annotation (Line(
           points={{-70,-20},{-70,-50},{40,-50},{40,-40}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(C.n, load.n) annotation (Line(
           points={{40,-40},{40,-50},{60,-50},{60,-40}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(open.p, closed.p)       annotation (Line(
           points={{-10,60},{-34,60},{-34,0},{-10,0}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(open.n, closed.n)       annotation (Line(
           points={{10,60},{30,60},{30,0},{10,0}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(open.n, C.p)  annotation (Line(
           points={{10,60},{30,60},{30,30},{40,30},{40,-20}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(C.p, load.p) annotation (Line(
           points={{40,-20},{40,-10},{60,-10},{60,-20}},
           color={0,0,255},
           smooth=Smooth.None));
       transition(
           open,
           closed,open.v > 0,
           immediate=false,
           reset=true,
           synchronize=false,
           priority=1)
                     annotation (Line(
           points={{8,48},{8,12}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{20,-6},{20,-12}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(open)  annotation (Line(
           points={{0,72},{0,84}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(Ri.p, open.p) annotation (Line(
           points={{-40,30},{-34,30},{-34,60},{-10,60}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Ri.n, sineVoltage.p) annotation (Line(
           points={{-60,30},{-70,30},{-70,0}},
           color={0,0,255},
           smooth=Smooth.None));
       transition(
           closed,
           open,closed.p.i < 0,immediate=false,
                                              reset=true,synchronize=false,
         priority=1)     annotation (Line(
           points={{-8,12},{-8,48}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-24,6},{-24,12}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       annotation (experiment(StopTime=4), Diagram(coordinateSystem(
               preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                                                          graphics));
     end IdealRectifier;

     model RectifierFault1
      "Switching of two state machines (one with a broken rectifier), variant 1"
      extends Modelica.Icons.Example;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
       Modelica.Electrical.Analog.Basic.Resistor brokenDiode(R=1000)
         annotation (Placement(transformation(extent={{0,2},{20,22}})));
       Modelica.Electrical.Analog.Ideal.IdealDiode     diode
         annotation (Placement(transformation(extent={{0,40},{20,60}})));
       Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(     freqHz=10, V=0.4)
                                                                       annotation (
           Placement(transformation(
             extent={{-10,-10},{10,10}},
             rotation=270,
             origin={-50,-40})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=1)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=270,
             origin={88,-40})));
       MultiModeModels.CapacitorAndResistor CR(R(R=0.001)) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=270,
             origin={40,-40})));
       Modelica.Electrical.Analog.Basic.Resistor Ri(R=1)
         annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
       Modelica.Electrical.Analog.Basic.Resistor load(R=1)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=270,
             origin={120,-40})));
     equation
       connect(sineVoltage.n, ground.p) annotation (Line(
           points={{-50,-50},{-50,-80}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(sineVoltage.n, CR.n)
                                   annotation (Line(
           points={{-50,-50},{-50,-70},{40,-70},{40,-50}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(CR.n, R2.n)  annotation (Line(
           points={{40,-50},{40,-70},{88,-70},{88,-50}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(diode.p, brokenDiode.p) annotation (Line(
           points={{0,50},{-10,50},{-10,12},{0,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(diode.n, brokenDiode.n) annotation (Line(
           points={{20,50},{30,50},{30,12},{20,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(diode.n, CR.p)
                             annotation (Line(
           points={{20,50},{30,50},{30,30},{40,30},{40,-30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(CR.p, R2.p)  annotation (Line(
           points={{40,-30},{40,-10},{88,-10},{88,-30}},
           color={0,0,255},
           smooth=Smooth.None));
       transition(
         diode,
         brokenDiode,time > 0.9,immediate=true,reset=true,synchronize=false,priority=1)
                     annotation (Line(
           points={{10,38},{10,24}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{14,14},{14,8}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(diode) annotation (Line(
           points={{10,62},{10,74}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(sineVoltage.p, Ri.p) annotation (Line(
           points={{-50,-30},{-50,30},{-40,30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Ri.n, diode.p) annotation (Line(
           points={{-20,30},{-10,30},{-10,50},{0,50}},
           color={0,0,255},
           smooth=Smooth.None));
       transition(
           CR,
           R2,time > 0.3 and time < 0.5,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1)               annotation (Line(
           points={{52,-32},{76,-32}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{10,10},{10,4}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       transition(
           R2,
           CR,
           time > 0.7,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1)
                     annotation (Line(
           points={{76,-48},{52,-48}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{6,-8},{6,-2}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       initialState(CR)
                       annotation (Line(
           points={{48,-28},{48,-16}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(R2.p, load.p) annotation (Line(
           points={{88,-30},{88,-10},{120,-10},{120,-30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.n, load.n) annotation (Line(
           points={{88,-50},{88,-70},{120,-70},{120,-50}},
           color={0,0,255},
           smooth=Smooth.None));
       annotation ( experiment(StopTime=1.0), Diagram(coordinateSystem(
               preserveAspectRatio=false, extent={{-100,-100},{140,100}}),
                                                          graphics), Icon(
             coordinateSystem(extent={{-100,-100},{100,100}})));
     end RectifierFault1;

     model RectifierFault2
      "Switching of two state machines (one with a broken rectifier), variant 2"
      extends Modelica.Icons.Example;
       Modelica.Electrical.Analog.Basic.Ground ground
         annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
       Modelica.Electrical.Analog.Basic.Resistor brokenDiode(R=0.0001)
         annotation (Placement(transformation(extent={{0,2},{20,22}})));
       Modelica.Electrical.Analog.Ideal.IdealDiode     diode
         annotation (Placement(transformation(extent={{0,40},{20,60}})));
       Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(     freqHz=10, V=0.4)
                                                                       annotation (
           Placement(transformation(
             extent={{-10,-10},{10,10}},
             rotation=270,
             origin={-50,-40})));
       Modelica.Electrical.Analog.Basic.Resistor R2(R=1)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=270,
             origin={88,-40})));
       MultiModeModels.CapacitorAndResistor CR(R(R=0.001)) annotation (Placement(
             transformation(
             extent={{-10,-10},{10,10}},
             rotation=270,
             origin={40,-40})));
       Modelica.Electrical.Analog.Basic.Resistor Ri(R=1)
         annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
       Modelica.Electrical.Analog.Basic.Resistor load(R=1)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=270,
             origin={120,-40})));
     equation
       connect(sineVoltage.n, ground.p) annotation (Line(
           points={{-50,-50},{-50,-80}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(sineVoltage.n, CR.n)
                                   annotation (Line(
           points={{-50,-50},{-50,-70},{40,-70},{40,-50}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(CR.n, R2.n)  annotation (Line(
           points={{40,-50},{40,-70},{88,-70},{88,-50}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(diode.p, brokenDiode.p) annotation (Line(
           points={{0,50},{-10,50},{-10,12},{0,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(diode.n, brokenDiode.n) annotation (Line(
           points={{20,50},{30,50},{30,12},{20,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(diode.n, CR.p)
                             annotation (Line(
           points={{20,50},{30,50},{30,30},{40,30},{40,-30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(CR.p, R2.p)  annotation (Line(
           points={{40,-30},{40,-10},{88,-10},{88,-30}},
           color={0,0,255},
           smooth=Smooth.None));
       transition(
         diode,
         brokenDiode,time > 0.9,immediate=true,reset=true,synchronize=false,priority=1)
                     annotation (Line(
           points={{10,38},{10,24}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{14,14},{14,8}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(diode) annotation (Line(
           points={{10,62},{10,74}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(sineVoltage.p, Ri.p) annotation (Line(
           points={{-50,-30},{-50,30},{-40,30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Ri.n, diode.p) annotation (Line(
           points={{-20,30},{-10,30},{-10,50},{0,50}},
           color={0,0,255},
           smooth=Smooth.None));
       transition(
           CR,
           R2,time > 0.3 and time < 0.5,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1)               annotation (Line(
           points={{52,-32},{76,-32}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{10,10},{10,4}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       transition(
           R2,
           CR,
           time > 0.7,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1)
                     annotation (Line(
           points={{76,-48},{52,-48}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{6,-8},{6,-2}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       initialState(CR)
                       annotation (Line(
           points={{48,-28},{48,-16}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(R2.p, load.p) annotation (Line(
           points={{88,-30},{88,-10},{120,-10},{120,-30}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(R2.n, load.n) annotation (Line(
           points={{88,-50},{88,-70},{120,-70},{120,-50}},
           color={0,0,255},
           smooth=Smooth.None));
       annotation ( experiment(StopTime=1.0), Diagram(coordinateSystem(
               preserveAspectRatio=false, extent={{-100,-100},{140,100}}),
                                                          graphics), Icon(
             coordinateSystem(extent={{-100,-100},{100,100}})));
     end RectifierFault2;

     model ChangedInertia1 "Drive train with changing inertia, variant 1"
      extends Modelica.Icons.Example;
       Modelica.Mechanics.Rotational.Sources.Torque
                                 torque(useSupport=false)
                                annotation (Placement(transformation(extent={{-56,-8},
                 {-40,8}},         rotation=0)));
       Modelica.Mechanics.Rotational.Components.Inertia
                                     inertia1(J=1,
         phi(fixed=false, start=0),
         w(fixed=false, start=0))
         annotation (Placement(transformation(extent={{-26,-8},{-10,8}},
               rotation=0)));
       Modelica.Mechanics.Rotational.Components.Inertia smallInertia(J=1)
         annotation (Placement(transformation(extent={{4,-8},{20,8}},  rotation=
                 0)));
       Modelica.Mechanics.Rotational.Components.Inertia
                                     inertia2(        J=2,
         phi(fixed=false, start=0),
         w(fixed=false))
                     annotation (Placement(transformation(extent={{34,-8},{50,8}},
               rotation=0)));
       Modelica.Blocks.Sources.Sine sine(amplitude=1, freqHz=1)
         annotation (Placement(transformation(extent={{-86,-8},{-70,8}},
               rotation=0)));
       Modelica.Mechanics.Rotational.Components.Inertia bigInertia(J=10)
         annotation (Placement(transformation(extent={{4,-38},{20,-22}})));
       Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=1, d=1)
         annotation (Placement(transformation(extent={{58,-10},{78,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{82,-10},{102,10}})));
     equation
       connect(inertia1.flange_b, smallInertia.flange_a)
         annotation (Line(points={{-10,0},{4,0}},   color={0,0,0}));
       connect(smallInertia.flange_b, inertia2.flange_a)
         annotation (Line(points={{20,0},{34,0}},color={0,0,0}));
       connect(sine.y,torque. tau) annotation (Line(points={{-69.2,0},{-57.6,0}},
             color={0,0,127}));
       connect(torque.flange,inertia1. flange_a) annotation (Line(
           points={{-40,0},{-26,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(smallInertia.flange_a, bigInertia.flange_a)
                                                          annotation (Line(
           points={{4,0},{-4,0},{-4,-30},{4,-30}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(smallInertia.flange_b, bigInertia.flange_b)
                                                          annotation (Line(
           points={{20,0},{26,0},{26,-30},{20,-30}},
           color={0,0,0},
           smooth=Smooth.None));
       transition(
           smallInertia,
           bigInertia,
           time > 0.5 and time < 0.6,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1) annotation (Line(
           points={{10,-10},{10,-20}},
           color={175,175,175},
           pattern=LinePattern.None,
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,4},{-4,10}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(smallInertia)
                                annotation (Line(
           points={{12,10},{12,20}},
           color={175,175,175},
           pattern=LinePattern.None,
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       transition(
           bigInertia,
           smallInertia,
           time > 0.8,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1) annotation (Line(
           points={{16,-20},{16,-10}},
           color={175,175,175},
           pattern=LinePattern.None,
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{4,-4},{4,-10}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       connect(springDamper.flange_b, fixed.flange) annotation (Line(
           points={{78,0},{92,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia2.flange_b, springDamper.flange_a) annotation (Line(
           points={{50,0},{58,0}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}),      graphics));
     end ChangedInertia1;

     model ChangedInertia2 "Drive train with changing inertia, variant 2"
         extends Modelica.Icons.Example;
       Modelica.Mechanics.Rotational.Sources.Torque
                                 torque(useSupport=false)
                                annotation (Placement(transformation(extent={{-56,-8},
                 {-40,8}},         rotation=0)));
       Modelica.Mechanics.Rotational.Components.Inertia
                                     inertia1(J=1,
         phi(fixed=false, start=0),
         w(fixed=false, start=0))
         annotation (Placement(transformation(extent={{-26,-8},{-10,8}},
               rotation=0)));
       Modelica.Mechanics.Rotational.Components.Inertia smallInertia(J=1)
         annotation (Placement(transformation(extent={{4,-8},{20,8}},  rotation=
                 0)));
       Modelica.Mechanics.Rotational.Components.Inertia
                                     inertia2(        J=2,
         phi(fixed=false, start=0),
         w(fixed=false))
                     annotation (Placement(transformation(extent={{34,-8},{50,8}},
               rotation=0)));
       Modelica.Blocks.Sources.Sine sine(amplitude=1, freqHz=1)
         annotation (Placement(transformation(extent={{-86,-8},{-70,8}},
               rotation=0)));
       Modelica.Mechanics.Rotational.Components.Inertia bigInertia(J=10)
         annotation (Placement(transformation(extent={{4,-38},{20,-22}})));
       Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=1, d=1)
         annotation (Placement(transformation(extent={{58,-10},{78,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{82,-10},{102,10}})));
     equation
       connect(inertia1.flange_b, smallInertia.flange_a)
         annotation (Line(points={{-10,0},{4,0}},   color={0,0,0}));
       connect(sine.y,torque. tau) annotation (Line(points={{-69.2,0},{-57.6,0}},
             color={0,0,127}));
       connect(torque.flange,inertia1. flange_a) annotation (Line(
           points={{-40,0},{-26,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(smallInertia.flange_a, bigInertia.flange_a)
                                                          annotation (Line(
           points={{4,0},{-4,0},{-4,-30},{4,-30}},
           color={0,0,0},
           smooth=Smooth.None));
       transition(
           smallInertia,
           bigInertia,
           time > 0.5 and time < 0.6,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1) annotation (Line(
           points={{10,-10},{10,-20}},
           color={175,175,175},
           pattern=LinePattern.None,
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,4},{-4,10}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(smallInertia)
                                annotation (Line(
           points={{12,10},{12,20}},
           color={175,175,175},
           pattern=LinePattern.None,
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       transition(
           bigInertia,
           smallInertia,
           time > 0.8,
           immediate=true,
           reset=true,
           synchronize=false,
           priority=1) annotation (Line(
           points={{16,-20},{16,-10}},
           color={175,175,175},
           pattern=LinePattern.None,
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{4,-4},{4,-10}},
           lineColor={95,95,95},
           fontSize=8,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Left));
       connect(springDamper.flange_b, fixed.flange) annotation (Line(
           points={{78,0},{92,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia2.flange_b, springDamper.flange_a) annotation (Line(
           points={{50,0},{58,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia1.flange_b, inertia2.flange_a) annotation (Line(
           points={{-10,0},{-4,0},{-4,28},{28,28},{28,0},{34,0}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}),      graphics));
     end ChangedInertia2;

     model BreakingInertia1
      "Drive train with breaking inertia, variant 1 (switching from one to two unconnected inertias)"
       extends Modelica.Icons.Example;
       inner Real phi(start=0, fixed=true);
       inner Real w;

       model State1
         outer Real phi;
         outer Real w;
         Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=10, w(start=1, fixed=true))
           annotation (Placement(transformation(extent={{-70,-70},{70,70}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
          "Right flange of shaft"
           annotation (Placement(transformation(extent={{90,-10},{110,10}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
          "Left flange of shaft"
           annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
       equation
         phi = inertia1.phi;
         w = inertia1.w;
         connect(inertia1.flange_a, flange_a1) annotation (Line(
             points={{-70,0},{-100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         connect(inertia1.flange_b, flange_b1) annotation (Line(
             points={{70,0},{100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                   100}}),      graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateName",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end State1;
       State1 state1
         annotation (Placement(transformation(extent={{17,14},{43,40}})));
       model State2
         outer Real phi;
         outer Real w;
         Modelica.Mechanics.Rotational.Components.Inertia inertia2a(
           J=9,
           phi(start=phi, fixed=true),
           w(start=w, fixed=true))
           annotation (Placement(transformation(extent={{-88,-40},{-8,40}})));
         Modelica.Mechanics.Rotational.Components.Inertia inertia2b(
           phi(start=phi, fixed=true),
           w(start=w, fixed=true),
           J=1) annotation (Placement(transformation(extent={{10,-40},{90,40}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
          "Right flange of shaft"
           annotation (Placement(transformation(extent={{90,-10},{110,10}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
          "Left flange of shaft"
           annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
       equation
         connect(inertia2b.flange_b, flange_b1) annotation (Line(
             points={{90,0},{100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         connect(inertia2a.flange_a, flange_a1) annotation (Line(
             points={{-88,0},{-100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                   {100,100}}), graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateName",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end State2;
       State2 state2
         annotation (Placement(transformation(extent={{6,-52},{54,-6}})));
       Modelica.Mechanics.Rotational.Sources.Torque torque
         annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
       Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=5000, d=20,
         stateSelect=StateSelect.never)
         annotation (Placement(transformation(extent={{68,-10},{88,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{90,-10},{110,10}})));
       Modelica.Blocks.Sources.Step step
         annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
     equation
       transition(
           state1,
           state2,time > 0.5,immediate=false,
                                            reset=true,synchronize=false,priority=1)
                       annotation (Line(
           points={{30,12},{30,-4}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,-4},{-4,-10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(state1) annotation (Line(
           points={{30,42},{30,50}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(state1.flange_b1, state2.flange_b1) annotation (Line(
           points={{43,27},{62,27},{62,-29},{54,-29}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(springDamper.flange_a, state2.flange_b1) annotation (Line(
           points={{68,0},{62,0},{62,-29},{54,-29}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(springDamper.flange_b, fixed.flange) annotation (Line(
           points={{88,0},{100,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(step.y, torque.tau) annotation (Line(
           points={{-45,0},{-34,0}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(torque.flange, state1.flange_a1) annotation (Line(
           points={{-12,0},{-4,0},{-4,27},{17,27}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(torque.flange, state2.flange_a1) annotation (Line(
           points={{-12,0},{-4,0},{-4,-29},{6,-29}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0, Tolerance=1e-006), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{160,100}}),      graphics), Icon(coordinateSystem(extent={
                 {-100,-100},{100,100}})));
     end BreakingInertia1;

     model BreakingInertia2
      "Drive train with breaking inertia, variant 2 (switching from one to two unconnected inertias)"

       extends Modelica.Icons.Example;
       inner Real phi(start=0, fixed=true) = inertia3.phi;
       inner Real w(start=0,fixed=true) = inertia3.w;

         model State1
         outer Real phi;
         outer Real w;
         Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=10)
           annotation (Placement(transformation(extent={{-70,-70},{70,70}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
          "Right flange of shaft"
           annotation (Placement(transformation(extent={{90,-10},{110,10}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
          "Left flange of shaft"
           annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
         equation
         connect(inertia1.flange_a, flange_a1) annotation (Line(
             points={{-70,0},{-100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         connect(inertia1.flange_b, flange_b1) annotation (Line(
             points={{70,0},{100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                   100}}),      graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateName",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
         end State1;
       State1 state1
         annotation (Placement(transformation(extent={{17,14},{43,40}})));
       model State2
         outer Real phi;
         outer Real w;
         Modelica.Mechanics.Rotational.Components.Inertia inertia2a(
           J=9)
           annotation (Placement(transformation(extent={{-88,-40},{-8,40}})));
         Modelica.Mechanics.Rotational.Components.Inertia inertia2b(
           phi(start=phi, fixed=true),
           w(start=w, fixed=true),
           J=1,
           stateSelect=StateSelect.always)
                annotation (Placement(transformation(extent={{10,-40},{90,40}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
          "Right flange of shaft"
           annotation (Placement(transformation(extent={{90,-10},{110,10}})));
         Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
          "Left flange of shaft"
           annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
       equation
         connect(inertia2b.flange_b, flange_b1) annotation (Line(
             points={{90,0},{100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         connect(inertia2a.flange_a, flange_a1) annotation (Line(
             points={{-88,0},{-100,0}},
             color={0,0,0},
             smooth=Smooth.None));
         annotation (
           Icon(graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%name")}),
           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                   {100,100}}), graphics={Text(
                 extent={{-100,100},{100,-100}},
                 lineColor={0,0,0},
                 textString="%stateName",
                 fontSize=10)}),
           __Dymola_state=true,
           showDiagram=true,
           singleInstance=true);
       end State2;
       State2 state2
         annotation (Placement(transformation(extent={{6,-52},{54,-6}})));
       Modelica.Mechanics.Rotational.Sources.Torque torque
         annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
       Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=5000, d=20)
         annotation (Placement(transformation(extent={{68,-10},{88,10}})));
       Modelica.Mechanics.Rotational.Components.Fixed fixed
         annotation (Placement(transformation(extent={{90,-10},{110,10}})));
       Modelica.Blocks.Sources.Step step
         annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
       Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=2, stateSelect=
             StateSelect.always)
         annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));

     equation
       transition(
           state1,
           state2,time > 0.5,immediate=false,
                                            reset=true,synchronize=false,priority=1)
                       annotation (Line(
           points={{30,12},{30,-4}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier), Text(
           string="%condition",
           extent={{-4,-4},{-4,-10}},
           lineColor={95,95,95},
           fontSize=10,
           textStyle={TextStyle.Bold},
           horizontalAlignment=TextAlignment.Right));
       initialState(state1) annotation (Line(
           points={{30,42},{30,50}},
           color={175,175,175},
           thickness=0.25,
           smooth=Smooth.Bezier,
           arrow={Arrow.Filled,Arrow.None}));
       connect(state1.flange_b1, state2.flange_b1) annotation (Line(
           points={{43,27},{62,27},{62,-29},{54,-29}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(springDamper.flange_a, state2.flange_b1) annotation (Line(
           points={{68,0},{62,0},{62,-29},{54,-29}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(springDamper.flange_b, fixed.flange) annotation (Line(
           points={{88,0},{100,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(step.y, torque.tau) annotation (Line(
           points={{-65,0},{-58,0}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(torque.flange, inertia3.flange_a) annotation (Line(
           points={{-36,0},{-30,0}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia3.flange_b, state1.flange_a1) annotation (Line(
           points={{-10,0},{-2,0},{-2,27},{17,27}},
           color={0,0,0},
           smooth=Smooth.None));
       connect(inertia3.flange_b, state2.flange_a1) annotation (Line(
           points={{-10,0},{-2,0},{-2,-29},{6,-29}},
           color={0,0,0},
           smooth=Smooth.None));
       annotation (experiment(StopTime=1.0, Tolerance=1e-006), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{160,100}}),      graphics), Icon(coordinateSystem(extent={
                 {-100,-100},{100,100}})));
     end BreakingInertia2;

    annotation (Documentation(info="<html>
<p>
Multi-model models based a Modelica extension that generalizes
synchronous state machines to continuous-time state machines.
All models in this sub-library simulate in a Dymola prototype.
</p>
</html>"));
   end WithStateMachine;

  package WithStateMachine_Fails
    "Multi-mode examples with continous-time state machines (requiring an extension to Modelica 3.3) but translation or simulation fails in Dymola"

     extends Modelica.Icons.Package;
    model CircuitParallelCapacitors
      "Varying index system (switching from a capacitor to a resistor and these components are in parallel to a capacitor)"
      extends Modelica.Icons.Example;
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-46,-74},{-26,-54}})));
      Modelica.Electrical.Analog.Basic.Resistor R1(R=100)
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Electrical.Analog.Basic.Capacitor C1(C=0.001) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-30})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12)
       annotation (Placement(transformation(
           extent={{-10,-10},{10,10}},
           rotation=-90,
           origin={-36,-28})));
      Modelica.Electrical.Analog.Basic.Resistor R2(R=100)
                                                        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={57,-30})));
      Modelica.Electrical.Analog.Basic.Capacitor C2(C=0.002) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={82,-31})));
    equation
     connect(voltage.p, R1.p) annotation (Line(
         points={{-36,-18},{-36,0},{-12,0}},
         color={0,0,255},
         smooth=Smooth.None));

      initialState(C1) annotation (Line(
          points={{-11,-30},{-20,-30}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier,
          arrow={Arrow.Filled,Arrow.None}));
      transition(
            C1,
            R2,
            time > 0.5,
            priority=1,
            immediate=true,
            reset=true,
            synchronize=false) annotation (Line(
          points={{11,-30},{46,-30}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier), Text(
          string="%condition",
          extent={{-4,4},{-4,10}},
          lineColor={95,95,95},
          fontSize=10,
          textStyle={TextStyle.Bold},
          horizontalAlignment=TextAlignment.Right));
     connect(voltage.n, ground.p) annotation (Line(
         points={{-36,-38},{-36,-54}},
         color={0,0,255},
         smooth=Smooth.None));
      connect(C1.p, R1.n) annotation (Line(
          points={{1.88738e-015,-20},{0,-20},{0,-14},{26,-14},{26,0},{8,0}},
          color={0,0,255},
          smooth=Smooth.None));
     connect(R1.n, R2.p) annotation (Line(
         points={{8,0},{26,0},{26,-14},{57,-14},{57,-20}},
         color={0,0,255},
         smooth=Smooth.None));
      connect(C1.n, R2.n) annotation (Line(
          points={{-1.83187e-015,-40},{-1.83187e-015,-48},{57,-48},{57,-40}},
          color={0,0,255},
          smooth=Smooth.None));
     connect(ground.p, R2.n) annotation (Line(
         points={{-36,-54},{27,-54},{27,-48},{57,-48},{57,-40}},
         color={0,0,255},
         smooth=Smooth.None));
      connect(R1.n, C2.p) annotation (Line(
          points={{8,0},{26,0},{26,-14},{82,-14},{82,-21}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(C2.n, R2.n) annotation (Line(
          points={{82,-41},{82,-48},{57,-48},{57,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (experiment(StopTime=1.0), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}},
           grid={1,1})), Icon(
           coordinateSystem(
           preserveAspectRatio=false,
           extent={{-100,-100},{100,100}},
           grid={1,1})));
    end CircuitParallelCapacitors;

    model BreakingShaft "Varying index system (breaking shaft)"
      extends Modelica.Icons.Example;

      model State1

        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
          "Right flange of shaft"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
          "Left flange of shaft"
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      equation
        connect(flange_a1, flange_b1) annotation (Line(
            points={{-100,0},{100,0}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (
          Icon(graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                               graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%stateName",
                fontSize=10)}),
          __Dymola_state=true,
          showDiagram=true,
          singleInstance=true);
      end State1;
      State1 state1
        annotation (Placement(transformation(extent={{21,12},{39,30}})));
      model State2

        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
          "Right flange of shaft"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
          "Left flange of shaft"
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque zeroTorque1(
            tau_constant=0)
          annotation (Placement(transformation(extent={{-20,-30},{-80,30}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque zeroTorque2(
            tau_constant=0)
          annotation (Placement(transformation(extent={{20,-30},{80,30}})));
      equation
        connect(zeroTorque1.flange, flange_a1) annotation (Line(
            points={{-80,0},{-100,0}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(zeroTorque2.flange, flange_b1) annotation (Line(
            points={{80,0},{100,0}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (
          Icon(graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),      graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%stateName",
                fontSize=10)}),
          __Dymola_state=true,
          showDiagram=true,
          singleInstance=true);
      end State2;
      State2 state2
        annotation (Placement(transformation(extent={{19,-29},{41,-10}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque
        annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=5000, d=20,
        stateSelect=StateSelect.never)
        annotation (Placement(transformation(extent={{96,-10},{116,10}})));
      Modelica.Mechanics.Rotational.Components.Fixed fixed
        annotation (Placement(transformation(extent={{116,-10},{136,10}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=9)
        annotation (Placement(transformation(extent={{-26,-10},{-6,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1)
        annotation (Placement(transformation(extent={{56,-10},{76,10}})));
    equation
      transition(
          state1,
          state2,time > 0.5,immediate=false,
                                           reset=true,synchronize=false,priority=1)
                      annotation (Line(
          points={{30,10},{30,-8}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier), Text(
          string="%condition",
          extent={{-4,-4},{-4,-10}},
          lineColor={95,95,95},
          fontSize=10,
          textStyle={TextStyle.Bold},
          horizontalAlignment=TextAlignment.Right));
      initialState(state1) annotation (Line(
          points={{30,32},{30,42}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier,
          arrow={Arrow.Filled,Arrow.None}));
      connect(state1.flange_b1, state2.flange_b1) annotation (Line(
          points={{39,21},{50,21},{50,-19.5},{41,-19.5}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(springDamper.flange_b, fixed.flange) annotation (Line(
          points={{116,0},{126,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(step.y, torque.tau) annotation (Line(
          points={{-65,0},{-54,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(inertia2.flange_a, state2.flange_b1) annotation (Line(
          points={{56,0},{50,0},{50,-19.5},{41,-19.5}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(state1.flange_a1, inertia1.flange_b) annotation (Line(
          points={{21,21},{0,21},{0,0},{-6,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(inertia1.flange_b, state2.flange_a1) annotation (Line(
          points={{-6,0},{0,0},{0,-19.5},{19,-19.5}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(inertia2.flange_b, springDamper.flange_a) annotation (Line(
          points={{76,0},{96,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(torque.flange, inertia1.flange_a) annotation (Line(
          points={{-32,0},{-26,0}},
          color={0,0,0},
          smooth=Smooth.None));
      annotation (experiment(StopTime=1.0, Tolerance=1e-006), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{160,100}}),      graphics), Icon(coordinateSystem(extent={
                {-100,-100},{100,100}})));
    end BreakingShaft;


    annotation (Documentation(info="<html>
<p>
The multi-model models in this sub-library do not yet simulate in
the Dymola prototype.
</p>
</html>"));
  end WithStateMachine_Fails;

   package Interfaces
    "Special switch components to emulate continuous-time state machines in Modelica 3.2"
      extends Modelica.Icons.InterfacesPackage;
     model ConnectPinsOf2States
      "Connect a pin to the pins on two states of the same state machine"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       parameter Real Gsmall=1e-10 "Virtual conductor for symbolic processing";
       Modelica.Electrical.Analog.Interfaces.PositivePin p
         annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
             iconTransformation(extent={{-70,-10},{-50,10}})));
       Modelica.Electrical.Analog.Interfaces.NegativePin n1 annotation (Placement(
             transformation(extent={{90,50},{110,70}}),  iconTransformation(extent={{50,30},
                 {70,50}})));
       Modelica.Electrical.Analog.Interfaces.NegativePin n2 annotation (Placement(
             transformation(extent={{90,-70},{110,-50}}), iconTransformation(extent={{50,-50},
                 {70,-30}})));
     equation
       // Equation for potential variables
       p.v = if state==1 then n1.v else n2.v;

       // Equation for flow variables
       0 = p.i + (if state==1 then n1.i else n2.i);

       // Dummy equation to handle not-connected state
       0 = if state==1 then Gsmall*n2.v - n2.i else Gsmall*n1.v - n1.i;

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={0,0,255},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-50,0},{28,24}},
               color={0,0,255},
               smooth=Smooth.None),
             Text(
               extent={{-15,47},{37,29}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               textString="n1",
               horizontalAlignment=TextAlignment.Right),
             Text(
               extent={{-13,-26},{39,-44}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               textString="n2",
               horizontalAlignment=TextAlignment.Right),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}),
         Documentation(info="<html>
<p>
The details of this switch are described in section 3.1 of
(Elmqvist, Mattsson, Otter 2014).
</p>

</html>"));
     end ConnectPinsOf2States;

     model ConnectPinsOf2States_specialCase
      "Connect a pin to pins on two states of the same state machine"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       Modelica.Electrical.Analog.Interfaces.PositivePin p
         annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
             iconTransformation(extent={{-70,-10},{-50,10}})));
       Modelica.Electrical.Analog.Interfaces.NegativePin n1 annotation (Placement(
             transformation(extent={{90,50},{110,70}}),  iconTransformation(extent={{50,30},
                 {70,50}})));
       Modelica.Electrical.Analog.Interfaces.NegativePin n2 annotation (Placement(
             transformation(extent={{90,-70},{110,-50}}), iconTransformation(extent={{50,-50},
                 {70,-30}})));
     equation
       // Equation for potential variables
       n1.v = noEvent(if state==1 then p.v else 0);
       n2.v = noEvent(if state==2 then p.v else 0);

       // Equation for flow variables
       0 = p.i + (if state==1 then n1.i else n2.i);

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={0,0,255},
               fillColor={255,255,0},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-50,0},{28,24}},
               color={0,0,255},
               smooth=Smooth.None),
             Text(
               extent={{-15,47},{37,29}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               textString="n1",
               horizontalAlignment=TextAlignment.Right),
             Text(
               extent={{-13,-26},{39,-44}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               textString="n2",
               horizontalAlignment=TextAlignment.Right),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}));
     end ConnectPinsOf2States_specialCase;

     model ConnectFlangesOf2States
      "Connect a flange to flanges on two states of the same state machine"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       parameter Real Gsmall=1e-10 "Virtual conductor for symbolic processing";
       Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
           Placement(transformation(extent={{-110,-10},{-90,10}}),
             iconTransformation(extent={{-70,-10},{-50,10}})));
       Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1 annotation (
           Placement(transformation(extent={{90,50},{110,70}}), iconTransformation(
               extent={{50,30},{70,50}})));
       Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b2 annotation (
           Placement(transformation(extent={{90,-70},{110,-50}}), iconTransformation(
               extent={{50,-50},{70,-30}})));
     equation
       // Equation for potential variables
       flange_a.phi = if state==1 then flange_b1.phi else flange_b2.phi;

       // Equation for flow variables
       0 = flange_a.tau + (if state==1 then flange_b1.tau else flange_b2.tau);

       // Dummy equation to handle not-connected state
       0 = if state==1 then Gsmall*flange_b2.phi - flange_b2.tau else Gsmall*flange_b1.phi - flange_b1.tau;

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={95,95,95},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-50,0},{28,24}},
               color={95,95,95},
               smooth=Smooth.None),
             Text(
               extent={{-38,50},{41,34}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="1"),
             Text(
               extent={{-27,-38},{44,-51}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="2"),
             Text(
              extent={{-150,95},{150,65}},
              lineColor={0,0,255},
              textString="%name")}));
     end ConnectFlangesOf2States;

     model ConnectFlangesOf2States_specialCase
      "Connect a flange to flanges on two states of the same state machine"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
           Placement(transformation(extent={{-110,-10},{-90,10}}),
             iconTransformation(extent={{-70,-10},{-50,10}})));
       Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1 annotation (
           Placement(transformation(extent={{90,50},{110,70}}), iconTransformation(
               extent={{50,30},{70,50}})));
       Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b2  annotation (
           Placement(transformation(extent={{90,-70},{110,-50}}), iconTransformation(
               extent={{50,-50},{70,-30}})));

       Modelica.SIunits.Angle phi;
       Modelica.SIunits.AngularVelocity w;
    protected
       constant Modelica.SIunits.Inertia J=1;
     equation
       // Equation for potential variables
       flange_a.phi = noEvent(if state==1 then flange_b1.phi else flange_b2.phi);

       // Equation for flow variables
       0 = flange_a.tau + (if state==1 then flange_b1.tau else flange_b2.tau);

       // Dummy equations to handle not-connected state
       phi = flange_b1.phi - flange_b2.phi;
       w = der(phi);
       der(w) = (if state==1 then flange_b2.tau else flange_b1.tau)/J;

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={95,95,95},
               fillColor={255,255,0},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-50,0},{28,24}},
               color={95,95,95},
               smooth=Smooth.None),
             Text(
               extent={{-38,50},{41,34}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="1"),
             Text(
               extent={{-27,-38},{44,-51}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="2"),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}));
     end ConnectFlangesOf2States_specialCase;

     model ConnectInputsOf2States
      "Connect an output to two inputs of the same state machine"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(
               extent={{-100,-20},{-60,20}}), iconTransformation(extent={{-100,-20},{
                 -60,20}})));
       Modelica.Blocks.Interfaces.RealOutput y1 annotation (Placement(transformation(
               extent={{60,30},{80,50}}), iconTransformation(extent={{60,30},{80,50}})));
       Modelica.Blocks.Interfaces.RealOutput y2 annotation (Placement(transformation(
               extent={{60,-50},{80,-30}}), iconTransformation(extent={{60,-50},{80,-30}})));
     equation
       y1 = u;
       y2 = u;
       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},
                 {60,60}}),      graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={0,0,127},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-60,0},{23,24}},
               color={0,0,127},
               smooth=Smooth.None),
             Text(
               extent={{-1,46},{51,28}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
               textString="y1"),
             Text(
               extent={{-1,-30},{51,-48}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
               textString="y2"),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}));
     end ConnectInputsOf2States;

     model ConnectOutputsOf2States
      "Connect two outputs of the same state machine to one input"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
      Modelica.Blocks.Interfaces.RealInput u1
        annotation (Placement(transformation(extent={{-100,20},{-60,60}})));
      Modelica.Blocks.Interfaces.RealInput u2
        annotation (Placement(transformation(extent={{-100,-60},{-60,-20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
     equation
       y = if state==1 then u1 else u2;

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,
                 -60},{60,60}}), graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={0,0,127},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-21,35},{60,0}},
               color={0,0,127},
               smooth=Smooth.None),
             Text(
               extent={{-53,49},{-1,31}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Left,
               textString="u1"),
             Text(
               extent={{-51,-31},{1,-49}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Left,
               textString="u2"),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}));
     end ConnectOutputsOf2States;

     model ConnectFramesOf2States
      "Connect a frame to frames on two states of the same state machine"
       import Modelica.Mechanics.MultiBody.Frames;
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
         annotation (Placement(transformation(extent={{-76,-16},{-44,16}})));
       Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b1
         annotation (Placement(transformation(extent={{44,24},{76,56}})));
       Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b2
         annotation (Placement(transformation(extent={{44,-56},{76,-24}})));
     equation
       Connections.branch(frame_a.R, frame_b1.R);
       Connections.branch(frame_a.R, frame_b2.R);

       // Equations for position
       frame_b1.r_0 = frame_a.r_0;
       frame_b2.r_0 = frame_a.r_0;

       // Equations for orientation
       frame_b1.R = frame_a.R;
       frame_b2.R = frame_a.R;

       // Equations for forces
       zeros(3) = frame_a.f + (if state==1 then frame_b1.f else frame_b2.f);
       zeros(3) = frame_a.t + (if state==1 then frame_b1.t else frame_b2.t);

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},
                 {60,60}}),      graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={95,95,95},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-50,0},{28,24}},
               color={95,95,95},
               smooth=Smooth.None),
             Text(
               extent={{-38,50},{41,34}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="1"),
             Text(
               extent={{-27,-38},{44,-51}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="2"),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}));
     end ConnectFramesOf2States;

     model ConnectFlangesOf2States_specialCaseb
      "Connect a flange to flanges on two states of the same state machine"
       input Integer state=1 "State (1 or 2) as function of time" annotation(Dialog);
       parameter Real Gsmall=1e-10 "Virtual conductor for symbolic processing";
       Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
           Placement(transformation(extent={{-110,-10},{-90,10}}),
             iconTransformation(extent={{-70,-10},{-50,10}})));
       Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1 annotation (
           Placement(transformation(extent={{90,50},{110,70}}), iconTransformation(
               extent={{50,30},{70,50}})));
       Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b2 annotation (
           Placement(transformation(extent={{90,-70},{110,-50}}), iconTransformation(
               extent={{50,-50},{70,-30}})));
       Real phi_rel,w;
       parameter Real J=0 annotation(Evaluate=false);
     equation
       // Equation for potential variables
       flange_a.phi = noEvent(if state==1 then flange_b1.phi else flange_b2.phi);

       // Equation for flow variables
       0 = flange_a.tau + (if state==1 then flange_b1.tau else flange_b2.tau);

       // Dummy equations to handle not-connected state
       phi_rel = flange_b1.phi - flange_b2.phi;
       w = der(phi_rel);
       J*der(w) = if state==1 then flange_b2.tau else flange_b1.tau;

       when change(state) then
         reinit(phi_rel,0);
         reinit(w,0);
       end when;

       annotation (defaultComponentName="switch1",Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                 -100},{100,100}}), graphics), Icon(coordinateSystem(
               preserveAspectRatio=false, extent={{-60,-60},{60,60}},
             grid={1,1}),                                                  graphics={
             Rectangle(extent={{-60,60},{60,-60}},     lineColor={95,95,95},
               fillColor={255,255,0},
               fillPattern=FillPattern.Solid),
             Line(
               points={{-50,0},{28,24}},
               color={95,95,95},
               smooth=Smooth.None),
             Text(
               extent={{-38,50},{41,34}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="1"),
             Text(
               extent={{-27,-38},{44,-51}},
               lineColor={0,0,0},
               fillColor={255,255,255},
               fillPattern=FillPattern.Solid,
               horizontalAlignment=TextAlignment.Right,
              textString="2"),
             Text(
               extent={{-150,95},{150,65}},
               lineColor={0,0,255},
               textString="%name")}));
     end ConnectFlangesOf2States_specialCaseb;
   end Interfaces;

  annotation (uses(Modelica(version="3.2.1")),
  preferredView="info",
version="0.2",
versionBuild=0,
versionDate="2014-09-02",
dateModified = "2014-09-02 08:44:41Z",
revisionId="$Id:: MultiModeExamples.mo 2335 2016-04-26 #$",
    Documentation(info="<html>
<p>
Library <b>MultiModeExamples</b> is a Modelica package
to evaluate and test concepts for multi-model modeling.
</p>

<p>
The library contains multi-mode examples that should work with any Modelica 3.2
tool, by using special switch components
(sublibrary <a href=\"MultiModeExamples.WithSpecialSwitch\">WithSpecialSwitch</a>).
These examples allow to analyze the
occuring equation structures and the needed symbolic transformation algorithms
for multi-mode systems, without actually supporting continuous-time state machines.
</p>

<p>
The core of the library are multi-model models based on a 
Modelica extension that generalizes
synchronous state machines to continuous-time state machines
(sublibrary <a href=\"MultiModeExamples.WithStateMachine\">WithStateMachine</a>).
All models in this sub-library simulate in a Dymola prototype
(if you would like to get this prototype within the MODRIO project, please
contact <a href=\"mailto:Dan.HENRIKSSON@3ds.com\">Dan.HENRIKSSON@3ds.com</a>).
The details of the Modelica extensions and the symbolic transformation
algorithms are described in the publication:
</p>

<blockquote>
Elmqvist, Mattsson, Otter (2014): <b>Modelica extensions for Multi-Mode DAE Systems</b>.
Proceedings of the 10th International ModelicaConference, March 10-12, 2014,
Lund, Sweden, pp. 183-193.
</blockquote>

<p>
A contribution to this library is welcome. If you contribute, you agree that your
contribution is published under the Modelica License 2 license (see below).
You will then also be listed under \"Authors\".
</p>

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
<b>Copyright &copy; 2014, DLR Institute of System Dynamics and Control,
  Oberpfaffenhofen Germany and<br>
  Dassault Systèmes AB, Lund, Sweden.</b>
</p>
</html>"));
end MultiModeExamples;
