ModelicaXML unified XML Schemas for Modelica Models
===================================================
The ModelicaXML XML schemas aim to standardize an XML format for symbolic interchange of Modelica models, in a unified format that allows to represent models in various stages of processing.
The schemas are currently in development; the repository <https://github.com/modelica-association/ModelicaXML> holds the current version and is meant to be a platform for collaboration on schema development.

The schemas are meant to be general enough to allow to express Modelica models on at least four levels of processing:

1. Modelica source code. This format is an abstract representation of Modelica code and should provide a one to one mapping between Modelica source code and a corresponding XML representation. 
2. Instantiated hierarchical models. In this model representation, Modelica modifiers have taken effect, and the hierarchical structure is preserved. 
3. Flattened Modelica models. This model representation is flat in the sense that it does not contain hierarchical model elements. It may, however, contain array declarations.
4. Flattened transformed models. In this representation, array variables and equations have been scalarized, alias variables have been eliminated and DAE index has been reduced, if needed.

So far, the focus has been on level 4.

Much of the schema development work has taken place within the ITEA2 [MODRIO](https://itea3.org/project/modrio.html) project, deliverables D3.1.2, and D5.1.3.
The schemas are based on unified schemas by Hilding Elmqvist, and have taken influence from earlier discussion within the Modelica Association.
They are independent of the FMI XML schemas.
