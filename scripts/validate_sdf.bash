#!/bin/bash

# Pass the dirs of the models by command line args; if not set, defaults are used
MODELS_DIR=${1:-"../models"}
WORLDS_DIR=${2:-"../worlds"}
# Pass another schema as a 3rd argument, if one wants to use a customized or local schema
# Default: http://sdformat.org/schemas/root.xsd
SCHEMA=${3:-"http://sdformat.org/schemas/root.xsd"}

# Validate the models SDF schemas according to http://sdformat.org/schemas/root.xsd format
# Note: ignoring delta_wing.sdf as the SDFormat schema doesn't currently consider Xacro xmlns declarations
find ${MODELS_DIR} -type f -name '*.sdf' ! -name 'delta_wing.sdf' -exec xmllint --schema ${SCHEMA} {} --noout \;

# Validate the models SDF schemas according to http://sdformat.org/schemas/root.xsd format
find ${WORLDS_DIR} -type f -name '*.world' -exec xmllint --schema ${SCHEMA} {} --noout \;
