#!/bin/bash

# Pass the dirs of the models by command line args; if not set, defaults are used
MODELS_DIR=${1:-"../models"}
WORLDS_DIR=${2:-"../worlds"}

# Validate the models SDF schemas according to http://sdformat.org/schemas/root.xsd format
find ${MODELS_DIR} -type f  -name '*.sdf' -exec xmllint --format -schema http://sdformat.org/schemas/root.xsd {} --noout \;

# Validate the models SDF schemas according to http://sdformat.org/schemas/root.xsd format
find ${WORLDS_DIR} -type f  -name '*.world' -exec xmllint --format -schema http://sdformat.org/schemas/root.xsd {} --noout \;
