#!/bin/bash

VERSION_JAR=2017-0.7
VERSION_DIRECTORY=2017-1.7

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$SCRIPT_DIRECTORY/../../massim/massim-$VERSION_DIRECTORY/server"

java -jar "server-$VERSION_JAR-jar-with-dependencies.jar" --monitor

#Monitor available at http://localhost:8000/
