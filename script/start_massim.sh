#!/bin/bash

VERSION_JAR=2017-0.2
VERSION_DIRECTORY=2017-1.1

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$SCRIPT_DIRECTORY/../massim-$VERSION_DIRECTORY/server"

java -jar "server-$VERSION_JAR-jar-with-dependencies.jar" --monitor

#Monitor available at http://localhost:8000/ 
