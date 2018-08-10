#!/bin/bash

EXECUTION_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$EXECUTION_DIRECTORY/../massim/massim18/server"

#IT IS NECESSARY TO RUN ONCE BEFORE "mvn install" IN THE ROOT PROJECT OF MASSIM

mvn exec:java -Dexec.args="--monitor -conf conf/BigConfig.json"

#Monitor available at http://localhost:8000/
