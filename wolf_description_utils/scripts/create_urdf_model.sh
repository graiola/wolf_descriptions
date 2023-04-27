#!/usr/bin/env bash

# Get this script's path
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

USAGE="Usage: \n create_urdf_model.sh [OPTIONS...]
\n\n
Help Options:
\n
-h,--help \tShow help options
\n\n
Application Options:
\n
-r,--robot \tRobot name, example: -r spot\n
-d,--destination  \tDestination, example: -d /tmp"

ROBOT_NAME=""
DESTINATION="/tmp"

while [ -n "$1" ]; do # while loop starts
        case "$1" in
        -r|--robot)
                ROBOT_NAME="$2"
                shift
                ;;
        -d|--destination)
                DESTINATION="$2"
                shift
                ;;
        esac
        shift
done

if [[ ( $ROBOT_NAME == "")]]
then
        echo "Wrong robot option!"
        echo -e $USAGE
        exit 0
fi

ROBOT_PATH=`rospack find ${ROBOT_NAME}_description`
mkdir -p $DESTINATION

echo "I am going to parse the following file: ${ROBOT_PATH}/robots/${ROBOT_NAME}.urdf.xacro"

# Create URDF
rosrun xacro xacro ${ROBOT_PATH}/robots/${ROBOT_NAME}.urdf.xacro > /${DESTINATION}/${ROBOT_NAME}.urdf

echo "Finished!"
echo "Created URDF model in ${DESTINATION}/${ROBOT_NAME}.urdf"
