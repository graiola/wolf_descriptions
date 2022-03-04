#!/usr/bin/env bash

# Get this script's path
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

USAGE="Usage: \n create_gazebo_model [OPTIONS...] 
\n\n
Help Options:
\n 
-h,--help \tShow help options
\n\n
Application Options:
\n 
-r,--robot \tRobot name, example: -r spot"

ROBOT_NAME=""

while [ -n "$1" ]; do # while loop starts
	case "$1" in
	-r|--robot)
		ROBOT_NAME="$2"
		shift
		;;
	*) echo "Option $1 not recognized!" 
		echo -e $USAGE
		exit 0;;
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
mkdir -p $ROBOT_PATH/gazebo_model

# Create SDF
rosrun xacro xacro --check-order ${ROBOT_PATH}/robots/${ROBOT_NAME}.urdf.xacro > /tmp/${ROBOT_NAME}.urdf
gz sdf --print /tmp/${ROBOT_NAME}.urdf > ${ROBOT_PATH}/gazebo_model/model.sdf

# Create config

cat >> ${ROBOT_PATH}/gazebo_model/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>${ROBOT_NAME}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Gennaro Raiola</name>
    <email>gennaro.raiola@gmail.com</email>
  </author>
  <author>
    <name>Gennaro Raiola</name>
  </author>
  <description>
    ${ROBOT_NAME} robot model
  </description>
</model>
EOF

