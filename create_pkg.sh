#! /usr/bin/env bash

# A helper script to create a ROS2 package based on 
# https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/
# 
# Tom Howard, 2024

ask() {
    local reply
    echo -e -n "[INPUT] $1 [y/n] >> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

if [[ $# -eq 0 ]]; then
    echo -n "[INPUT] Please enter a name for your package >> "
    read -r PKG_NAME </dev/tty
else
    PKG_NAME=$1
fi

if [ -z "$PKG_NAME" ]; then
    echo "[EXITING] No package name provided."
    exit 0
fi
echo "$PKG_NAME"

if [ -z "$COLCON_PREFIX_PATH" ]; then
    echo "[EXITING] No ROS2 Workspaces detected."
    exit 0
fi

IFS=':' read -r -a COLCON_WS <<< "$COLCON_PREFIX_PATH"
echo "${#COLCON_WS[@]}"

NUM_WORKSPACES=${#COLCON_WS[@]}
VALID_INPUTS=""
ROS2_WS=""
if [[ $NUM_WORKSPACES -gt 1 ]]; then
    echo "Multiple ROS2 Workspaces detected..."
    for index in "${!COLCON_WS[@]}";
    do
        echo "  [$(($index+1))] ${COLCON_WS[index]}"
        if [[ $index -eq 0 ]]; then
            VALID_INPUTS="$(($index+1))"
        else
            VALID_INPUTS="$VALID_INPUTS, $(($index+1))"
        fi
    done
    echo -n "[INPUT] Which one do you want to use [$VALID_INPUTS]? >> "
    read -r WORKSPACE_IDX </dev/tty

    if [ -z "$WORKSPACE_IDX" ]; then
        echo "[EXITING] No option selected."
        exit 0
    elif (( $WORKSPACE_IDX > 0 && $WORKSPACE_IDX <= $NUM_WORKSPACES )); then
        ROS2_WS="$(dirname "${COLCON_WS[$(($WORKSPACE_IDX-1))]}")/src"
    else
        echo "[EXITING] Invalid option: '$WORKSPACE_IDX'."
        exit 0
    fi
else
    ROS2_WS="$(dirname "$COLCON_WS")/src"
fi

# if [ -z "$ROS2_WS" ]; then
#     echo "[EXITING] No path selected."
#     exit 0    
# fi

PKG_PATH="$ROS2_WS/$PKG_NAME"
echo "$PKG_PATH"

if [ -d "$PKG_PATH" ]; then
    echo "[WARNING] The '$PKG_NAME' ROS package (or a directory of the same name) already exists!"
    if ask "Do you want to replace it?"; then
        echo "Removing the existing directory..."
        rm -rf $PKG_PATH
    else
        echo "[EXITING] Start again with a new package name, or remove the existing one."
        exit 0
    fi
fi

echo "Creating the '$PKG_NAME' package at:"
echo "  $PKG_PATH"

ORG_PKG_NAME=ros2_pkg_template
git clone --quiet https://github.com/tom-howard/$ORG_PKG_NAME.git $PKG_PATH

cd $PKG_PATH 
rm -rf .git
mv $ORG_PKG_NAME/ $PKG_NAME/
mv include/$ORG_PKG_NAME/ include/$PKG_NAME/
sed -i '/<name>/s/'$ORG_PKG_NAME'/'$PKG_NAME'/' package.xml
sed -i '2 s/'$ORG_PKG_NAME'/'$PKG_NAME'/' CMakeLists.txt
