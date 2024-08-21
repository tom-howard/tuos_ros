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

if [ -z "$COLCON_PREFIX_PATH" ]; then
    echo "[EXITING] No ROS2 Workspaces detected."
    exit 0
fi

ROS2_WS=""
for path in ${COLCON_PREFIX_PATH//:/ }; do
    targetdir="$(dirname "$path")/src"
    echo "ROS2 Workspace detected at: '$targetdir'."
    if ask "Do you want to create your package here?"; then
        ROS2_WS=$targetdir
        break
    else
        continue
    fi
done

if [ -z "$ROS2_WS" ]; then
    echo "[EXITING] No path selected."
    exit 0    
fi

echo -n "[INPUT] Please enter a name for your package >> "
read -r PKG_NAME </dev/tty

if [ -z "$PKG_NAME" ]; then
    echo "[EXITING] No package name provided."
    exit 0
fi

PKG_PATH="$ROS2_WS/$PKG_NAME"

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
