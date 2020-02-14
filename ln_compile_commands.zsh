#!/bin/zsh
(){
cd `dirname $0`

source /opt/ros/$ROS_DISTRO/setup.zsh
export COLCON_HOME=$PWD
export COLCON_BUILD_PATH=$COLCON_HOME"/build"

colcon_list=$(colcon list -p)
echo $colcon_list | while read package_dir
do
  package_cmake=$package_dir"/CMakeLists.txt"
  if [[ -e $package_cmake ]]; then
    package_name=$(echo $(cat $package_cmake | grep "^project") | sed -e "s/^.*(\(.*\)).*$/\1/")
    if [ -n "$package_name" ]; then
      echo $package_name
      compile_commands=$COLCON_BUILD_PATH"/"$package_name"/compile_commands.json"
      if [[ -e $compile_commands ]]; then
        ln -sf $compile_commands $package_dir"/"
      fi
    fi
  fi
done
}
