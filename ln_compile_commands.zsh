#!/bin/zsh
(){
cd `dirname $0`

source /opt/ros/$ROS_DISTRO/setup.zsh
export COLCON_HOME=$PWD
export COLCON_BUILD_PATH=$COLCON_HOME"/build"

_c_paths=$(colcon list -p)
echo $_c_paths | while read _c_path
do
  _cmakelists_path=$_c_path"/CMakeLists.txt"
  if [[ -e $_cmakelists_path ]]; then
    project_name=$(echo $(cat $_cmakelists_path | grep "^project") | sed -e "s/^.*(\(.*\)).*$/\1/")
    if [ -n "$project_name" ]; then
      echo $project_name
      origin_ccj_path=$COLCON_BUILD_PATH"/"$project_name"/compile_commands.json"
      if [[ -e $origin_ccj_path ]]; then
        ln -sf $origin_ccj_path $_c_path"/"
      fi
    fi
  fi
done
}
