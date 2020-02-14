#!/bin/zsh
(){
source /opt/ros/$ROS_DISTRO/setup.zsh
export COLCON_HOME=$PWD"/"
export COLCON_LOG_PATH=$COLCON_HOME"log/"

_c_paths=$(colcon list -p)

echo $_c_paths | while read _c_path
do
  _cmakelists_path=$_c_path"/CMakeLists.txt"
  if [[ -e $_cmakelists_path ]]; then
    project_name=$(echo $(cat $_cmakelists_path | grep "^project") | sed -e "s/^.*(\(.*\)).*$/\1/")
    if [ -n "$project_name" ]; then
      num=$(echo $_cmakelists_path | sed -e "s/^\///" | sed -e "s/\//\n/g" | wc -l)
      for i in {$num..2}
      do
        cand_path=$(echo $_cmakelists_path | cut -d "/" -f1-$i)"/build"
        if [[ -e $cand_path"/.built_by" ]]; then
          origin_ccj_path=$cand_path"/"$project_name"/compile_commands.json"
          if [[ -e $origin_ccj_path ]]; then
            ln -s $origin_ccj_path $_c_path"/"
          fi
        fi
      done
    fi
  fi
done
}
