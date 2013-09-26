FILE(REMOVE_RECURSE
  "srv_gen"
  "src/daVinciGazeboPlugins/srv"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/daVinciGazeboPlugins/srv/__init__.py"
  "src/daVinciGazeboPlugins/srv/_SetJointState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
