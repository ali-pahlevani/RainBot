cmake_minimum_required(VERSION 3.8)
project(pickplace_arm_description)

find_package(ament_cmake REQUIRED)

# `meshes/` and `urdf/vendor/` hold vendored third-party geometry (Husky A200,
# FR3 arm + Franka Hand, RealSense/SICK housings); upstream licences sit
# alongside them as LICENSE.<package>.
# `fuel_cache/` is a vendored copy of the Gazebo Fuel models the Tugbot world
# pulls in, so it loads with no network access.
# `aws_hospital_models/` holds the AWS RoboMaker hospital models that
# aws_hospital.sdf references by model:// URI. Omitting it fails at runtime,
# not at build time.
install(
  DIRECTORY urdf launch rviz config worlds models meshes fuel_cache
            aws_hospital_models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
