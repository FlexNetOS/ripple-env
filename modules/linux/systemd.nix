# systemd user services
# ROS2 related systemd configurations
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
in
{
  # Example systemd user services for ROS2
  # Uncomment and customize as needed

  # systemd.user.services.ros2-daemon = {
  #   Unit = {
  #     Description = "ROS2 Daemon";
  #     After = [ "network.target" ];
  #   };
  #   Service = {
  #     Type = "simple";
  #     ExecStart = "${pkgs.bash}/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 daemon start'";
  #     ExecStop = "${pkgs.bash}/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 daemon stop'";
  #     Restart = "on-failure";
  #     RestartSec = 5;
  #   };
  #   Install = {
  #     WantedBy = [ "default.target" ];
  #   };
  # };

  # Example: Auto-start a ROS2 node
  # systemd.user.services.my-ros2-node = {
  #   Unit = {
  #     Description = "My ROS2 Node";
  #     After = [ "network.target" "ros2-daemon.service" ];
  #     Requires = [ "ros2-daemon.service" ];
  #   };
  #   Service = {
  #     Type = "simple";
  #     ExecStart = "${pkgs.bash}/bin/bash -c 'source /path/to/ws/install/setup.bash && ros2 run my_pkg my_node'";
  #     Restart = "on-failure";
  #     RestartSec = 10;
  #     Environment = [
  #       "ROS_DOMAIN_ID=0"
  #       "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  #     ];
  #   };
  #   Install = {
  #     WantedBy = [ "default.target" ];
  #   };
  # };

  # Keep this as a reference file
  home.file.".config/ros2/systemd-templates/ros2-node.service" = {
    text = ''
      # Template for ROS2 node systemd service
      # Copy to ~/.config/systemd/user/ and customize

      [Unit]
      Description=My ROS2 Node
      After=network.target

      [Service]
      Type=simple
      # Update these paths for your workspace
      ExecStart=/bin/bash -c 'source ~/ros2_ws/install/setup.bash && ros2 run my_pkg my_node'
      Restart=on-failure
      RestartSec=10
      Environment=ROS_DOMAIN_ID=0

      [Install]
      WantedBy=default.target
    '';
  };
}
