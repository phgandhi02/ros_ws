# ros_ws
This is my ROS2 Workspace

## What's in this repo directory?:
1. .vscode/
    - This directory holds all the files to ensure the vscode workspace is reproducable. Additionally, the development workflow can be easily used by simply opening this repo as the workspace in VS Code. All vscode tasks, extensions, and setting configurations are setup in here.
2. src/
    - This directory contains all the ROS packages. Each ROS2 package follows a basic structure. Additionally, each package should contain a README file that documents the code and file directory.
3. .gitignore
    - This file prevents git from tracking files and directories. Some reasons for not tracking files/directories are that the files are a result of build, log, and install functions ie. build/, log/, and install/. These directories appear after `colcon build` is run which autogenerates the following directories.
4. LICENSE
    - This file contains the License and usage parameters of the code.
5. README.md
    - This is the current file and it seeks to outline the purpose of this repo and detail any important install steps, common applications, and give a quick introduction to the repo. 