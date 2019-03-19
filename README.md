# Talon Robotics 2019 Robot Code

This project uses [ezAuton](https://github.com/ezAuton/ezAuton)
----

### Features

- Custom-made autonomous path tracking using [ezAuton](https://github.com/ezAuton/ezAuton)
- Vision processing in Python with OpenCV using [Vision2019](https://github.com/Team-2502/Vision2019)
  and custom-made socket communication code
    - Can align on all vision targets for quick, accurate pickup and placement of hatches
- Localization using both encoders and the Pigeon IMU
- Unit testing with Kotlin

### Setup Instructions

#### Cloning
##### Via command line

- `git clone --recursive https://github.com/Team-2502/RobotCode2019`

- `cd RobotCode2019`

- `./gradlew downloadAll`
 
##### Via GitKraken
 
 - `File -> Clone Repo
 
 - Clone RobotCode2019 repository

 <img src="https://i.imgur.com/BN0kRNt.jpg" width="498" height="318" />

#### Opening

##### In IntelliJ 
 
 - File -> New -> Project From Existing Sources...
 
 - Select the `RobotCode2019` directory
 
 - Import the project from the external model `Gradle`
 
  ![Imgur](https://i.imgur.com/qPqQpUv.jpg?1)
 
 - Select `Use auto-import`
 
  ![Imgur](https://i.imgur.com/eQeEA97.jpg)
  
  - then import `Team2502CS.xml` as a codestyle and use it for the project.
  
##### In Eclipse
 
 - Instructions found [here](http://lmgtfy.com/?q=how+to+install+intellij),
  with more documentation [here](https://www.scientificamerican.com/article/the-science-of-irrational/)

### Package Functions
- `com.team2502.robot2019.command`
  > Contains all commands and ezAuton actions
  
- `com.team2502.robot2019.command.autonomous.ingredients`
  > Contains base commands and actions for autonomous command groups
  
- `com.team2502.robot2019.command.teleop`
  > Contains all commands and ezAuton actions for teleop subsystems including drivetrain and active intake
  
- `com.team2502.robot2019.command.vision`
   > Contains all commands and ezAuton actions for vision based movement
   
- `com.team2502.robot2019.subsystem`
  > Contains all subsytems used by the robot
  
- `com.team2502.robot2019.subsystem.vision`
  > Contains the code for interfacing with the Raspberry Pi
