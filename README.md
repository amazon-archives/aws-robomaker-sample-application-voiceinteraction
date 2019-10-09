# AWS RoboMaker Sample Application - Voice Interaction

This sample application demonstrates a robot that has voice controls via Amazon Lex, 
and can speak up if it encounters trouble using Amazon Polly.

Keywords: Robot Voice Interaction, AWS, Lex, Polly

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/voice-interaction/?region=us-east-1)._

## Requirements

- [ROS2 Dashing](https://index.ros.org//doc/ros2/Installation/Dashing) - Other versions of ROS2 may work, however they have not been tested
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html) - Used for building and bundling the application. 

## AWS Account Setup
	 
### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files](https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html) helpful.

### AWS Permissions
To run this application you will need an IAM user with the following permissions:
```
  lex:PostContent
  lex:PostText
  polly:SynthesizeSpeech
```

You can find instructions for creating a new IAM Policy [here](https://docs.aws.amazon.com/IAM/latest/UserGuide/access_policies_create.html#access_policies_create-start). In the JSON tab paste the following policy document:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Sid": "VoiceInteractionRobotRole",
      "Effect": "Allow",
      "Action": [
        "lex:PostContent",
        "lex:PostText", 
        "polly:SynthesizeSpeech"
      ],
      "Resource": "*"
    }
  ]
}
```

## Build 

### Pre-build commands

```bash
sudo apt-get update
rosdep update
```

### Robot

```bash
cd robot_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Simulation

```bash
cd simulation_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run 

First, you must create a LexBot by importing the JSON file `robot_ws/src/voice_interaction_robot/config/VoiceInteractionRobot.json` into Lex.  
You can read detailed instructions on how to import [here](https://docs.aws.amazon.com/lex/latest/dg/import-from-lex.html).

After the import is complete you must build the bot, and then publish it to an alias. 

Once the bot has been built and published to an alias, edit the file `robot_ws/src/voice_interaction_robot/config/lex_config.yaml` 
changing the bot_name and bot_alias variables to the bot you published in the Lex console. 

Launch the application with the following commands:

- *Running Robot Application on a Robot*
    ```bash
    source robot_ws/install/local_setup.sh
    ros2 launch voice_interaction_robot deploy_voice_interaction.launch.py
    ```

- *Running Robot Application Elsewhere*
    ```bash
    source robot_ws/install/local_setup.sh
    ros2 launch voice_interaction_robot voice_interaction.launch.py
    ```

- *Running Simulation Application*
    ```bash
    source simulation_ws/install/local_setup.sh
    ros2 launch voice_interaction_simulation bookstore.launch.py
    ```

### Run Options

You can set the following environment variables to configure your robot:

- `ROS_AWS_REGION` - Set the AWS region of the Lex bot you are connecting to. Defaults to the value of `aws_client_configuration.region` in `config/lex_config.yaml` if unset. 
- `LEX_USER_ID` - Set the UserID used when talking to the Lex bot. This is useful if you have multiple robots talking to the same Lex bot at the same time, as Lex will error if you send multiple commands using the same UserID to the same Lex bot at the same time. Defaults to the value of `lex_configuration.user_id` in `config/lex_config.yaml` if unset.  

## Test 

First, run the robot following the commands in the "Run" section. Then try some of the possible ways to test below.

#### All Possible Commands

- Wake:
    - jarvis
    - turtlebot
- Move:
    - move
    - go 
- Turn:
    - rotate 
    - turn
- Stop:
    - stop
    - halt
    - freeze

#### Testing with Text

```bash
source robot_ws/install/local_setup.sh
ros2 run voice_interaction_robot text_input.py
```

Try the following inputs:

```
jarvis
move
forward
5
```

This should send a move command to the robot


#### Testing with Audio files

```bash
source robot_ws/install/local_setup.sh
cd robot_ws/src/voice_interaction_robot/
ros2 run voice_interaction_robot audio_input.py
```

Once the script has loaded try the following inputs

```
jarvis
turn
clockwise
2
```

This will tell the robot to turn clockwise at a speed of "2". 

This loads .wav files from the `robot_ws/src/voice_interaction_robot/assets` directory and plays them to the robot. 
You can type the name of any of the audio files in this `assets` directory, or create your own via the commands in the "Creating Audio" section of this README.

#### Testing on a Robot
See the "Run" section for launching this sample application on a robot.

The microphone will be listening by default via the `audio_input` node however this package contains no node to send the wake command to the robot, and the robot will not send any audio to lex until it is awoken. 

To wake the robot up publish a message to the `/wake_word` topic with: `rostopic pub /wake_word std_msgs/String "wake"` or you can run the audio_input.py or text_input.py scripts on the robot and wake it using them. 

## Troubleshooting 

#### Audio Recording not working

Run `arecord -l` on your robot, which will list all recording devices connected to your robot. 
If no devices are listed there's an issue with your microphone not being detected by the robot. 

If a device is present then create the file `~/.asoundrc` with the following settings:

```
pcm.record {
    type plug;
    slave {
        pcm "hw:<CARD_ID>,<DEVICE_ID>"
    }
}
```

Replace `<CARD_ID>` and `<DEVICE_ID>` with the card number and device number listed in `arecord -l` for your device.

#### Audio Output not working

Run `aplay -l` on your robot, which will list all recording devices connected to your robot. 
If no devices are listed there's an issue with your speakers not being detected by the robot. 

If a device is present then create the file `~/.asoundrc` with the following settings:

```
pcm.play {
    type plug;
    slave {
        pcm "hw:<CARD_ID>,<DEVICE_ID>"
    }
}
```

Replace `<CARD_ID>` and `<DEVICE_ID>` with the card number and device number listed in `aplay -l` for your device.

### Creating Audio Files

You can create your own audio files for testing using the following command

```bash
aws polly synthesize-speech --output-format pcm --sample-rate 16000 --voice-id Joanna --text 'Hi lexbot' hi-lexbot.wav
```

## Using this sample with RoboMaker

You first need to install colcon-ros-bundle. Python 3.5 or above is required. 

```bash
pip3 install colcon-ros-bundle
```

After colcon-ros-bundle is installed you need to build your robot or simulation, then you can bundle with:

```bash
# Bundling Robot Application
cd robot_ws
source install/local_setup.sh
colcon bundle

# Bundling Simulation Application
cd simulation_ws
source install/local_setup.sh
colcon bundle
```

This produces the artifacts `robot_ws/bundle/output.tar` and `simulation_ws/bundle/output.tar` respectively. 

You'll need to upload these to an s3 bucket, then you can use these files to 
[create a robot application](https://docs.aws.amazon.com/robomaker/create-robot-application.html),  
[create a simulation application](https://docs.aws.amazon.com/robomaker/create-simulation-application.html), 
and [create a simulation job](https://docs.aws.amazon.com/robomaker/create-simulation-job.html) in RoboMaker.

### Interacting with this sample in RoboMaker

As the RoboMaker simulator doesn't support audio input or output you'll have to use the `audio_input.py` and `text_input.py` 
scripts to move the robot around. First, open a RoboMaker terminal from the simulation job page, then run the commands below:

```bash
# Initialize ROS environment variables, message types etc
eval $AWS_ROBOMAKER_ROBOT_APPLICATION_SETUP

# Run Text Input test script
ros2 run voice_interaction_robot text_input.py

# Run Audio Input test script
ros2 run voice_interaction_robot audio_input.py
```

## AWS ROS Packages used by this Sample

- RoboMakerUtils-Common
- RoboMakerUtils-ROS1
- RoboMaker-Lex-ROS1
- RoboMaker-TTS-ROS1

## ROS Nodes launched by this Sample 

### Nodes created by dependent packages 

```
/lex_node
/polly_node
/synthesizer_node
/tts_node
```

### Nodes created by this sample

```
/audio_output
/voice_command_translator
/voice_input
/voice_interaction
/voice_output
```

## ROS topics used by this sample

```
/clock
/cmd_vel
/tts/cancel
/tts/feedback
/tts/goal
/tts/result
/tts/status
/audio_input
/text_input
/wake_word
/audio_output
/text_output
/voice_interaction_node/fulfilled_command
/voice_output_node/speak
```

## License

MIT-0 - See LICENSE.txt for further information

## How to Contribute

Create issues and pull requests against this repository on Github


