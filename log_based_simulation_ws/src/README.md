# AWS RoboMaker Sample Application - Voice Interaction Log Based Simulation
This sample application demonstrates a robot that has voice controls via Amazon Lex, and can speak up if it encounters trouble using Amazon Polly.
In Usage section below, we are providing a tutorial to run this application is AWS RoboMaker. To learn more about AWS RoboMaker, please visit our web page https://aws.amazon.com/robomaker/.

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/voice-interaction/?region=us-east-1)._

# Usage
See https://aws.amazon.com/blogs/machine-learning/introducing-log-based-simulation-for-aws-robomaker/

## Regression Testing via AWS RoboMaker
If you would like to run the sample application as a regression test for the Voice Interaction robot application,
use `play_unpaused.launch` as the launch file in your AWS RoboMaker simulation job. This means that you can see the result from your simulation faster.

## Interacting via AWS RoboMaker
To inspect and interact with the job, start playback as paused. Still follow the [usage](#Usage) to setup resources but, instead, use `play_paused.launch` as the launch file. You may then interact via AWS RoboMaker console. 

### Pause/Unpause via AWS RoboMaker Terminal and the ROS CLI
1. In AWS RoboMaker console, open the Terminal GUI tool for your job.
1. To unpause log playback, run `rosservice call /rosbag_play/pause_playback '{data: False}'`
1. This will start the clock. To see, run `rostopic echo /clock`. 
1. To see if the `cmd_vel` on the robot is changed, run `rostopic echo /cmd_vel` and you should see the following being printed:
```
linear: 
  x: 5.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
```

See [AWS RoboMaker documentation](https://docs.aws.amazon.com/robomaker/latest/dg/simulation-job-playback-rosbags.html#simulation-job-playback-rosbags-cancel) for more usage commands.
    
# License

MIT-0 - See LICENSE.txt for further information

# How to Contribute

Create issues and pull requests against this Repository on Github
