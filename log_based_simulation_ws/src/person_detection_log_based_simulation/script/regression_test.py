#!/usr/bin/env python

import sys
import argparse
import rospy
from geometry_msgs.msg import Twist, Vector3
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags


IS_CANCELLED = False
DEFAULT_BAG_CLOCK_TIMEOUT_SECONDS = 60
X_CMD_VELOCITY = 5


def cancel_job():
    """
        Cancel a job. Use a global shared with other functions as the
        cancellation may take serveral seconds.
    """
    global IS_CANCELLED
    request_cancel = rospy.ServiceProxy('/robomaker/job/cancel', Cancel)
    response = request_cancel()
    if response.success:
        IS_CANCELLED = True
        rospy.loginfo("Successfully requested cancel job")
    else:
        rospy.logerr("Cancel request failed: %s", response.message)


def add_tags(tags):
    """
        See Tag key and value rules:
        https://docs.aws.amazon.com/robomaker/latest/dg/API_TagResource.html
    """
    request_add_tags = rospy.ServiceProxy('/robomaker/job/add_tags', AddTags)
    response = request_add_tags(tags)
    if not response.success:
        rospy.logerr("AddTags request failed for tags (%s): %s",
                     tags, response.message)


def has_changed_velocity(cmd_vel):
    """
        Pass the test if the command velocity on the robot changes
        and the x coordinate of the linear equals 5 (move forward at 5m/s)
    """
    if IS_CANCELLED:
        return

    rospy.loginfo("Test is checking if command velocity has changed.")
    if not IS_CANCELLED and cmd_vel.linear.x == X_CMD_VELOCITY:
        rospy.loginfo(
            "Command velocity changed on the robot, test passed and cancelling job")
        add_tags([Tag(key="status", value="passed")])
        cancel_job()


def timeout_test(timeout):
    """
        Cancel the test if it times out. The timeout is based on the
        /clock topic to simulate playback taking too long. Use the
        RoboMaker simulation job duration to timeout based on a
        wallclock duration.
    """
    rospy.loginfo("Test timeout called")
    if not IS_CANCELLED:
        rospy.loginfo("Test timed out, cancelling job")
        add_tags([Tag(key="status", value="timeout cancelled job")])
        cancel_job()
    else:
        rospy.loginfo("Test timed out, job already cancelled")
        add_tags([Tag(key="status", value="timeout job already cancelled")])


def run(clock_timeout):
    """
        Run the test. Connects to the /robomaker services for tagging and cancelling.
        Then subscribes to check if /cmd_vel is changed on the robot.
        Will timeout if /clock exceeds the specified duration.
    """
    rospy.init_node('robomaker_voice_interaction_regression_test')
    rospy.loginfo('Running AWS RoboMaker voice interaction regression test')
    rospy.on_shutdown(on_shutdown)

    # The timeout ensures the /robomaker services are running before continuing.
    rospy.wait_for_service('/robomaker/job/cancel', timeout=clock_timeout)
    rospy.wait_for_service('/robomaker/job/add_tags', timeout=clock_timeout)
    rospy.wait_for_service('/robomaker/job/list_tags', timeout=clock_timeout)

    # Tag this job as a test
    add_tags([Tag(key="robomaker_voice_interaction", value="regression_test")])

    # Now subscribe so we can check that the command velocity on the robot has changed
    rospy.Subscriber('/cmd_vel', Twist, has_changed_velocity)

    # If the /clock takes too long, then timeout the test
    # Note: this will not timeout if the bags are not played back. For a wallclock
    #       timeout, use the Job duration.
    rospy.Timer(rospy.Duration(clock_timeout), timeout_test, oneshot=True)
    rospy.spin()


def on_shutdown():
    """ Log a message when shutting down. """
    rospy.loginfo(
        "Shutting down AWS RoboMaker Voice Interaction regression test node")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--clock-timeout', type=int, default=DEFAULT_BAG_CLOCK_TIMEOUT_SECONDS,
                        help='/clock topic timeout in seconds to cancel job')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    run(clock_timeout=args.clock_timeout)
