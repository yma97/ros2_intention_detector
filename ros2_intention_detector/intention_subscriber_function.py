import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#from justhink_world import create_world, load_log, show_world
#from justhink_world.agent import Human, Robot
#from justhink_world.domain.action import PickAction, ClearAction, \
#AttemptSubmitAction, ContinueAction, SubmitAction, AgreeAction, DisagreeAction

class IntentionSubscriber(Node):

    def __init__(self):
        super().__init__('intention_subscriber')
        self.subscription = self.create_subscription(
            String,
            'intention',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # initialize the activity
#        # Create a world.
#        world = create_world('collaboration-1')

#        # Visualise the world on the current screen.
#        show_world(world, screen_index=0)

    def listener_callback(self, msg):
        self.get_logger().info('User wants to: "%s"' % msg.data)
        
        # Excute the action in the activity
 #       if msg.data == agree:
 #           world.act(AgreeAction(agent=Human))
 #       elif msg.data == disagree:
 #           world.act(DisagreeAction(agent=Human))


def main(args=None):
    rclpy.init(args=args)

    intention_subscriber = IntentionSubscriber()

    rclpy.spin(intention_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    intention_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
